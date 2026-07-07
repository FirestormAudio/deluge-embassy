# Osc part 3 — single-cycle band-limited wavetable oscillator — design & spec

The third cut of the `Osc` suite. The first
([anti-aliased classic oscillators](2026-07-07-osc-antialiased-oscillators-design.md))
and second ([expressive PWM/PM/FM](2026-07-07-osc-expressive-pwm-fm-design.md)) are
merged. This cut adds a **single-cycle wavetable oscillator**: play an arbitrary
single-cycle waveform without aliasing, via a **build-time-generated, mip-mapped,
band-limited table pyramid** the `no_std` kernel merely reads and interpolates.

> **Status:** design proposal. Depends on merged Osc-1 (PolyBLEP kernel + the
> phase machinery), Osc-2 (the `freq`/`pmod` port scheme), QA (`deluge-dsp-test`,
> realfft), and the existing (but not-yet-node-integrated) `Pool` in
> `deluge-audio-graph`. This resolves the Osc-1 open item "wavetable oscillator —
> its own sub-project; needs table storage (the buffer pool)".

**Scope (chosen):** single-cycle band-limited wavetable, with **both** table
sources — built-in named static tables **and** user-supplied dynamic tables
(Pool-backed). **Deferred:** multi-frame morphing wavetables (position/scan),
hard sync, noise variants (separate sub-projects).

**Execution split (three plans under this one spec):**
- **3a — named static wavetables** (MERGED): `mipgen` additive builder, kernel
  `WtOsc`, generated static consts, `Osc.wavetable(WT.x, freq)`.
- **3b — user-supplied dynamic tables** (this plan): Pool-in-Engine, a
  `Host::upload_table` seam, `SlotApi` list-read, `Wavetable.from`,
  `TableSrc::Pooled`, `Node.free()` → pool free — all on the **additive**
  `mipgen` (no `deluge-fft` change).
- **3c — IRFFT optimization**: add an inverse real FFT to `deluge-fft`, switch
  `mipgen` to the IFFT build path (~16× cheaper per table), with additive kept
  as the equivalence oracle. Pure performance; ships after 3b.

---

## 1. Goals & non-goals

**Goals**

- **Anti-aliased arbitrary-table playback.** A `Kind::Wavetable` node plays a
  single-cycle waveform pitch-safely across the audio range, using a mip pyramid
  (one band-limited table per octave) selected by frequency + interpolated.
- **Two table sources, one kernel.** Built-in **named static** tables
  (`&'static` pre-baked mips) and **user-supplied dynamic** tables (built at
  runtime, stored in the `Pool`) share the exact same DSP path — the kernel reads
  a borrowed `MipSet` and is blind to the backing.
- **First real Pool→node integration.** The dynamic path allocates a Pool region,
  fills it with the mip pyramid, binds it to the node, and frees it on
  `Node.free()`.
- **`no_std`-pure player, `no_std` builder.** The kernel does mip-select +
  interpolation only (no FFT, no alloc). The FFT-based mip **builder** (`mipgen`)
  uses the project's own `no_std` **`deluge-fft`** (stack scratch, fixed 2048
  tables) and **writes directly into a caller-provided buffer** (the Pool slice,
  or a host buffer for static consts) — so it needs no heap either. QA verifies
  the result with the *independent* `deluge-dsp-test` realfft (build with one FFT,
  grade with another).
- **Modulation reuse.** Reuses Osc-2's `freq` (port 0) and `pmod` (port 1);
  phase-mod indexes a phase-addressed table naturally.
- **Transparency.** Purely additive — new `Kind`, new modules. Existing
  Osc-1/Osc-2 goldens and all prior tests pass unchanged.
- **QA-proven.** Pitch-safety, mip-crossfade continuity, and builder band-limit
  correctness all gated with measured `worst_alias_db`/spectral checks.

**Non-goals (deferred / out of scope)**

- **Multi-frame morphing wavetables** (a stack of frames + a `position`/scan
  input) — the natural follow-on; this cut is single-cycle. Port 2 (`width`) is
  reserved for the morph `position` then.
- **Hard sync**, **noise variants** (pink/brown) — separate sub-projects.
- **Feedback FM on wavetables** (phase-feedback on a table; niche — revisit with
  morph).
- Building tables whose base length differs from the fixed size (2048) — one base
  length this cut; variable lengths are a later refinement.
- Bumping `MAX_INPUTS`. (`deluge-fft` already brings SIMD to the build path; the
  kernel interpolation stays scalar this cut.)

---

## 2. Architecture — one kernel, two backings

### 2.1 The kernel (`deluge-dsp-kernels`, new `wavetable.rs`)

`WtOsc::process` is source-agnostic: it reads its mips through a borrowed
`MipSet<'a>` — a view over N band-limited mip levels (each a power-of-two
single-cycle table) — and neither knows nor cares whether the backing is
`&'static` consts or a `Pool` region.

```rust
pub struct WtOsc { phase: f32 }
// A borrowed view of the mip pyramid for one block. Concrete form — an array of
// sub-slices vs a single data slice + a level-layout descriptor (len, count) —
// is a plan-time detail; the Pooled path favors one contiguous slice + layout.
pub struct MipSet<'a> { /* levels, resolved from static consts or pool.slice() */ }
// process(&mut self, mips: MipSet, freq: In, pmod: In, dt, out)
// level 0 = fullest band; higher levels = progressively fewer harmonics.
```

Per sample:
1. `dtp = freq.at(i) * dt` (cycles/sample) → **mip-select**: pick the level whose
   highest harmonic is still below Nyquist at this `dtp` (≈ `log2` of the phase
   increment), and the adjacent finer level for the crossfade.
2. **Effective phase** `ph = frac(phase + pmod.at(i))` (reuses Osc-2's phase-mod;
   `frac(x) = x - floorf(x)`).
3. **Sample the table(s)** at `ph` with **cubic Hermite** interpolation within a
   level, then **linear crossfade** between the two bracketing mip levels (kills
   the octave-boundary zipper).
4. Advance `phase += dtp`, wrap via `floorf`.

The kernel is `no_std`, no alloc, no FFT. Mip parameters (**2048-sample base
tables; ≈1 mip/octave, ~11 levels; cubic-within + linear-between**) are the
standard sweet spot; QA measures and confirms (like Osc-1's thresholds).

### 2.2 Two backings

| | Named / static | User-supplied / dynamic |
|---|---|---|
| Owner of bytes | `&'static [f32]` consts | `Pool` region (`PoolHandle`) |
| When built | build time (`mipgen` generator) | call/load time (`mipgen::build`) |
| Node binding | `TableSrc::Static(&'static MipSet)` | `TableSrc::Pooled(PoolHandle)` |
| Freed | never (static) | `Node.free()` → `Pool.free(handle)` |

Both resolve to a `MipSet` at process time; the DSP path is identical.

### 2.3 Node/graph (`deluge-audio-graph`)

- New `Kind::Wavetable` → `State::Wt(WtOsc)`. The osc-family dispatch passes
  `ins[0]` (freq) and `ins[1]` (pmod) to `WtOsc::process`; port 2 unused this cut.
- `WtOsc` node state carries a `TableSrc` (`Static(&'static …)` or
  `Pooled(PoolHandle)`); each block the engine resolves it to a `MipSet` — static
  slices directly, or `pool.slice(handle)` sub-divided into the N mip levels by a
  known layout — and calls `process`.
- **Pool ownership:** the Engine owns the Pool (the render reads it). The Host
  exposes `upload_table(&[f32]) -> Option<PoolHandle>` that runs `mipgen` and
  fills the Pool between blocks; the node-create carries the resulting handle.
  Exhaustion → `None` → the node **degrades gracefully** (fall back to a default
  static table), never panics.

---

## 3. Data flow & the `mipgen` builder

### 3.1 `mipgen` — a `no_std` builder using `deluge-fft`

Builds via the project's own `no_std`, no-heap `deluge-fft` (`RealFft<2048, …>`,
compile-time twiddles, stack scratch): base single-cycle → forward FFT → for each
octave mip, zero every harmonic above that octave's safe count → inverse FFT → one
mip level, **written directly into a caller-provided output slice** (level by
level, by a known layout). No heap, no std in the builder itself.

- The **player** (kernel `WtOsc`) is pure `no_std`.
- The **builder** (`mipgen`) is also `no_std`/no-alloc — the fixed 2048 table size
  bounds all scratch to the stack, and output goes into the caller's buffer. So
  even a `no_std`-*without-alloc* host can build user tables (bounded by the fixed
  table size); it isn't limited to named static tables.
- **QA independence:** `mipgen` builds with `deluge-fft`; the acceptance tests
  measure the result with `deluge-dsp-test`'s *independent* realfft — we never
  grade a table with the same FFT that produced it.

The only std-side pieces are the build-time `gen-tables` generator (a host binary
that emits the `&'static` const file) and the Wren host reading a script's sample
list into a fixed scratch buffer.

### 3.2 Named-table flow (static, build-time)

`mipgen` runs at build time via a small generator (`cargo run -p mipgen --bin
gen-tables`) that emits a **committed** `tables_generated.rs` of `&'static` mip
consts. Deterministic, reviewable, regenerated on demand — no FFT in the build
graph. QA gates the generated tables' alias floor.

Named set (first cut): **`WT.Saw`, `WT.Square`, `WT.Sine`, `WT.Tri`, + 2 richer**
(e.g. a formant-ish and a bright/organ-ish single cycle) — final palette chosen
during implementation.

### 3.3 User-table flow (dynamic, runtime)

```
Wavetable.from(samples)              // Wren list of single-cycle samples
  → binding reads list → scratch [f32]
  → mipgen::build(&samples)          // control side (std/alloc), FFT
  → Host.upload_table(...) → Pool.alloc + fill → PoolHandle
  → node = Kind::Wavetable bound to TableSrc::Pooled(handle)
  → per block: engine resolves handle → MipSet → WtOsc::process(mips, freq, pmod, dt, out)
  → Node.free() → Pool.free(handle)
```

### 3.4 Open question (resolved at plan time)

Reading a Wren **list** of floats into Rust needs a slot-list-read primitive
(count + element-get) on both the `wren-sys` backend and the test `SlotApi`. If it
isn't already present, the plan adds it (small) or falls back to a foreign buffer
object. Verified during planning.

---

## 4. Wren surface

```wren
// Named (static, no Pool):
var s = Osc.wavetable(WT.Saw, 110)     // WT.Saw / .Square / .Sine / .Tri / +2 richer
s.pm = lfo                              // reuses Osc-2 phase-mod (port 1)

// User-supplied (dynamic, Pool-backed):
var w = Wavetable.from(mySamples)       // builds mips + uploads once
var v = Osc.wavetable(w, 220)           // play a built table
```

- `WT` is a small holder of named-table ids.
- `Wavetable.from(list)` returns a handle bound to a Pool region (freed when it
  and its nodes are released).
- `Osc.wavetable(src, freq)` accepts either a `WT` name or a `Wavetable` handle.
- Existing `Osc.saw/square/…` and all Osc-2 setters are **untouched** (transparency
  — purely additive).

---

## 5. QA acceptance & testing

**Kernel (`deluge-dsp-kernels`, `WtOsc`; via the `deluge-dsp-test` dev-dep):**

- **Pitch-safety (core gate).** Each named table at 2k/5k/8k Hz @ 48k →
  `worst_alias_db` below the **measured** floor; mip-select keeps it band-limited.
- **Improvement over naive.** A single-table-no-mips reference at the same pitch
  aliases badly; the mip'd version is **> N dB** better (N set from measurement).
- **Mip-crossfade continuity.** Sweep frequency across an octave boundary → **no
  click/zipper**; level + spectral continuity through the switch (what the linear
  inter-mip crossfade buys).
- **Low-freq fidelity.** At ~100 Hz the intended waveform is intact (fundamental +
  harmonic series present, THD sane) — cubic interp doesn't distort.
- **Bounds proptest.** Output finite + bounded (±1.1 or a documented
  interp-overshoot bound) over random table/freq/pmod.

**Builder (`mipgen` crate):**

- **Band-limit correctness.** With the *independent* `deluge-dsp-test` realfft,
  FFT each generated mip and assert energy above its target harmonic cutoff ≈ 0
  (built by `deluge-fft`, graded by a different FFT).
- **Determinism.** Fixed input → identical mips (named consts and user tables
  reproduce host/device).

**Graph (`deluge-audio-graph`):**

- **Pool lifecycle.** User-table alloc → fill → play → `Node.free()` → region
  reclaimed (reuse works). **Exhaustion → graceful degrade** (`None` → fallback),
  never panics.

**Wren (`deluge-wren-core`):**

- **Both authoring paths.** `Osc.wavetable(WT.Saw, f)` (static) and
  `Wavetable.from(samples)` (dynamic) each produce a node that renders the expected
  spectrum; the list-read path round-trips the samples.

**Transparency (regression).** Purely additive — the existing Osc-1/Osc-2 goldens
(`golden_saw_lpf_env_first_block`, `golden_saw_lpf_renders_expected_block`) and all
prior tests pass **unchanged**.

**Determinism.** `mipgen` is deterministic (FFT of fixed input); static tables are
identical host/device; the `WtOsc` playback has no RNG.

---

## 6. Deferred (tracked follow-ups)

- **Multi-frame morphing wavetables** — a frame stack + a `position`/scan input
  (port 2), inter-frame interpolation, and its anti-aliasing interactions. The
  direct successor.
- **Hard sync**, **noise variants** (pink/brown) — separate sub-projects.
- Feedback FM on wavetables; SIMD table interpolation; through-zero refinements.
- Larger/curated named-table palette; user-table hot-reload.
- **Inverse real FFT in `deluge-fft`** — added as a companion to **3b** (runtime
  user-table building), where it cuts per-table build cost ~16× vs additive
  resynthesis. 3a's additive `mipgen` (`build with deluge-fft forward + additive
  synth`) becomes the IRFFT path's **equivalence oracle** in 3b's tests. Not
  needed for 3a (build-time named tables; additive and IFFT are identical for a
  periodic single cycle).
- **Wavetable region ownership** — pooled table memory is currently freed
  node-scoped (`Cmd::Free`), not by the `Wavetable` object's GC; an unbound or
  multiply-bound `Wavetable` can leak or alias its region (graceful →
  silence, never UB). A proper object-scoped ownership model
  (finalizer/refcount) is a follow-on.

---

## 7. Open questions (resolved during implementation)

- Wren list-read primitive vs foreign buffer object for `Wavetable.from` (§3.4).
- Exact mip parameters (table length, mips/octave, interp order) — set from the
  §5 pitch-safety + continuity measurements, like Osc-1.
- Whether named tables are generated by a committed `gen-tables` binary or a
  `build.rs` (leaning committed binary → reviewable generated file).
- The graceful-degrade fallback on Pool exhaustion (default static table vs
  silence) — decided from the graph lifecycle test.
- Pool `CAP`/`CHUNK` sizing for a realistic mip pyramid (2048-base × ~11 levels ≈
  a few k f32s per table) — set when the Pool is instantiated in the engine.
