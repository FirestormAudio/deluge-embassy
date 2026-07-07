# Wavetable mip storage compaction — per-level resolution — design & spec

A foundational storage refactor for the [wavetable suite](2026-07-07-osc-wavetable-design.md),
and the enabler for **multi-frame morph** (256-frame banks all-resident). Today every
mip level is stored at the full base length `N = 2048` (`11×N ≈ 22528` f32/table). Each
level only carries `kmax(L) = (N/2)>>L` harmonics, representable in `~2·kmax` samples —
so a per-level-resolution pyramid is `~2N`, a **~5.5× storage reduction**, benefiting
every wavetable (single-cycle today; 256-frame banks next).

> **Status:** design proposal. Depends on merged 3a (`WtOsc`, `MipSet`, static tables),
> 3b (Pool-in-Engine, `Wavetable.from`, `PYRAMID_LEN`), 3c (IFFT `mipgen`). It is the
> first of two sub-projects toward Serum-scale morph (this, then multi-frame morph).

**Scope (chosen):** store each mip level at its own resolution (`~2N` total), with a
QA-measured oversampling factor. **Deferred:** the multi-frame morph itself; streaming.

---

## 1. Goals & non-goals

**Goals**

- **Per-level mip resolution.** Level `L` stored at `level_len(L)` samples (halving down
  the pyramid, floored + capped), total `COMPACT_LEN ≈ 2N` instead of `11N`.
- **Near-unchanged kernel.** `WtOsc::interp_cubic` already uses `table.len()` as its
  period and `MipSet { levels: &[&[f32]] }` already allows varying-length level slices —
  so the kernel *type and DSP* are unchanged; only the data it points at is smaller.
  Mip-select (pitch→level index) is unchanged.
- **Shared computed layout.** `level_len(L)`, `level_offset(L)`, `COMPACT_LEN` are pure
  functions of `L` (and the oversample factor) — the builder, the kernel/pooled reader,
  the pooled-region assembly, and the static registry all compute the same layout; no
  stored descriptor, no desync.
- **QA-tuned oversampling.** A factor `F` trades memory vs cubic-interp headroom at each
  level's top harmonic (`F=1` critical ~2N/5.5×; `F=2` ~4N/2.75× with headroom). The
  plan **measures the alias floor at `F=1` and `F=2` and picks the smallest that holds**
  (measure, don't predict).
- **Transparency.** The audio is band-limited-equivalent (no harmonic-content loss);
  alias floors re-measured; only `wavetables_generated.rs` bytes change (an intended
  re-pin). The wavetable tests are threshold-based (`worst_alias_db < gate`), so they
  stay green; the Osc-1/Osc-2 exact goldens are saw/LPF (not wavetables) — untouched.

**Non-goals (deferred / out of scope)**

- Multi-frame morph (the next sub-project; compaction is its enabler).
- Streaming / on-demand mip generation (all-resident is feasible once compact).
- Changing the mip-**select** math or the cubic/crossfade DSP (only per-level *length*
  changes).
- Changing `N`, `LEVELS`, or the number of harmonics per level.

---

## 2. Layout functions (`mipgen`)

The one source of truth, pure functions of the level index and a const oversample
factor `OS_FACTOR` (QA-set to 1 or 2):

```rust
pub const N: usize = 2048;
pub const LEVELS: usize = 11;
pub const N_MIN: usize = 32;      // interp floor (>= 4 for cubic; power of two)
pub const OS_FACTOR: usize = 1;   // QA-measured: 1 (critical, ~2N) or 2 (~4N)

/// Samples stored for mip level `L`. `kmax(L) = (N/2)>>L` harmonics need `2*kmax = N>>L`
/// critical samples; oversample by `OS_FACTOR`, clamp to `[N_MIN, N]`, round to pow2.
pub const fn level_len(level: usize) -> usize { /* clamp(OS_FACTOR*(N>>level), N_MIN, N), pow2 */ }
/// Byte/f32 offset of level `L` in a flat compact pyramid (prefix sum of level_len).
pub const fn level_offset(level: usize) -> usize { /* sum of level_len(0..level) */ }
/// Total f32 in one compact pyramid = level_offset(LEVELS).
pub const COMPACT_LEN: usize = /* level_offset(LEVELS) */;
```

`level_len` must be a power of two (for the per-size inverse FFT and clean interp),
monotonically non-increasing, `level_len(0) == N`, `level_len(L) >= N_MIN`.

---

## 3. Builder (`mipgen`)

Each level is built at its own `level_len(L)`, into a flat `[f32; COMPACT_LEN]` region:

- **Build each level via a per-size inverse FFT.** For level `L`, take the base's
  `kmax(L)+1` low bins (DC zeroed, matching 3c), inverse-real-FFT at size `level_len(L)`
  → `level_len(L)` samples. `deluge-fft`'s `RealFft<M, LANES>` is const-generic on size,
  so a `match`/const-generic dispatch over the (few, fixed) level sizes gives each level
  its own inverse. This is *cheaper* than the current uniform-`N` inverse (smaller
  transforms). **Fallback** (if per-size const-generic dispatch is fiddly): IFFT at `N`
  then decimate to `level_len(L)` — simpler, one transform size, slightly slower.
- Public API mirrors 3c but writes the compact layout:
  - `build_pyramid_flat_compact(base: &[f32], region: &mut [f32])` — region len `COMPACT_LEN`;
    writes level `L` to `region[level_offset(L) .. level_offset(L)+level_len(L)]`.
  - `build_all_additive` stays as the equivalence oracle (a compact-additive variant, or
    the existing full-`N` additive decimated per level for the cross-check).
- `gen_tables` emits flat compact `[f32; COMPACT_LEN]` per waveform + the registry.

---

## 4. Readers (kernel/graph)

- **Static registry** (`deluge-dsp-kernels`): `static_mipset(id)` builds a
  `MipSet { levels }` whose level `L` slice is `flat[level_offset(L) .. +level_len(L)]`
  from the generated flat array (instead of the current `[[f32;N];11]` nested arrays).
- **Pooled assembly** (`node.rs` `Kind::Wavetable` arm): assemble the `[&[f32]; LEVELS]`
  view from the flat pool region using `level_offset`/`level_len` instead of `L*N`; the
  exact-size guard becomes `region.len() == COMPACT_LEN` (or `>=`).
- `WtOsc::process` is **unchanged** — it reads whatever lengths `MipSet::levels` carry.
  (Confirm `interp_cubic` handles the smallest `N_MIN=32` level: it needs ≥4 points for
  the 4-tap wrap — 32 ≫ 4, fine.)

---

## 5. Ripples

- `deluge_wren_core::PYRAMID_LEN` → `COMPACT_LEN` (used by firmware `pool_alloc`, the 3b
  `build_pyramid_into`, and 3b/device tests). `build_pyramid_into` calls the new
  `build_pyramid_flat_compact`.
- **Pool sizing:** `PCAP` in the `Engine<...>` aliases can shrink (compact tables are
  ~5.5× smaller) or hold more tables. Update the test/firmware aliases; the firmware
  `const _: () = assert!(PCAP >= COMPACT_LEN)` still holds with margin.
- `wavetables_generated.rs` regenerated (flat compact) — an intended re-pin; committed.

---

## 6. QA acceptance & testing

- **Oversampling `F` decision (the key measurement).** Build the named tables at `F=1`
  and `F=2`; measure `worst_alias_db` per table at the same freqs as 3a/3b (the six
  named tables, 2k/5k). Pick the smallest `F` whose floors stay within the existing
  gates (the 3b per-table gate was `< -21 dB`). Record both measurements + the choice.
- **Audio equivalence (compact vs full-`N`).** A compact-built level, rendered, matches
  the full-`N`-built level's spectrum within tolerance (same harmonics, alias floor not
  worse than the gate) — the compaction doesn't change the *sound*, only the storage.
- **Per-table band-limit (regression).** The 3b `all_static_tables_band_limited` test
  (all six named tables) still clears its gate on the compact tables.
- **Layout functions.** Unit tests: `level_len(0)==N`, non-increasing, `>=N_MIN`, all
  powers of two; `level_offset` is the prefix sum; `COMPACT_LEN == level_offset(LEVELS)`
  and `≈ 2N` (for `F=1`).
- **Pooled + static round-trip.** A pooled wavetable built via `build_pyramid_flat_compact`
  and a static table both render correctly through `WtOsc` (bounded/non-silent; the 3b
  pooled-render test + a static-table test, updated for the compact layout).
- **`interp_cubic` at `N_MIN`.** The smallest level (32 samples) interpolates without
  OOB/panic (the 4-tap wrap holds).
- **Transparency.** Osc-1/Osc-2 exact goldens unchanged (not wavetables). All 3a/3b/3c
  threshold tests green on compact tables (re-measured, documented).

---

## 7. Deferred (tracked follow-ups)

- **Multi-frame morph** (the next sub-project) — `[frame][mip][phase]`, the `position`
  input, inter-frame interpolation, 256-frame banks all-resident on compact mips.
- Streaming / on-demand mips (only if banks exceed resident SDRAM).
- Per-level SIMD tuning of the smaller inverse transforms.

---

## 8. Open questions (resolved during implementation)

- `OS_FACTOR` (1 vs 2) — set from the §6 alias-floor measurement.
- `N_MIN` (interp floor) — 32 proposed; confirm no audible degradation at the top mip
  band (measure; a level this small is only used at the very highest pitches).
- Per-size inverse FFT (const-generic dispatch over level sizes) vs IFFT-at-`N`+decimate
  — implementer's call; both must pass the equivalence test.
- Whether `level_len` rounds up or down to a power of two when `OS_FACTOR*(N>>L)` isn't
  one (with `OS_FACTOR ∈ {1,2}` and `N` a power of two, `N>>L` is already a power of two,
  so rounding only matters if a future non-pow2 factor is chosen).
