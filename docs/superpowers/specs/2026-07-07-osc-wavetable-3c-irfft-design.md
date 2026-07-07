# Osc 3c — inverse real FFT + IFFT-based mipgen — design & spec

A performance follow-on to the [wavetable sub-project](2026-07-07-osc-wavetable-design.md).
3a (named static wavetables) and 3b (user-supplied dynamic tables) are merged, both
building band-limited mip pyramids by **additive resynthesis**. This cut adds a
**dedicated inverse real FFT** to `deluge-fft` and switches `mipgen`'s runtime
build path to **forward-FFT → zero high bins → inverse-FFT**, ~16× cheaper per
table — with additive kept as the equivalence oracle.

> **Status:** design proposal. Depends on merged `deluge-fft` (forward
> `RealFft`/`Fft`, compile-time twiddles), `mipgen` (additive builder), and the
> 3b `upload_table` runtime path. This resolves the wavetable spec's deferred
> item "**Inverse real FFT in `deluge-fft`** — added as a companion to 3b".

**Scope (chosen):** a **dedicated half-size inverse real FFT** (not the
full-N conjugation trick), + a `mipgen` IFFT build path, + switching the 3b
runtime `build_pyramid_into` onto it. **Deferred:** nothing new; this is purely
a speed/primitive addition.

**Why now:** it makes both remaining follow-ons cheaper — most directly the
**device-upload path**, whose real-time budget for building a table shrinks ~16×.

---

## 1. Goals & non-goals

**Goals**

- **A reusable `deluge-fft` inverse real FFT.** `RealFft::process_inverse`
  (`N/2+1` complex bins → `N` reals), the exact inverse of the merged forward
  `RealFft::process`, at the same ~2×-over-full-complex cost (half-size
  real-packing preserved). A genuinely reusable primitive (spectral effects,
  convolution, overlap-add), not just for wavetables.
- **`mipgen` IFFT build path.** Build a band-limited mip by forward-FFT of the
  base, zeroing bins above the level's cutoff, then inverse-FFT — ~16× cheaper
  than additive's `O(N·kmax)` per level.
- **Additive stays as the oracle.** The proven additive builder remains, renamed
  to make the two paths explicit; a test asserts the IFFT mips equal the additive
  mips within tolerance. Two independent references (round-trip + additive)
  validate the new inverse.
- **Runtime path switches; build-time doesn't.** The 3b `upload_table` →
  `build_pyramid_into` runtime path uses IFFT (the ~16× win where it matters).
  The static `gen_tables` build-time generator stays additive — no regeneration
  of the committed `wavetables_generated.rs`.
- **Transparency.** All existing tests pass; the committed static tables are
  **not** regenerated (byte-identical); the runtime-built tables change only
  within f32 tolerance (still clear the QA alias floors).

**Non-goals (deferred / out of scope)**

- The full-N conjugation-trick inverse (rejected: ~2× slower — it discards the
  real-packing half-size structure). *Note:* conjugation is still used for the
  **inner N/2 complex** inverse (see §2) — that's harmless and doesn't cost the 2×.
- A dedicated inverse *complex* FFT butterfly (the inner N/2 inverse reuses the
  forward via conjugation — same cost, near-zero new code).
- Regenerating the static named tables via IFFT (build-time; no benefit).
- Multi-frame morph, device-upload real-time path — separate sub-projects.
- SIMD-tuning the inverse beyond what reusing the forward radix-4 gives.

---

## 2. Architecture — Part A: the inverse real FFT (`deluge-fft`)

The forward `RealFft::process` does: real `x[N]` → **pack** `z[k]=x[2k]+j·x[2k+1]`
(half-size `z[N/2]`) → **forward FFT_{N/2}** (`process_r4_simd_soa`, unscaled) →
**split** post-step recovering the `N/2+1` one-sided bins `X` from conjugate
symmetry (using `W_N^k` from `TwiddleTable::<N>`, with the DC/Nyquist bins forced
purely real).

`RealFft::process_inverse(bins: &[Complex; N/2+1], out: &mut [f32; N])` reverses
each step exactly, in reverse order:

1. **Inverse-split** (`X → Z`, O(N)): the algebraic inverse of the forward split.
   For `k = 1..N/2-1`, given `X[k]` and `X[N/2-k]`, recover `Z[k]` and `Z[N/2-k]`
   (the forward split is a 2×2 linear map between the `(X[k], X[nk])` pair and the
   `(Z[k], Z[nk])` pair; invert it with the same `W_N^k`). Handle the DC and
   Nyquist bins (`X[0]`, `X[N/2]`, purely real) to reconstruct `Z[0]` (whose
   `re`/`im` carry the two real values). **The exact algebra is derived against
   the forward code and pinned by the round-trip oracle (§4)** — the spec fixes
   the structure, the implementer fixes the constants and proves them.
2. **Inverse FFT_{N/2}** (`Z → z`): reuse the *existing forward* complex FFT via
   `ifft(v) = conj(fft(conj(v)))`. Add `Complex::conj()` (trivial `{re, -im}`).
   This is the same flop cost as a dedicated inverse butterfly (one N/2 complex
   FFT) — the ~2× penalty only afflicts a *full-N* complex FFT, which this is not.
   No new butterfly/twiddle code.
3. **Unpack + normalize** (`z → x`, O(N)): `x[2k] = z[k].re`, `x[2k+1] = z[k].im`,
   scaled by the overall `1/N`. The forward chain is unscaled and the inner
   `conj(fft(conj))` is unscaled, so the total normalization is a single constant
   — its exact value is fixed by the round-trip oracle, not guessed.

**API shape** (matching the crate's const-generic bound idiom `[(); N]:`,
`[(); 2*(N/2)]:`):

```rust
impl Complex { pub fn conj(self) -> Self { Self { re: self.re, im: -self.im } } }

impl<const N: usize, const LANES: usize> RealFft<N, LANES> where /* same bounds */ {
    // existing:
    pub fn process(input: &[f32; N], out: &mut [Complex; N/2 + 1]);
    // new:
    pub fn process_inverse(bins: &[Complex; N/2 + 1], out: &mut [f32; N]);
}
```

`process_inverse` is exported the same way `process` is (via the `RealFft`
re-export in `lib.rs`).

---

## 3. Architecture — Part B: the `mipgen` IFFT path

`mipgen` currently: `analyze` (forward `RealFft` → `Harmonics{amp,phase}`) +
`synth_level`/`build_all` (additive). This cut adds an IFFT path and keeps
additive as the oracle:

- **Rename for clarity:** the additive `build_all` → `build_all_additive`
  (and keep `synth_level` for it + the oracle test). `analyze`/`max_harmonic`/`N`/
  `LEVELS`/`Harmonics` unchanged.
- **New IFFT path:** `build_level_ifft(spectrum: &[Complex; N/2+1], level, out: &mut [f32; N])`
  — copy `spectrum`, **zero every bin index `> max_harmonic(level)`**, then
  `RealFft::<N,4>::process_inverse` into `out`. And `build_all_ifft(base, out)`:
  forward-`RealFft` the base once → `build_level_ifft` per level.
- **`build_all` becomes the IFFT path** (the default the runtime uses). So
  `build_all` = `build_all_ifft`; additive is the explicitly-named alternative +
  oracle.

**Consumers:**
- **Runtime (3b):** `deluge-wren-core::build_pyramid_into` (called by
  `Host::upload_table`) builds level-by-level into a *flat* pool region, so it
  can't call the `[[f32;N];LEVELS]`-shaped `build_all`. Its switch: replace its
  `analyze` + per-level `synth_level` with one forward `RealFft::process(base)` →
  spectrum, then `build_level_ifft(&spectrum, level, &mut region[level*N..])` per
  level (this is why `build_level_ifft` is public). No signature change to
  `build_pyramid_into`; the ~16× win lands on every user-table upload. Requires
  `mipgen` to depend on `deluge-fft` already (it does) and `deluge-wren-core` to
  reach `RealFft` (via `mipgen`'s re-export or a direct dep — plan decides).
- **Build-time:** `mipgen/src/bin/gen_tables.rs` **stays on additive**
  (`build_all_additive`) — build speed is irrelevant, and this avoids
  regenerating (and re-pinning) the 1.87 MB committed `wavetables_generated.rs`.

---

## 4. QA acceptance & testing

**`deluge-fft` (Part A):**
- **Round-trip (the key gate).** `process_inverse(process(x)) ≈ x` for random real
  `x`, across sizes {16, 64, 256, 512, 1024} and multiple xorshift seeds, tolerance
  `~1e-2·sqrt(N)` — mirroring the existing `rustfft_oracle.rs` idiom.
- **vs `rustfft` inverse.** `process_inverse(bins)` vs `rustfft`'s
  `plan_fft_inverse` (real) on the same bins, same tolerance — an independent
  reference (the crate's test-dep already has `rustfft`).
- **Impulse / DC / Nyquist edge cases.** A single-bin spectrum inverts to the
  expected cosine; DC-only → constant; Nyquist-only → alternating — pinning the
  special-bin handling.
- `Complex::conj()` unit test (trivial).

**`mipgen` (Part B):**
- **IFFT ≡ additive (equivalence oracle).** For a saw (and an arbitrary-phase)
  base, `build_all_ifft` mips equal `build_all_additive` mips within tolerance,
  per level. This is the cross-check that the new inverse is correct *in the
  wavetable context*, independent of the round-trip test.
- **Band-limit correctness (independent FFT).** Each IFFT-built mip, measured with
  the independent `deluge-dsp-test` realfft, has ≈zero energy above its cutoff —
  the same gate 3a applied to additive.
- **Determinism.** Fixed input → identical IFFT mips.

**Runtime path (3b regression):**
- The `deluge-wren-core` dynamic-wavetable round-trip test (from 3b) still passes
  with `build_pyramid_into` now on IFFT — finite/non-silent/bounded, and the
  per-table band-limit still clears the measured floors. Any float drift from
  additive→IFFT stays within the existing gate margins (verify; re-measure if a
  gate is tight).

**Transparency:**
- The committed `wavetables_generated.rs` is **unchanged** (gen_tables stays
  additive) — verify it isn't regenerated.
- All merged 3a/3b/Osc-1/Osc-2 tests and both goldens pass unchanged.

**Determinism note:** the inverse is deterministic; IFFT vs additive differ only
by f32 rounding, well inside the QA margins.

---

## 5. Deferred (tracked follow-ups)

- **Device-upload real-time path** (the sibling follow-on) — now with a ~16×
  cheaper build, easing its real-time budget.
- **Multi-frame morphing wavetables** — the marquee expressive follow-on.
- A dedicated inverse *complex* FFT butterfly (if a future consumer wants the
  inner inverse without the two `conj` sweeps — negligible today).
- SIMD-specific tuning of the inverse-split / unpack passes.

---

## 6. Open questions (resolved during implementation)

- The exact inverse-split algebra and the single `1/N` normalization constant —
  structure fixed here; constants derived against the forward code and pinned by
  the round-trip oracle (§4).
- Whether `process_inverse` reuses `process_r4_simd_soa` directly (via
  `conj`-wrapping into an `FftBuf`) or a thin `ifft` helper — implementer's call;
  both same cost.
- Whether to expose `mipgen::build_level_ifft` publicly or keep it internal to
  `build_all_ifft` (public helps the device-upload path build incrementally).
