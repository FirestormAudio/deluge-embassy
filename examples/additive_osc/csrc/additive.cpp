// Additive oscillator DSP core — C++/Argon (ARM NEON SIMD), reached from Rust
// over a C ABI.
//
// This is the "real-world DSP" half of the `additive_osc` example. The Rust app
// (src/main.rs) owns the pads, envelopes and the OLED scope; every audio block
// it hands the active voices to `additive_render`, which synthesises the actual
// sound here using Argon — a header-only, zero-overhead C++ wrapper over ARM's
// NEON intrinsics (https://github.com/stellar-aria/argon).
//
// Why Argon / SIMD at all? Additive synthesis is a sum of many sine partials,
// which is exactly the kind of wide, regular, branch-free arithmetic SIMD eats
// for breakfast. We process four samples at a time (one `Argon<float>` = a NEON
// `float32x4_t`), and inside each group sum every harmonic. On the device this
// compiles straight to NEON; on the x86-64 simulator host Argon transparently
// falls back to SIMDe, so the same source runs under `cargo deluge sim`.
//
// Argon ships no transcendentals (it is deliberately not an algorithms library),
// so the sine itself is ours: a vectorised parabolic approximation, the SIMD
// twin of the scalar `sine()` in the stock `test_osc` example.
//
// The firmware links newlib's libc (for `memset`) but NOT its libm, and is built
// without C++ exceptions/RTTI — so this file stays libm-free (no powf/floorf:
// the roll-off is an integer order built by repeated multiply, and phase is
// wrapped by integer truncation) and self-contained.

#include <argon.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {

constexpr float kPi = 3.14159265358979323846f;

/// Largest harmonic count the caller may request. Bounds the per-block amplitude
/// table so we never touch the heap.
constexpr std::size_t kMaxHarmonics = 64;

/// Fractional part of a non-negative vector, i.e. `x - floor(x)`. Each partial's
/// phase is `harmonic * fundamental_phase`, which we wrap into [0, 1) before
/// evaluating the sine. `x` is always >= 0 here, so truncation == floor: convert
/// to int (NEON truncates toward zero) and back, then subtract.
inline Argon<float> Fract(Argon<float> x) {
  Argon<float> truncated = x.ConvertTo<std::int32_t>().ConvertTo<float>();
  return x - truncated;
}

/// Vectorised parabolic sine. `phase` is in cycles; only its [0, 1) fraction is
/// meaningful (the caller wraps it). Mirrors `test_osc`'s scalar `sine()`:
///   a   = (phase*2 - 1) * PI            // map [0,1) -> [-PI, PI)
///   y   = (4/PI)*a - (4/PI^2)*a*|a|     // base parabola
///   out = -(0.225*(y*|y| - y) + y)      // one refinement pass for low THD
///        = -(0.225*y*|y| + 0.775*y)
///
/// Every multiply-then-add/sub is a fused `MultiplyAdd`/`MultiplySubtract`, so
/// each maps to a single NEON `vfma`/`vfms` rather than a separate multiply and
/// add — the reason to reach for Argon in the first place.
inline Argon<float> ParabolicSine(Argon<float> phase) {
  // Map phase [0,1) -> a in [-PI, PI):  a = 2*PI*phase - PI   (fold *PI in: one FMA)
  Argon<float> a = Argon<float>{-kPi}.MultiplyAdd(phase, 2.0f * kPi);

  // Base parabola:  y = (4/PI)*a - (4/PI^2)*(a*|a|)
  Argon<float> a_abs_a = a.Multiply(a.Absolute());
  Argon<float> base = a.Multiply(4.0f / kPi);
  Argon<float> y = base.MultiplySubtract(a_abs_a, 4.0f / (kPi * kPi));

  // One refinement pass:  out = -(0.225*(y*|y|) + 0.775*y)
  Argon<float> y_abs_y = y.Multiply(y.Absolute());
  Argon<float> weighted = y.Multiply(0.775f);
  Argon<float> refined = weighted.MultiplyAdd(y_abs_y, 0.225f);
  return -refined;
}

}  // namespace

/// One synth voice, shared field-for-field with the Rust `#[repr(C)]` struct.
/// `phase` is the voice's *fundamental* phase in cycles [0, 1); we read it at the
/// top of the block and write the advanced, wrapped value back so partials stay
/// phase-locked across blocks. Tracking only the fundamental is enough: harmonic
/// `h`'s phase is just `fract(h * phase)`.
/// The envelope arrives as a PAIR and is interpolated across the block. Holding
/// one `amp` constant for all `n` samples makes the envelope a staircase that
/// steps every block: silent in steady state (successive values are equal), but
/// through an attack or release it is a burst of ~10 amplitude discontinuities,
/// which is audible as static on note onset and release.
///
/// Field order must match the Rust `#[repr(C)] struct AdditiveVoice`.
struct AdditiveVoice {
  float freq;       ///< fundamental frequency, Hz
  float amp_start;  ///< envelope at the first sample of the block
  float amp;        ///< envelope at the last sample of the block
  float phase;      ///< fundamental phase in cycles, [0, 1); updated in place
};

extern "C" {

/// Render `n` mono samples summing every active voice's additive tone into
/// `out` (overwritten, not accumulated onto prior contents).
///
/// Each voice contributes `n_harmonics` sine partials at `h * freq` with a
/// `1 / h^rolloff` spectral roll-off (rolloff 0 = all partials equal/buzzy,
/// 1 ≈ sawtooth, 2 ≈ darker/triangle-ish), the whole stack normalised so a
/// voice's peak tracks its `amp` regardless of harmonic count. Partials above
/// Nyquist are dropped per voice to avoid aliasing.
///
/// `out` length `n` must be a multiple of 4 (the audio block, 128, always is).
/// Each voice's `phase` is advanced by `n` samples and written back.
void additive_render(float* out,
                     std::size_t n,
                     AdditiveVoice* voices,
                     std::size_t n_voices,
                     std::uint32_t n_harmonics,
                     std::uint32_t rolloff,
                     float sample_rate) {
  std::memset(out, 0, n * sizeof(float));

  if (n_harmonics < 1) {
    n_harmonics = 1;
  }
  if (n_harmonics > kMaxHarmonics) {
    n_harmonics = kMaxHarmonics;
  }

  // Per-block harmonic amplitude table: 1/h^rolloff (built libm-free by repeated
  // division), normalised so the partials sum to 1 — keeps total level
  // independent of harmonic count / roll-off. Depends only on
  // (n_harmonics, rolloff), so it is built once for all voices.
  float harm_amp[kMaxHarmonics];
  float sum = 0.0f;
  for (std::uint32_t h = 1; h <= n_harmonics; ++h) {
    float a = 1.0f;
    for (std::uint32_t k = 0; k < rolloff; ++k) {
      a /= static_cast<float>(h);
    }
    harm_amp[h - 1] = a;
    sum += a;
  }
  const float norm = sum > 0.0f ? 1.0f / sum : 1.0f;

  const float nyquist = sample_rate * 0.5f;
  const std::size_t groups = n / 4;  // 4 samples per NEON vector

  for (std::size_t v = 0; v < n_voices; ++v) {
    AdditiveVoice& voice = voices[v];
    const float inc = voice.freq / sample_rate;  // cycles per sample
    const float amp0 = voice.amp_start;
    const float amp1 = voice.amp;

    // Skip silent voices, but still advance their phase so they re-enter in
    // phase when the envelope opens again. Test BOTH ends: the first block of
    // an attack starts at 0, and testing only the start would drop it (and
    // testing only the end would drop the last block of a release).
    if ((amp0 > 1.0e-5f || amp1 > 1.0e-5f) && voice.freq > 0.0f) {
      // Drop partials that would alias past Nyquist for this voice's pitch.
      std::uint32_t h_max = n_harmonics;
      if (inc > 0.0f) {
        std::uint32_t limit = static_cast<std::uint32_t>(nyquist / voice.freq);
        if (limit < h_max) {
          h_max = limit;
        }
      }

      // Per-lane phase of the four samples in the current group:
      // phase + inc * {0,1,2,3} (one FMA), advanced by inc*4 each group.
      Argon<float> phase = Argon<float>{voice.phase}.MultiplyAdd(Argon<float>::Iota(0.0f), inc);
      const Argon<float> phase_step{inc * 4.0f};

      // Per-lane envelope, ramped linearly from amp_start to amp across the
      // block: amp0 + damp * {0,1,2,3}, advanced by damp*4 each group — the
      // same shape as the phase ramp above.
      const float damp = (amp1 - amp0) / static_cast<float>(n);
      Argon<float> amp_v = Argon<float>{amp0}.MultiplyAdd(Argon<float>::Iota(0.0f), damp);
      const Argon<float> amp_step{damp * 4.0f};

      for (std::size_t g = 0; g < groups; ++g) {
        Argon<float> acc{0.0f};
        for (std::uint32_t h = 1; h <= h_max; ++h) {
          Argon<float> partial_phase = Fract(phase.Multiply(static_cast<float>(h)));
          // Envelope is NOT folded in here: summing the partials unscaled lets
          // it be applied once per group below instead of once per harmonic,
          // which is both cheaper and what makes the per-sample ramp possible
          // (the gain is now a vector, not a scalar).
          float gain = harm_amp[h - 1] * norm;
          // acc += sine(partial_phase) * gain  — a single NEON FMA.
          acc = acc.MultiplyAdd(ParabolicSine(partial_phase), gain);
        }
        float* slot = out + g * 4;
        Argon<float>::Load(slot).MultiplyAdd(acc, amp_v).StoreTo(slot);
        phase = phase.Add(phase_step);
        amp_v = amp_v.Add(amp_step);
      }
    }

    // Advance and wrap the fundamental phase for the next block. `next` is always
    // >= 0, so truncation == floor — no libm.
    float next = voice.phase + inc * static_cast<float>(n);
    next -= static_cast<float>(static_cast<std::int32_t>(next));
    voice.phase = next;
  }
}

}  // extern "C"
