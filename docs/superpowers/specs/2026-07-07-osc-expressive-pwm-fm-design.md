# Osc part 2 — expressive oscillators (PWM + phase-mod/FM) — design & spec

The second cut of the `Osc` suite (the first,
[anti-aliased classic oscillators](2026-07-07-osc-antialiased-oscillators-design.md),
is merged). Add the modulation features that make the analytic oscillators
musically expressive: **pulse-width modulation** (variable-width square),
**phase modulation / FM** (DX-style, including operator **feedback**) — all built
on the merged PolyBLEP/BLAMP foundation.

> **Status:** design proposal. Depends on the merged Osc-1 kernel (PolyBLEP saw/
> square, BLAMP triangle) and the QA harness (`deluge-dsp-test`). **Hard sync is a
> separate follow-on** (it needs a new graph coupling + anti-aliased reset).

**Scope (chosen):** PWM + phase-mod/FM + feedback. **Deferred:** hard sync,
wavetable oscillator, noise variants (separate sub-projects).

---

## 1. Goals & non-goals

**Goals**

- **PWM:** the square gains a `width` input (audio-rate) — a moving falling edge,
  anti-aliased with a BLEP at the moved edge.
- **Phase modulation / FM:** every oscillator gains a `phase-mod` input (audio-
  rate) added to the phase before waveshaping — true DX-style PM (dedicated
  operators beyond the "patch a node into freq" basic FM that already works).
- **Feedback FM:** an operator can modulate itself with its (averaged) previous
  output; feedback depth is a control-rate scalar (`SetParam`).
- **Transparency:** with modulation off, output is bit-for-bit the current
  band-limited oscillator — the existing goldens still pass.
- **QA-proven:** PWM aliasing gated with `worst_alias_db`; FM/feedback gated for
  bounded/stable output + the expected spectral structure (FM aliasing is
  inherent, so not a hard alias floor).

**Non-goals (deferred / out of scope)**

- **Hard sync** — its own follow-on (new graph coupling: slave receives the
  master's phase/reset; anti-aliased reset).
- Wavetable oscillator, noise variants — separate sub-projects.
- Audio-rate feedback *modulation* (feedback depth is a control-rate scalar).
- Fully band-limiting deep FM/PM (impossible — inherent to FM; §4).
- SIMD; bumping `MAX_INPUTS` (this cut fits the existing 3 input ports).

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/osc.rs`)

`Osc::process` grows to take the modulation inputs (all `In` — const or
audio-rate) and the oscillator carries feedback state:

```rust
pub struct Osc { phase: f32, last: f32, last2: f32, feedback: f32 }
// process(wave, freq: In, pmod: In, width: In, dt, out)
// feedback is a scalar field set via `set_feedback` (control-rate, SetParam).
```

Per sample:

1. **Effective phase** (feedback + PM folded in *before* waveshaping — true PM,
   not freq-FM):
   `ph = frac(phase + pmod.at(i) + feedback * 0.5 * (last + last2))`
   - `pmod` is in **cycles** (a modulator patched as `carrier.pm = modulator *
     index`).
   - Feedback uses the **averaged** previous two outputs (the DX7 stabilization;
     plain `last` self-oscillates), scaled by the `feedback` scalar (clamped to a
     safe range, e.g. `[-1.0, 1.0]`).
2. **Waveshape at `ph`** with the merged band-limiting: `poly_blep` at the wrap
   for saw/square, `poly_blamp` for triangle, `fast_sin` for sine — using the
   carrier increment `dtp = freq.at(i) * dt` as the correction width.
3. **PWM (square only):** `width` moves the falling edge from 0.5 to `width`:
   `y = (if ph < w { 1.0 } else { -1.0 }) + poly_blep(ph, dtp)
        - poly_blep(frac(ph - w), dtp)`
   (reuses `poly_blep` at the moved edge; `frac(x) = x - floorf(x)`).
4. Advance `phase += freq.at(i) * dt` (wrap via `floorf`); update
   `last2 = last; last = y`.

**Width defaulting.** The node's default input is `Const(0.0)`, and a raw width of
0 is degenerate, so the square arm maps **`w <= 0.0 → 0.5`** (i.e. 0 means "default
50%"); real widths use `w.clamp(0.01, 0.99)`. Width applies only to `Square`;
other waves ignore it.

**Transparency.** `pmod = 0`, `feedback = 0`, `width <= 0 (→ 0.5)` reproduce the
current band-limited output **exactly** — so the existing oscillator goldens pass
unchanged (a §4 test).

**FM/PM aliasing is inherent and accepted.** The BLEP tracks the *carrier* wrap;
deep PM/feedback generate harmonics above Nyquist that cannot be band-limited
(true of all FM). The design keeps modulation **bounded and stable**, not
alias-free — §4 gates accordingly.

**`set_feedback`.** `Osc::set_feedback(&mut self, f: f32)` stores the clamped
feedback scalar; the graph calls it from `SetParam` (§3).

---

## 3. Graph & Wren surface

No `MAX_INPUTS` change — the oscillator fits the three existing input ports; the
control-rate feedback rides `SetParam`.

**`deluge-audio-graph`:**
- Osc input ports: **0 = freq, 1 = pmod, 2 = width** (all audio-rate `In`).
- `node.rs` osc dispatch: pass `ins[0]`/`ins[1]`/`ins[2]` as freq/pmod/width to
  `Osc::process`; the osc `State` holds the feedback scalar.
- `engine.rs` `apply(Cmd::SetParam { node, param, value })`: **wire it** (currently
  a no-op) so `param == 0` on an osc node calls `Osc::set_feedback(value)`. `Osc`
  is the first kind to use `SetParam`.

**`deluge-wren-core`:**
- Add Node setters: `pm=(v)` → `SetInput` port 1; `width=(v)` → `SetInput` port 2;
  `feedback=(v)` → `SetParam` param 0. Keep `freq=` (port 0). (`bindings_audio.rs`
  + `register_audio` + the `METHODS` table + the prelude `Node` class.)
- `Osc.square(f)` is unchanged and gains `.width`; existing scripts untouched.

**Transparency across layers.** Unset ports default to `Const(0.0)` (pmod 0;
width 0 → kernel 0.5) and feedback defaults to 0 — so a plain `Osc.saw(110)`
renders identically to today.

---

## 4. QA acceptance & testing

`deluge-dsp-kernels` tests (host, via the `deluge-dsp-test` dev-dependency):

- **PWM aliasing.** Render a band-limited PWM square at widths {0.1, 0.3, 0.5}
  and high freq → `worst_alias_db` below the measured floor (expect ~−28 dB;
  narrow widths are harder — measure and gate just below), a **duty-cycle** check
  (the +1 fraction ≈ `width` at low freq), and improvement over a naïve PWM square.
- **PM/FM — stability & structure (not a hard alias gate).**
  - **Bounded & finite** over randomized `pmod`/index (proptest, within ±1.1 or a
    documented modulation bound).
  - **Expected FM structure:** at a moderate index with a simple sine modulator,
    the spectrum shows sidebands at `carrier ± k·modulator` — proving it FMs.
  - `pmod = 0` on a sine is still a clean sine.
- **Feedback — stability.** Bounded/finite at feedback up to the clamp maximum
  (the averaged-outputs form must not blow up) — a proptest sweeping feedback.
- **Transparency (key regression).** With `pmod = 0`, `width ≤ 0 (→ 0.5)`,
  `feedback = 0`, the oscillator output equals the current band-limited output —
  a direct null-test against a saved reference block, **and** the existing
  oscillator goldens (`deluge-audio-graph`, `deluge-wren-core`) still pass.

**Determinism:** oscillators have no RNG, so all measurements/goldens reproduce
across host and device (and default/`--features simd`).

---

## 5. Deferred (tracked follow-ups)

- **Hard sync** — new graph coupling (slave receives master phase/reset) +
  anti-aliased reset (BLEP at the sync discontinuity). Its own sub-project.
- **Wavetable oscillator**, **noise variants** — separate sub-projects.
- Audio-rate feedback modulation (currently control-rate scalar).
- SIMD oscillator path; through-zero FM refinements.

---

## 6. Open questions (resolved during implementation)

- Exact `pmod` scaling convention (cycles vs a fixed index scale) — start with
  raw cycles; adjust from the FM-structure test.
- The feedback clamp range and whether the averaged-outputs form needs a further
  DC/leak term for stability at max feedback (decided from the stability proptest).
- Whether the `Osc::process` signature stays positional (`freq, pmod, width`) or
  moves to a small `OscInputs` struct if the arg list feels unwieldy.
- PWM `worst_alias_db` thresholds per width (set from measurement, like Osc-1).
