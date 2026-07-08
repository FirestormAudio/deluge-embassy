# Fi-3: Huovilainen Moog ladder filter — design & spec

The third sub-project of the **Fi (Filter) suite**: an authentic **Huovilainen/Stilson-Smith
Moog transistor-ladder** filter — the warm, self-oscillating 4-pole. Built as a thin `Moog`
kernel **reusing the Fi-2 `DiodeLadder<STAGES>` core + `pade_tanh`** (which are already the
Huovilainen one-pole-tanh-cascade topology), adding the one missing piece — the
resonance-dependent gain compensation — plus a switchable slope (12/24 dB) and a drive knob.

> **Status:** design proposal. Depends on merged Fi-2 (`DiodeLadder<STAGES>`, `pade_tanh` in
> `deluge-dsp-kernels/src/filter.rs`), the Fi-1 filter QA harness, the P0 graph (incl. the
> existing `Cmd::SetParam` control-param path), and P1 Wren. `spark` has **no** authentic
> Huovilainen to port (only a rejected simplified `ladder.rs`), so this is from-scratch DSP
> over our own proven ladder — not a port.

**Fi roadmap:** Fi-1 SVF ✅ → Fi-2 TB-303 ✅ → **Fi-3 Moog** (this spec) → Fi-4 MS-20 →
Fi-5 modal resonator.

---

## 1. Goals & non-goals

**Goals**
- **Authentic Moog ladder, reusing the existing core.** Our `DiodeLadder<STAGES>` is already
  the canonical Huovilainen ODE cascade: per-stage `pade_tanh` (`ẏᵢ = fh·(tanh(yᵢ₋₁) −
  tanh(yᵢ))`), global `pade_tanh(y_last)·res·4` feedback (the classic Moog `k = 4·res`), and
  RK2 sub-stepping at 2× (Huovilainen-canonical). The Moog uses this ladder's **native
  feedback** — the path the 303 bypassed. **No changes to `DiodeLadder`/`Tb303`.**
- **Resonance gain compensation (the missing piece).** Real Huovilainen scales the input to
  keep the passband/−3 dB cutoff stable as resonance rises. Add this in the `Moog` wrapper as
  an input pre-scale — nothing else is missing from the topology.
- **Switchable slope.** 24 dB/oct (4-pole, `DiodeLadder<4>`) and 12 dB/oct (2-pole,
  `DiodeLadder<2>`) — a signature Moog control, free from the const-generic core.
- **Drive.** A pre-ladder input gain into the `pade_tanh` for the overdriven-Moog growl.
- **Self-oscillating** at max resonance (both slopes), bounded.
- **QA-proven** via the Fi-1 harness: cutoff stability vs resonance (the compensation gate),
  self-oscillation, measured slope (≈24/≈12 dB/oct), drive raises THD, resonant-peak tracking,
  boundedness.

**Non-goals (deferred / out of scope)**
- **Audio-rate drive.** The graph node has 3 input ports (input/cutoff/res); drive is a
  **control param** (via the existing `Cmd::SetParam` path, like `feedback=`), not a 4th
  audio-rate input. Audio-rate drive would need a `MAX_INPUTS` bump — deferred.
- **Thermal-voltage (`Vt`) asymmetry / temperature drift** — `pade_tanh`'s fixed steepness is
  the nonlinearity; a `Vt` drive-shaping knob is a later refinement.
- **Extra slopes / multimode** (6/18 dB taps, Oberheim-style LP/BP/HP tap mixing) — later.
- **Cross-voice SIMD** (voice/`Sy` layer). No changes to `DiodeLadder`, `pade_tanh`, or `Tb303`.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/filter.rs`, new `Moog<POLES>`)

```rust
pub struct Moog<const POLES: usize> {
    ladder: DiodeLadder<POLES>,
    drive: f32,            // control param, default 1.0 (clean); >1 overdrives the tanh
}

impl<const POLES: usize> Moog<POLES> {
    pub fn new() -> Self { Moog { ladder: DiodeLadder::new(), drive: 1.0 } }
    pub fn set_drive(&mut self, d: f32) { self.drive = d.max(0.0); }
    pub fn process(&mut self, input: In, cutoff: In, res: In, dt: f32, out: &mut [f32]);
}
```

**Per-sample (`process`)** — `OS = TB303_OVERSAMPLE` (=2; rename to a shared
`LADDER_OVERSAMPLE` or reuse — 2× is the Huovilainen-canonical choice too):
```
cutoff = cutoff.at(i).clamp(20.0, ~18_000.0)     // Moog range (wider than the 303's 2 kHz)
res    = res.at(i).clamp(0.0, 1.0)
fh     = 2π·cutoff·dt / OS
x      = input.at(i) · self.drive · (1.0 + MOOG_COMP·res)     // drive + resonance compensation
out[i] = self.ladder.process(x, fh, res · MOOG_K, OS)          // native ladder feedback = (res·MOOG_K)·4
```

**The two tuned constants (the DSP substance), both set by measurement (§4):**
- **`MOOG_COMP`** — the input compensation gain. Tuned so the measured `minus_3db_hz` cutoff
  stays put (within a small tolerance) as `res` sweeps `0→1`. Without it, high resonance
  drops and detunes the passband. (May be a single constant, or per-slope if needed.)
- **`MOOG_K[POLES]`** — the feedback scale so `res=1` self-oscillates cleanly at each slope.
  The ladder's feedback is `pade_tanh(y_last)·(res·MOOG_K)·4`; a 4-pole self-oscillates at
  loop gain 4 (`MOOG_K[4] ≈ 1.0`), a 2-pole needs less (`MOOG_K[2] < 1.0`). Set from the
  self-oscillation gate per slope.

`no_std`; pure `f32` arithmetic in the hot loop; deterministic; reuses `DiodeLadder`/`pade_tanh`
verbatim (no edits to them). No SIMD.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- Add `Kind::MoogLp4`, `Kind::MoogLp2`. `Node::new` maps them to `State::Moog4(Moog::<4>::new())`
  / `State::Moog2(Moog::<2>::new())`. Render arm calls `m.process(ins[0], ins[1], ins[2], dt,
  outs.port(0))` — ports 0=input, 1=cutoff, 2=res. `out_width` = 1.
- **Drive** via `Node::set_param`: extend the existing match (currently only `State::Osc`
  feedback) with `State::Moog4(m)|State::Moog2(m) if param==0 => m.set_drive(value)`.

**`deluge-wren-core`:**
- A `Moog` Wren class: `Moog.lp(input, cutoff, res)` (24 dB → `Kind::MoogLp4`) and
  `Moog.lp2(input, cutoff, res)` (12 dB → `Kind::MoogLp2`) — two factories over a shared
  `node_moog_impl(kind)` helper (mirroring how `node_svf_impl` maps a code → `Kind`).
- **`drive=`** instance setter → `node_set_drive_impl` → `audio::set_param(self_id, 0, v)`
  (verbatim the `feedback=` pattern), reused by both slopes. `cutoff=`/`res=` reused (ports
  1/2). Registration in both binding tables + the `class Moog` sugar in `prelude.wren`.

---

## 4. QA acceptance & testing

Reuse the Fi-1 harness (`deluge_dsp_test::filter_meas::{magnitude_db, minus_3db_hz,
self_osc_hz_and_rms}`) and `spectrum::{analyze, slope_db_per_octave, thd, level_at}`.

**`deluge-dsp-kernels` tests:**
- **Cutoff stability vs resonance (the compensation gate — the key one).** For a fixed
  `cutoff`, sweep `res` `0→0.9`; the measured `minus_3db_hz` stays within a tolerance (e.g.
  ±15%) of the low-res value. This is what `MOOG_COMP` buys — a wrong/zero compensation makes
  the cutoff drift with resonance and fails this.
- **Slope is correct.** `slope_db_per_octave` in the stopband is ≈ **−24 dB/oct** for
  `Moog<4>` and ≈ **−12 dB/oct** for `Moog<2>` (within a measured tolerance). Proves the
  pole-count/slope switch works.
- **Self-oscillation both slopes.** At `res=1` with a brief excitation, each slope sustains a
  bounded tone near `cutoff` (via `self_osc_hz_and_rms`) — validates `MOOG_K[POLES]`.
- **Resonant peak tracks cutoff.** The high-resonance spectral peak sits near `cutoff` and
  rises monotonically with it.
- **Drive adds harmonics.** At a fixed `cutoff`/`res`, `thd` (or high-harmonic energy) at
  `drive=4` is meaningfully greater than at `drive=1` — proves drive overdrives the tanh.
- **Boundedness (P0 gate).** Proptest: `cutoff ∈ [20, 18k]`, `res ∈ [0, 1]`, `drive ∈ [1, 8]`,
  input incl. overdrive → every output finite and within a measured bound (the `pade_tanh`
  stages bound it).

**Graph/Wren:** `Kind::MoogLp4`/`MoogLp2` nodes render bounded/non-silent; `Moog.lp`/`Moog.lp2`
emit the right `Cmd`; `drive=` (via `Cmd::SetParam`) changes the output (render twice, assert
differ — mirroring the existing `setparam_feedback_renders_bounded` test); `cutoff=`/`res=`
update the node.

**Determinism:** no RNG; reproducible; proptests seed-fixed.

---

## 5. Deferred / follow-ups

- **Audio-rate drive** (bump `MAX_INPUTS` to 4, or a dedicated modulation path).
- **Thermal `Vt`** drive-shaping for temperature/asymmetry character.
- **More slopes / Oberheim-style multimode** (6/18 dB, LP/BP/HP tap mixing off the ladder).
- **Cross-voice SIMD** at the `Sy` voice layer.
- Fi-4 MS-20 Sallen-Key, Fi-5 modal resonator — separate specs, reusing this suite's harness.
