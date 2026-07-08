# Hard sync oscillator — design & spec

Completes the deferred [Osc](2026-07-07-osc-antialiased-oscillators-design.md) "hard
sync" item. A single-node `SyncOsc`: an internal **master** oscillator resets the
**slave** each master cycle; the master frequency sets the perceived pitch, the slave
frequency (swept up) sets the timbre — the classic sync-sweep sound. The reset is a step
discontinuity, anti-aliased with a **BLEP** (reusing the merged PolyBLEP).

> **Status:** design proposal. Depends on merged Osc-1 (`poly_blep`, waveshaping) and QA
> (`deluge-dsp-test` `worst_alias_db`). Self-contained: a `SyncOsc` kernel + four node
> kinds + a Wren factory. **No new inter-node graph coupling** (master is internal).

---

## 1. Goals & non-goals

**Goals**
- **`SyncOsc` kernel:** `{ master_phase, slave_phase }`; `process(wave, master_freq,
  slave_freq, dt, out)`. Master wrap → slave reset; slave produces the (existing)
  band-limited waveform + a **reset BLEP** at the sub-sample reset position.
- **Four sync kinds** `SyncSine/SyncSaw/SyncSquare/SyncTri` → `State::Sync(SyncOsc)`,
  mirroring how `Osc`'s four kinds map to `State::Osc` (kind→wave at render).
- **Wren:** `Node.sync_(waveCode, master, slave)` (wave→kind, like `Node.src_`) +
  `Osc.syncSaw/syncSquare/syncTri/syncSine(master, slave)` sugar.
- **QA-proven:** the sync sweep clears a measured `worst_alias_db` floor and beats a
  naive (no reset-BLEP) sync; bounded/finite; the expected spectral sideband motion as
  the slave sweeps.

**Non-goals (deferred)**
- Inter-node sync coupling (any osc → any osc via a new graph edge).
- Extreme slave≫master ratios where simple PolyBLEP degrades (multiple discontinuities
  per sample) — tested up to a slave freq where it holds, like Osc-1's ceiling.
- Sync PWM/PM/feedback (Osc-2 features on the sync slave) — a later refinement.
- Through-zero / sub-sample master-freq modulation refinements beyond the basic reset.

---

## 2. Kernel DSP (`deluge-dsp-kernels/src/osc.rs`, new `SyncOsc`)

```rust
pub struct SyncOsc { master_phase: f32, slave_phase: f32 }
// process(&mut self, wave: Wave, master_freq: In, slave_freq: In, dt, out)
```

Per sample `i` (`dtp_m = master_freq.at(i)*dt`, `dtp_s = slave_freq.at(i)*dt`):

1. **Advance master; detect wrap.** `mp = master_phase + dtp_m`. If `mp >= 1.0` a reset
   occurs this sample at sub-sample position `t_reset = (1.0 - master_phase) / dtp_m`
   (in `[0,1)`); `master_phase = mp - 1.0` (wrapped). Else no reset; `master_phase = mp`.
2. **Slave value + natural-wrap BLEP.** Compute the slave waveform at `slave_phase` using
   the SAME band-limited shaping as `Osc` (`poly_blep` at the slave's own wrap for
   saw/square, `poly_blamp` for tri, `fast_sin` for sine), with `dtp_s` as the correction
   width — i.e. factor `Osc`'s per-sample waveshaping into a shared helper both use.
3. **Reset BLEP (the new piece).** If a reset occurred: the output steps from the slave
   value at its pre-reset phase to the value at phase 0. Compute the **step magnitude**
   `d = wave_value(0) - wave_value(slave_phase_at_reset)` (the discontinuity), and add a
   `poly_blep(t_reset, ...)`-scaled correction of `-d` (or `+d`, per the derivation) so
   the corrected output is continuous through the reset. Then set `slave_phase` to the
   post-reset advance `(1.0 - t_reset) * dtp_s` (the slave restarts from 0 and advances
   the remaining fraction of the sample).
4. **Advance slave** (when no reset): `slave_phase += dtp_s`, wrap via `floorf`.

**The exact reset-BLEP formula (magnitude + sign + sub-sample placement) is derived
against the alias floor** — §4 gates it by measurement. The structure is fixed here; the
implementer validates the step/sign by minimizing `worst_alias_db` (a wrong sign makes
aliasing *worse* than naive — an unambiguous signal). Reuse `poly_blep`; no new kernel
math beyond the reset step.

**Shared waveshaping.** Extract `Osc`'s per-sample "waveshape at final phase `ph` with
increment `dtp`" (the `match wave { … }` block — the band-limited saw/square/tri/sine
shaping) into a helper `fn wave_sample(wave, ph, dtp, width) -> f32`. `Osc` computes
`ph = frac(phase + pmod + feedback)` and calls it with its `width` (so `Osc::process`
stays **bit-exact** — a behavior-preserving extraction, regression-checked). `SyncOsc`
calls it with the default `width` (0.5 → plain square). The reset-BLEP step (§2.3) uses
the **naïve** (pre-BLEP) waveform values — a separate trivial `naive_wave(wave, ph)` (or
the shaping with the correction zeroed), since the step magnitude is the ideal
discontinuity the BLEP cancels.

`no_std`; deterministic; state is just the two phases.

---

## 3. Graph & Wren surface

**`deluge-audio-graph`:**
- Add `Kind::SyncSine`, `Kind::SyncSaw`, `Kind::SyncSquare`, `Kind::SyncTri`. `Node::new`
  maps all four to `State::Sync(SyncOsc::new())`. The render arm maps the sync kind →
  `Wave` (as the `Osc` kinds do) and calls `sync.process(wave, ins[0], ins[1], dt,
  outs.port(0))` — port 0 = master, port 1 = slave. `out_width` = 1.

**`deluge-wren-core`:**
- `Node.sync_(waveCode, master, slave)` static factory (`node_sync_impl` → `sync_kind(code)`
  → the four kinds, mirroring `node_src_impl`/`src_kind`), emitting `NewNode{kind, args:
  [master, slave, Const0]}`. 4-place registration.
- `Osc.syncSaw(m,s)` / `syncSquare` / `syncTri` / `syncSine` prelude sugar → `Node.sync_(code, m, s)`.

---

## 4. QA acceptance & testing

`deluge-dsp-kernels` tests (via `deluge-dsp-test`):

- **Aliasing gate (the sync sweep).** Render a synced saw/square at a musical master
  fundamental (e.g. 110/220 Hz) with the slave swept over a range (e.g. slave = 1–6×
  master), FFT, and assert `worst_alias_db` below the **measured** floor across the
  sweep (set from measurement, like Osc-1). The reset BLEP is what buys this.
- **Improvement over naive.** Same sweep with the reset BLEP DISABLED (naive hard reset)
  aliases badly; assert the BLEP version is meaningfully better (**> N dB**, N measured).
  A wrong reset-BLEP sign/magnitude makes it WORSE than naive — this test catches it.
- **Sync sound / sidebands.** At a fixed master with the slave at a non-integer ratio,
  the spectrum shows the sync formant structure (energy clusters around the slave
  frequency / its harmonics) that moves as the slave sweeps — proving it syncs.
- **Bounded/finite** over randomized master/slave freqs (proptest; ±1.2 like the other
  band-limited oscillators — reset BLEP can overshoot slightly).
- **`Osc` unchanged.** Extracting the shared `wave_sample` helper keeps `Osc::process`
  bit-exact — all Osc-1/Osc-2 oscillator tests + goldens pass unchanged.
- **Determinism:** no RNG; reproducible host/device.

Graph/Wren: a `Kind::SyncSaw` node renders bounded/non-silent; `Osc.syncSaw(m,s)` emits
the right `Cmd` and renders.

---

## 5. Deferred / follow-ups

- Inter-node sync coupling (any osc → any osc).
- Osc-2 features (PWM/PM/feedback) on the sync slave.
- Higher slave≫master ratios (multi-discontinuity-per-sample anti-aliasing / MinBLEP).
- Through-zero sync; soft/windowed sync.
