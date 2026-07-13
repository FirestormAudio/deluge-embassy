# Aux / Mixer Sugar (IO-2d) — Design Spec

**Date:** 2026-07-13
**Status:** Implemented inline
**Sub-project:** IO suite — IO-2 mixer/bus expansion, FINAL sub-slice (completes IO-2). Pure
Wren-prelude sugar composing the 2a-2f primitives; no engine/binding change. See
[[io-suite-routing]].

## Design (prelude.wren only)

**`class Aux`** — effect-return presets. Each creates an aux bus, wires a FULLY-WET effect fed
by that bus back into `target`, returns the aux bus:
`Aux.reverb(target, roomsize, damp)` / `hall(target, size, damp)` / `plate(target, size, damp)`
/ `delay(target, time, feedback)` / `chorus(target, rate, depth)`. The effect reads the aux one
block late (IO-2e). Top-level only (effects abort in poly). Create `target` BEFORE the aux so
ids increase (the from > to send rule, IO-2c).

**`class Mixer`** — `construct new()` (master bus), `master` getter, `channel(src)` (creates a
channel bus, writes `src`, sends to master at unity, returns the channel Bus). The channel's
`.gain=` is a REAL post-fader fader (IO-2f) and `.send(aux, level)` its aux sends. Create the
mixer first, then auxes, then channels (from > to).

`construct`+instance-fields verified to compile in this Wren fork (probe). Multi-line
explicit-return bodies (single-line explicit-return is the [[wren-prelude-single-line-body]]
gotcha).

## Testing (tests/audio_bindings.rs)
- Cmd-capture: `Aux.reverb` emits `NewNode{Room}` + `BusWriteGains` into the target;
  `Mixer.channel` emits `BusSend{from:ch, to:master, 1.0}`.
- e2e: `Aux.reverb` fed a saw renders non-silent wet (4096-frame window so the tail ramps);
  `Mixer` channel `ch.gain = 0.5` ~halves the render (proves the post-fader fader).

## Non-breaking
Additive prelude-only (two Wren classes); no engine/binding/Cmd change. Both configs green;
device cross-build clean. Completes IO-2 (routing table + gain + bus→bus + bus-fed effects +
post-fader + Aux/Mixer sugar).
