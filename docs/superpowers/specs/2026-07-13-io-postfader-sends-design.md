# Post-Fader Sends (IO-2f) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), implemented inline
**Sub-project:** IO suite — IO-2 mixer/bus expansion, an ordering refinement enabling a proper
live channel fader (prerequisite for the IO-2d Mixer). See [[io-suite-routing]].

## Context

IO-2c applied bus→bus sends BEFORE IO-2b's per-bus gain (render order: node-writes → sends →
gain), making sends **pre-fader** — a bus's `gain` did not scale its outgoing sends. So a
Mixer channel's `ch.gain` could not act as a real fader (it wouldn't scale the channel's
contribution to master). IO-2f swaps the two seams so **per-bus gain runs BEFORE bus→bus
sends** (post-fader sends), matching console semantics: `ch.gain` scales the channel's master
contribution AND its aux sends, and it's a live per-block param.

## Design

Engine-internal only. Swap the two render seams:

```
before:  node-writes → bus→bus sends → per-bus gain → master chain → clamp
after:   node-writes → per-bus gain → bus→bus sends → master chain → clamp
```

`Cmd::BusGain`/`bus_gain`, `Cmd::BusSend`/`bus_sends`, and their loops are otherwise
unchanged — only their order in `render` swaps.

## Non-breaking

**Byte-identical for all existing tests:** IO-2b's gain tests use a root bus with no outgoing
sends; IO-2c's send tests all use `gain = 1.0`. The only NEW behavior is the intended one — a
bus with a non-unity gain AND an outgoing send now sends its post-gain signal. `no_std`,
no-panic. Both configs green; device cross-build clean. MIT/Apache-2.0.

**Doc note:** a master bus's own `gain` applies to its direct writes before aux returns fold
in (returns land post-master-fader). Master gain is normally 1.0 (output shaped by the
limiter), so this is a non-issue in practice.

## Testing

- `bus_send_is_post_fader`: node0=0.8 → busA; `BusGain{busA,0.5}` → busA=0.4;
  `BusSend{busA→master,1.0}` → master=0.4 (a pre-fader send would give 0.8). Proves post-fader.
- All existing IO-2b/2c/2e tests stay green (byte-identical) both configs.
