# Per-Bus Gain (IO-2b) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-2 mixer/bus expansion, SECOND sub-slice. IO-2 slices: IO-2a
persistent routing table (DONE), **IO-2b per-bus gain/trim** (this spec), IO-2c bus→bus
routing, IO-2d sends/returns/aux API.

## Context

Buses sum at **unity** today — there is no per-bus level control (`bus_l[b][i] += v*gl` in the
render write loop; `Input::Bus` reads `bus_l[b]+bus_r[b]` unscaled; the root bus copies to
output). The only gains are the per-*write* side gains `gl`/`gr` (used for L/R panning of a
stereo source). IO-2b adds the **channel-fader primitive**: a per-bus mono gain, settable from
Wren via `Bus.gain=`. See [[io-suite-routing]].

**Model (chosen):** a per-bus **mono** gain `bus_gain: [f32; BUSES]` (default `1.0`), applied to
each bus's L/R rows **after** the write-accumulation loop and **before** the master chain — so
the root→output copy, the master chain, and the next block's `Input::Bus` read all see the
gained bus. Per-side/balance deferred (a fader is mono).

## Design

### Engine — `crates/deluge-audio-graph/src/engine.rs` + `cmd.rs`

- **Field:** `bus_gain: [f32; BUSES]` on `Engine` (e.g. after `bus_r`). Init `[1.0; BUSES]` in
  `Engine::new`; reset to `[1.0; BUSES]` in the `Cmd::Reset` arm (the one reset field whose
  value is nonzero — the others are `0.0`/`None`).
- **`Cmd::BusGain { bus: BusId, gain: f32 }`** (new variant in `cmd.rs`) + an `Engine::apply`
  arm: `let b = bus.0 as usize; if b < BUSES { self.bus_gain[b] = gain; }` (bounds-guarded).
  Plus a `pub fn set_bus_gain(&mut self, bus: BusId, gain: f32)` helper mirroring `set_root`.
- **Gain seam:** insert between the write-accumulation loop and the master-chain block:
  ```rust
  // Per-bus gain (default 1.0 = no-op, byte-identical): fader before the master chain.
  for b in 0..BUSES {
      let g = self.bus_gain[b];
      if g != 1.0 {
          for i in 0..BLOCK {
              self.bus_l[b][i] *= g;
              self.bus_r[b][i] *= g;
          }
      }
  }
  ```
  The `if g != 1.0` guard keeps the default path **byte-identical** (matching the master-chain
  `Option`-gated house style + its `_disabled_is_byte_identical` tests) and skips the multiply
  on unused/unity buses. Ordering: gain → master chain (DC-block/EQ/limiter, root only) → clamp
  — the natural "channel fader before master processing" order; the stateful master processors
  react to the post-gain level, which is correct for a fader feeding a master chain.

### Wren — `Bus.gain=` (`crates/deluge-wren-core/`)

- **`audio::set_bus_gain(bus: u16, gain: f32)`** facade → `Cmd::BusGain` (mirrors `audio::
  set_root`; `NULL_ID` guard).
- **`bus_set_gain_impl<S>(vm)`** (`bindings_audio.rs`, an INSTANCE method like `bus_write_impl`):
  `let id = unsafe { vm.foreign_mut::<BusObj>(0) }.id;` (receiver at slot 0), `let g = vm.get_f
  (1) as f32;` (safe), `audio::set_bus_gain(id, g);`. Plus the `#[cfg(feature="wren-sys-backend")]`
  extern-C shim `bus_set_gain`.
- Register as an INSTANCE method in BOTH tables: `method("Bus", "gain=(_)", bindings_audio::
  bus_set_gain)` (`bindings.rs`) and `method("main", "Bus", false, "gain=(_)", bus_set_gain_impl
  ::<S>)` (`bindings_audio.rs` — note `false` = instance, mirroring `write_(_)`).
- **Prelude `foreign class Bus`:** add `foreign gain=(v)` (a bare foreign instance setter, exactly
  like `Node.gain=` — no wrapper/poly-guard needed). Usage: `var m = Bus.new(); m.write(synth);
  m.gain = 0.5; Out.patch(m)`.

## Non-breaking

Additive: one engine field, one `Cmd` variant + `apply` arm + `Reset` reset, one `audio.rs`
facade, one Wren instance setter (both tables + shim + prelude). The default path is
byte-identical (the `if g != 1.0` guard — no `BusGain` ⇒ every bus stays at 1.0 ⇒ the seam is a
no-op). The `writes[]` model, IO-2a invalidation, master chain, and clamp are untouched. `no_std`,
no-heap, no-panic (bounds-guarded apply, guarded multiply, `NULL_ID` facade guard). 32-bit-`usize`
safe. Both configs green; device cross-build clean.

## Constraints (global)

- `no_std`, no heap, no panic on any input. 32-bit-`usize` safe. MIT/Apache-2.0.
- Per-bus MONO gain (both L/R scaled by one factor); default `1.0` (byte-identical when unset).
- Wren reads only `get_f` + the receiver `BusObj` (the same `foreign_mut::<BusObj>(0)` read
  `bus_write_impl` already does) — no new UB surface.

## Out of scope / deferred

- **Per-side (balance) gain**, VCA/mute groups, per-bus pan, dB-unit gain (this is linear),
  gain smoothing/ramping (instant this slice).
- The other IO-2 slices (2c bus→bus, 2d sends/returns API) and IO-4.

## Testing

- **Engine:** set the root bus's gain to 0.5, render a known level → output halved; **default
  (no `BusGain`) → output byte-identical** (a unity patch renders exactly as before); a non-root
  aux-bus gain scales what a node reading `Input::Bus(aux)` sees (one block late — the existing
  bus-read latency); `Cmd::Reset` restores all buses to unity; `BusGain` with an out-of-range
  bus index is a no-op (no panic).
- **Wren** (Cmd-capture): `m.gain = 0.5` (on a `Bus.new()` handle) emits `Cmd::BusGain { bus:
  <m.id>, gain: 0.5 }`.
- **e2e** (real-`EngineHost`): a source routed through a bus at `gain = 0.5` renders at ~half the
  level of the same patch at unity; existing synths unchanged.

## Likely task decomposition (for writing-plans)

1. **Engine per-bus gain** (`engine.rs` + `cmd.rs`): the `bus_gain` field + `new` init + `Reset`
   reset; `Cmd::BusGain` + `apply` arm + `set_bus_gain` helper; the gain seam. Engine tests
   (halved-when-set, byte-identical-default, aux-bus-read-scaled, Reset-restores, out-of-range
   no-op).
2. **Wren `Bus.gain=` + e2e** (`audio.rs`/`bindings_audio.rs`/`bindings.rs` + `prelude.wren` +
   `tests/audio_bindings.rs`): `set_bus_gain` facade + `bus_set_gain_impl` + both tables + shim +
   prelude `foreign gain=(v)`. Cmd-capture test + real-`EngineHost` e2e (bus at half gain renders
   quieter).
