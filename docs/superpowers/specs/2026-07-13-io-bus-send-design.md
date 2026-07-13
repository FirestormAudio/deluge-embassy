# Bus→Bus Routing (IO-2c) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-2 mixer/bus expansion, THIRD sub-slice. IO-2 slices: IO-2a
persistent routing table (DONE), IO-2b per-bus gain (DONE), **IO-2c bus→bus routing** (this
spec), IO-2d sends/returns/aux API.

## Context

A bus cannot feed another bus today: the render write loop stubs `Input::Bus(_) => 0.0`
because there is no deterministic bus evaluation order. Render order is: zero buses →
`render_block` (nodes) → node-write loop fills buses → per-bus gain → master chain (root
only) → clamp. IO-2c adds **stereo-preserving bus→bus sends** — fold an aux/sub-mix bus into
another bus (return→master, sub-mix trees) — via a dedicated mechanism (not the scalar
`Input` path, which would mono-collapse). See [[io-suite-routing]], [[wren-binding-safety]].

**Model (chosen):** a dedicated `bus_sends` list of `(from, to, gain)`, applied after the
node-write loop, folding `bus_l/r[from] * gain` into `bus_l/r[to]`. Sends process in
**descending source-id order** with the rule **`from > to`** (master=0 is the sink) —
deterministic and cycle-free, no topological sort. **Pre-fader** (sends read the raw
node-summed source bus, before that bus's per-bus gain). `Bus.send(dst, level)` in Wren.

## Design

### Engine — `crates/deluge-audio-graph/src/engine.rs` + `cmd.rs`

- **Field:** `bus_sends: [Option<(BusId, BusId, f32)>; NODES]` (`(from, to, gain)`) +
  `bus_sends_len: usize` on `Engine`, mirroring `writes`/`writes_len`. Init empty in
  `Engine::new`; cleared (`[None; NODES]`, `bus_sends_len = 0`) in `Cmd::Reset`.
- **`Cmd::BusSend { from: BusId, to: BusId, gain: f32 }`** (new variant) + an `Engine::apply`
  arm that appends via a `bus_send` helper (hole-reuse like `bus_write_gains`, bounds/validity
  left to the render seam).
- **Send seam:** between the node-write loop and the per-bus-gain seam:
  ```rust
  // Bus→bus sends (stereo-preserving, pre-fader): fold source buses into targets
  // in descending `from` id (rule: from > to, master=0 = sink) so a source is
  // fully filled (nodes + higher-bus sends) before it feeds a lower bus.
  for from in (0..BUSES).rev() {
      for s in 0..self.bus_sends_len {
          if let Some((f, t, g)) = self.bus_sends[s] {
              let (fi, ti) = (f.0 as usize, t.0 as usize);
              if fi == from && fi < BUSES && ti < BUSES && fi != ti {
                  for i in 0..BLOCK {
                      self.bus_l[ti][i] += self.bus_l[fi][i] * g;
                      self.bus_r[ti][i] += self.bus_r[fi][i] * g;
                  }
              }
          }
      }
  }
  ```
  `fi != ti` guards a self-send; `fi/ti < BUSES` guards out-of-range; the descending-`from`
  loop with the `from > to` convention is cycle-free by construction. Stereo-preserving
  (L→L, R→R). When `bus_sends_len == 0` the seam is a no-op ⇒ **render byte-identical**.
- **Order rationale:** processing by descending `from` id means when bus `k`'s sends run, all
  buses with id `> k` have already been finalized (their node-writes AND their own outgoing
  sends applied), so bus `k`'s content is complete before it feeds a lower bus. A chain
  `aux2(2) → aux1(1) → master(0)` routes correctly same-block.

### Wren — `Bus.send(dst, level)` (`crates/deluge-wren-core/`)

- **`audio::bus_send(from: u16, to: u16, gain: f32)`** facade → `Cmd::BusSend` (NULL_ID guard
  on both ids).
- **`bus_send_impl<S>(vm)`** (`bindings_audio.rs`, a `Bus` INSTANCE method): `from = unsafe {
  vm.foreign_mut::<BusObj>(0) }.id` (the receiver — guaranteed a Bus); `let dst_id = match
  checked_tagged_foreign::<BusObj, _>(vm, 1, TAG_BUS) { Some(o) => o.id, None => return };`
  (the `dst` ARG is user-supplied → the [[wren-binding-safety]] checked guard, degrade to
  no-op on a non-Bus); `let level = vm.get_f(2) as f32;`; `audio::bus_send(from, dst_id,
  level);`. Plus the `#[cfg(feature="wren-sys-backend")]` extern-C shim `bus_send`.
- Register as a `Bus` INSTANCE method in BOTH tables (`method("Bus", "send_(_,_)",
  bindings_audio::bus_send)` and `method("main", "Bus", false, "send_(_,_)", bus_send_impl
  ::<S>)` — `false` = instance, mirroring `write_`).
- **Prelude `foreign class Bus`:** add `foreign send_(dst, level)` + a `send(dst, level) {
  send_(dst, level) }` wrapper (implicit-return one-liner — see [[wren-prelude-single-line-body]];
  a bare `foreign send(dst, level)` also works, but a `send_`/`send` pair matches the `write_`/
  `write` house style). Usage: `var aux = Bus.new(); aux.write(reverb); aux.send(master, 0.3)`
  — a stereo aux return into master at 0.3.

## Non-breaking

Additive: one engine list + `Cmd` + facade + Wren instance method. The no-sends path is
byte-identical (the seam is a no-op when `bus_sends_len == 0`). Node `writes[]`, IO-2a
invalidation, IO-2b per-bus gain, the master chain, and the clamp are untouched. `no_std`,
no-heap, no-panic (bounds/self-send guards, `checked_tagged_foreign` for `dst`, `NULL_ID`
facade guard). 32-bit-`usize` safe. Both configs green; device cross-build clean.

## Constraints (global)

- `no_std`, no heap, no panic on any input. 32-bit-`usize` safe. MIT/Apache-2.0.
- **Sends flow high-id → low-id** (`from > to`; master=0 is the ultimate sink). Natural: create
  a destination bus before the buses that feed it (increasing ids satisfy `from > to`). A
  low→high send is applied but does not propagate through the target's own sends that block
  (one-block-late for that path) — documented, not a crash.
- Stereo-preserving (L→L, R→R). Pre-fader (reads the raw node-summed source bus).
- Wren: the `dst` foreign arg uses the `checked_tagged_foreign::<BusObj>` guard; `get_f` for
  the level is safe.

## Out of scope / deferred

- **Removable individual sends** (handle-based) — IO-2d, with removable writes.
- **Low→high (one-block-late) routing**, **post-fader sends**, **per-send pan**, a topological
  bus-eval (any-order) model, feedback/looped bus routing.
- The remaining IO-2 slice (2d sends/returns/aux API sugar) and IO-4.

## Testing

- **Engine:** node→busA (id 1), `Cmd::BusSend { from: 1, to: 0, gain: 0.5 }`, root = bus0 →
  bus0 receives busA's content × 0.5 on BOTH channels (feed an asymmetric L≠R source to prove
  stereo-preserving); a chain `bus2 → bus1 → bus0` (each `from > to`) routes same-block
  (bus0 gets bus2's content through bus1); **no sends → byte-identical**; self-send (`from ==
  to`) is a no-op; out-of-range bus id is a no-op (no panic); `Cmd::Reset` clears sends.
- **Wren** (Cmd-capture): `aux.send(m, 0.3)` emits `Cmd::BusSend { from: <aux.id>, to: <m.id>,
  gain: 0.3 }`; `aux.send(<non-Bus>, 0.3)` emits nothing (checked guard degrades to no-op).
- **e2e** (real-`EngineHost`): a source on an aux bus, `aux.send(master, 0.5)`,
  `Out.patch(master)` renders the aux's contribution at ~half level (vs the same aux patched
  directly at unity); existing synths unchanged.

## Likely task decomposition (for writing-plans)

1. **Engine bus→bus sends** (`engine.rs` + `cmd.rs`): the `bus_sends`/`bus_sends_len` fields +
   `new` init + `Reset` clear; `Cmd::BusSend` + `apply` arm + `bus_send` helper (hole-reuse);
   the descending-`from` send seam. Engine tests (stereo fold, chain, byte-identical-default,
   self-send no-op, out-of-range no-op, Reset).
2. **Wren `Bus.send` + e2e** (`audio.rs`/`bindings_audio.rs`/`bindings.rs` + `prelude.wren` +
   `tests/audio_bindings.rs`): `bus_send` facade + `bus_send_impl` (checked `dst`) + both tables
   + shim + prelude `send_`/`send`. Cmd-capture test (emits `BusSend`; non-Bus dst → no-op) +
   real-`EngineHost` e2e (aux→master at half level).
