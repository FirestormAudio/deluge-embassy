# Multi-Output Foundation (IO-4a) — Design Spec

**Date:** 2026-07-13
**Status:** Approved (design), pending implementation plan
**Sub-project:** IO suite — IO-4 multiple outputs, FIRST sub-slice (the engine foundation).
IO-4 decomposes: **IO-4a engine multi-output foundation** (this spec), IO-4b Wren USB routing
API, IO-4c firmware USB TX + speaker/jack policy (HW-gated). See [[io-suite-routing]].

## Context

The Deluge outputs: **main L/R** (codec stereo, the existing render root → `out`; the internal
**speaker** mirrors it, amp-gated) and **USB** — up to **8 independently-routable MONO
channels** (the SCUX USB-audio TX path). Today the engine renders ONE root bus to ONE `out`;
there is no USB channel routing. IO-4a adds the engine-side 8-channel USB output map + a
`fill_usb` render, all host-testable. The actual USB DMA + speaker policy are later HW-gated
slices (IO-4c).

**Model (chosen):** per-mono-channel routing — each USB channel routes from a bus SIDE (L or R)
or a node port.

## Design

Engine-internal (`deluge-audio-graph`). No Wren/binding change (that is IO-4b).

### `OutputSrc` + `USB_CHANNELS` (`ids.rs`)

```rust
pub const USB_CHANNELS: usize = 8;

/// A mono source feeding one USB output channel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OutputSrc {
    Silent,                            // channel off (default)
    BusL(BusId), BusR(BusId),          // a bus's left / right row (e.g. drums.left)
    Node { node: NodeId, port: u8 },   // a node output port (already mono)
}
```
Re-export `OutputSrc`/`USB_CHANNELS` from `lib.rs`.

### Engine (`engine.rs` + `cmd.rs`)

- Field `usb_out: [OutputSrc; USB_CHANNELS]` (default `Silent` in `Engine::new`; cleared to
  `Silent` in `Cmd::Reset`).
- `Cmd::SetUsbOut { channel: u8, src: OutputSrc }` (all-`Copy`, satisfies `Cmd`'s derives) +
  an `Engine::apply` arm: `let c = channel as usize; if c < USB_CHANNELS { self.usb_out[c] = src; }`.
- **`pub fn fill_usb(&self, usb: &mut [[f32; BLOCK]; USB_CHANNELS])`** — called AFTER `render`
  (buses hold this block's final content; node outputs still in the arena). For each channel,
  resolve `usb_out[ch]`: `Silent` → zeros; `BusL(b)`/`BusR(b)` → `bus_l[b]`/`bus_r[b]`
  (bounds-guarded `b < BUSES`, else zeros); `Node { node, port }` → the node's output row
  (`arena.out_base(node)` + `port`, guarded `base+port < OUTS`, dangling/OOB → zeros). Reads
  only; no panic (mirrors the existing write-loop resolution discipline).

`render` and every existing path are UNCHANGED — `fill_usb` is a separate read-only method, so
no render-signature churn.

## Non-breaking

Additive: one enum + const, one engine field, one `Cmd` + apply arm + `Reset` clear, one
method. A graph that never routes USB is byte-identical (the field is unused). `no_std`,
no-heap, no-panic, 32-bit-`usize` safe. Both configs green; device cross-build clean.
MIT/Apache-2.0.

## Constraints (global)

- `no_std`, no heap, no panic. 32-bit-`usize` safe. MIT/Apache-2.0. Engine-internal only
  (Wren API = IO-4b; USB DMA + speaker policy = IO-4c, HW-gated).
- USB channels are MONO (8 slots), each routed from a bus side or node port.
- `fill_usb` is called after `render`, reading this block's final bus/node state.

## Out of scope / deferred

- **IO-4b** Wren USB routing API (`Usb.channel(n).route(...)`). **IO-4c** firmware USB TX
  (SCUX) + speaker auto-mute on jack-detect (HW-gated). Note: USB audio is mutually exclusive
  with the codec in the SDK today — IO-4c resolves that.
- Stereo-pair USB routing, dynamic channel count, per-channel gain (compose via a Mul/bus).

## Testing

- **`fill_usb_routes_bus_side_and_node`**: node0 = const 0.3 → bus1 (center write, so
  `bus_l[1]=0.3`); route USB ch0 = `BusL(bus1)`, ch1 = `Node{node0, port0}`. Render, `fill_usb`
  → `usb[0]` == the bus's left row (0.3), `usb[1]` == node0's output (0.3), other channels
  silent (0). Proves both source kinds route.
- **`set_usb_out_of_range_is_noop`**: `SetUsbOut { channel: 99, .. }` is a no-op (no panic).
- **`usb_dangling_node_is_silent`**: route a channel to a non-existent node → `fill_usb` yields
  silence (no panic).
- **`reset_clears_usb_map`**: route channels, `Cmd::Reset`, re-`fill_usb` → all silent.
- All existing `deluge-audio-graph` tests stay green both configs (additive, `render`
  unchanged).

## Likely task decomposition (for writing-plans)

Single task (`ids.rs` + `lib.rs` + `cmd.rs` + `engine.rs`):
1. `OutputSrc`/`USB_CHANNELS` + re-export; `Cmd::SetUsbOut`; engine `usb_out` field + `new`
   init + `Reset` clear + apply arm + `fill_usb`; the four tests. Full graph crate green both
   configs; dependent crates build.
