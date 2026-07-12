# Wren Slot-Read UB Hardening — Design Spec

**Date:** 2026-07-11
**Status:** Approved (design), pending implementation plan
**Sub-project:** Sa-2 follow-up (flagged by the Sa-2 whole-branch review)

## Problem

This repo's Wren backend (`wren-sys`) compiles the vendored C VM's `ASSERT`
type/bounds checks to **no-ops**: `wren-sys/build.rs` never defines the C
`DEBUG` macro, so `ASSERT(cond, msg)` expands to `do {} while(false)`
(`wren-rs/ext/wren/src/vm/wren_common.h`). The VM also uses `WREN_NAN_TAGGING`.
Consequently the VM's slot accessors do **no** runtime validation:

- `wrenGetSlotForeign` / `wrenGetListCount` / `wrenGetListElement` /
  `wrenGetSlotBytes` each `ASSERT(IS_X(...))` then unconditionally
  `AS_FOREIGN`/`AS_LIST`/`AS_STRING`, i.e. `(T*)AS_OBJ(value)` where
  `AS_OBJ` is a raw bit-mask reinterpret of the tagged 64-bit slot value
  followed by a struct-field dereference. On a mistyped slot this is a **wild /
  out-of-bounds pointer read — real undefined behavior, not a catchable Rust
  panic.**
- `wrenGetSlotDouble` (`AS_NUM` = bit-reinterpret, no deref) and
  `wrenGetSlotBool` (`AS_BOOL` = equality test) are **safe**: worst case a
  garbage number/bool, never memory unsafety.

The `SlotApi` trait (`crates/deluge-wren-core/src/slotapi.rs`) and its
`wren-sys` impl are 1:1 pass-throughs with zero Rust-side checks.

The Sa-2 work fixed two instances of this (`keymap_from_impl`,
`node_polysampleplayer_impl`) but a repo-wide audit found the same UB class in
**9 distinct user-reachable `_impl` functions**. This spec hardens all 9.

### Reachability nuance (heap over-read, not just field-reinterpret)

`WtObj` / `SampleObj` / `KeymapObj` are documented (`bindings_audio.rs`) as
**larger** than the 4-byte `NodeObj` / `PortObj` / `BusObj`. Passing a small
4-byte foreign where a large one is expected reads **past the allocated
foreign block** into adjacent VM heap — a genuine heap buffer over-read.
`arg_input`'s own safety comment relies on never casting to a large type on an
unverified slot; the unguarded sites violate exactly that invariant.

## The 9 sites

All are reachable from a public Wren surface (`prelude.wren`). `Node.xxx_(…)`
underlying foreigns can be called directly, bypassing any prelude-level
`is`-guard, so the fix **must** live in Rust (the `_impl`), not only in the
Wren wrapper.

| # | `_impl` (file) | Read | Public Wren call → malformed arg |
|---|---|---|---|
| 1 | `sample_from_impl` (`bindings_audio.rs`) | `get_list_count(1)` / `get_list_element(1,i,2)` | `SampleBuffer.from(5)` |
| 2 | `wavetable_from_impl` (`bindings_audio.rs`) | `get_list_count(1)` … | `Wavetable.from(5)` |
| 3 | `wavetable_from2d_impl` (`bindings_audio.rs`) | `get_list_count(1)` outer + `get_list_count(2)` inner | `Wavetable.from2d(5)` / `Wavetable.from2d([5,6])` |
| 4 | `node_polywt_pooled_impl` (`bindings_audio.rs`) | `foreign_mut::<WtObj>(1)` | `Node.polywt_pooled_(5, 220)` |
| 5 | `node_wavetable_pooled_impl` (`bindings_audio.rs`) | `foreign_mut::<WtObj>(1)` | `Node.wavetable_pooled_(5, 220)` |
| 6 | `node_player_impl` (`bindings_audio.rs`) | `foreign_mut::<SampleObj>(1)` | `Player.new(5)` / `Node.player_(5)` |
| 7 | `node_steps_impl` (`bindings_audio.rs`) | `get_list_count(1)` | `Steps.new(5, clock)` |
| 8 | `node_patch_impl` (`bindings_audio.rs`) | `*foreign_mut::<u8>(1)` tag peek with **no `slot_type==Foreign` check** — worse than #4–#6 | `Out.patch(5)` / `Out.patch("x")` / `Out.patch([1,2])` — the most-used call in every example script |
| 9 | `oled_text_impl` (`bindings.rs`) | `get_str(3)` (UB-class per above) | `Oled.text(0, 0, 42)` |

Also confirmed safe / out of scope: self/receiver `foreign_mut::<T>(0)` reads
(guaranteed correct by Wren single-dispatch); ~100+ `get_f`/`get_bool` scalar
reads (safe by the NaN-tagging analysis above — no hardening for memory safety,
only optional input-validation UX which we are **not** doing here).

## Design

### Shared checked-read helpers

Backend-generic free functions over `&impl SlotApi`, in
`crates/deluge-wren-core/src/slotapi.rs` (reachable from both `bindings.rs`
and `bindings_audio.rs`), generalizing the idiom `keymap_from_impl` /
`node_polysampleplayer_impl` already hand-roll:

- `checked_list_count<S: SlotApi>(vm: &S, slot: i32) -> usize`
  — `if vm.slot_type(slot) == WrenType::List { vm.get_list_count(slot).max(0) as usize } else { 0 }`.
  A subsequent `0..count` walk is then always in-bounds.
- `checked_list_element<S: SlotApi>(vm: &S, list: i32, idx: usize, count: usize, elem: i32) -> bool`
  — guards **fixed-index** sub-reads: returns `false` (caller skips / uses a
  default) when `idx >= count`; else performs `get_list_element` and returns
  `true`. (For plain `0..count` loops, `checked_list_count` alone suffices;
  this helper is for non-sequential reads.)
- `checked_tagged_foreign<S: SlotApi, T>(vm: &S, slot: i32, want_tag: u8) -> Option<&mut T>`
  — `slot_type==Foreign` → peek the tag byte via `foreign_mut::<u8>(slot)` →
  if it equals `want_tag`, return `Some(unsafe { foreign_mut::<T>(slot) })`,
  else `None`; non-foreign → `None`. This is the safe generalization of the
  Sa-2 tag-dispatch and the fix for the heap-over-read (a small foreign's tag
  won't match, so the large cast never happens). Requires the concrete `T`'s
  tag constant at the call site (`TAG_SAMPLE`, `TAG_KEYMAP`, and the `WtObj`
  tag — implementer confirms the exact constant names).
- `checked_str<S: SlotApi>(vm: &S, slot: i32) -> &str`
  — `if slot_type==String { get_str } else { "" }`.

### Per-site guard + degrade target

Degrade targets follow the crate's established "never panic, degrade to
silence" contract (identical to `keymap_from_impl` / `sample_new`).

| Site | Guard | Degrades to |
|---|---|---|
| `sample_from` | `checked_list_count` + bounded copy | empty buffer (len 0) |
| `wavetable_from` | `checked_list_count` | empty/zero table (existing None-handle path) |
| `wavetable_from2d` | `checked_list_count` outer + inner | 0 frames |
| `node_steps` | `checked_list_count` | 0 steps |
| `node_polywt_pooled` | `checked_tagged_foreign::<WtObj>` | `None` handle → silent node |
| `node_wavetable_pooled` | `checked_tagged_foreign::<WtObj>` | `None` handle → silent node |
| `node_player` | `checked_tagged_foreign::<SampleObj>` | `None` handle → silent node |
| `node_patch` | `slot_type==Foreign` + tag gate **before** the `TAG_BUS` peek | fall through to the existing `arg_input` / master-bus path (already tolerant of non-`Bus` foreigns) |
| `oled_text` | `checked_str` | skip the draw (empty string) |

### Error-handling policy

**Silent degrade only.** No `Fiber.abort` / prelude `is`-guards added — the
Rust guard is the single mechanism, matching the Sa-2 precedent. A malformed
argument yields the safe default and the call returns normally.

## Non-breaking guarantee

On a **well-formed** argument every guard is a pure pass-through: the
`slot_type` check succeeds and the exact same read runs as before, producing a
byte-identical Cmd stream and render. The helpers are additive. No correct
script changes behavior. The full existing suite must stay green in **both**
feature configs (default + `--features deluge-dsp-kernels/simd`).

## Testing

- **Helper unit tests:** each `checked_*` returns the safe default on a
  wrong-type slot and the real value on a correct one.
- **Per-site malformed-input regression test:** call each public Wren surface
  with a wrong-type argument via `run_script_ok` and assert a clean,
  non-crashing return (mirrors the three `keymap_from_malformed_*` tests). The
  `wren-sys` backend actually executes the C VM, so these tests genuinely
  exercise the previously-UB path.
- **Non-breaking:** all pre-existing valid-input tests stay green, both configs.

## Constraints (global)

- `no_std`, no heap, **no panic AND no UB** on any input.
- MIT/Apache-2.0 only.
- Scalar oracle / `simd` fast path parity; tests pass in both configs.
- Purely additive to correct-script behavior (byte-identical happy path).

## Out of scope / deferred

- Prelude-level `is`-type guards with `Fiber.abort` for nicer error messages
  (chosen against: silent degrade matches precedent).
- Input-validation UX for `get_f`/`get_bool` scalar args (memory-safe already).
- The separate `validateApiSlot` slot-index-bounds axis (not user-triggerable
  via normal fixed-arity calls).
- Any refactor of the already-guarded reference sites (`keymap_from_impl`,
  `node_polysampleplayer_impl`, `arg_input`, `write_source_to_bus`); they may
  optionally be re-expressed in terms of the new shared helpers if it reduces
  duplication without changing behavior, but that is not required.

## Likely task decomposition (for writing-plans)

1. Shared `checked_*` helpers in `slotapi.rs` + their unit tests.
2. The four list-arg sites (`sample_from`, `wavetable_from`,
   `wavetable_from2d`, `node_steps`).
3. The three tagged-foreign sites (`node_polywt_pooled`,
   `node_wavetable_pooled`, `node_player`).
4. `node_patch` (`Out.patch`) — the highest-traffic site.
5. `oled_text` + `checked_str` (in `bindings.rs`).
6. A consolidated malformed-input regression suite across all 9 surfaces.
