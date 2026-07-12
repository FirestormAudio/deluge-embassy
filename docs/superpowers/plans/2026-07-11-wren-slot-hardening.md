# Wren Slot-Read UB Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** No malformed Wren argument can trigger memory-unsafety in the bindings — guard all 9 user-reachable slot reads that currently type-confuse or heap-over-read on a wrong-type/out-of-range argument, degrading each to the crate's safe default.

**Architecture:** Add three backend-generic `checked_*` free functions to `slotapi.rs` that wrap `SlotApi` reads with a `slot_type` (and, for lists, a count) guard, returning a safe default on mismatch. Replace each unguarded read at the 9 sites with the matching helper. Correct scripts take the identical code path (the guard is a pass-through on well-formed input).

**Tech Stack:** Rust `no_std` (`deluge-wren-core`), the `wren-sys` C-VM backend (its `ASSERT` type/bounds checks are compiled out, which is the root cause), the `run_script_ok` integration-test harness in `crates/deluge-wren-core/tests/audio_bindings.rs`.

## Global Constraints

- `no_std`, no heap, **no panic AND no UB** on any input.
- MIT/Apache-2.0 only.
- Scalar path is the oracle; `#[cfg(feature="simd")]` fast path must match; tests pass in BOTH configs (default + `--features deluge-dsp-kernels/simd`).
- **Silent degrade only** — no `Fiber.abort`, no prelude `is`-guards. The Rust guard is the sole mechanism; a malformed arg yields the safe default and the call returns normally.
- **Non-breaking:** on a well-formed argument every guard is a pure pass-through — byte-identical Cmd stream and render. Purely additive; the full existing suite stays green in both configs.
- Tag constants (verified): `TAG_NODE=0, TAG_PORT=1, TAG_BUS=2, TAG_WT=3, TAG_SAMPLE=4, TAG_KEYMAP=5` in `bindings_audio.rs`. All audio foreigns are `#[repr(C)]` with `tag: u8` first.
- Test invocation is per-crate, NEVER `--workspace`; use `-- name1 name2` to scope: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- <names>`. LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## File Structure

- `crates/deluge-wren-core/src/slotapi.rs` — defines `trait SlotApi` + `enum WrenType`. **Add the three `checked_*` helpers here** (both binding files can `use crate::slotapi::…`).
- `crates/deluge-wren-core/src/bindings_audio.rs` — 8 of the 9 sites (all audio factories/setters). Add `use` for the helpers.
- `crates/deluge-wren-core/src/bindings.rs` — the 9th site (`oled_text_impl`).
- `crates/deluge-wren-core/tests/audio_bindings.rs` — the `run_script_ok` harness; add malformed-input regression tests here.

## Helper design (introduced across Tasks 1–2, defined once in `slotapi.rs`)

```rust
/// List length if `slot` is actually a List, else 0. A subsequent `0..count`
/// walk is then always in-bounds (the VM does NO bounds/type checking).
pub(crate) fn checked_list_count<S: SlotApi>(vm: &S, slot: i32) -> usize {
    if vm.slot_type(slot) == WrenType::List {
        vm.get_list_count(slot).max(0) as usize
    } else {
        0
    }
}

/// `&str` if `slot` is a String, else `""`. Guards `get_str`/`wrenGetSlotBytes`,
/// which dereferences `AS_STRING` unconditionally (UB on a non-String slot).
pub(crate) fn checked_str<S: SlotApi>(vm: &S, slot: i32) -> &str {
    if vm.slot_type(slot) == WrenType::String {
        vm.get_str(slot)
    } else {
        ""
    }
}

/// `Some(&mut T)` iff `slot` is a Foreign whose leading tag byte == `want_tag`,
/// else `None`. Prevents the heap over-read where a small 4-byte foreign
/// (Node/Port/Bus) is cast to a larger `WtObj`/`SampleObj`: the tag won't
/// match, so the large cast never happens. Mirrors `arg_input`'s tag peek.
pub(crate) fn checked_tagged_foreign<T, S: SlotApi>(vm: &S, slot: i32, want_tag: u8) -> Option<&mut T> {
    if vm.slot_type(slot) != WrenType::Foreign {
        return None;
    }
    // SAFETY: slot is a Foreign, so it is at least the 1-byte tag (every foreign
    // is >= 4 bytes). Read only the tag, copy it out, then drop that borrow.
    let tag = unsafe { *vm.foreign_mut::<u8>(slot) };
    if tag != want_tag {
        return None;
    }
    // SAFETY: tag == want_tag guarantees the foreign really is a T.
    Some(unsafe { vm.foreign_mut::<T>(slot) })
}
```

> `WrenType` variants used: `List`, `String`, `Foreign` (defined in `slotapi.rs`). The `checked_tagged_foreign` turbofish binds `T` first, `S` inferred: `checked_tagged_foreign::<SampleObj>(vm, 1, TAG_SAMPLE)`.
>
> **Why no dedicated helper unit tests:** there is no mock `SlotApi` in the crate (the `wren-sys` `Vm` is the only impl, and the real UB only manifests through the C VM). Each helper's branches are covered by integration tests through the real VM: the malformed-input tests below exercise the **default** branch, and the pre-existing valid-input tests (`SampleBuffer.from([...])`, `Out.patch(s.out)`, etc.) cover the **happy** branch.

---

### Task 1: List- and string-argument guards (`checked_list_count`, `checked_str`)

Guards the 4 list-arg sites + the OLED string read. Public surfaces: `SampleBuffer.from`, `Wavetable.from`, `Wavetable.from2d`, `Steps.new`, `Oled.text`.

**Files:**
- Modify: `crates/deluge-wren-core/src/slotapi.rs` (add `checked_list_count` + `checked_str`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`sample_from_impl`, `wavetable_from_impl`, `wavetable_from2d_impl`, `node_steps_impl`; add `use`)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (`oled_text_impl`; add `use`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `SlotApi::slot_type/get_list_count/get_str`, `WrenType::{List,String}`.
- Produces: `pub(crate) fn checked_list_count<S: SlotApi>(vm: &S, slot: i32) -> usize`; `pub(crate) fn checked_str<S: SlotApi>(vm: &S, slot: i32) -> &str`. Task 2 adds `checked_tagged_foreign` alongside them.

- [ ] **Step 1: Write the failing malformed-input tests**

In `crates/deluge-wren-core/tests/audio_bindings.rs` (use the existing `run_script_ok` harness — read the `keymap_from_malformed_*` tests for the exact import/style):

```rust
#[test]
fn sample_from_malformed_arg_not_a_list_does_not_crash() {
    assert!(run_script_ok("var b = SampleBuffer.from(5)"), "SampleBuffer.from(non-list) degrades, no crash");
}

#[test]
fn wavetable_from_malformed_arg_not_a_list_does_not_crash() {
    assert!(run_script_ok("var w = Wavetable.from(5)"), "Wavetable.from(non-list) degrades, no crash");
}

#[test]
fn wavetable_from2d_malformed_args_do_not_crash() {
    assert!(run_script_ok("var w = Wavetable.from2d(5)"), "from2d(non-list) degrades");
    assert!(run_script_ok("var w = Wavetable.from2d([5, 6])"), "from2d([non-list frames]) degrades");
}

#[test]
fn steps_malformed_values_not_a_list_does_not_crash() {
    assert!(run_script_ok("var s = Steps.new(5, 1)"), "Steps.new(non-list values) degrades to 0 steps, no crash");
}

#[test]
fn oled_text_malformed_non_string_does_not_crash() {
    assert!(run_script_ok("Oled.text(0, 0, 42)"), "Oled.text(non-string) skips draw, no crash");
}
```

> If `Oled.text` is not reachable in `audio_bindings.rs`'s harness (it lives in `bindings.rs`, not the audio bindings), place `oled_text_malformed_non_string_does_not_crash` in whichever integration test file boots the full prelude + host via `run_script_ok`; confirm the surface is callable there before finalizing. If `Steps.new`'s clock arg cannot be a bare number in this prelude, substitute a valid clock source (e.g. a `Metro`/`Osc` per the existing Steps tests) — the point is that the *values* arg (`5`) is the malformed one.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- sample_from_malformed wavetable_from_malformed wavetable_from2d_malformed steps_malformed oled_text_malformed`
Expected: these tests either FAIL or the process ABORTS/segfaults (the reads are UB today — a crash that prevents the run from completing is itself the "fail"). If the harness catches nothing because the specific malformed value happens not to fault on this run, that is acceptable pre-implementation noise; the guarantee is that after the fix they pass deterministically.

- [ ] **Step 3: Add the two helpers to `slotapi.rs`**

Append to `crates/deluge-wren-core/src/slotapi.rs` (after the `SlotApi` trait). `WrenType` is already in scope in this module:

```rust
/// List length if `slot` is actually a List, else 0. A subsequent `0..count`
/// walk is then always in-bounds (the wren-sys VM does NO bounds/type checking
/// — its C `ASSERT`s are compiled out).
pub(crate) fn checked_list_count<S: SlotApi>(vm: &S, slot: i32) -> usize {
    if vm.slot_type(slot) == WrenType::List {
        vm.get_list_count(slot).max(0) as usize
    } else {
        0
    }
}

/// `&str` if `slot` is a String, else `""`. Guards `get_str`, which the VM
/// backs with an unconditional `AS_STRING` deref (UB on a non-String slot).
pub(crate) fn checked_str<S: SlotApi>(vm: &S, slot: i32) -> &str {
    if vm.slot_type(slot) == WrenType::String {
        vm.get_str(slot)
    } else {
        ""
    }
}
```

- [ ] **Step 4: Apply the guards at the 5 sites**

In `crates/deluge-wren-core/src/bindings_audio.rs`, ensure the helper is imported (add to the existing `use crate::slotapi::…` line, or add one): `use crate::slotapi::checked_list_count;`

**`wavetable_from_impl`** — change the first line:
```rust
// BEFORE
    let count = vm.get_list_count(1);
    let mut base = [0.0f32; mipgen::N];
    let n = (count.max(0) as usize).min(base.len());
// AFTER
    let count = checked_list_count(vm, 1);
    let mut base = [0.0f32; mipgen::N];
    let n = count.min(base.len());
```

**`wavetable_from2d_impl`** — guard BOTH the outer and inner counts:
```rust
// BEFORE (outer):
    let nframes = (vm.get_list_count(1).max(0)) as usize;
// AFTER (outer):
    let nframes = checked_list_count(vm, 1);
```
```rust
// BEFORE (inner, inside the closure):
        vm.get_list_element(1, f as i32, 2); // frame f -> slot 2
        let n = (vm.get_list_count(2).max(0) as usize).min(base.len());
// AFTER (inner):
        vm.get_list_element(1, f as i32, 2); // frame f -> slot 2
        let n = checked_list_count(vm, 2).min(base.len());
```
> `get_list_element(1, f, 2)` here is in-bounds because `f < nframes` and `nframes` now comes from `checked_list_count(vm, 1)` (0 if slot 1 isn't a list → the `upload_table_2d` loop never runs). The inner `checked_list_count(vm, 2)` then guards a frame element that isn't itself a list.

**`sample_from_impl`** — change the first line:
```rust
// BEFORE
    let count = vm.get_list_count(1).max(0) as usize;
// AFTER
    let count = checked_list_count(vm, 1);
```

**`node_steps_impl`** — change the first line:
```rust
// BEFORE
    let count = vm.get_list_count(1).max(0) as usize;
// AFTER
    let count = checked_list_count(vm, 1);
```

In `crates/deluge-wren-core/src/bindings.rs`, add `use crate::slotapi::checked_str;` and change **`oled_text_impl`**:
```rust
// BEFORE
    let s = vm.get_str(3);
// AFTER
    let s = checked_str(vm, 3);
```

- [ ] **Step 5: Run the malformed tests + full regression, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- sample_from_malformed wavetable_from_malformed wavetable_from2d_malformed steps_malformed oled_text_malformed`
Expected: PASS (all degrade cleanly).
Then full suite both configs:
`cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` and again with `--features deluge-dsp-kernels/simd`.
Expected: all green (existing valid-input tests unchanged — the guards are pass-throughs on well-formed lists/strings).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/slotapi.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "fix(wren): guard list/string slot reads — checked_list_count/checked_str (4 factories + Oled.text)"
```

---

### Task 2: Foreign-argument guards (`checked_tagged_foreign` + `node_patch` gate)

Guards the 3 tagged-foreign sites + the `Out.patch` tag peek. Public surfaces: `Player.new`/`Node.player_`, `Node.wavetable_pooled_`, `Node.polywt_pooled_` (all reachable directly, bypassing any prelude `is`-guard), and `Out.patch`.

**Files:**
- Modify: `crates/deluge-wren-core/src/slotapi.rs` (add `checked_tagged_foreign`)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_player_impl`, `node_wavetable_pooled_impl`, `node_polywt_pooled_impl`, `node_patch_impl`; add `use`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `SlotApi::{slot_type, foreign_mut}`, `WrenType::Foreign`, `TAG_WT`, `TAG_SAMPLE`, `TAG_BUS`, the `WtObj`/`SampleObj`/`BusObj` structs, `checked_list_count`/`checked_str` (Task 1, same module).
- Produces: `pub(crate) fn checked_tagged_foreign<T, S: SlotApi>(vm: &S, slot: i32, want_tag: u8) -> Option<&mut T>`.

- [ ] **Step 1: Write the failing malformed-input tests**

In `crates/deluge-wren-core/tests/audio_bindings.rs`:

```rust
#[test]
fn player_malformed_buffer_not_a_foreign_does_not_crash() {
    assert!(run_script_ok("var p = Player.new(5)"), "Player.new(non-foreign) -> unbound silent node, no crash");
}

#[test]
fn wavetable_pooled_malformed_not_a_wavetable_does_not_crash() {
    assert!(run_script_ok("var n = Node.wavetable_pooled_(5, 220)"), "wavetable_pooled_(non-foreign) degrades, no crash");
    // A wrong-tag foreign (a Bus, not a Wavetable) must also degrade, not heap-over-read:
    assert!(run_script_ok("var b = Bus.new()\nvar n = Node.wavetable_pooled_(b, 220)"), "wavetable_pooled_(Bus) degrades, no over-read");
}

#[test]
fn polywt_pooled_malformed_not_a_wavetable_does_not_crash() {
    assert!(run_script_ok("var n = Node.polywt_pooled_(5, 220)"), "polywt_pooled_(non-foreign) degrades, no crash");
    assert!(run_script_ok("var b = Bus.new()\nvar n = Node.polywt_pooled_(b, 220)"), "polywt_pooled_(Bus) degrades, no over-read");
}

#[test]
fn out_patch_malformed_non_source_does_not_crash() {
    assert!(run_script_ok("Out.patch(5)"), "Out.patch(Num) -> patches silence, no crash");
    assert!(run_script_ok("Out.patch(\"x\")"), "Out.patch(String) -> patches silence, no crash");
    assert!(run_script_ok("Out.patch([1, 2])"), "Out.patch(List) -> patches silence, no crash");
}
```

> Confirm the exact public names against `prelude.wren`: `Player.new`, `Bus.new`, `Out.patch`, and the `Node.wavetable_pooled_`/`Node.polywt_pooled_` foreign statics (the audit found these callable directly). If `Bus.new` isn't the constructor name, use whatever the prelude exposes for a Bus; the wrong-tag case just needs *some* foreign of a different tag than `WtObj`. If a bare number can't be a `Player.new` arg for a syntactic reason, wrap as needed — the arg `5` must reach `node_player_impl` as a non-foreign slot.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- player_malformed wavetable_pooled_malformed polywt_pooled_malformed out_patch_malformed`
Expected: FAIL or process abort/segfault (UB today — the wrong-tag `Bus` cases in particular read past the 4-byte `BusObj` into adjacent heap when cast to the larger `WtObj`).

- [ ] **Step 3: Add `checked_tagged_foreign` to `slotapi.rs`**

Append to `crates/deluge-wren-core/src/slotapi.rs`:

```rust
/// `Some(&mut T)` iff `slot` is a Foreign whose leading tag byte == `want_tag`,
/// else `None`. Prevents the heap over-read where a small 4-byte foreign
/// (Node/Port/Bus) is cast to a larger `WtObj`/`SampleObj`: a mismatched tag
/// yields `None` before the large cast happens. Mirrors `arg_input`'s tag peek.
pub(crate) fn checked_tagged_foreign<T, S: SlotApi>(vm: &S, slot: i32, want_tag: u8) -> Option<&mut T> {
    if vm.slot_type(slot) != WrenType::Foreign {
        return None;
    }
    // SAFETY: slot is a Foreign, so it is at least the 1-byte tag (every foreign
    // is >= 4 bytes). Read only the tag, copy it out, then drop that borrow.
    let tag = unsafe { *vm.foreign_mut::<u8>(slot) };
    if tag != want_tag {
        return None;
    }
    // SAFETY: tag == want_tag guarantees the foreign really is a T.
    Some(unsafe { vm.foreign_mut::<T>(slot) })
}
```

- [ ] **Step 4: Apply the guards at the 4 sites**

In `crates/deluge-wren-core/src/bindings_audio.rs`, add `use crate::slotapi::checked_tagged_foreign;` (or extend the existing `use crate::slotapi::…`).

**`node_player_impl`** (reads `handle` + `len` from a `SampleObj`):
```rust
// BEFORE
    let obj = unsafe { vm.foreign_mut::<SampleObj>(1) };
    let handle = obj.handle;
    let len = obj.len;
// AFTER
    let (handle, len) = match checked_tagged_foreign::<SampleObj>(vm, 1, TAG_SAMPLE) {
        Some(obj) => (obj.handle, obj.len),
        None => (None, 0),
    };
```

**`node_wavetable_pooled_impl`** (reads `handle` from a `WtObj`):
```rust
// BEFORE
    let handle = unsafe { vm.foreign_mut::<WtObj>(1) }.handle;
// AFTER
    let handle = checked_tagged_foreign::<WtObj>(vm, 1, TAG_WT).and_then(|o| o.handle);
```

**`node_polywt_pooled_impl`** (reads `handle` + `frames` from a `WtObj`):
```rust
// BEFORE
    let wt = unsafe { vm.foreign_mut::<WtObj>(1) };
    let handle = wt.handle;
    let kind = poly_wt_kind(wt.frames as usize);
// AFTER
    let (handle, frames) = match checked_tagged_foreign::<WtObj>(vm, 1, TAG_WT) {
        Some(wt) => (wt.handle, wt.frames as usize),
        None => (None, 1), // frames=1 (single-cycle) is inert; handle None -> plain silent node
    };
    let kind = poly_wt_kind(frames);
```

**`node_patch_impl`** — gate the tag peek on `slot_type == Foreign` (mirrors `write_source_to_bus` one function above, which already does this correctly):
```rust
// BEFORE
    let tag = unsafe { *vm.foreign_mut::<u8>(1) };
    if tag == TAG_BUS {
        let bus = unsafe { vm.foreign_mut::<BusObj>(1) }.id;
        audio::set_root(bus);
    } else {
        write_source_to_bus(vm, 1, audio::MASTER_BUS);
        audio::set_root(audio::MASTER_BUS);
    }
// AFTER
    // Only peek the tag byte if arg 1 is actually a Foreign; a Num/String/List/
    // Null slot must NOT be reinterpreted as an Obj pointer (UB). Non-Bus (and
    // non-Foreign) falls through to write_source_to_bus, which already tolerates
    // non-Foreign args via arg_input.
    let is_bus = vm.slot_type(1) == WrenType::Foreign
        && unsafe { *vm.foreign_mut::<u8>(1) } == TAG_BUS;
    if is_bus {
        let bus = unsafe { vm.foreign_mut::<BusObj>(1) }.id;
        audio::set_root(bus);
    } else {
        write_source_to_bus(vm, 1, audio::MASTER_BUS);
        audio::set_root(audio::MASTER_BUS);
    }
```

- [ ] **Step 5: Run the malformed tests + full regression, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- player_malformed wavetable_pooled_malformed polywt_pooled_malformed out_patch_malformed`
Expected: PASS (all degrade cleanly, including the wrong-tag `Bus` over-read cases).
Then full suite both configs:
`cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` and again with `--features deluge-dsp-kernels/simd`.
Expected: all green — the existing `Player.new(buffer)`, `Osc.wavetable(wt, f)`, and `Out.patch(s.out)` tests still pass (a correctly-tagged foreign matches `want_tag`, so `checked_tagged_foreign` returns `Some` and the identical read runs).

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/slotapi.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "fix(wren): guard foreign slot reads — checked_tagged_foreign (player/wavetable_pooled/polywt_pooled) + Out.patch Foreign gate"
```

---

## Self-Review

**Spec coverage:** All 9 sites from the spec are covered — Task 1: `sample_from`, `wavetable_from`, `wavetable_from2d`, `node_steps`, `oled_text` (5); Task 2: `node_player`, `node_wavetable_pooled`, `node_polywt_pooled`, `node_patch` (4). The three helpers (`checked_list_count`, `checked_str`, `checked_tagged_foreign`) match the spec. `checked_list_element` from the spec's helper list is intentionally DROPPED (YAGNI): every target list site is a sequential `0..count` walk that `checked_list_count` fully guards; the only fixed-index sub-reads live in the already-guarded `keymap_from_impl`, which the spec leaves as-is. Silent-degrade-only policy honored (no `Fiber.abort`). Non-breaking pass-through honored.

**Placeholder scan:** No TBD/TODO/"handle edge cases". Every code step shows exact before/after. The two "confirm the public name / substitute a valid clock" notes are verification instructions with concrete fallbacks, not deferred design.

**Type consistency:** `checked_list_count(&S,i32)->usize`, `checked_str(&S,i32)->&str`, `checked_tagged_foreign::<T,S>(&S,i32,u8)->Option<&mut T>` used identically across both tasks. Tags (`TAG_SAMPLE`, `TAG_WT`, `TAG_BUS`) and structs (`SampleObj`, `WtObj`, `BusObj`) match the verified source. `WrenType::{List,String,Foreign}` are real variants in `slotapi.rs`.
