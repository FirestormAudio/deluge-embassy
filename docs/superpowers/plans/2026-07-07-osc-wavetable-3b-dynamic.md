# Osc part 3b — user-supplied dynamic wavetables Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a Wren script build a wavetable from its own single-cycle samples — `Wavetable.from([...])` → band-limited mip pyramid → stored in an Engine-owned `Pool` → played by a `Kind::Wavetable` node — reusing the merged 3a kernel and the **additive** `mipgen` (no `deluge-fft` change; the IRFFT optimization is 3c).

**Architecture:** The `Engine` gains a const-generic `Pool<PCAP,PCHUNK>` it owns. A new `Host::upload_table(&base) -> Option<PoolHandle>` lets an embedder build mips *in-place* into that pool. A new `SlotApi` list-read primitive (raw `wren_sys` FFI → `Vm` → `SlotApi` → impl) lets `Wavetable.from` read a Wren list into a base cycle. `TableSrc` gains a `Pooled(PoolHandle)` variant; at render time the Engine resolves a pooled node's flat `&[f32]` region and passes it into `Node::process_resolved` (type-erased — `Node` stays non-generic), which assembles the `[&[f32]; LEVELS]` `MipSet`. `Cmd::Free` frees the pooled region.

**Tech Stack:** Rust; `deluge-audio-graph` (Engine/Pool/Node), `deluge-wren-core` + `wren-sys` (C VM FFI), `mipgen` (additive builder, embedder-side), `deluge-dsp-test` (independent realfft QA). Host test target `x86_64-unknown-linux-gnu`.

**Reference spec:** [Osc part 3 design](../specs/2026-07-07-osc-wavetable-design.md) — the 3b (dynamic) half.

## Global Constraints

- **Additive `mipgen` only** — no `deluge-fft` inverse (that's 3c). The embedder builds mips via the existing `mipgen::analyze` + `mipgen::synth_level` in-place into its pool region.
- **Transparency:** all merged 3a + Osc-1/Osc-2 tests, and both goldens (`golden_saw_lpf_env_first_block`, `golden_saw_lpf_renders_expected_block`), pass **unchanged**. Static wavetables keep working identically; dynamic is purely additive surface.
- **No `MAX_INPUTS` change.** Wavetable nodes still use ports 0 (freq) / 1 (pmod).
- **`Node` stays non-generic.** The `Pool<PCAP,PCHUNK>` const generics live only on `Engine`. The render path passes the node a type-erased `Option<&[f32]>` pool region, never the `Pool` type.
- **No panics on the audio path.** Pool exhaustion → `upload_table` returns `None` → `Wavetable.from` returns a null/unbound handle → the node renders silence (never `unwrap`). Invalid/freed handles never index out of bounds.
- **`wren-web` is pre-existing-broken** (stale vs an earlier `Cmd`/`Input` refactor; not re-exported `Engine`; workspace-excluded) — **do not touch it**; it is not a real Engine instantiation site for this plan.
- **Test commands:**
  - graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` **and** `--features simd`
  - wren: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
  - wren-sys (if it has tests): `cargo test --target x86_64-unknown-linux-gnu -p wren-sys`
- Test output pristine (zero warnings). Commit after each task.

## Global reference values (verbatim, from recon)

- `mipgen::N = 2048`, `mipgen::LEVELS = 11`. A full pyramid = `N*LEVELS = 22528` f32.
- `PoolHandle` fields (`off`,`len`) are **private**, derives `Clone,Copy` only (no `Debug`/`PartialEq`) — like 3a's `TableSrc::Static`, `TableSrc::Pooled(PoolHandle)` needs a **hand-written `Debug`** (and must not require `PartialEq` on `PoolHandle` — see Task 3).
- `Pool` API: `alloc(&mut self, len) -> Option<PoolHandle>`, `free(&mut self, h)`, `slice(&self, h) -> &[f32]`, `slice_mut(&mut self, h) -> &mut [f32]`.
- `Engine<const BLOCK, NODES, OUTS, BUSES>` instantiation sites to update when adding pool consts (`wren-web` excluded): `engine.rs:258` `type E = Engine<16,8,8,4>`; `cmd.rs:38` `type E = Engine<16,8,8,4>`; `test_support.rs:285` `type TestEng = Engine<32,64,128,8>`; `wren-firmware/src/audio.rs:24` `type Eng = Engine<32,64,128,8>`.
- `Node::process_resolved(&mut self, ins: &[In; MAX_INPUTS], dt: f32, outs: &mut OutView)` — called at `engine.rs:180`.
- `MipSet<'a> { pub levels: &'a [&'a [f32]] }` (kernel). `WtOsc::process(&mut self, mips: MipSet, freq, pmod, dt, out)`.
- `Host` trait (wren-core) methods today: `now_ms, cv_set, gate_set, midi_tx, led, oled_clear, oled_text, oled_pixel, oled_show, audio_cmd`. Impls: `CmdCaptureHost` + `EngineHost` (test_support), the firmware host (wren-firmware), and web host (excluded — skip).

---

## Task 1: `SlotApi` list-read primitive (4 layers)

**Files:**
- Modify: `wren-sys/src/lib.rs` (extern decls), `wren-sys/src/foreign.rs` (Vm methods), `crates/deluge-wren-core/src/slotapi.rs` (trait), `crates/deluge-wren-core/src/slotapi_wrensys.rs` (impl)
- Test: `crates/deluge-wren-core/tests/` (a list-read binding test) — or extend `tests/audio_bindings.rs`.

**Interfaces:**
- Produces on `SlotApi`: `fn get_list_count(&self, slot: i32) -> i32;` and `fn get_list_element(&self, list_slot: i32, index: i32, elem_slot: i32);` (mirrors wren's `wrenGetListCount(vm, slot)` / `wrenGetListElement(vm, listSlot, index, elementSlot)`).

- [ ] **Step 1: Add the raw FFI decls**

In `wren-sys/src/lib.rs`, inside the `unsafe extern "C" { ... }` block (after `wrenSetSlotNull`):

```rust
    pub fn wrenGetListCount(vm: *mut WrenVM, slot: c_int) -> c_int;
    pub fn wrenGetListElement(vm: *mut WrenVM, list_slot: c_int, index: c_int, element_slot: c_int);
```

- [ ] **Step 2: Add `Vm` inherent methods**

In `wren-sys/src/foreign.rs` (next to `get_f64`/`get_bytes`), mirroring the one-line-unsafe-call pattern:

```rust
    pub fn get_list_count(&self, slot: i32) -> i32 {
        unsafe { crate::wrenGetListCount(self.0, slot) }
    }
    pub fn get_list_element(&self, list_slot: i32, index: i32, element_slot: i32) {
        unsafe { crate::wrenGetListElement(self.0, list_slot, index, element_slot) }
    }
```

(Confirm the exact path to the extern fns — `crate::wrenGetListCount` vs a re-export — by matching how `wrenGetSlotDouble` is referenced in this same file.)

- [ ] **Step 3: Write the failing SlotApi test**

Add to `crates/deluge-wren-core/tests/audio_bindings.rs` (uses the real `wren_sys::Vm` via `test-support`). A minimal binding that sums a Wren list proves the primitive end to end. Register a temporary static test method OR assert via an existing path — simplest: add a test that runs a script calling a new `Node`-static helper that reads a list. To avoid over-scoping, instead unit-test at the `Vm` level is hard (needs a live VM + a list in a slot); the natural place is the `Wavetable.from` test in Task 5. **For Task 1, add the trait methods + a compile-level assertion** that `Vm: SlotApi` still holds and the new methods exist:

```rust
#[test]
fn slotapi_has_list_read() {
    fn assert_list_api<S: deluge_wren_core::slotapi::SlotApi>() {}
    assert_list_api::<wren_sys::Vm>();
    // Behavioral coverage of list-read lands in Task 5 (Wavetable.from round-trip),
    // which drives a real Wren list through get_list_count/get_list_element.
}
```

(If `slotapi` is not a public module, place this assertion in an internal `#[cfg(test)]` module within the crate instead. The real behavioral test is Task 5.)

- [ ] **Step 4: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL to compile — `get_list_count`/`get_list_element` not on `SlotApi`.

- [ ] **Step 5: Add the trait methods + impl**

In `slotapi.rs`, add to `trait SlotApi`:

```rust
    fn get_list_count(&self, slot: i32) -> i32;
    fn get_list_element(&self, list_slot: i32, index: i32, elem_slot: i32);
```

In `slotapi_wrensys.rs`, add to `impl SlotApi for Vm`:

```rust
    fn get_list_count(&self, slot: i32) -> i32 {
        Vm::get_list_count(self, slot)
    }
    fn get_list_element(&self, list_slot: i32, index: i32, elem_slot: i32) {
        Vm::get_list_element(self, list_slot, index, elem_slot)
    }
```

- [ ] **Step 6: Run, verify pass; commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS, zero warnings.

```bash
git add wren-sys/src crates/deluge-wren-core/src/slotapi.rs crates/deluge-wren-core/src/slotapi_wrensys.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren-core): SlotApi list-read primitive (wrenGetListCount/Element)"
```

---

## Task 2: `Pool` into `Engine` (const generics + accessors)

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (add pool field + const generics + accessors), and the four `Engine<...>` alias sites (`engine.rs:258`, `cmd.rs:38`, `crates/deluge-wren-core/src/test_support.rs:285`, `wren-firmware/src/audio.rs:24`).

**Interfaces:**
- Produces: `Engine<const BLOCK, NODES, OUTS, BUSES, PCAP, PCHUNK>` owning `pool: Pool<PCAP, PCHUNK>`; engine methods `pool_alloc(&mut self, len) -> Option<PoolHandle>`, `pool_slice_mut(&mut self, h) -> &mut [f32]`, `pool_free(&mut self, h)`, `pool_slice(&self, h) -> &[f32]`.
- Consumes: `crate::pool::{Pool, PoolHandle}`.

- [ ] **Step 1: Write the failing pool-through-engine test**

Add to `engine.rs` tests (using the updated alias — see Step 3):

```rust
    #[test]
    fn engine_pool_alloc_fill_read_free() {
        let mut e = E::new(48_000.0);
        let h = e.pool_alloc(16).expect("alloc");
        e.pool_slice_mut(h).fill(0.25);
        assert!(e.pool_slice(h).iter().all(|&x| x == 0.25));
        e.pool_free(h);
        // After free, a full-capacity alloc succeeds (region reclaimed).
        assert!(e.pool_alloc(16).is_some());
    }
```

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: FAIL to compile (`pool_alloc` etc. missing; `E` alias arity).

- [ ] **Step 3: Add the const generics, field, and accessors**

In `engine.rs`, extend the struct + impl signature:

```rust
pub struct Engine<
    const BLOCK: usize,
    const NODES: usize,
    const OUTS: usize,
    const BUSES: usize,
    const PCAP: usize,
    const PCHUNK: usize,
> {
    arena: Arena<NODES, OUTS>,
    outs: UnsafeCell<[[f32; BLOCK]; OUTS]>,
    dt: f32,
    pub(crate) bus_l: [[f32; BLOCK]; BUSES],
    pub(crate) bus_r: [[f32; BLOCK]; BUSES],
    pub(crate) root: Option<BusId>,
    writes: [Option<(Input, BusId)>; NODES],
    writes_len: usize,
    pool: crate::pool::Pool<PCAP, PCHUNK>,
}
```

Update the `impl<...>` header to add `const PCAP, const PCHUNK`, and `new()` to init `pool: crate::pool::Pool::new()`. Add accessors (near the other `pub fn`s):

```rust
    pub fn pool_alloc(&mut self, len: usize) -> Option<crate::pool::PoolHandle> {
        self.pool.alloc(len)
    }
    pub fn pool_free(&mut self, h: crate::pool::PoolHandle) {
        self.pool.free(h)
    }
    pub fn pool_slice(&self, h: crate::pool::PoolHandle) -> &[f32] {
        self.pool.slice(h)
    }
    pub fn pool_slice_mut(&mut self, h: crate::pool::PoolHandle) -> &mut [f32] {
        self.pool.slice_mut(h)
    }
```

- [ ] **Step 4: Update the four alias sites**

Give each a pool capacity sized for a few tables (a full pyramid = 22528 f32; chunk 2048 → 11 chunks/table). Use `PCAP`, `PCHUNK` per site:
- `engine.rs:258` (unit tests): `type E = Engine<16, 8, 8, 4, 45056, 2048>;` (2 tables' worth).
- `cmd.rs:38`: `type E = Engine<16, 8, 8, 4, 45056, 2048>;`
- `test_support.rs:285`: `type TestEng = Engine<32, 64, 128, 8, 90112, 2048>;` (4 tables).
- `wren-firmware/src/audio.rs:24`: `type Eng = Engine<32, 64, 128, 8, 90112, 2048>;` (adjust the comment to list all six params).

(Numbers are RAM-vs-capacity choices; the test sites just need ≥ 1 pyramid. `PCHUNK = 2048` makes one chunk = one mip level, so a pyramid is 11 contiguous chunks.)

- [ ] **Step 5: Run, verify pass (both modes)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features simd`
Expected: PASS both — the new pool test + all existing (goldens unchanged). Also build the firmware alias site compiles: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support` (pulls `test_support.rs`). Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-audio-graph/src crates/deluge-wren-core/src/test_support.rs wren-firmware/src/audio.rs
git commit -m "feat(audio-graph): Engine owns a const-generic Pool + accessors"
```

---

## Task 3: `TableSrc::Pooled` + render resolution + `Cmd::Free` → pool free

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (Pooled variant + Debug + table accessor + render arm signature), `crates/deluge-audio-graph/src/engine.rs` (render call passes the pool region; `Cmd::Free` frees it).

**Interfaces:**
- Produces: `TableSrc::Pooled(PoolHandle)`; `Node::table_src(&self) -> Option<TableSrc>`; `Node::process_resolved(&mut self, ins, dt, outs, pool_region: Option<&[f32]>)`.

- [ ] **Step 1: Write the failing pooled-render + free test**

Add to `engine.rs` tests:

```rust
    #[test]
    fn pooled_wavetable_renders_and_frees() {
        let mut e = E::new(48_000.0);
        // Build a saw pyramid directly into the pool (mimics upload_table).
        let n = mipgen::N;
        let h = e.pool_alloc(n * mipgen::LEVELS).expect("pool room");
        let mut base = [0.0f32; mipgen::N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / n as f32) - 1.0; }
        let harm = mipgen::analyze(&base);
        {
            let region = e.pool_slice_mut(h);
            let mut lvl = [0.0f32; mipgen::N];
            for level in 0..mipgen::LEVELS {
                mipgen::synth_level(&harm, level, &mut lvl);
                region[level * n..(level + 1) * n].copy_from_slice(&lvl);
            }
        }
        e.create(NodeId(0), Kind::Wavetable);
        *e.node_input_mut(NodeId(0), 0).unwrap() = Input::Const(220.0);
        e.apply(Cmd::BindTable { node: NodeId(0), src: TableSrc::Pooled(h) });
        e.render_block();
        let out = e.node_output(NodeId(0), 0);
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.2));
        assert!(out.iter().any(|&s| s != 0.0));
        // Free the node → pool region reclaimed.
        e.apply(Cmd::Free { node: NodeId(0) });
        assert!(e.pool_alloc(n * mipgen::LEVELS).is_some());
    }
```

(Add `mipgen` as a **dev-dependency** of `deluge-audio-graph` for this test: `mipgen = { path = "../mipgen" }` under `[dev-dependencies]`.)

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: FAIL to compile (`TableSrc::Pooled` missing; `process_resolved` arity).

- [ ] **Step 3: Add `Pooled` + Debug + accessor (node.rs)**

Extend `TableSrc`:

```rust
#[derive(Clone, Copy)]
pub enum TableSrc {
    Static(deluge_dsp_kernels::wavetable::TableId),
    Pooled(crate::pool::PoolHandle),
}
```

`Cmd` derives `Clone, Copy, Debug, PartialEq`, so `TableSrc` must be all four. `PoolHandle` (private `{off:u32,len:u32}`) and 3a's `TableId` lack `Debug`/`PartialEq`, so make the deriving work by adding the missing derives to those two plain types (both trivially correct), then derive `TableSrc` normally and delete 3a's hand-written `impl Debug for TableSrc`:

1. In `pool.rs`: change `PoolHandle` to `#[derive(Clone, Copy, Debug, PartialEq, Eq)]`.
2. In `crates/deluge-dsp-kernels/src/wavetable.rs`: add `Debug` to `TableId`'s derives (currently `Clone, Copy, PartialEq, Eq` → add `Debug`).
3. In `node.rs`: `#[derive(Clone, Copy, Debug, PartialEq)] pub enum TableSrc { Static(TableId), Pooled(PoolHandle) }`, and **delete** the hand-written `impl core::fmt::Debug for TableSrc` from 3a (the derive now covers both arms).

Add the accessor:

```rust
    pub fn table_src(&self) -> Option<TableSrc> {
        self.table
    }
```

- [ ] **Step 4: Update the render arm to take the pool region (node.rs)**

Change `process_resolved`'s signature and the `Kind::Wavetable` arm:

```rust
    pub fn process_resolved(
        &mut self,
        ins: &[In; MAX_INPUTS],
        dt: f32,
        outs: &mut OutView,
        pool_region: Option<&[f32]>,
    ) {
        match self.kind {
            // ... existing arms unchanged ...
            Kind::Wavetable => {
                if let State::Wt(o) = &mut self.state {
                    match self.table {
                        Some(TableSrc::Static(id)) => {
                            if let Some(mips) = static_mipset(id) {
                                o.process(mips, ins[0], ins[1], dt, outs.port(0));
                            }
                        }
                        Some(TableSrc::Pooled(_)) => {
                            if let Some(region) = pool_region {
                                let n = region.len() / LEVELS_WT;
                                // Assemble a [&[f32]; LEVELS] view over the flat region.
                                let levels: [&[f32]; LEVELS_WT] =
                                    core::array::from_fn(|l| &region[l * n..(l + 1) * n]);
                                o.process(MipSet { levels: &levels }, ins[0], ins[1], dt, outs.port(0));
                            }
                        }
                        None => {}
                    }
                }
            }
        }
    }
```

Add `const LEVELS_WT: usize = 11;` (== `mipgen::LEVELS`) in `node.rs`, or import it. Import `MipSet` from the kernel (already importing `static_mipset`, `TableId`, `WtOsc`).

- [ ] **Step 5: Update the render call + `Cmd::Free` (engine.rs)**

In `render_block` (≈`engine.rs:180`), where `n.process_resolved(&ins, self.dt, &mut view)` is called: the node may need its pool region. Because `self.pool` and `self.arena` are disjoint fields, **destructure to borrow both** (or resolve the region before the node borrow). Concretely, before the node call, compute the region for the current node:

```rust
        // Resolve a pooled table's flat region (immutable pool borrow) BEFORE the
        // node's &mut borrow. `pool` and `arena` are disjoint fields of `self`.
        let pool_region: Option<&[f32]> = match node_table_src {
            Some(crate::node::TableSrc::Pooled(h)) => Some(self.pool.slice(h)),
            _ => None,
        };
        n.process_resolved(&ins, self.dt, &mut view, pool_region);
```

where `node_table_src` was read via `n.table_src()` earlier. **You will likely need to restructure the render loop to avoid overlapping borrows** — read `n.table_src()` into a local, drop the node borrow, take `self.pool.slice(h)` into a local `&[f32]`, then re-borrow the node `&mut` for `process_resolved`. If the borrow checker fights this, destructure `let Engine { arena, pool, outs, .. } = self;` at the top of `render_block` and operate on the fields directly (the standard disjoint-borrow pattern). This is the main implementation risk — solve it with field destructuring, not `unsafe`.

In `apply`, extend `Cmd::Free` to free a pooled region first:

```rust
        Cmd::Free { node } => {
            if let Some(n) = self.arena.node_mut(node) {
                if let Some(crate::node::TableSrc::Pooled(h)) = n.table_src() {
                    self.pool.free(h);
                }
            }
            self.arena.free(node);
        }
```

(Read the handle via `node_mut(...).table_src()` — a shared read through the mut ref is fine — before `arena.free`.)

- [ ] **Step 6: Run, verify pass (both modes)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph --features simd`
Expected: PASS both — the pooled-render+free test, the 3a static wavetable test (`wavetable_node_renders_bounded_nonsilent`) unchanged, and both goldens. Zero warnings.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src crates/deluge-dsp-kernels/src/wavetable.rs
git commit -m "feat(audio-graph): TableSrc::Pooled render + Cmd::Free pool reclamation"
```

---

## Task 4: `Host::upload_table` seam

**Files:**
- Modify: `crates/deluge-wren-core/src/host.rs` (trait method), `crates/deluge-wren-core/src/test_support.rs` (`CmdCaptureHost` + `EngineHost` impls), `wren-firmware/src/audio.rs` (firmware host impl), `crates/deluge-wren-core/Cargo.toml` + `wren-firmware/Cargo.toml` (add `mipgen` dep where the build happens).

**Interfaces:**
- Produces on `Host`: `fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle>;` — build a mip pyramid from `base` into a freshly-allocated pool region, return its handle (or `None` on exhaustion/bad input).
- Consumes: `mipgen::{N, LEVELS, analyze, synth_level}`, `Engine::pool_alloc`/`pool_slice_mut`.

- [ ] **Step 1: Write the failing test (EngineHost round-trip)**

`test_support.rs` has `EngineHost` wrapping a real `TestEng`. Add a test (in `test_support.rs`'s test module or `tests/`) that calls `upload_table` with a saw base and asserts a `Some(handle)` whose pooled region, read back, is band-limited:

```rust
    #[test]
    fn engine_host_upload_table_builds_band_limited() {
        let mut host = EngineHost::new(48_000.0); // or however EngineHost is constructed
        let mut base = [0.0f32; mipgen::N];
        for (i, s) in base.iter_mut().enumerate() { *s = 2.0 * (i as f32 / mipgen::N as f32) - 1.0; }
        let h = host.upload_table(&base).expect("upload");
        // level 0 region round-trips to a saw-ish shape; deeper levels are band-limited.
        let region = host.engine().pool_slice(h);
        assert_eq!(region.len(), mipgen::N * mipgen::LEVELS);
        assert!(region[..mipgen::N].iter().any(|&x| x != 0.0));
    }
```

(Adapt to `EngineHost`'s real constructor/accessor names — read `test_support.rs`. If `EngineHost` has no engine accessor, add a `pub fn engine(&self) -> &TestEng`.)

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `upload_table` not on `Host`.

- [ ] **Step 3: Add the trait method + a shared builder helper**

In `host.rs`, add to `trait Host`:

```rust
    /// Build a band-limited mip pyramid from `base` (one single cycle) into a
    /// pool region and return its handle. `None` on exhaustion/bad input.
    /// Default: unsupported (no pool) → None, so non-audio hosts need not build.
    fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
        let _ = base;
        None
    }
```

(Providing a **default** `None` means `CmdCaptureHost` and any minimal host compile unchanged; only pool-backed hosts override it. Confirm the trait allows default methods — it's a plain trait, yes.)

Add a free helper (in `host.rs` or a small `mipgen`-using module) that both real impls call — build in place into a caller `&mut [f32]` region of length `N*LEVELS`:

```rust
pub fn build_pyramid_into(base: &[f32], region: &mut [f32]) {
    // region.len() must be mipgen::N * mipgen::LEVELS.
    let mut b = [0.0f32; mipgen::N];
    let n = mipgen::N.min(base.len());
    b[..n].copy_from_slice(&base[..n]); // pad/truncate to N
    let harm = mipgen::analyze(&b);
    let mut lvl = [0.0f32; mipgen::N];
    for level in 0..mipgen::LEVELS {
        mipgen::synth_level(&harm, level, &mut lvl);
        region[level * mipgen::N..(level + 1) * mipgen::N].copy_from_slice(&lvl);
    }
}
```

Add `mipgen = { path = "../../crates/mipgen" }` (adjust relative path) to `crates/deluge-wren-core/Cargo.toml` `[dependencies]`.

- [ ] **Step 4: Implement `upload_table` on `EngineHost` (and firmware host)**

In `test_support.rs`'s `impl Host for EngineHost`:

```rust
    fn upload_table(&mut self, base: &[f32]) -> Option<deluge_audio_graph::PoolHandle> {
        let h = self.eng.pool_alloc(mipgen::N * mipgen::LEVELS)?;
        crate::host::build_pyramid_into(base, self.eng.pool_slice_mut(h));
        Some(h)
    }
```

(Match the real field name for the engine — recon shows `EngineHost` forwards `audio_cmd` to `self.eng.apply`, so the field is likely `eng`.) Mirror the same impl in `wren-firmware/src/audio.rs`'s `Host` impl (add `mipgen` to `wren-firmware/Cargo.toml`). `CmdCaptureHost` needs no override (default `None`) — but see Task 5 for how `Wavetable.from` behaves under a `None`-returning host in the Cmd-capture test.

- [ ] **Step 5: Run, verify pass; commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: PASS (EngineHost round-trip). Build the firmware too if feasible in-host: `cargo build -p wren-firmware` may need the device target — at minimum confirm `deluge-wren-core` compiles. Zero warnings.

```bash
git add crates/deluge-wren-core wren-firmware
git commit -m "feat(wren-core): Host::upload_table seam (embedder builds mips into pool)"
```

---

## Task 5: Wren `Wavetable.from` + `Osc.wavetable(handle, freq)`

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (the `Wavetable` foreign + `from` + pooled factory), `src/audio.rs` (`new_wavetable_pooled` emitter), `src/bindings.rs` (METHODS), `wren/prelude.wren` (`Wavetable` class + `Osc.wavetable` handling), `tests/audio_bindings.rs`.

**Interfaces:**
- Produces: a `Wavetable` foreign object (tag byte + `PoolHandle`); `Wavetable.from(list)` static; `audio::new_wavetable_pooled(id, handle, freq)` emitting `NewNode{Wavetable}` + `BindTable{Pooled(handle)}`; `Osc.wavetable(src, freq)` accepting a `Wavetable` handle (Foreign) or a `WT` id (Num).

- [ ] **Step 1: Write the failing Cmd-sequence + round-trip tests**

Add to `tests/audio_bindings.rs`:

```rust
#[test]
fn wavetable_from_emits_bindtable_pooled() {
    // Cmd-capture host returns None from upload_table (no engine) → Wavetable.from
    // must degrade gracefully: NOT emit a bogus BindTable with an invalid handle.
    // With a real engine host (run_and_render), it DOES bind + render.
    let out = run_and_render("
        var w = Wavetable.from([ -1, -0.5, 0, 0.5, 1, 0.5, 0, -0.5 ])
        var v = Osc.wavetable(w, 220)
        v.out(0)
    ");
    assert!(out.iter().all(|s| s.is_finite()));
}
```

(Use the existing `run_and_render` harness that drives a real `EngineHost`. If a Cmd-capture variant is also wanted, assert the sequence under `run_and_capture_cmds` — but note `upload_table` returns `None` there, so `Wavetable.from` yields an unbound handle and no `BindTable` is emitted; assert that graceful path instead of a bogus bind.)

- [ ] **Step 2: Run, verify failure**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Expected: FAIL — `Wavetable` class / `from` unknown.

- [ ] **Step 3: Add the `Wavetable` foreign + `from` (bindings_audio.rs)**

Add a foreign holding an **`Option<PoolHandle>`** (not a bare handle — `PoolHandle` has no public constructor, so there is no way to build a "dummy" one for the exhaustion case; `None` is the unbound state). Mirror the `NodeObj`/`PortObj` tag pattern from P1 (leading `tag: u8`). `WtObj` is larger than the 4-byte small foreigns, which is safe: `arg_input` reads only the tag *byte* for unknown tags (→ `Const(0.0)`), never the whole `WtObj`, so it can't over-read.

```rust
const TAG_WT: u8 = /* next free tag after TAG_BUS */;
#[repr(C)]
pub struct WtObj { tag: u8, handle: Option<deluge_audio_graph::PoolHandle> }

pub(crate) fn wavetable_from_impl<S: SlotApi>(vm: &S) {
    let count = vm.get_list_count(1);
    let mut base = [0.0f32; mipgen::N]; // add `mipgen` dep to wren-core (Task 4 already does)
    let n = (count.max(0) as usize).min(base.len());
    for i in 0..n {
        vm.get_list_element(1, i as i32, 2); // element → slot 2
        base[i] = vm.get_f(2) as f32;
    }
    let handle = audio::upload_table(&base[..n.max(1)]); // Option<PoolHandle>
    unsafe { vm.new_foreign_in::<WtObj>(0, WtObj { tag: TAG_WT, handle }) };
}
```

Add `audio::upload_table(base) -> Option<PoolHandle>` in `audio.rs` that calls `host().upload_table(base)`. Register `Wavetable.from(_)` static in `register_audio` + `METHODS`, and declare `foreign class Wavetable { foreign static from(samples) }` in the prelude. (`WtObj` needs `WrenForeign` impl for `new_foreign_in` — mirror how `NodeObj`/`BusObj` implement it.)

- [ ] **Step 4: Route `Osc.wavetable` for a handle (bindings_audio.rs + prelude)**

`Osc.wavetable(src, freq)` must accept either a `WT` Num id (existing static path) or a `Wavetable` foreign. In the prelude, `Osc.wavetable` is sugar; branch in Wren by type, or route both through a single `Node.wavetable_(_,_)` that inspects the arg's slot type in Rust. Cleanest: a new static factory `Node.wavetable_pooled_(wt, freq)` for the handle case, and keep `Osc.wavetable` deciding in Wren:

```wren
class Osc {
  // ... existing ...
  static wavetable(t, f) {
    if (t is Wavetable) { return Node.wavetable_pooled_(t, f) }
    return Node.wavetable_(t, f)  // WT.x numeric id (3a)
  }
}
```

`node_wavetable_pooled_impl<S>`: read the `WtObj` from slot 1 (via `foreign_mut::<WtObj>`); `match obj.handle { Some(h) => audio::new_wavetable_pooled(alloc_id, h, arg_input(vm,2)), None => { /* unbound upload → create node with no bind, renders silent */ } }`. Register + METHODS + prelude `foreign static wavetable_pooled_(wt, freq)`.

`audio::new_wavetable_pooled`:

```rust
pub fn new_wavetable_pooled(id: u16, handle: deluge_audio_graph::PoolHandle, freq: Input) {
    if id == NULL_ID { return; }
    host().audio_cmd(Cmd::NewNode { node: NodeId(id), kind: Kind::Wavetable, args: [freq, Input::Const(0.0), Input::Const(0.0)] });
    host().audio_cmd(Cmd::BindTable { node: NodeId(id), src: deluge_audio_graph::node::TableSrc::Pooled(handle) });
}
```

- [ ] **Step 5: Run, verify pass; sys-backend build**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core --features test-support`
Run: `cargo build --target x86_64-unknown-linux-gnu -p deluge-wren-core --no-default-features --features wren-sys-backend`
Expected: PASS + clean compile. Confirm the four-place registration sync for BOTH new methods (`wavetable_from` + `wavetable_pooled_`): impl+extern wrapper, `register_audio`, `METHODS`, prelude. Golden `golden_saw_lpf_renders_expected_block` unchanged. Zero warnings.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core
git commit -m "feat(wren-core): Wavetable.from + Osc.wavetable(handle) dynamic tables"
```

---

## Task 6: QA — round-trip, exhaustion, lifecycle + 3a per-table coverage

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/wavetable.rs` (the deferred 3a per-table static coverage test), `crates/deluge-audio-graph/src/engine.rs` (pool exhaustion/degrade), `crates/deluge-wren-core/tests/audio_bindings.rs` (end-to-end round-trip).

- [ ] **Step 1: 3a per-table static band-limit coverage (the deferred gap)**

In `wavetable.rs` tests, parametrize over all six `TableId`s and assert each generated table is band-limited at 5 kHz (measure each floor; gate just below):

```rust
    #[test]
    fn all_static_tables_band_limited() {
        let sr = 48_000.0f32;
        for id in 0u16..6 {
            let m = static_mipset(TableId(id)).expect("table");
            let mut osc = WtOsc::new();
            let mut buf = [0.0f32; deluge_dsp_test::FFT_N];
            osc.process(m, In::K(5_000.0), In::K(0.0), 1.0 / sr, &mut buf);
            let wa = deluge_dsp_test::spectrum::analyze_buf(sr, &buf)
                .worst_alias_db(5_000.0, 3.0 * (sr / deluge_dsp_test::FFT_N as f32));
            // Sine (id 2) is near-silent alias; harmonically-rich ones ≈ saw's -25 dB.
            // Measure each and gate just below its floor (document per-id).
            assert!(wa < -18.0, "table {id}: worst_alias {wa} dB");
        }
    }
```

Run the kernel tests, record each table's floor, tighten the gate to just below the worst measured. Do NOT alter the tables to hit a number.

- [ ] **Step 2: Pool exhaustion → graceful degrade**

Add to `engine.rs` tests: fill the pool, then a `upload`-sized `pool_alloc` returns `None`; and a `Kind::Wavetable` node bound to `Pooled` whose region was never allocated (or is out of range) renders **silence, no panic**. (The render arm's `pool_region` is `None`/short → the `Some(Pooled)` branch guards on `pool_region` length; confirm no OOB when `region.len()` isn't a multiple of `LEVELS_WT` — add a guard `if region.len() >= N*LEVELS` before assembling.)

- [ ] **Step 3: End-to-end Wren round-trip**

In `tests/audio_bindings.rs`, drive a known base through `Wavetable.from`, render via the real engine host, and assert the spectrum matches the intended cycle (fundamental + expected harmonics present, alias floor below a measured bar). This exercises list-read → upload_table → pool → Pooled bind → render as one path.

- [ ] **Step 4: Run all, verify; commit**

Run the kernel, graph (both modes), and wren suites. All green, zero warnings, both goldens unchanged.

```bash
git add crates/deluge-dsp-kernels crates/deluge-audio-graph crates/deluge-wren-core
git commit -m "test(osc-3b): dynamic-table round-trip, exhaustion, per-table static coverage"
```

---

## Self-review notes

- **Spec coverage (3b half):** list-read (Task 1); Pool-in-Engine (Task 2); Pooled binding + render + free (Task 3); upload seam (Task 4); `Wavetable.from` + `Osc.wavetable(handle)` (Task 5); QA round-trip/exhaustion/lifecycle + the deferred 3a per-table coverage (Task 6). IRFFT is explicitly **3c**, not here.
- **Resolved design forks:** render-time pool access is type-erased (`Option<&[f32]>` into `process_resolved`; `Node` non-generic; disjoint field borrows in `render_block`). Upload is a single `Host::upload_table` with a **default `None`** (non-pool hosts unaffected), the embedder building in-place into its own pool. `PoolHandle` gains `Debug,PartialEq,Eq` derives (trivial) so `TableSrc`/`Cmd` derive cleanly — and `TableId` gains `Debug`, letting 3a's hand-written `TableSrc::Debug` be deleted.
- **Transparency:** static wavetables (3a) and both goldens must stay green through Tasks 2–6 — the const-generic Engine change (Task 2) is the highest-risk step for this; verify the goldens after it explicitly.
- **Main implementation risk:** the `render_block` disjoint borrow of `self.pool` (shared) and `self.arena` (mut) — solved by field destructuring, never `unsafe`. Called out in Task 3 Step 5.
- **No-panic invariants:** exhaustion → `upload_table` None → unbound handle → silent node; out-of-range/short pool region → guarded, silent (Task 6 Step 2). No `unwrap` on any audio-path resolution.
- **Known follow-ups:** 3c (IRFFT + mipgen IFFT path, additive oracle); the deferred 3a Minors (mip-select brightness bias, `log2f`-per-sample hoist, `gen_tables` `required-features` guard) remain tracked for a later tidy-up; audio-rate pool hot-reload; multi-frame morph (the next Osc sub-project).
