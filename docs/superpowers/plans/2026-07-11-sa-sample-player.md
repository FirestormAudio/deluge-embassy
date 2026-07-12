# Sa-1: In-memory Sample Buffer + Player Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `SampleBuffer.from([…])` uploads raw PCM to the pool; `Player.new(buffer)` plays it back Hermite-interpolated at `speed × 2^(semitones/12)`, one-shot or looped (loop points), `trigger`-restartable. Mono source.

**Architecture:** A `SamplePlayer` kernel (position accumulator + 4-point Hermite + one-shot/loop). A `Kind::SamplePlayer` pooled source reading its PCM via `pool_region` (Wavetable/Delay template). A raw-PCM upload path (`pool_set` host method) + Wren `SampleBuffer`/`Player`, reusing the existing `Node.trigger`.

**Tech Stack:** Rust `no_std` (kernels → graph → wren-core), `libm` (`exp2f`), Wren. Buffer pool + `pool_region` threading already exist.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/upload paths. `libm` (`exp2f`) — NO std float methods. Bounded loops. Pool never panics on exhaustion (`None` → graceful silence).
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** additive — new kernel, new `Kind`, new `pool_set` host method (trait default no-op), new Wren classes. No existing kernel/node/effect/wavetable behavior changes. `Node.trigger` gains ONE match arm (additive).
- **Two registration tables:** new foreigns register in BOTH `install_methods` (bindings_audio.rs) AND `METHODS` (bindings.rs) + prelude decls/classes. All proposed names verified FREE (`Player`, `SampleBuffer`, `speed=`/`semitones=`/`loopStart=`/`loopEnd=`/`loop=`, `node_player`, `node_sample`, `Kind::SamplePlayer`, `TAG_SAMPLE=4`). `trigger` is REUSED from the existing `Node.trigger()` (do NOT add a new trigger selector).
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

---

### Task 1: `SamplePlayer` kernel

**Files:**
- Create: `crates/deluge-dsp-kernels/src/sampler.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod sampler;` — alphabetical: after `pub mod reverb;`, before `pub mod shape;`)
- Test: `sampler.rs` `#[cfg(test)]` module

**Interfaces:**
- Consumes: `libm::exp2f`, the PCM as a `&[f32]` (the pool region).
- Produces: `pub struct SamplePlayer` (`new()`, `set_speed`/`set_semitones`/`set_loop_start`/`set_loop_end`/`set_loop_mode`, `trigger()`, `process(&mut self, pcm: &[f32], dt: f32, out: &mut [f32])`), `#[derive(Clone, Copy)]`. Used by Task 2.

- [ ] **Step 1: Write the failing oracle tests**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    fn player() -> SamplePlayer { SamplePlayer::new() }

    #[test]
    fn plays_buffer_verbatim_at_speed_1() {
        // Hermite interpolates its knots, so at integer positions (speed 1) the
        // output equals the PCM sample-for-sample.
        let pcm: [f32; 8] = core::array::from_fn(|i| (i as f32) * 0.1 - 0.3);
        let mut p = player(); // defaults: speed 1, one-shot, loop_end will be set below
        p.set_loop_end(pcm.len() as f32);
        p.trigger();
        let mut out = [0.0f32; 8];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        for i in 0..8 { assert!((out[i] - pcm[i]).abs() < 1e-5, "sample {} verbatim", i); }
    }

    #[test]
    fn speed_2_reads_every_other() {
        let pcm: [f32; 8] = core::array::from_fn(|i| i as f32);
        let mut p = player();
        p.set_loop_end(8.0);
        p.set_speed(2.0);
        p.trigger();
        let mut out = [0.0f32; 4];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        // pos 0,2,4,6 → pcm 0,2,4,6
        assert!((out[0] - 0.0).abs() < 1e-5 && (out[1] - 2.0).abs() < 1e-5
             && (out[2] - 4.0).abs() < 1e-5 && (out[3] - 6.0).abs() < 1e-5);
    }

    #[test]
    fn semitone_12_doubles_rate() {
        // +12 semitones == speed 2: same read pattern as speed_2.
        let pcm: [f32; 8] = core::array::from_fn(|i| i as f32);
        let mut p = player();
        p.set_loop_end(8.0);
        p.set_semitones(12.0);
        p.trigger();
        let mut out = [0.0f32; 4];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        assert!((out[3] - 6.0).abs() < 1e-4, "+12 semis doubles rate, got {}", out[3]);
    }

    #[test]
    fn one_shot_stops_at_end_and_retriggers() {
        let pcm = [1.0f32; 4];
        let mut p = player();
        p.set_loop_end(4.0);
        p.trigger();
        let mut out = [0.0f32; 8];
        p.process(&pcm, 1.0 / 48_000.0, &mut out); // 8 out, 4-sample buffer
        assert!(out[0..4].iter().all(|&v| (v - 1.0).abs() < 1e-5), "plays the buffer");
        assert!(out[4..8].iter().all(|&v| v.abs() < 1e-6), "silence after end (one-shot)");
        // retrigger → plays again
        let mut out2 = [0.0f32; 4];
        p.trigger();
        p.process(&pcm, 1.0 / 48_000.0, &mut out2);
        assert!(out2.iter().all(|&v| (v - 1.0).abs() < 1e-5), "retrigger replays");
    }

    #[test]
    fn loop_wraps_within_region() {
        // loop the whole 4-sample buffer; render 12 samples → 3 repeats.
        let pcm: [f32; 4] = [10.0, 20.0, 30.0, 40.0];
        let mut p = player();
        p.set_loop_start(0.0);
        p.set_loop_end(4.0);
        p.set_loop_mode(true);
        p.trigger();
        let mut out = [0.0f32; 12];
        p.process(&pcm, 1.0 / 48_000.0, &mut out);
        for i in 0..12 { assert!((out[i] - pcm[i % 4]).abs() < 1e-4, "loop repeats at {}", i); }
    }
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels sampler` (or `-- plays_buffer speed_2 semitone_ one_shot loop_wraps`)
Expected: FAIL (`SamplePlayer` not defined).

- [ ] **Step 3: Implement the kernel**

`crates/deluge-dsp-kernels/src/sampler.rs`:

```rust
//! In-memory sample playback (Sa-1): a `SamplePlayer` reads a PCM buffer at a
//! playback rate (speed × semitone transpose) with 4-point Hermite interpolation,
//! one-shot or looped (with loop points), restartable via `trigger`. Mono,
//! `no_std`, no heap.

/// 4-point (Catmull-Rom) Hermite. At `frac == 0` returns `y1` exactly, so integer
/// read positions reproduce the source sample verbatim.
#[inline]
fn hermite(y0: f32, y1: f32, y2: f32, y3: f32, frac: f32) -> f32 {
    let c0 = y1;
    let c1 = 0.5 * (y2 - y0);
    let c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
    let c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
    ((c3 * frac + c2) * frac + c1) * frac + c0
}

#[derive(Clone, Copy)]
pub struct SamplePlayer {
    speed: f32,
    semitones: f32,
    loop_start: f32,
    loop_end: f32,
    loop_mode: bool,
    pos: f32,
    playing: bool,
}

impl SamplePlayer {
    pub fn new() -> SamplePlayer {
        SamplePlayer {
            speed: 1.0,
            semitones: 0.0,
            loop_start: 0.0,
            loop_end: 0.0, // set to buffer len by the graph/Wren on create
            loop_mode: false,
            pos: 0.0,
            playing: false,
        }
    }

    pub fn set_speed(&mut self, v: f32) { self.speed = v; }
    pub fn set_semitones(&mut self, v: f32) { self.semitones = v; }
    pub fn set_loop_start(&mut self, v: f32) { self.loop_start = v.max(0.0); }
    pub fn set_loop_end(&mut self, v: f32) { self.loop_end = v.max(0.0); }
    pub fn set_loop_mode(&mut self, on: bool) { self.loop_mode = on; }
    pub fn trigger(&mut self) {
        self.pos = if self.loop_mode { self.loop_start } else { 0.0 };
        self.playing = true;
    }

    pub fn process(&mut self, pcm: &[f32], dt: f32, out: &mut [f32]) {
        let _ = dt;
        let len = pcm.len();
        if len == 0 {
            for o in out.iter_mut() { *o = 0.0; }
            return;
        }
        let rate = self.speed * libm::exp2f(self.semitones / 12.0);
        // effective loop region, clamped to the buffer
        let ls = self.loop_start.max(0.0);
        let le = self.loop_end.min(len as f32);
        let loopable = self.loop_mode && le > ls + 1.0;
        for o in out.iter_mut() {
            if !self.playing {
                *o = 0.0;
                continue;
            }
            // index helper: clamp (one-shot) or wrap into [ls, le) (loop)
            let read = |idx: isize| -> f32 {
                let n = if loopable {
                    let span = (le - ls) as isize;
                    let base = ls as isize;
                    let mut k = (idx - base) % span;
                    if k < 0 { k += span; }
                    base + k
                } else {
                    idx.clamp(0, len as isize - 1)
                };
                pcm[n as usize]
            };
            let i = libm::floorf(self.pos) as isize;
            let frac = self.pos - (i as f32);
            *o = hermite(read(i - 1), read(i), read(i + 1), read(i + 2), frac);
            self.pos += rate;
            if loopable {
                if self.pos >= le { self.pos -= le - ls; }
                if self.pos < ls { self.pos += le - ls; } // guard reverse/underrun
            } else if self.pos >= len as f32 {
                self.playing = false;
            }
        }
    }
}

impl Default for SamplePlayer {
    fn default() -> Self { SamplePlayer::new() }
}
```

Add `pub mod sampler;` to lib.rs (after `reverb`, before `shape`).

- [ ] **Step 4: Run to verify pass (both configs)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels sampler` (+ `--features simd`).
Then FULL kernels crate both configs (purely additive). Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/sampler.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(kernels): SamplePlayer — Hermite PCM playback, speed/semitone, one-shot/loop"
```

---

### Task 2: `Kind::SamplePlayer` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import; `Kind` enum; `State` enum; constructor; `Node::trigger` at ~line 316; `process_resolved` arm ~near the `Kind::Delay`/`Kind::Wavetable` arms 614-676)
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `SamplePlayer` (Task 1); `TableSrc`/`table_src()`/`bind_table`; the `pool_region` param of `process_resolved`; the `Kind::Delay` pool_region arm as the template.
- Produces: `Kind::SamplePlayer` (out_width 1, is_poly false, poly_in_count 0), `State::SamplePlayer(SamplePlayer)`, `set_param(0..4)`, `Node::trigger` arm. Used by Tasks 3–4.

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn sample_player_node_reads_pool_and_triggers() {
    assert_eq!(Node::out_width(Kind::SamplePlayer), 1);
    assert!(!Node::is_poly(Kind::SamplePlayer));
    let mut n = Node::new(Kind::SamplePlayer, 0);
    n.set_param(3, 4.0); // loop_end = 4 (buffer len)
    n.trigger();          // start playback
    // Drive it with a pool region = the PCM. one-shot, speed 1 → verbatim.
    let pcm = [0.25f32, 0.5, -0.5, -0.25];
    let mut region = pcm; // a &mut [f32] pool region
    let ins = [In::A(&[0.0; 4]), In::A(&[0.0; 4]), In::A(&[0.0; 4])];
    let mut buf = [0.0f32; 4];
    {
        let mut outs = OutView::single(&mut buf);
        n.process_resolved(&ins, 1.0 / 48_000.0, &mut outs, Some(&mut region));
    }
    for i in 0..4 { assert!((buf[i] - pcm[i]).abs() < 1e-5, "plays pool PCM at {}", i); }
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph sample_player_node`
Expected: FAIL (`Kind::SamplePlayer` not found).

- [ ] **Step 3: Wire `Kind::SamplePlayer`**

In `crates/deluge-audio-graph/src/node.rs`:
1. **Import** (with the other kernel imports): `use deluge_dsp_kernels::sampler::SamplePlayer;` (match how sibling single-kernel imports are written).
2. **Kind enum:** add `SamplePlayer,`.
3. **State enum:** add `SamplePlayer(SamplePlayer),`.
4. **Constructor** (the `Kind::X => State::X(...)` block): `Kind::SamplePlayer => State::SamplePlayer(SamplePlayer::new()),`.
5. **`out_width`:** falls into `_ => 1` (do NOT add to the width-2 list).
6. **`is_poly`:** NOT listed → false. **`poly_in_count`:** NOT listed → 0. (Leave both — the default arms are correct; do not add SamplePlayer to either list.)
7. **`set_param`** (with the other `State::X => match param` arms):
```rust
    State::SamplePlayer(p) => match param {
        0 => p.set_speed(value),
        1 => p.set_semitones(value),
        2 => p.set_loop_start(value),
        3 => p.set_loop_end(value),
        4 => p.set_loop_mode(value != 0.0),
        _ => {}
    },
```
8. **`Node::trigger`** (node.rs:316-323 — add an arm):
```rust
    State::SamplePlayer(p) => p.trigger(),
```
9. **`process_resolved` arm** (beside `Kind::Delay` at node.rs:653; the Delay arm is the template — a raw `&mut [f32]` pool_region consumer, but SamplePlayer only READS so take `.as_deref()` for `&[f32]`):
```rust
    Kind::SamplePlayer => {
        let pcm: Option<&[f32]> = pool_region.as_deref();
        if let (State::SamplePlayer(p), Some(region)) = (&mut self.state, pcm) {
            p.process(region, dt, outs.port(0));
        } else {
            // no bound buffer → silence
            let port = outs.port(0);
            for o in port.iter_mut() { *o = 0.0; }
        }
    }
```

- [ ] **Step 4: Run + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph sample_player_node` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs (existing nodes untouched). Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): Kind::SamplePlayer — pooled PCM source (out_width 1, set_param 0-4, trigger)"
```

---

### Task 3: Raw-PCM upload + `SampleBuffer.from`

**Files:**
- Modify: `crates/deluge-wren-core/src/host.rs` (add `pool_set` trait method, default no-op)
- Modify: `crates/deluge-wren-core/src/test_support.rs` + `wren-firmware/src/audio.rs` (implement `pool_set` on the two `Host` embedders)
- Modify: `crates/deluge-wren-core/src/audio.rs` (add `pool_set` facade wrapper)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`SampleObj` foreign + `sample_from_impl` + register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`class SampleBuffer` + `foreign class` decl if needed)

**Interfaces:**
- Consumes: `audio::alloc_buffer` (existing — pool_allocs + zero-fills), `get_list_count`/`get_list_element`/`get_f`, `PoolHandle`, the `WtObj` foreign pattern.
- Produces: `Host::pool_set(&mut self, h: PoolHandle, index: usize, value: f32)`; `audio::pool_set`; `SampleObj { tag, handle: Option<PoolHandle>, len: u32 }`; Wren `SampleBuffer.from([…])`. Used by Task 4.

- [ ] **Step 1: The `pool_set` host path**

- `host.rs` — add to the `Host` trait a default no-op (mirror `alloc_buffer`'s default at host.rs:97):
```rust
    fn pool_set(&mut self, h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
        let _ = (h, index, value);
    }
```
- `test_support.rs` `EngineHost` and `wren-firmware/src/audio.rs` — override it (they already implement `alloc_buffer` with the real engine):
```rust
    fn pool_set(&mut self, h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
        let region = self.eng.pool_slice_mut(h);
        if index < region.len() { region[index] = value; }
    }
```
(Match the exact `self.eng`/engine field name each embedder uses.)
- `audio.rs` — facade wrapper (mirror `alloc_buffer` at audio.rs:295):
```rust
pub fn pool_set(h: deluge_audio_graph::PoolHandle, index: usize, value: f32) {
    host().pool_set(h, index, value);
}
```

- [ ] **Step 2: `SampleObj` + `sample_from_impl`** (mirror `WtObj`/`wavetable_from_impl`)

In `bindings_audio.rs`:
```rust
pub(crate) const TAG_SAMPLE: u8 = 4; // next free after TAG_WT=3

#[repr(C)]
pub(crate) struct SampleObj {
    pub tag: u8,
    pub handle: Option<deluge_audio_graph::PoolHandle>,
    pub len: u32,
}
impl WrenForeign for SampleObj {
    fn module_name() -> &'static str { "main" }
    fn class_name() -> &'static str { "SampleBuffer" }
}

/// `SampleBuffer.from([f32…])` — upload raw PCM verbatim into a pool region.
pub(crate) fn sample_from_impl<S: SlotApi>(vm: &S) {
    let count = vm.get_list_count(1).max(0) as usize;
    let handle = audio::alloc_buffer(count); // pool_allocs + zero-fills `count`
    if let Some(h) = handle {
        vm.ensure_slots(3);
        for i in 0..count {
            vm.get_list_element(1, i as i32, 2);
            audio::pool_set(h, i, vm.get_f(2) as f32);
        }
    }
    unsafe { vm.new_foreign_in::<SampleObj>(0, SampleObj { tag: TAG_SAMPLE, handle, len: count as u32 }) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn sample_from(raw: *mut WrenVM) {
    let vm = Vm(raw);
    sample_from_impl(&vm);
}
```
(`ensure_slots(3)` + the `get_list_element(1, i, 2)` + `get_f(2)` loop is verbatim the `wavetable_from_impl` pattern; only the store changes to `pool_set` instead of a local base array.)

- [ ] **Step 3: Register `SampleBuffer.from` in BOTH tables + prelude**

- `install_methods` (bindings_audio.rs, near `wavetable_from` registration ~2270): `method("main", "SampleBuffer", true, "from(_)", sample_from_impl::<S>);`
- `METHODS` (bindings.rs, near the Wavetable entry): `static_method("SampleBuffer", "from(_)", bindings_audio::sample_from),`
- prelude.wren: `foreign class SampleBuffer { foreign static from(samples) }` (mirror `foreign class Wavetable`, prelude.wren:417). Register the foreign class in the CLASSES list the same way `Wavetable` is (grep how `Wavetable` is added to `crate::CLASSES` / the foreign-class registration).

- [ ] **Step 4: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Gate: compiles clean (all 3 crates — host.rs trait change ripples to both embedders) + full existing suite green BOTH configs. Nothing exercises `SampleBuffer` yet (Task 5). Expected: PASS both. (Known pre-existing `golden_sim` flake — rerun if seen.)

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/host.rs crates/deluge-wren-core/src/test_support.rs wren-firmware/src/audio.rs crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): SampleBuffer.from — raw-PCM pool upload (pool_set host path)"
```

---

### Task 4: `Player.new` + setters

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (`new_sample_player` create helper)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_player_impl` factory + 5 setters + register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (Node foreign setters + `class Player`)

**Interfaces:**
- Consumes: `SampleObj` (Task 3, its `handle`+`len`); `Kind::SamplePlayer` + `set_param(0..4)` (Task 2); the `node_delay_impl`/`new_delay` pooled-create template; the existing `Node.trigger()` (node_trigger_impl) — REUSED, no new trigger.
- Produces: Wren `Player.new(buffer)` + setters `speed=`/`semitones=`/`loopStart=`/`loopEnd=`/`loop=`.

- [ ] **Step 1: `new_sample_player` create helper** (mirror `new_delay`, audio.rs:304)

```rust
pub fn new_sample_player(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, len: u32) {
    if id == NULL_ID { return; }
    host().audio_cmd(Cmd::NewNode {
        node: NodeId(id),
        kind: Kind::SamplePlayer,
        args: [Input::Const(0.0), Input::Const(0.0), Input::Const(0.0)], // pure source, no signal inputs
    });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable {
            node: NodeId(id),
            src: deluge_audio_graph::node::TableSrc::Pooled(h),
        });
        host().audio_cmd(Cmd::SetParam { node: NodeId(id), param: 3, value: len as f32 }); // loop_end = buffer len
    }
}
```
(Match the exact `Cmd::SetParam` field names used elsewhere in audio.rs.)

- [ ] **Step 2: `node_player_impl` factory** (mirror `node_wavetable_pooled_impl`, bindings_audio.rs:648)

```rust
pub(crate) fn node_player_impl<S: SlotApi>(vm: &S) {
    let obj = unsafe { vm.foreign_mut::<SampleObj>(1) };
    let handle = obj.handle;
    let len = obj.len;
    let id = audio::alloc_node_id();
    audio::new_sample_player(id, handle, len);
    unsafe { return_node(vm, id) };
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_player(raw: *mut WrenVM) {
    let vm = Vm(raw);
    node_player_impl(&vm);
}
```

- [ ] **Step 3: The 5 setters** (mirror `comp_set_threshold_impl` — `set_param` by index)

`node_set_speed`(0), `node_set_semitones`(1), `node_set_loop_start`(2), `node_set_loop_end`(3), `node_set_loop_mode`(4). Each:
```rust
pub(crate) fn node_set_speed_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    audio::set_param(self_id(vm), 0, v);
}
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn node_set_speed(raw: *mut WrenVM) { let vm = Vm(raw); node_set_speed_impl(&vm); }
```
(`loop=` passes its number straight to `set_param(4, v)` — the node arm maps `!= 0.0` → loop on.)

- [ ] **Step 4: Register in BOTH tables**

`install_methods` (bindings_audio.rs, near the effect factories):
```rust
method("main", "Node", true,  "player_(_)", node_player_impl::<S>);
method("main", "Node", false, "speed=(_)",      node_set_speed_impl::<S>);
method("main", "Node", false, "semitones=(_)",  node_set_semitones_impl::<S>);
method("main", "Node", false, "loopStart=(_)",  node_set_loop_start_impl::<S>);
method("main", "Node", false, "loopEnd=(_)",    node_set_loop_end_impl::<S>);
method("main", "Node", false, "loop=(_)",       node_set_loop_mode_impl::<S>);
```
`METHODS` (bindings.rs):
```rust
static_method("Node", "player_(_)", bindings_audio::node_player),
method("Node", "speed=(_)",     bindings_audio::node_set_speed),
method("Node", "semitones=(_)", bindings_audio::node_set_semitones),
method("Node", "loopStart=(_)", bindings_audio::node_set_loop_start),
method("Node", "loopEnd=(_)",   bindings_audio::node_set_loop_end),
method("Node", "loop=(_)",      bindings_audio::node_set_loop_mode),
```

- [ ] **Step 5: Prelude foreign decls + `class Player`**

`Node` class (prelude.wren):
```
  foreign static player_(buffer)
  foreign speed=(v)
  foreign semitones=(v)
  foreign loopStart=(v)
  foreign loopEnd=(v)
  foreign loop=(v)
```
> **Wren keyword check:** if `loop=(_)` fails to parse (Wren keyword), rename the selector to `looping=(_)` consistently across both tables + here; report the change. (`loop` is believed free but VERIFY on compile.)

`class Player` near `class Comp` (with the `polyMode_` guard; trigger is inherited from the returned Node — no method needed here):
```
class Player {
  static new(buffer) {
    if (Node.polyMode_ == 1) Fiber.abort("Player is a sample source — not usable in a Synth yet (Sa-2)")
    return Node.player_(buffer)
  }
}
```
(`Player.new(buf)` returns a `Node`, so `p.trigger()` uses the existing `Node.trigger()`, and `p.speed = …` uses the new `Node.speed=` foreign.)

- [ ] **Step 6: Extend `Node.trigger` note**

`Node.trigger()` already dispatches to `State::SamplePlayer(p) => p.trigger()` (added in Task 2). Confirm `player.trigger()` works via the existing `foreign trigger()` on `Node` — no new binding.

- [ ] **Step 7: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Gate: compiles + full existing suite green BOTH configs; no "metaclass does not implement" / "Node does not implement player_". Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): Player.new — sample player factory + speed/semitone/loop setters"
```

---

### Task 5: End-to-end

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `SampleBuffer.from` (Task 3), `Player.new` + setters + `trigger` (Task 4); `run_and_render`.

- [ ] **Step 1: Write the failing e2e tests**

The test host provides a real pool (`EngineHost`). NEWLINE-separated statements.

```rust
#[test]
fn sample_player_plays_buffer() {
    let mut out = [StereoFrame::default(); 64];
    // 4-sample buffer, looped so the 64-frame render stays non-silent.
    run_and_render(
        "var b = SampleBuffer.from([0.5, 0.5, -0.5, -0.5])\nvar p = Player.new(b)\np.loop = 1\np.trigger()\nOut.patch(p)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "sample plays (non-silent)");
}

#[test]
fn one_shot_goes_silent_after_length() {
    let mut out = [StereoFrame::default(); 128];
    // tiny 2-sample one-shot buffer: sounds early, silent later.
    run_and_render(
        "var b = SampleBuffer.from([0.8, 0.8])\nvar p = Player.new(b)\np.trigger()\nOut.patch(p)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite()), "finite");
    assert!(out[0..8].iter().any(|f| f.l.abs() > 1e-3), "sounds at the start");
    assert!(out[64..128].iter().all(|f| f.l.abs() < 1e-4), "one-shot silent well after its 2 samples");
}

#[test]
fn player_pitch_and_build() {
    // speed / semitone setters build + render.
    assert!(run_script_ok("var b = SampleBuffer.from([0.3, 0.6, -0.6, -0.3])\nvar p = Player.new(b)\np.speed = 2\np.semitones = 12\np.loop = 1\np.trigger()\nOut.patch(p)"), "pitch setters build/run");
}
```

- [ ] **Step 2: Run (classes live from Tasks 3–4)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- sample_player_plays one_shot_goes player_pitch` (+ `--features deluge-dsp-kernels/simd`).
If `sample_player_plays_buffer` renders silent, the pool upload or the player's `pool_region` read isn't connected (verify `SampleBuffer.from` returned a handle — the test host has a real pool; and that `p.trigger()` set `playing`). If `one_shot_goes_silent_after_length` never goes silent, the one-shot stop isn't firing. Investigate; do NOT weaken. Expected: PASS both.

- [ ] **Step 3: Full regression**

Run the FULL wren-core suite both configs. Existing synths/effects/wavetables byte-unchanged. Expected: PASS. (Known pre-existing `golden_sim` flake — rerun if seen.)

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): sample player e2e — plays buffer, one-shot silences, pitch setters build"
```

---

## Self-Review Notes (for the executor)

- **Hermite exactness is the key oracle:** at integer read positions the Catmull-Rom Hermite returns `y1` (the source sample) exactly, so `speed 1` replays the buffer verbatim — `plays_buffer_verbatim_at_speed_1` proves it. Do not loosen.
- **`rate = speed · 2^(semitones/12)`** via `libm::exp2f`. `speed 2` ≡ `semitones 12`.
- **One-shot vs loop:** one-shot clamps read indices to `[0, len)` and stops (`playing=false`) at end; loop wraps read indices AND `pos` within `[loop_start, loop_end)`. `trigger` restarts (`pos = loop_start` in loop, `0` in one-shot).
- **`Node.trigger` is REUSED** — Task 2 adds the `State::SamplePlayer` arm; `player.trigger()` is the existing `foreign trigger()` on Node. Do NOT add a new trigger selector.
- **`pool_set` ripples across 3 crates:** the `Host` trait gets a default no-op (host.rs), overridden in BOTH embedders (test_support.rs + wren-firmware/src/audio.rs). Missing an embedder override → uploads silently no-op there.
- **Upload has no large scratch:** `alloc_buffer(n)` reserves the region, then per-element `pool_set` writes the Wren list in — the same element-by-element pattern as `wavetable_from_impl`, avoiding a big stack array.
- **`loop=` selector:** believed free; if Wren rejects it as a keyword, rename to `looping=` across both tables + prelude and report it.
- **`Player` is a source, `polyMode_`-guarded** (poly sample voices are Sa-2) — abort inside a Synth block.
- **Purely additive** except the one `Node::trigger` match arm. No existing kernel/node/effect/wavetable behavior changes.
- **Deferred (do NOT implement):** poly/keymap voices (Sa-2), SR-aware root pitch, SD streaming (Sa-3), granular (Sa-4), per-object refcounting, gate-to-stop, reverse.
