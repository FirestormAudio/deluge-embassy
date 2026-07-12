# Sa-2: Poly Sample Voices + Basic Keymap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Play samples as polyphonic voices inside `Synth.new { |p| Sample.new(p, x) * Env.adsr(…) }`, pitched by note relative to a root, with a basic note-zone keymap (`x` = a `SampleBuffer` or a `Keymap`).

**Architecture:** A `PolySamplePlayer` kernel (`[voice; VOICES]` over one shared concatenated PCM region + a zone table, mirroring `PolyWt`); a `Kind::PolySamplePlayer` poly node; a `triggers` fan-out added to the voice allocators; `Keymap.from`/`Sample.new` Wren, threading a `poly_record_trigger` like `poly_record_gate`.

**Tech Stack:** Rust `no_std` (kernels → graph → wren-core), `libm` (`exp2f`/`log2f`/`floorf`/`roundf`), Wren. VOICES=8.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/upload paths. `libm` only (NO std float methods). Bounded loops. `VOICES == 8`. Pool `None` on exhaustion → graceful silence.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** additive — new kernel, new `Kind`, new Wren classes, a `trigger_voice` arm, and an additive `triggers` fan-out in the allocators (**a note_on registering no triggers emits byte-identical Cmds to today**). Sa-1 mono `SamplePlayer`/`Player`/`SampleBuffer` behavior unchanged (the shared-helper refactor in Task 1 must keep Sa-1 oracle tests green). `VoiceAllocator::new`/`MonoAllocator::new` gain params (Task 3) → wren-core won't compile until Task 4 (expected interim).
- **Two registration tables:** new foreigns in BOTH `install_methods` (bindings_audio.rs) AND `METHODS` (bindings.rs) + prelude. Names verified FREE: `class Sample`/`class Keymap`, `Sample.new`/`Keymap.from`, `root=`, `polysampleplayer_`, `node_polysampleplayer`, `Kind::PolySamplePlayer`, `MAX_ZONES`, `MAX_TRIGGERS`, `TAG_KEYMAP=5`.
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

---

### Task 1: `PolySamplePlayer` kernel (+ shared read helper)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/sampler.rs` (factor the Hermite read into a shared `hermite_read`; ADD `Zone`, `MAX_ZONES`, `PolySamplePlayer`)
- Test: `sampler.rs` test module

**Interfaces:**
- Consumes: `crate::In`, `libm`.
- Produces: `pub const MAX_ZONES: usize = 8`; `pub struct Zone { offset: u32, len: u32, low: u8, high: u8, root: u8 }`; `pub struct PolySamplePlayer` (`new`, `set_n_zones`/`set_loop_mode`/`set_zone_field`, `trigger_voice(v)`, `process_voice(&mut self, v: usize, pcm: &[f32], hz: In, dt: f32, out: &mut [f32])`), `#[derive(Clone, Copy)]`. Used by Task 2.

- [ ] **Step 1: Write the failing tests**

```rust
#[test]
fn poly_voice_pitch_is_hz_over_root() {
    // 1 zone, full range, root 60. mtof(60) → rate 1 (verbatim); mtof(72) → rate 2.
    let pcm: [f32; 8] = core::array::from_fn(|i| i as f32);
    let mtof = |n: f32| 440.0 * libm::exp2f((n - 69.0) / 12.0);
    let mut p = PolySamplePlayer::new();
    p.set_n_zones(1);
    p.set_zone_field(0, 0, 0.0);   // offset
    p.set_zone_field(0, 1, 8.0);   // len
    p.set_zone_field(0, 2, 0.0);   // low
    p.set_zone_field(0, 3, 127.0); // high
    p.set_zone_field(0, 4, 60.0);  // root
    // voice 0 at root pitch → rate 1 → verbatim
    p.trigger_voice(0);
    let hz0 = [mtof(60.0); 4];
    let mut o0 = [0.0f32; 4];
    p.process_voice(0, &pcm, In::A(&hz0), 1.0 / 48_000.0, &mut o0);
    assert!((o0[0] - 0.0).abs() < 1e-4 && (o0[1] - 1.0).abs() < 1e-4 && (o0[2] - 2.0).abs() < 1e-4);
    // voice 1 an octave up → rate 2 → reads pcm[0,2,4,6]
    p.trigger_voice(1);
    let hz1 = [mtof(72.0); 4];
    let mut o1 = [0.0f32; 4];
    p.process_voice(1, &pcm, In::A(&hz1), 1.0 / 48_000.0, &mut o1);
    assert!((o1[0] - 0.0).abs() < 1e-4 && (o1[1] - 2.0).abs() < 1e-4 && (o1[2] - 4.0).abs() < 1e-4);
}

#[test]
fn poly_zone_selection_by_note() {
    // zone A: notes 0-59 → pcm region [0,4) content ~10; zone B: 60-127 → [4,8) ~20.
    let pcm: [f32; 8] = [10.0, 10.0, 10.0, 10.0, 20.0, 20.0, 20.0, 20.0];
    let mtof = |n: f32| 440.0 * libm::exp2f((n - 69.0) / 12.0);
    let mut p = PolySamplePlayer::new();
    p.set_n_zones(2);
    // zone 0
    p.set_zone_field(0, 0, 0.0); p.set_zone_field(0, 1, 4.0); p.set_zone_field(0, 2, 0.0); p.set_zone_field(0, 3, 59.0); p.set_zone_field(0, 4, 48.0);
    // zone 1
    p.set_zone_field(1, 0, 4.0); p.set_zone_field(1, 1, 4.0); p.set_zone_field(1, 2, 60.0); p.set_zone_field(1, 3, 127.0); p.set_zone_field(1, 4, 72.0);
    // note 48 (root of zone 0) → plays A content (~10) at rate 1
    p.trigger_voice(0);
    let ha = [mtof(48.0); 4]; let mut oa = [0.0f32; 4];
    p.process_voice(0, &pcm, In::A(&ha), 1.0 / 48_000.0, &mut oa);
    assert!(oa.iter().all(|&v| (v - 10.0).abs() < 1e-3), "note 48 plays zone A");
    // note 72 (root of zone 1) → plays B content (~20)
    p.trigger_voice(1);
    let hb = [mtof(72.0); 4]; let mut ob = [0.0f32; 4];
    p.process_voice(1, &pcm, In::A(&hb), 1.0 / 48_000.0, &mut ob);
    assert!(ob.iter().all(|&v| (v - 20.0).abs() < 1e-3), "note 72 plays zone B");
    // note 100 in no... (covered by zone 1). Test an unmapped note with a 1-zone map elsewhere:
}

#[test]
fn poly_unmapped_note_is_silent() {
    let pcm = [1.0f32; 4];
    let mtof = |n: f32| 440.0 * libm::exp2f((n - 69.0) / 12.0);
    let mut p = PolySamplePlayer::new();
    p.set_n_zones(1);
    p.set_zone_field(0, 0, 0.0); p.set_zone_field(0, 1, 4.0); p.set_zone_field(0, 2, 60.0); p.set_zone_field(0, 3, 60.0); p.set_zone_field(0, 4, 60.0); // only note 60
    p.trigger_voice(0);
    let h = [mtof(48.0); 4]; let mut o = [0.0f32; 4]; // note 48, unmapped
    p.process_voice(0, &pcm, In::A(&h), 1.0 / 48_000.0, &mut o);
    assert!(o.iter().all(|&v| v.abs() < 1e-6), "unmapped note is silent");
}
```

Also keep the existing Sa-1 mono `SamplePlayer` tests (they must still pass after the refactor).

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly_voice poly_zone poly_unmapped`
Expected: FAIL (`PolySamplePlayer` not defined).

- [ ] **Step 3: Factor the shared read helper + implement `PolySamplePlayer`**

First, factor the per-sample read out of the mono `SamplePlayer::process` (the current `read` closure at ~line 69-80) into a free fn — and rewrite mono `process` to call it (mono behavior MUST stay identical; the Sa-1 oracle tests are the proof):

```rust
/// 4-point Hermite read of `pcm` at absolute fractional position `pos`, with
/// taps clamped (one-shot) or wrapped (loop) inside the window `[lo, hi)`.
#[inline]
fn hermite_read(pcm: &[f32], pos: f32, lo: isize, hi: isize, loopable: bool) -> f32 {
    let i = libm::floorf(pos) as isize;
    let frac = pos - (i as f32);
    let tap = |idx: isize| -> f32 {
        let span = hi - lo;
        let n = if loopable && span > 0 {
            let mut k = (idx - lo) % span;
            if k < 0 { k += span; }
            lo + k
        } else {
            idx.clamp(lo, hi - 1)
        };
        pcm[n as usize]
    };
    hermite(tap(i - 1), tap(i), tap(i + 1), tap(i + 2), frac)
}
```

Rewrite mono `SamplePlayer::process`'s inner sample loop to use it (one-shot window `[0, len)`, loop window `[ls, le)`):
```rust
    // inside the `for o in out.iter_mut()` loop, replacing the `read` closure + hermite call:
    let (lo, hi) = if loopable { (ls as isize, le as isize) } else { (0, len as isize) };
    *o = hermite_read(pcm, self.pos, lo, hi, loopable);
```
(Keep the `pos += rate` + stop/wrap logic exactly as-is — only the read is factored.)

Then add the poly types:
```rust
pub const MAX_ZONES: usize = 8;

#[derive(Clone, Copy)]
pub struct Zone { pub offset: u32, pub len: u32, pub low: u8, pub high: u8, pub root: u8 }
impl Zone { const fn empty() -> Zone { Zone { offset: 0, len: 0, low: 0, high: 0, root: 60 } } }

#[derive(Clone, Copy)]
struct SVoice { pos: f32, rate: f32, off: u32, len: u32, playing: bool, latch: bool }
impl SVoice { const fn new() -> SVoice { SVoice { pos: 0.0, rate: 1.0, off: 0, len: 0, playing: false, latch: false } } }

#[derive(Clone, Copy)]
pub struct PolySamplePlayer {
    voices: [SVoice; VOICES],
    zones: [Zone; MAX_ZONES],
    n_zones: usize,
    loop_mode: bool,
}

impl PolySamplePlayer {
    pub fn new() -> PolySamplePlayer {
        PolySamplePlayer { voices: [SVoice::new(); VOICES], zones: [Zone::empty(); MAX_ZONES], n_zones: 0, loop_mode: false }
    }
    pub fn set_n_zones(&mut self, n: f32) { self.n_zones = (n.max(0.0) as usize).min(MAX_ZONES); }
    pub fn set_loop_mode(&mut self, on: bool) { self.loop_mode = on; }
    /// field: 0=offset,1=len,2=low,3=high,4=root
    pub fn set_zone_field(&mut self, zone: usize, field: usize, value: f32) {
        if zone >= MAX_ZONES { return; }
        let z = &mut self.zones[zone];
        match field {
            0 => z.offset = value.max(0.0) as u32,
            1 => z.len = value.max(0.0) as u32,
            2 => z.low = value.clamp(0.0, 127.0) as u8,
            3 => z.high = value.clamp(0.0, 127.0) as u8,
            4 => z.root = value.clamp(0.0, 127.0) as u8,
            _ => {}
        }
    }
    pub fn trigger_voice(&mut self, v: usize) {
        if v < VOICES { self.voices[v] = SVoice { playing: true, latch: true, ..SVoice::new() }; }
    }

    pub fn process_voice(&mut self, v: usize, pcm: &[f32], hz: In, _dt: f32, out: &mut [f32]) {
        if v >= VOICES { return; }
        let loop_mode = self.loop_mode;
        // zone latch on first process after trigger
        if self.voices[v].latch {
            let h = hz.at(0).max(1e-6);
            let note = libm::roundf(69.0 + 12.0 * libm::log2f(h / 440.0));
            let mut found = false;
            for z in 0..self.n_zones {
                let zn = self.zones[z];
                if note >= zn.low as f32 && note <= zn.high as f32 && zn.len > 0 {
                    let root_hz = 440.0 * libm::exp2f((zn.root as f32 - 69.0) / 12.0);
                    self.voices[v].off = zn.offset;
                    self.voices[v].len = zn.len;
                    self.voices[v].rate = h / root_hz;
                    found = true;
                    break;
                }
            }
            if !found { self.voices[v].playing = false; }
            self.voices[v].latch = false;
        }
        let vc = &mut self.voices[v];
        let lo = vc.off as isize;
        let hi = (vc.off + vc.len) as isize;
        for o in out.iter_mut() {
            if !vc.playing || vc.len == 0 || (vc.off as usize + vc.len as usize) > pcm.len() {
                *o = 0.0;
                continue;
            }
            *o = hermite_read(pcm, vc.off as f32 + vc.pos, lo, hi, loop_mode);
            vc.pos += vc.rate;
            let l = vc.len as f32;
            if loop_mode {
                if vc.pos >= l { vc.pos -= l; }
            } else if vc.pos >= l {
                vc.playing = false;
            }
        }
    }
}

impl Default for PolySamplePlayer {
    fn default() -> Self { PolySamplePlayer::new() }
}
```

- [ ] **Step 4: Run to verify pass (both configs) + Sa-1 regression**

Run the new `poly_*` tests + the existing Sa-1 `plays_buffer`/`speed_2`/`one_shot`/`loop_wraps` tests (BOTH must pass — the refactor kept mono behavior): `cargo test -p deluge-dsp-kernels sampler` (+ `--features simd`), then full kernels crate. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/sampler.rs
git commit -m "feat(kernels): PolySamplePlayer — poly sample voices with note-zone keymap + rate=hz/root"
```

---

### Task 2: `Kind::PolySamplePlayer` graph node

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs` (import; `Kind`; `State`; constructor; `out_width`/`is_poly`/`poly_in_count`; `set_param`; `trigger_voice` ~line 337; `poly_process` arm beside `Kind::PolyWt` ~989)
- Test: `node.rs` test module

**Interfaces:**
- Consumes: `PolySamplePlayer`/`MAX_ZONES` (Task 1); the `Kind::PolyWt` poly_process arm (de-interleave/re-interleave + `pool_region`) as the template; `Node::trigger_voice`.
- Produces: `Kind::PolySamplePlayer` (out_width VOICES, is_poly, poly_in_count 1), `State::PolySamplePlayer(PolySamplePlayer)`, `set_param` zone scheme, `trigger_voice` arm. Used by Task 6.

- [ ] **Step 1: Write the failing test**

```rust
#[test]
fn poly_sample_node_wires_and_reads_pool() {
    assert_eq!(Node::out_width(Kind::PolySamplePlayer), VOICES);
    assert!(Node::is_poly(Kind::PolySamplePlayer));
    assert_eq!(Node::poly_in_count(Kind::PolySamplePlayer), 1);
    let mut n = Node::new(Kind::PolySamplePlayer, 0);
    // 1 zone, root 60, offset 0 len 4, full range. set_param scheme: 0=n_zones,1=loop, 2+z*5+f.
    n.set_param(0, 1.0);           // n_zones
    n.set_param(1, 0.0);           // loop off
    n.set_param(2, 0.0);           // zone0 offset
    n.set_param(3, 4.0);           // zone0 len
    n.set_param(4, 0.0);           // zone0 low
    n.set_param(5, 127.0);         // zone0 high
    n.set_param(6, 60.0);          // zone0 root
    n.trigger_voice(0);
    // pool region = PCM; poly_in[0] = an interleaved Hz tile at mtof(60) for lane 0.
    let pcm = [0.5f32, -0.5, 0.5, -0.5];
    let mut region = pcm;
    let hz = 440.0 * libm::exp2f((60.0 - 69.0) / 12.0);
    // one sample block: VOICES-interleaved pitch tile, lane 0 = hz, others 0.
    let mut pitch = [0.0f32; VOICES]; pitch[0] = hz;
    let ins: [In; MAX_INPUTS] = core::array::from_fn(|_| In::zero());
    let mut out = [0.0f32; VOICES]; // out_width VOICES, 1 sample
    n.poly_process(&ins, [Some(&pitch[..]), None], 1.0 / 48_000.0, &mut out, Some(&mut region));
    // lane 0 at root → pcm[0] = 0.5
    assert!((out[0] - 0.5).abs() < 1e-4, "lane 0 plays pool PCM at root pitch, got {}", out[0]);
}
```
> Confirm `In::zero()`/`In::A`/`poly_process`/`MAX_INPUTS`/`VOICES` idioms against the existing `Kind::PolyWt` node test; adapt if the sibling test constructs the interleaved tile differently.

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph poly_sample_node`
Expected: FAIL (`Kind::PolySamplePlayer` not found).

- [ ] **Step 3: Wire `Kind::PolySamplePlayer`** (mirror `Kind::PolyWt`)

1. **Import:** `use deluge_dsp_kernels::sampler::{PolySamplePlayer, MAX_ZONES};` (add to the sampler import from Task-2-of-Sa-1 or a new use).
2. **Kind enum:** add `PolySamplePlayer,`.
3. **State enum:** add `PolySamplePlayer(PolySamplePlayer),`.
4. **Constructor:** `Kind::PolySamplePlayer => State::PolySamplePlayer(PolySamplePlayer::new()),`.
5. **`out_width`:** add `Kind::PolySamplePlayer` to the `=> VOICES` arm (alongside PolyWt/PolyOsc/etc.). **`is_poly`:** add it to the poly `matches!` list. **`poly_in_count`:** add it to the arm returning `1` (alongside PolyWt).
6. **`set_param`** (zone scheme):
```rust
    State::PolySamplePlayer(p) => match param {
        0 => p.set_n_zones(value),
        1 => p.set_loop_mode(value != 0.0),
        _ => {
            let idx = param as usize - 2;
            p.set_zone_field(idx / 5, idx % 5, value);
        }
    },
```
7. **`trigger_voice`** (node.rs:337 — add an arm): `State::PolySamplePlayer(p) => p.trigger_voice(v),`.
8. **`poly_process` arm** (beside `Kind::PolyWt` ~989 — mirror its de-interleave/re-interleave + `pool_region.as_deref()`):
```rust
    Kind::PolySamplePlayer => {
        let pcm: Option<&[f32]> = pool_region.as_deref();
        if let (State::PolySamplePlayer(p), Some(pitch), Some(region)) =
            (&mut self.state, poly_in[0], pcm) {
            let n = out.len() / VOICES;
            let mut col = [0.0f32; MAX_BLOCK];
            let mut ocol = [0.0f32; MAX_BLOCK];
            for v in 0..VOICES {
                for i in 0..n { col[i] = pitch[i * VOICES + v]; }
                p.process_voice(v, region, In::A(&col[..n]), dt, &mut ocol[..n]);
                for i in 0..n { out[i * VOICES + v] = ocol[i]; }
            }
        }
    }
```

- [ ] **Step 4: Run + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph poly_sample_node` (+ `--features deluge-dsp-kernels/simd`), then FULL graph crate both configs. Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): Kind::PolySamplePlayer — poly pooled sample source (VOICES-wide, zone set_param, trigger_voice)"
```

---

### Task 3: Per-lane source trigger in the allocators

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs` (`MAX_TRIGGERS` const; `VoiceAllocator` + `MonoAllocator` structs/`new`/`note_on`; a `trigger_all` helper)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `Cmd::TriggerVoice` (already exists), `NodeId`.
- Produces: `pub const MAX_TRIGGERS: usize = 4`; `VoiceAllocator::new`/`MonoAllocator::new` gain `triggers: [NodeId; MAX_TRIGGERS]` + `n_triggers: usize` params (appended LAST); `note_on` fans `Cmd::TriggerVoice{node, voice: lane}` to each registered trigger. Used by Task 4.

- [ ] **Step 1: Write the failing tests**

```rust
#[test]
fn poly_note_on_fans_trigger_to_registered_sources() {
    // allocator with a sample trigger node = NodeId(50).
    let mut trig = [NodeId(0); MAX_TRIGGERS]; trig[0] = NodeId(50);
    let mut a = mk_poly_with_triggers(trig, 1); // helper: mk_poly + triggers
    let c = on(&mut a, 69, 100);
    // one TriggerVoice to NodeId(50) on the allocated lane.
    let trigs: std::vec::Vec<u8> = c.iter().filter_map(|cmd| match cmd {
        Cmd::TriggerVoice { node: NodeId(50), voice } => Some(*voice), _ => None }).collect();
    assert_eq!(trigs.len(), 1, "fans one TriggerVoice to the registered source");
}

#[test]
fn poly_no_triggers_is_byte_identical() {
    // no registered triggers → note_on emits NO TriggerVoice (unchanged Cmd stream).
    let mut a = mk_poly(); // existing helper, 0 triggers
    let c = on(&mut a, 69, 100);
    assert_eq!(c.iter().filter(|cmd| matches!(cmd, Cmd::TriggerVoice { .. })).count(), 0);
}
```
Add an `mk_poly_with_triggers(triggers, n)` helper mirroring `mk_poly` but passing the trigger list to `VoiceAllocator::new`.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- poly_note_on_fans poly_no_triggers`
Expected: FAIL (`new` takes no triggers).

- [ ] **Step 3: Add `triggers` to both allocators**

`MAX_TRIGGERS` const near `MAX_GATES` (voice.rs:16): `pub const MAX_TRIGGERS: usize = 4;` (re-export it in lib.rs beside `MAX_GATES`).

`VoiceAllocator` struct: add `triggers: [NodeId; MAX_TRIGGERS]` + `n_triggers: usize`. `VoiceAllocator::new`: append `triggers: [NodeId; MAX_TRIGGERS], n_triggers: usize` params (LAST), init the fields. Add:
```rust
    fn trigger_all(&self, lane: usize, emit: &mut impl FnMut(Cmd)) {
        for t in &self.triggers[..self.n_triggers] {
            emit(Cmd::TriggerVoice { node: *t, voice: lane as u8 });
        }
    }
```
In `note_on`, right after `self.gate_all(lane, true, emit);` (voice.rs:176): `self.trigger_all(lane, emit);`.

`MonoAllocator` struct/`new`: same `triggers`/`n_triggers` (appended LAST to `new`). Add the same `trigger_all`. In `note_on`, inside the `if from_silence` block, right after `self.gate_all(u, true, emit);` (voice.rs:317): `self.trigger_all(u, emit);` (a mono sampler retriggers from silence, alongside the slew snap + gates).

- [ ] **Step 4: Fix the graph crate's own test call sites + run**

`VoiceAllocator::new`/`MonoAllocator::new` arity changed → update every `voice.rs` test constructor with an empty trigger list: `[NodeId(0); MAX_TRIGGERS], 0` appended (an empty-trigger allocator is byte-identical to today). No assertion changes.
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- poly_note_on_fans poly_no_triggers` (+ simd), then FULL graph crate both configs (existing allocator/unison/width tests green with only the constructor-arg edits). Expected: PASS.
> `deluge-wren-core` will NOT compile until Task 4 (the allocator `new` arity) — expected interim.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs crates/deluge-audio-graph/src/lib.rs
git commit -m "feat(graph): allocator per-lane source triggers — note_on fans TriggerVoice to registered sources"
```

---

### Task 4: `PolyCtx.triggers` + thread to the allocators (un-break wren-core)

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (`PolyCtx` triggers + `poly_record_trigger` + reset + `poly_end`/`mono_end` tuples)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_poly_end_impl`/`node_mono_end_impl` pass triggers to the allocator `new`)

**Interfaces:**
- Consumes: `MAX_TRIGGERS` (Task 3); the allocator `new` new params.
- Produces: `audio::poly_record_trigger(id)`; extended `poly_end`/`mono_end` returns. Used by Task 6.

- [ ] **Step 1: Extend `PolyCtx`** (mirror `gates`)

`audio.rs` `PolyCtx` struct: add `triggers: [u16; MAX_TRIGGERS]` + `trigger_count: u8`. `PolyCtx::new`: init `triggers: [NULL_ID; MAX_TRIGGERS]`, `trigger_count: 0`. Import `MAX_TRIGGERS` (beside `MAX_GATES`). Reset both in `poly_begin` AND `mono_begin` (beside the `gates`/`gate_count` reset). Add:
```rust
pub fn poly_record_trigger(id: u16) {
    let p = poly();
    if (p.trigger_count as usize) < MAX_TRIGGERS { p.triggers[p.trigger_count as usize] = id; }
    p.trigger_count = p.trigger_count.saturating_add(1);
}
```
Extend the returns:
```rust
pub fn poly_end() -> (u16, [u16; MAX_GATES], u8, u16, [u16; MAX_TRIGGERS], u8) {
    let p = poly(); p.mode = false;
    (p.pitch_ctrl, p.gates, p.gate_count, p.vel_node, p.triggers, p.trigger_count)
}
pub fn mono_end() -> (u16, u16, [u16; MAX_GATES], u8, u16, [u16; MAX_TRIGGERS], u8) {
    let p = poly(); p.mode = false;
    (p.pitch_ctrl, p.slew_node, p.gates, p.gate_count, p.vel_node, p.triggers, p.trigger_count)
}
```

- [ ] **Step 2: Pass triggers into the allocators**

`node_poly_end_impl` (bindings_audio.rs:1849): read the extended tuple, build the trigger array, pass to `VoiceAllocator::new`:
```rust
    let (pitch_ctrl, gates_raw, gate_count, vel_raw, trig_raw, trig_count) = audio::poly_end();
    // … gates/vel/sum as before …
    let triggers: [NodeId; deluge_audio_graph::MAX_TRIGGERS] = core::array::from_fn(|i| NodeId(trig_raw[i]));
    let n_triggers = (trig_count as usize).min(deluge_audio_graph::MAX_TRIGGERS);
    let alloc = SynthAlloc::Poly(deluge_audio_graph::VoiceAllocator::new(NodeId(pitch_ctrl), gates, n_gates, vel, NodeId(sum), triggers, n_triggers));
```
`node_mono_end_impl` (bindings_audio.rs:1880): same, reading the mono tuple and passing `triggers, n_triggers` last to `MonoAllocator::new`. Re-export `MAX_TRIGGERS` from `deluge_audio_graph` if needed.

- [ ] **Step 3: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Gate: wren-core compiles again + full existing suite green BOTH configs (a synth with no sample source registers no triggers → byte-identical). Expected: PASS both.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs
git commit -m "feat(wren): PolyCtx triggers — thread poly_record_trigger through to the voice allocators"
```

---

### Task 5: `Keymap.from` — concatenated multi-sample upload

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`KeymapObj` + `keymap_from_impl` + register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign class Keymap`)

**Interfaces:**
- Consumes: `audio::alloc_buffer`/`audio::pool_set` (Sa-1), `get_list_count`/`get_list_element`; the `SampleObj`/`sample_from_impl` pattern.
- Produces: `KeymapObj { tag: TAG_KEYMAP=5, handle, zones: [(u32,u32,u8,u8,u8); MAX_ZONES], n_zones }`; Wren `Keymap.from([[samples], low, high, root], …)`. Used by Task 6.

- [ ] **Step 1: `KeymapObj` + `keymap_from_impl`**

`TAG_KEYMAP: u8 = 5` (next after `TAG_SAMPLE=4`). `KeymapObj` foreign (mirror `SampleObj`), `class_name() = "Keymap"`, holding the zone table (offset/len/low/high/root per zone) + n_zones + the concatenated `handle`.

`keymap_from_impl<S>(vm)`: the args are a Wren list of zones, each zone a 4-element list `[samplesList, low, high, root]`.
1. First pass: sum the lengths of all zones' `samplesList` (via `get_list_count` on each nested samples list) → `total`.
2. `let handle = audio::alloc_buffer(total)`.
3. Second pass: for each zone z, record `offset` (running), copy its samples via `pool_set(handle, offset + i, value)` (element-by-element, nested-list reads — `get_list_element` to descend), record `(offset, len, low, high, root)` in the zone table, advance offset.
4. `new_foreign_in::<KeymapObj>(0, KeymapObj { tag: TAG_KEYMAP, handle, zones, n_zones })`.
> The nested-list read pattern (list of zones, each a list) uses `get_list_element(zonesSlot, z, elemSlot)` then `get_list_element(elemSlot=zone, 0, samplesSlot)` etc. Confirm the `SlotApi` supports nested `get_list_element` (the wavetable `from2d` reads a 2D nested list — mirror `wavetable_from2d_impl`'s nesting exactly). Cap `n_zones` at `MAX_ZONES`.

- [ ] **Step 2: Register `Keymap.from` in BOTH tables + prelude**

- `install_methods`: `method("main", "Keymap", true, "from(_)", keymap_from_impl::<S>);`
- `METHODS`: `static_method("Keymap", "from(_)", bindings_audio::keymap_from),`
- prelude: `foreign class Keymap { foreign static from(zones) }`.

- [ ] **Step 3: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`). Gate: compiles + existing suite green (nothing exercises Keymap yet — Task 6/7). Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): Keymap.from — concatenated multi-sample pool upload + zone table"
```

---

### Task 6: `Sample.new` — poly sample factory + routing

**Files:**
- Modify: `crates/deluge-wren-core/src/audio.rs` (`new_poly_sample_player` helper)
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_polysampleplayer_impl` factory + `root=` setter + register)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (Node foreign + `class Sample`)

**Interfaces:**
- Consumes: `SampleObj` (Sa-1) + `KeymapObj` (Task 5); `Kind::PolySamplePlayer` set_param zone scheme (Task 2); `audio::poly_record_trigger` (Task 4); `return_poly_node`; the `node_polywt_pooled_impl`/`new_polywt_pooled` template.
- Produces: Wren `Sample.new(pitch, x)` (x = SampleBuffer or Keymap) + `root=`.

- [ ] **Step 1: `new_poly_sample_player` create helper** (mirror `new_polywt_pooled`, audio.rs:277)

```rust
/// Create a Kind::PolySamplePlayer bound to `handle`, wired to the per-voice
/// `pitch` (PolyMtof Hz) tile, with the zone table emitted as SetParams.
pub fn new_poly_sample_player(id: u16, handle: Option<deluge_audio_graph::PoolHandle>, pitch: Input, zones: &[(u32, u32, u8, u8, u8)], loop_mode: bool) {
    if id == NULL_ID { return; }
    host().audio_cmd(Cmd::NewNode { node: NodeId(id), kind: Kind::PolySamplePlayer, args: [pitch, Input::Const(0.0), Input::Const(0.0)] });
    if let Some(h) = handle {
        host().audio_cmd(Cmd::BindTable { node: NodeId(id), src: deluge_audio_graph::node::TableSrc::Pooled(h) });
        let sp = |param: u8, value: f32| host().audio_cmd(Cmd::SetParam { node: NodeId(id), param, value });
        sp(0, zones.len() as f32);           // n_zones
        sp(1, if loop_mode { 1.0 } else { 0.0 });
        for (z, &(off, len, lo, hi, root)) in zones.iter().enumerate() {
            let base = 2 + (z as u8) * 5;
            sp(base, off as f32); sp(base + 1, len as f32); sp(base + 2, lo as f32); sp(base + 3, hi as f32); sp(base + 4, root as f32);
        }
    }
}
```

- [ ] **Step 2: `node_polysampleplayer_impl` factory** (mirror `node_polywt_pooled_impl`)

```rust
pub(crate) fn node_polysampleplayer_impl<S: SlotApi>(vm: &S) {
    let pitch = arg_input(vm, 1);
    // arg 2 is a SampleBuffer OR a Keymap foreign — dispatch by tag.
    let (handle, zones): (Option<PoolHandle>, [(u32,u32,u8,u8,u8); MAX_ZONES], usize) = /* read foreign at slot 2:
        if SampleObj → (handle, [(0, len, 0, 127, 60)], 1);
        if KeymapObj → (handle, its zone table, n_zones) */;
    let id = audio::alloc_node_id();
    audio::new_poly_sample_player(id, handle, pitch, &zones[..n_zones], /*loop_mode*/ false);
    audio::poly_record_trigger(id);
    unsafe { return_poly_node(vm, id) };
}
```
> Dispatch by the foreign's `tag` byte at slot 2 (`TAG_SAMPLE` vs `TAG_KEYMAP`) — read the tag via a small helper (the foreigns are `#[repr(C)]` with `tag` first, like `NodeObj`/`WtObj`). For a `SampleObj`, synthesize a single full-range zone `(0, len, 0, 127, default_root)`. For `root=` support on the single-sample form, store a mutable default root; simplest: `Sample.new` sets root 60 and the `root=` setter re-emits `SetParam(2 + 0*5 + 4, root)` (zone 0's root). Add `node_set_root_impl` → `set_param(6, v)` (zone 0 root index = 2+0*5+4 = 6). Paired shims for the factory + setter.

- [ ] **Step 3: Register + prelude**

- `install_methods`: `method("main","Node",true,"polysampleplayer_(_,_)", node_polysampleplayer_impl::<S>)` + `method("main","Node",false,"root=(_)", node_set_root_impl::<S>)`.
- `METHODS`: `static_method("Node","polysampleplayer_(_,_)", bindings_audio::node_polysampleplayer)` + `method("Node","root=(_)", bindings_audio::node_set_root)`.
- prelude `Node`: `foreign static polysampleplayer_(pitch, source)` + `foreign root=(v)`.
- prelude `class Sample` (requires poly context — abort outside a Synth):
```
class Sample {
  static new(pitch, source) {
    if (Node.polyMode_ != 1) Fiber.abort("Sample.new is a poly voice source — use it inside Synth.new/Synth.mono (top-level one-shot playback is Player)")
    return Node.polysampleplayer_(pitch, source)
  }
}
```

- [ ] **Step 4: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`). Gate: compiles + existing suite green; no "metaclass does not implement". Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/audio.rs crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): Sample.new — poly sample voice source (SampleBuffer|Keymap), root setter"
```

---

### Task 7: End-to-end

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `SampleBuffer.from` (Sa-1), `Keymap.from` (Task 5), `Sample.new` (Task 6); `Synth.new`/`Synth.mono`/`noteOn`; `run_and_render` (real-pool test host).

- [ ] **Step 1: Write the failing e2e tests**

```rust
#[test]
fn poly_sample_voice_plays_in_synth() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var b = SampleBuffer.from([0.6, 0.6, -0.6, -0.6])\nvar s = Synth.new { |p| Sample.new(p, b) * Env.adsr(0.001, 0.5, 1, 0.2) }\nOut.patch(s.out)\ns.noteOn(60, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "sample voice sounds");
}

#[test]
fn poly_sample_two_notes_two_voices() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var b = SampleBuffer.from([0.5, 0.5, -0.5, -0.5])\nvar s = Synth.new { |p| Sample.new(p, b) * Env.adsr(0.001, 0.5, 1, 0.2) }\nOut.patch(s.out)\ns.noteOn(60, 100)\ns.noteOn(64, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite()) && out.iter().any(|f| f.l.abs() > 1e-3), "two poly sample voices sound");
}

#[test]
fn keymap_and_scope_guard() {
    // A 2-zone keymap builds + renders in a Synth.
    assert!(run_script_ok("var k = Keymap.from([[0.5,0.5,-0.5,-0.5], 0, 59, 48], [[0.3,0.3,-0.3,-0.3], 60, 127, 72])\nvar s = Synth.new { |p| Sample.new(p, k) * Env.adsr(0.001,0.5,1,0.2) }\nOut.patch(s.out)\ns.noteOn(64,100)\ns.noteOn(72,100)"), "keymap synth builds/runs");
    // Sample.new OUTSIDE a Synth aborts.
    assert!(!run_script_ok("var b = SampleBuffer.from([0.5])\nOut.patch(Sample.new(Osc.saw(110), b))"), "Sample.new aborts outside a Synth");
}
```

- [ ] **Step 2: Run (surface live from Tasks 1–6)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- poly_sample_voice poly_sample_two keymap_and_scope` (+ `--features deluge-dsp-kernels/simd`).
If `poly_sample_voice_plays_in_synth` renders silent, the chain is broken: verify the sample buffer got a real handle (test host has a pool), the `Sample.new` registered a trigger (`poly_record_trigger`), the allocator fanned `TriggerVoice` on noteOn, and the node read the pool region at the note's rate. Investigate; do NOT weaken. Expected: PASS both.

- [ ] **Step 3: Full regression**

Run the FULL wren-core suite both configs. Existing synths/effects/Sa-1 unchanged. Expected: PASS. (Known pre-existing `golden_sim` flake — rerun if seen.)

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): poly sample voice e2e — plays in Synth, poly, keymap zones, scope guard"
```

---

## Self-Review Notes (for the executor)

- **The pitch model:** the poly sample source consumes the SAME Hz tile oscillators do (`PolyMtof`, `poly_in[0]`); per voice `rate = hz / mtof(root)`, and the note for zone selection is `round(69 + 12·log2(hz/440))`. No PolyCtrl-direct wiring.
- **Zone latch on trigger:** `trigger_voice` sets `latch`; the FIRST process reads the pitch, picks the zone, latches `(offset,len,rate)`. So a voice plays ONE zone for its note's duration.
- **Keymap = concatenation:** all zone PCM in ONE pool region + a zone table with offsets; a single `SampleBuffer` = a 1-zone full-range map. Keeps the one-bound-region model (no engine change).
- **Allocator triggers are additive:** `note_on` fans `TriggerVoice` only to registered triggers; a synth with none is byte-identical (the `poly_no_triggers_is_byte_identical` test proves it). Note-**off** does NOT stop the sample — the amp env releases it.
- **The `VoiceAllocator`/`MonoAllocator::new` arity change** (Task 3) breaks wren-core until Task 4 threads the trigger list through — expected interim; the graph crate stays green via its updated test constructors.
- **Refactor safety (Task 1):** factoring the mono `read` into `hermite_read` must keep Sa-1's mono oracle tests green (behavior identical).
- **Confirm exact idioms** before transcribing: the `Kind::PolyWt` poly_process arm (de-interleave/re-interleave, `MAX_BLOCK` scratch), the foreign `tag`-dispatch for SampleBuffer-vs-Keymap at slot 2, the nested `get_list_element` for `Keymap.from` (mirror `wavetable_from2d_impl`), and `return_poly_node`.
- **Deferred (do NOT implement):** velocity zones/layers, SR-aware root, per-zone loop points, reverse, gate-to-stop, SD streaming (Sa-3), granular (Sa-4).
