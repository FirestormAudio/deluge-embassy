# Sy-6a: Stereo Spread Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `synth.width = amount` (0..1) fans a note's U unison voices symmetrically across the stereo field (poly + mono); `width == 0` is byte-identical to today.

**Architecture:** A new `StereoVoiceSum` poly node (out_width==2, port0=L/port1=R) replaces the mono `VoiceSum` in the Synth build path, summing the VOICES lanes into L/R with a per-lane unity-center balance pan and the existing 1/√U gain. Both allocators emit a per-lane pan `SetParam` (only when width≠0) alongside the detune emit. The graph is already stereo below the sum, so a width-2 node routes to the stereo bus with no plumbing change.

**Tech Stack:** Rust `no_std` (3 crates: `deluge-dsp-kernels` → `deluge-audio-graph` → `deluge-wren-core`), Wren scripting, voice-interleaved SIMD (`f32x8` behind `simd`).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** `width == 0` is byte-identical to today (poly AND mono) in BOTH the emitted Cmd stream and the rendered audio. `StereoVoiceSum` default gain 1.0, default pan 0.0 (center) on every lane ⇒ `L = R = gain·Σ lane` (= mono `voice_sum`). The per-lane pan `SetParam` is emitted **only when `width_amount != 0.0`**, so a synth that never calls `width` emits the exact same Cmd stream as today — every existing allocator test passes unchanged. The unison `1/√U` gain (Sy-5e, `set_param(0)`) is on the same slot and keeps working.
- **Two registration tables:** the new `width=(_)` foreign setter goes in BOTH `register_audio` (`bindings_audio.rs`) AND the `wren-sys-backend` `METHODS` table (`bindings.rs`), plus a `foreign` decl in the prelude `Synth` class.
- **Test invocation (per-crate, never `--workspace`; both configs; cargo rejects multiple bare positional names — use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

---

### Task 1: `StereoVoiceSum` kernel + graph node

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (add `voice_sum_stereo` after `voice_sum` at ~line 251-259)
- Modify: `crates/deluge-audio-graph/src/node.rs` (Kind enum ~85, State enum ~153, constructor ~199, `out_width` ~244-254, `is_poly` ~258-263, `poly_in_count` ~267-270, `set_param` ~329, `poly_process` ~811-815)
- Test: kernel tests in `poly.rs`, node tests in `node.rs`

**Interfaces:**
- Consumes: `VOICES` (from `deluge_dsp_kernels::poly`, already imported in both files); `voice_sum` (the mono oracle, poly.rs:251).
- Produces: `pub fn voice_sum_stereo(tile: &[f32], out_l: &mut [f32], out_r: &mut [f32], gain: f32, pan: &[f32; VOICES])`; `Kind::StereoVoiceSum` (out_width 2, is_poly, poly_in_count 1); `State::StereoVoiceSum { gain: f32, pan: [f32; VOICES] }`, `set_param(0)=gain`, `set_param(1..=VOICES)=pan[param-1]`. Used by Tasks 2–4.

- [ ] **Step 1: Write the failing kernel test**

In `crates/deluge-dsp-kernels/src/poly.rs` test module (near the existing `voice_sum` tests):

```rust
#[test]
fn voice_sum_stereo_center_is_bit_identical_mono_and_pans() {
    // Build a 2-sample, VOICES-lane tile with distinct per-lane values.
    let mut tile = [0.0f32; VOICES * 2];
    for i in 0..2 {
        for v in 0..VOICES {
            tile[i * VOICES + v] = (i * VOICES + v) as f32 + 1.0;
        }
    }
    // (a) all-center pan ⇒ L == R == plain mono voice_sum (bit-identical).
    let center = [0.0f32; VOICES];
    let (mut l, mut r) = ([0.0f32; 2], [0.0f32; 2]);
    voice_sum_stereo(&tile, &mut l, &mut r, 1.0, &center);
    let mut mono = [0.0f32; 2];
    voice_sum(&tile, &mut mono, 1.0);
    assert_eq!(l, mono, "center L == mono sum");
    assert_eq!(r, mono, "center R == mono sum");
    assert_eq!(l, r, "center L == R");
    // (b) lane 0 panned hard right (+1) ⇒ contributes only to R (gl=0, gr=1).
    let mut pan = [0.0f32; VOICES];
    pan[0] = 1.0;
    let (mut l2, mut r2) = ([0.0f32; 2], [0.0f32; 2]);
    voice_sum_stereo(&tile, &mut l2, &mut r2, 1.0, &pan);
    // L2 = sum of lanes 1..VOICES (lane 0 dropped); R2 = full sum.
    let drop0: f32 = (1..VOICES).map(|v| tile[v]).sum();
    assert!((l2[0] - drop0).abs() < 1e-6, "hard-right lane 0 absent from L");
    assert!((r2[0] - mono[0]).abs() < 1e-6, "hard-right lane 0 present in R");
    // (c) gain scales both rows.
    let (mut lg, mut rg) = ([0.0f32; 2], [0.0f32; 2]);
    voice_sum_stereo(&tile, &mut lg, &mut rg, 0.5, &center);
    assert!((lg[0] - 0.5 * mono[0]).abs() < 1e-6 && (rg[0] - 0.5 * mono[0]).abs() < 1e-6);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels voice_sum_stereo`
Expected: FAIL (`voice_sum_stereo` not found).

- [ ] **Step 3: Implement `voice_sum_stereo`**

In `crates/deluge-dsp-kernels/src/poly.rs`, immediately after `voice_sum` (after line 259):

```rust
/// Collapse the `VOICES`-lane tile into a stereo pair, applying a per-lane
/// unity-center balance pan and an overall `gain` (the 1/√U unison
/// normalization). `pan[v]` ∈ [-1, 1]: -1 = hard left, 0 = center, +1 = hard
/// right. The unity-center law (`gl = clamp(1-p, 0, 1)`, `gr = clamp(1+p, 0, 1)`)
/// keeps pan 0 at gains (1, 1), so an all-center sum is bit-identical to
/// `voice_sum`. `tile.len() == VOICES * out_l.len()`, `out_l.len() == out_r.len()`.
pub fn voice_sum_stereo(
    tile: &[f32],
    out_l: &mut [f32],
    out_r: &mut [f32],
    gain: f32,
    pan: &[f32; VOICES],
) {
    // Per-lane L/R gains, computed once (pan is per-note, block-constant).
    let mut gl = [0.0f32; VOICES];
    let mut gr = [0.0f32; VOICES];
    for v in 0..VOICES {
        gl[v] = (1.0 - pan[v]).clamp(0.0, 1.0);
        gr[v] = (1.0 + pan[v]).clamp(0.0, 1.0);
    }
    for i in 0..out_l.len() {
        let mut sl = 0.0f32;
        let mut sr = 0.0f32;
        for v in 0..VOICES {
            let x = tile[i * VOICES + v];
            sl += gl[v] * x;
            sr += gr[v] * x;
        }
        out_l[i] = gain * sl;
        out_r[i] = gain * sr;
    }
}
```

(Scalar only — this is the correctness oracle. A `#[cfg(feature="simd")]` f32x8 path is NOT required for Sy-6a; if added later it must null-test lane-for-lane against this scalar. Do not add SIMD in this task.)

- [ ] **Step 4: Run to verify the kernel test passes**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels voice_sum_stereo` (then again with `--features simd`)
Expected: PASS both.

- [ ] **Step 5: Write the failing graph-node test**

In `crates/deluge-audio-graph/src/node.rs` test module. Mirror the existing `VoiceSum`-node poly_process test (the one near node.rs:1643 that drives a PolyCtrl tile → VoiceSum). Drive a StereoVoiceSum with a known VOICES-lane input tile and assert the two output rows:

```rust
#[test]
fn stereovoicesum_node_center_and_pan_and_gain() {
    assert_eq!(Node::out_width(Kind::StereoVoiceSum), 2);
    assert!(Node::is_poly(Kind::StereoVoiceSum));
    assert_eq!(Node::poly_in_count(Kind::StereoVoiceSum), 1);

    // A VOICES-lane tile, one sample: lane v = v+1.
    let mut tile = [0.0f32; VOICES * BLOCK];
    for v in 0..VOICES { tile[v] = (v + 1) as f32; }
    let sum: f32 = (1..=VOICES).map(|v| v as f32).sum();

    // out spans 2 rows (2*BLOCK); poly_process splits it L | R.
    let mut node = Node::new(Kind::StereoVoiceSum, 0);
    let ins: [In; MAX_INPUTS] = core::array::from_fn(|_| In::zero());
    let mut out = [0.0f32; 2 * BLOCK];
    // default (center, gain 1.0): L[0] == R[0] == sum.
    node.poly_process(&ins, [Some(&tile[..]), None], 0.0, &mut out, None);
    assert!((out[0] - sum).abs() < 1e-6, "L center == sum");
    assert!((out[BLOCK] - sum).abs() < 1e-6, "R center == sum");

    // set_param(0)=gain 0.5 halves both.
    node.set_param(0, 0.5);
    let mut out2 = [0.0f32; 2 * BLOCK];
    node.poly_process(&ins, [Some(&tile[..]), None], 0.0, &mut out2, None);
    assert!((out2[0] - 0.5 * sum).abs() < 1e-6 && (out2[BLOCK] - 0.5 * sum).abs() < 1e-6);

    // set_param(1)=pan lane 0 hard right (+1): lane 0 drops from L, stays in R.
    node.set_param(0, 1.0); // gain back to 1.0
    node.set_param(1, 1.0); // pan[0] = +1
    let mut out3 = [0.0f32; 2 * BLOCK];
    node.poly_process(&ins, [Some(&tile[..]), None], 0.0, &mut out3, None);
    let drop0: f32 = (2..=VOICES).map(|v| v as f32).sum(); // lanes 1..VOICES
    assert!((out3[0] - drop0).abs() < 1e-6, "lane0 absent from L");
    assert!((out3[BLOCK] - sum).abs() < 1e-6, "lane0 present in R");
}
```

> Confirm the exact names of the test-scaffolding items already used by neighboring node tests: `BLOCK`, `MAX_INPUTS`, `In`, `In::zero()` (or however a zero `In` is built — mirror the existing VoiceSum node test's construction). If `In::zero()` is not the idiom, copy whatever the sibling test uses to build an `[In; MAX_INPUTS]`.

- [ ] **Step 6: Run to verify it fails**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph stereovoicesum_node`
Expected: FAIL (`Kind::StereoVoiceSum` not found).

- [ ] **Step 7: Wire `Kind::StereoVoiceSum` into the graph**

In `crates/deluge-audio-graph/src/node.rs`:

1. **Kind enum** (after `VoiceSum,` at line 85):
```rust
    /// Stereo voice sum: VOICES lanes → L/R (port0=L, port1=R) with per-lane
    /// unity-center pan + 1/√U gain. The Synth's stereo-spread collapse node.
    StereoVoiceSum,
```

2. **State enum** (after `VoiceSum(f32),` at line 153):
```rust
    StereoVoiceSum { gain: f32, pan: [f32; VOICES] },
```

3. **Constructor** (after `Kind::VoiceSum => State::VoiceSum(1.0),` at line 199):
```rust
    Kind::StereoVoiceSum => State::StereoVoiceSum { gain: 1.0, pan: [0.0; VOICES] },
```

4. **`out_width`** (line 246 — add to the `=> 2` arm alongside `Split2 | Pan | …`):
```rust
    Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger | Kind::Room | Kind::Hall | Kind::Plate | Kind::StereoVoiceSum => 2,
```

5. **`is_poly`** (line 259 — add `| Kind::StereoVoiceSum` to the `matches!` list that already contains `Kind::VoiceSum`).

6. **`poly_in_count`** (line 269 — add `| Kind::StereoVoiceSum` to the arm that already contains `Kind::VoiceSum` and returns 1).

7. **`set_param`** (after `State::VoiceSum(g) if param == 0 => *g = value,` at line 329):
```rust
    State::StereoVoiceSum { gain, .. } if param == 0 => *gain = value,
    State::StereoVoiceSum { pan, .. } if (1..=VOICES).contains(&(param as usize)) => {
        pan[param as usize - 1] = value;
    }
```

8. **`poly_process`** (after the `Kind::VoiceSum` arm at node.rs:811-815):
```rust
    Kind::StereoVoiceSum => {
        if let (State::StereoVoiceSum { gain, pan }, Some(pin)) = (&self.state, poly_in[0]) {
            let half = out.len() / 2;                 // 2*BLOCK ⇒ BLOCK
            let (l, r) = out.split_at_mut(half);      // l = row base (port0=L), r = row base+1 (port1=R)
            voice_sum_stereo(pin, l, r, *gain, pan);
        }
    }
```
Import `voice_sum_stereo` alongside the existing `voice_sum` import at the top of node.rs.

> `set_param`'s `param` is a `u8`; guard the pan arm with `(1..=VOICES).contains(&(param as usize))` so params 1..=8 map to `pan[0..8]` and any out-of-range param is a no-op (matches the existing `set_param` convention). Gain lives on param 0 — the SAME slot `VoiceSum` uses, so Sy-5e's `set_param(out_node, 0, 1/√U)` keeps working when the sum node becomes `StereoVoiceSum`.

- [ ] **Step 8: Run to verify graph tests pass + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph stereovoicesum_node` (+ `--features deluge-dsp-kernels/simd`)
Then FULL graph crate both configs (existing VoiceSum + voice-render tests must stay green — `VoiceSum` is untouched): `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` (with and without `--features deluge-dsp-kernels/simd`).
Expected: PASS all.

- [ ] **Step 9: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs crates/deluge-audio-graph/src/node.rs
git commit -m "feat(kernels+graph): StereoVoiceSum — VOICES→L/R with per-lane unity-center pan"
```

---

### Task 2: `width_offset` helper + poly `VoiceAllocator` width

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs` (`width_offset` free fn near `unison_offset`; `VoiceAllocator` struct/`new`/setters/`note_on`)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `unison_offset` (Sy-5e, the sibling helper), `Cmd::SetParam`, `NodeId`, `VOICES`, `A440_NOTE`, `pick_lane`/`LaneState`, `gate_all`.
- Produces: `fn width_offset(u: usize, count: usize, amount: f32) -> f32`; `VoiceAllocator` gains `sum_node: NodeId` + `width_amount: f32` + `set_width(&mut self, amount: f32)`; `note_on` emits per-lane pan `SetParam(sum_node, lane+1, …)` when `width_amount != 0.0`. **`VoiceAllocator::new` gains a `sum_node` parameter** (used by Task 4).

> **Cross-task note (by design):** adding a `sum_node` param to `VoiceAllocator::new` breaks the wren-core call sites until Task 4 updates them — an interim non-compiling workspace between per-crate tasks, expected in this project. The `deluge-audio-graph` crate itself compiles and tests green after this task (its own test helpers pass a `sum_node`).

- [ ] **Step 1: Write the failing tests**

In `crates/deluge-audio-graph/src/voice.rs` test module:

```rust
#[test]
fn width_offset_spreads_symmetric_and_scaled() {
    assert_eq!(width_offset(0, 1, 1.0), 0.0); // single voice = center
    // U=2 @ amount=1 → -1, +1
    assert!((width_offset(0, 2, 1.0) - (-1.0)).abs() < 1e-6);
    assert!((width_offset(1, 2, 1.0) - 1.0).abs() < 1e-6);
    // U=3 @ amount=0.5 → -0.5, 0, +0.5
    assert!((width_offset(0, 3, 0.5) - (-0.5)).abs() < 1e-6);
    assert!(width_offset(1, 3, 0.5).abs() < 1e-6);
    assert!((width_offset(2, 3, 0.5) - 0.5).abs() < 1e-6);
}

#[test]
fn poly_width_emits_per_lane_pan_on_sum_node() {
    let mut a = mk_poly(); // pitch=NodeId(10), gate=NodeId(20), sum=NodeId(30), no vel
    a.set_unison(3);
    a.set_width(1.0);
    let c = on(&mut a, 69, 100);
    // 3 pan SetParams on the sum node (NodeId(30)), params lane+1, 3 symmetric values.
    let pans: std::vec::Vec<(u8, f32)> = c.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(30), param, value } => Some((*param, *value)),
        _ => None,
    }).collect();
    assert_eq!(pans.len(), 3, "3 pan SetParams on sum node");
    // params are lane+1 (>=1), distinct
    assert!(pans.iter().all(|(p, _)| *p >= 1));
    let mut params: std::vec::Vec<u8> = pans.iter().map(|(p, _)| *p).collect();
    params.sort(); params.dedup();
    assert_eq!(params.len(), 3, "3 distinct lane params");
    // values symmetric around 0: sorted ≈ -1, 0, +1
    let mut vals: std::vec::Vec<f32> = pans.iter().map(|(_, v)| *v).collect();
    vals.sort_by(|x, y| x.partial_cmp(y).unwrap());
    assert!((vals[0] - (-1.0)).abs() < 1e-6 && vals[1].abs() < 1e-6 && (vals[2] - 1.0).abs() < 1e-6);
}

#[test]
fn poly_width_zero_emits_no_pan() {
    let mut a = mk_poly();
    a.set_unison(3);
    // width defaults to 0.0 — no set_width call.
    let c = on(&mut a, 69, 100);
    assert_eq!(
        c.iter().filter(|cmd| matches!(cmd, Cmd::SetParam { node: NodeId(30), .. })).count(),
        0,
        "width=0 emits no pan SetParam (byte-identical Cmd stream)"
    );
}
```

Update the `mk_poly()` helper to construct the allocator with a `sum_node` (e.g. `NodeId(30)`), matching the new `VoiceAllocator::new` signature.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- width_offset poly_width`
Expected: FAIL (`width_offset`/`set_width`/`sum_node` not defined).

- [ ] **Step 3: Add `width_offset` + the fields/setters**

Free fn near `unison_offset` in voice.rs:

```rust
/// Pan position for unison voice `u` of `count`, spread symmetrically and evenly
/// over ±`amount` of the stereo field. `count <= 1` ⇒ 0.0 (center). Result ∈
/// [-1.0, +1.0]: -1 = hard left, 0 = center, +1 = hard right.
fn width_offset(u: usize, count: usize, amount: f32) -> f32 {
    if count <= 1 { return 0.0; }
    let t = -1.0 + 2.0 * (u as f32) / ((count - 1) as f32); // -1..+1
    t * amount
}
```

`VoiceAllocator` struct: add `sum_node: NodeId` and `width_amount: f32`. `VoiceAllocator::new` gains a `sum_node: NodeId` parameter (thread it into the struct; init `width_amount: 0.0`; keep all existing fields). Add:

```rust
pub fn set_width(&mut self, amount: f32) { self.width_amount = amount; }
```

- [ ] **Step 4: Emit per-lane pan in `note_on`**

Inside the existing `for u in 0..u_count` lane loop in `VoiceAllocator::note_on`, right after the pitch `SetParam` (and the optional velocity `SetParam`), before `gate_all(lane, true, emit)`:

```rust
if self.width_amount != 0.0 {
    emit(Cmd::SetParam {
        node: self.sum_node,
        param: (lane + 1) as u8, // param 0 is the gain slot on StereoVoiceSum
        value: width_offset(u, u_count, self.width_amount),
    });
}
```

- [ ] **Step 5: Run + full regression + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- width_offset poly_width` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs — **all existing allocator tests must pass unedited** (`width == 0` default ⇒ no pan emit ⇒ byte-identical Cmd stream; the `mk_poly` helper signature change is the only test-side edit). Expected: PASS.

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): poly stereo width — VoiceAllocator emits per-lane pan (width!=0)"
```

---

### Task 3: Mono `MonoAllocator` width

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs` (`MonoAllocator` struct/`new`/setters/`note_on`)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `width_offset` (Task 2), `Cmd::SetParam`, `NodeId`, `VOICES`, `A440_NOTE`.
- Produces: `MonoAllocator` gains `sum_node: NodeId` + `width_amount: f32` + `set_width`. **`MonoAllocator::new` gains a `sum_node` parameter** (used by Task 4).

- [ ] **Step 1: Write the failing tests**

```rust
#[test]
fn mono_width_emits_per_lane_pan_on_sum_node() {
    let mut m = mk_mono(); // pitch=NodeId(10), slew=NodeId(20), gate=NodeId(30), sum=NodeId(40), no vel
    m.set_unison(2);
    m.set_width(1.0);
    let c = mon(&mut m, 69, 100); // from silence, 2 unison voices
    // pan on lanes 0 and 1 (params 1 and 2) on the sum node (NodeId(40)).
    let pans: std::vec::Vec<u8> = c.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(40), param, .. } => Some(*param), _ => None }).collect();
    assert_eq!(pans, std::vec![1, 2], "pan params lane+1 for lanes 0 and 1");
}

#[test]
fn mono_width_zero_emits_no_pan() {
    let mut m = mk_mono();
    m.set_unison(2);
    let c = mon(&mut m, 69, 100);
    assert_eq!(
        c.iter().filter(|cmd| matches!(cmd, Cmd::SetParam { node: NodeId(40), .. })).count(),
        0, "width=0 emits no pan"
    );
}
```

Update `mk_mono()` to construct `MonoAllocator` with a `sum_node` (`NodeId(40)`), matching the new signature.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- mono_width`
Expected: FAIL.

- [ ] **Step 3: Add fields/setters + emit per-lane pan**

`MonoAllocator` struct/`new`: add `sum_node: NodeId` (new `new` param) + `width_amount: f32` (init 0.0) + `pub fn set_width(&mut self, amount: f32) { self.width_amount = amount; }`.

Inside the `for u in 0..u_count` loop in `MonoAllocator::note_on` (the lanes `0..U` loop), after the pitch `SetParam` (and optional velocity), before the `from_silence` trigger/gate block:

```rust
if self.width_amount != 0.0 {
    emit(Cmd::SetParam {
        node: self.sum_node,
        param: (u + 1) as u8,
        value: width_offset(u, u_count, self.width_amount),
    });
}
```

(Note: mono lanes are `0..U`, so the lane index IS `u`; pan param is `u + 1`.)

- [ ] **Step 4: Run + full regression + commit**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- mono_width` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs — existing mono tests pass unedited (only the `mk_mono` helper signature changes). Expected: PASS.

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): mono stereo width — MonoAllocator emits per-lane pan (width!=0)"
```

---

### Task 4: `SynthAlloc` dispatch + Wren `width` setter + StereoVoiceSum build path

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`SynthAlloc`, `node_poly_end_impl`/`node_mono_end_impl`, setter impl + extern shim, `register_audio`)
- Modify: `crates/deluge-wren-core/src/bindings.rs` (METHODS table)
- Modify: `crates/deluge-wren-core/wren/prelude.wren` (`foreign` decl)

**Interfaces:**
- Consumes: `set_width` on both allocators (Tasks 2–3); `VoiceAllocator::new`/`MonoAllocator::new` now take a `sum_node: NodeId`; `Kind::StereoVoiceSum` (Task 1); `return_node_w`; `deluge_audio_graph::VOICES`.
- Produces: Wren `synth.width = amount`.

- [ ] **Step 1: Build `StereoVoiceSum` instead of `VoiceSum` + pass `sum_node`**

In `crates/deluge-wren-core/src/bindings_audio.rs`, find `node_poly_end_impl` and `node_mono_end_impl` (~1447-1488) where the `sum` node is created as `Kind::VoiceSum`. In BOTH:
1. Change the sum node kind to `Kind::StereoVoiceSum`.
2. Mark it width-2 when returning the Synth's `out` node — the `SynthObj.out_node` is still that id; ensure `synth.out` returns it with width 2 (mirror `node_pan_impl`'s `return_node_w(vm, id, 2)` at bindings_audio.rs:1630 — apply the width-2 marking wherever `synth.out` is produced, i.e. `synth_out` at ~1523-1529 must report width 2). Confirm how `synth_out` currently wraps the node and set its width to 2.
3. Pass the `sum` node id into the allocator constructor as the new `sum_node` argument: `VoiceAllocator::new(NodeId(pitch_ctrl), gates, n_gates, vel, NodeId(sum))` and `MonoAllocator::new(NodeId(pitch), NodeId(slew), gates, n_gates, vel, NodeId(sum))` — match the exact new parameter ORDER from Tasks 2–3 (append `sum_node` last).

- [ ] **Step 2: `SynthAlloc::set_width` dispatch**

Add to `impl SynthAlloc` (beside `set_unison`/`set_detune`):

```rust
fn set_width(&mut self, amount: f32) {
    match self { SynthAlloc::Poly(a) => a.set_width(amount), SynthAlloc::Mono(m) => m.set_width(amount) }
}
```

- [ ] **Step 3: The setter impl + extern shim**

```rust
/// `synth.width = amount` — set the unison stereo spread (0..1) on the active
/// allocator. Pan rides the next note-on (no immediate SetParam).
pub(crate) fn synth_set_width_impl<S: SlotApi>(vm: &S) {
    let amount = vm.get_f(1) as f32;
    self_synth(vm).alloc.set_width(amount);
}
```

Add the paired `#[cfg(feature = "wren-sys-backend")] #[no_mangle] extern "C"` shim mirroring `synth_set_glide` / `synth_set_unison` exactly:

```rust
#[cfg(feature = "wren-sys-backend")]
pub(crate) unsafe extern "C" fn synth_set_width(raw: *mut WrenVM) {
    let vm = /* same wrap as synth_set_unison */;
    synth_set_width_impl(&vm);
}
```
(Copy the exact wrap from `synth_set_unison` at bindings_audio.rs:~1550.)

- [ ] **Step 4: Register in BOTH tables + prelude**

1. `register_audio` (bindings_audio.rs, near the `unison=(_)`/`detune=(_)` registrations): add
```rust
method("main", "Synth", false, "width=(_)", synth_set_width_impl::<S>);
```
2. `bindings.rs` METHODS table (near `Synth` `unison=(_)`): add `method("Synth", "width=(_)", synth_set_width)` (mirror the exact form used for `unison=(_)`, pointing at the `extern "C"` shim).
3. `prelude.wren` `Synth` class (near `foreign unison=(n)`): add
```
    foreign width=(amount)
```
No Wren wrapper/guard (width works in both mono and poly).

- [ ] **Step 5: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Nothing exercises `width` yet (Task 5), but the gate is: compiles clean + full existing suite green in BOTH configs. If "metaclass does not implement" appears → the bindings.rs METHODS registration is missing. Confirm existing synths (and Sy-5e unison, whose `set_param(out_node, 0, 1/√N)` now hits `StereoVoiceSum`'s param-0 gain slot) still pass.
Expected: PASS both.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/src/bindings.rs crates/deluge-wren-core/wren/prelude.wren
git commit -m "feat(wren): synth.width stereo spread — StereoVoiceSum build path + width= setter"
```

---

### Task 5: End-to-end

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `synth.width = amount` (Task 4); `run_and_render(src, &mut [StereoFrame; N])` (renders into an L/R buffer; `StereoFrame` has `.l`/`.r` f32 fields; `StereoFrame::default()` is silence); `run_script_ok`.

- [ ] **Step 1: Write the failing e2e tests**

Mirror the existing Synth render harness (e.g. `synth_saw_renders_sound` at audio_bindings.rs:1122; NEWLINE-separated statements, envelope required in the block). Add:

```rust
#[test]
fn synth_poly_width_renders_stereo_image() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.1,0.7,0.2) }\ns.unison = 4\ns.detune = 12\ns.width = 1\nOut.patch(s.out)\ns.noteOn(60,100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 8.0 && f.r.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3 || f.r.abs() > 1e-3), "sounds");
    assert!(out.iter().any(|f| (f.l - f.r).abs() > 1e-4), "stereo image: L != R somewhere");
}

#[test]
fn synth_mono_width_renders_stereo_image() {
    let mut out = [StereoFrame::default(); 32];
    run_and_render(
        "var s = Synth.mono { |p| Osc.saw(p) * Env.adsr(0.005,0.1,0.7,0.2) }\ns.unison = 3\ns.detune = 20\ns.width = 1\nOut.patch(s.out)\ns.noteOn(60,100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.r.is_finite() && f.l.abs() <= 8.0 && f.r.abs() <= 8.0), "bounded");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3 || f.r.abs() > 1e-3), "sounds");
    assert!(out.iter().any(|f| (f.l - f.r).abs() > 1e-4), "mono unison spread: L != R");
}

#[test]
fn synth_width_zero_is_mono_and_byte_identical() {
    // Same patch/note rendered twice: once with `s.width = 0`, once with no width call.
    let patch_width0 = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.1,0.7,0.2) }\ns.unison = 4\ns.detune = 0\ns.width = 0\nOut.patch(s.out)\ns.noteOn(60,100)";
    let patch_none   = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.1,0.7,0.2) }\ns.unison = 4\ns.detune = 0\nOut.patch(s.out)\ns.noteOn(60,100)";
    let mut a = [StereoFrame::default(); 32];
    let mut b = [StereoFrame::default(); 32];
    run_and_render(patch_width0, &mut a);
    run_and_render(patch_none, &mut b);
    // width=0 ⇒ L == R (dual-mono) ...
    assert!(a.iter().all(|f| (f.l - f.r).abs() < 1e-9), "width=0 is dual-mono (L==R)");
    // ... and byte-identical to the no-width render, frame by frame.
    for (fa, fb) in a.iter().zip(b.iter()) {
        assert_eq!(fa.l, fb.l, "width=0 L byte-identical to no-width");
        assert_eq!(fa.r, fb.r, "width=0 R byte-identical to no-width");
    }
    assert!(a.iter().any(|f| f.l.abs() > 1e-3), "still sounds");
}
```

> If `StereoFrame` is not directly nameable in this test file, use the same import the existing render tests use (grep the top of `audio_bindings.rs` for `StereoFrame`). The `.l`/`.r` fields and `StereoFrame::default()` match the existing `synth_saw_renders_sound` idiom.

- [ ] **Step 2: Run to verify pass (setters already implemented in Task 4)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- synth_poly_width synth_mono_width synth_width_zero` (+ `--features deluge-dsp-kernels/simd`).
These should PASS on correct Tasks 1–4. If `synth_width_zero_is_mono_and_byte_identical` FAILS, that is a real non-breaking regression to investigate (do NOT weaken the assertion) — the width=0 path must be byte-identical.
Expected: PASS both.

- [ ] **Step 3: Full regression**

Run the FULL wren-core suite both configs (existing 1-voice + unison synths byte-unchanged): `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` with and without `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): stereo spread e2e — poly + mono width renders stereo, width=0 byte-identical"
```

---

## Self-Review Notes (for the executor)

- **Non-breaking is the hard gate (two parts):** `width == 0` must be byte-identical in BOTH the Cmd stream (no pan `SetParam` emitted — guarded by `width_amount != 0.0`) AND the rendered audio (StereoVoiceSum all-center = mono `voice_sum`, routed width-2 = today's dual-mono). The Task-5 `synth_width_zero_is_mono_and_byte_identical` + the full regressions are the proof.
- **The `set_param(0)` gain slot is shared** — `StereoVoiceSum` param 0 IS the gain, same as `VoiceSum`, so Sy-5e's `synth.unison = N` → `set_param(out_node, 0, 1/√N)` keeps working unchanged when the sum node becomes stereo. Do NOT renumber it.
- **Width-2 poly node is a new combination** but needs NO engine change: the engine passes `out` as `out_width * BLOCK` (`arr[base..base+width].as_flattened_mut()`, engine.rs:251), so `poly_process` receives `2*BLOCK`; split at `out.len()/2` → L = row `base` (port0), R = row `base+1` (port1). The width-2 bus routing (`write_source_to_bus`, port0→L / port1→R) picks them up iff `synth.out` is returned width-2 (`return_node_w(…, 2)`).
- **Allocator constructor arity change** (`sum_node` appended to both `new`s in Tasks 2–3) leaves wren-core non-compiling until Task 4 fixes the call sites — expected interim state; the graph crate itself stays green each task via its updated test helpers.
- **Both registration tables** (`register_audio` + `bindings.rs` METHODS) for `width=(_)`, plus the prelude `foreign width=(amount)` decl (the Sy-2c registered-but-undeclared gotcha).
- **Confirm exact names** against the codebase before transcribing: `BLOCK`/`MAX_INPUTS`/`In` construction in node tests, the `set_param` param type (`u8`), `return_node_w`, `synth_out`'s width handling, `get_f`, `self_synth`, the `extern "C"` shim wrap, `VOICES` path (`deluge_audio_graph::VOICES`).
- **Deferred (do NOT implement):** group-partitioned poly-unison (Sy-6b, next); `Osc(...).spread=` interaction; live re-pan on width change; re-centering on width→0; f32x8 `voice_sum_stereo`; stereo global FX rework; per-lane pan as a modulatable signal; MPE.
