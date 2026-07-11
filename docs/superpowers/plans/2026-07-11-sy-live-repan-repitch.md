# Sy-6b: Live Re-Pan / Re-Pitch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `synth.detune = cents` and `synth.width = amount` re-emit per-voice pitch/pan for currently-sounding notes (poly + mono) — turn the knob, hear the unison spread move live.

**Architecture:** Control-plane only. Both allocators remember each sounding voice's group context (`u`-index, group size `U`); `set_detune`/`set_width` gain an `emit` sink and re-emit `SetParam`s for the sounding voices. No kernel or graph-node change; no new Wren foreign.

**Tech Stack:** Rust `no_std` (`deluge-audio-graph` allocators + `deluge-wren-core` bindings), Wren scripting.

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in control paths. `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** a synth that never calls the setters is unaffected. A setter call with NO sounding voice (the normal `s.detune = …` / `s.width = …` before `noteOn`) re-emits nothing → every note-on Cmd stream and the Sy-6a `width==0` byte-identical render proof are preserved. Existing allocator tests pass with only mechanical `emit`-arg threading through their `set_detune`/`set_width` call sites.
- **No new registration:** `detune=(_)`/`width=(_)` are already registered (Sy-5e/6a). Only the Rust setter impls change.
- **Test invocation (per-crate, never `--workspace`; both configs; use `-- name1 name2`):**
  - Graph: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

---

### Task 1: Poly `VoiceAllocator` — group memory + live re-emit

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs` (`VoiceAllocator` struct ~line 44, `new` ~line 64, `note_on` lane loop ~line 121, `set_detune`/`set_width` ~lines 81-82)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `unison_offset`/`width_offset` (voice.rs), `Cmd::SetParam`, `LaneState::Held(u8)`, `A440_NOTE`, `VOICES`.
- Produces: `VoiceAllocator` gains `lane_ctx: [(u8, u8); VOICES]`; **`set_detune(&mut self, cents: f32, emit: &mut impl FnMut(Cmd))`** and **`set_width(&mut self, amount: f32, emit: &mut impl FnMut(Cmd))`** (signature change — an `emit` param appended). Used by Task 3.

- [ ] **Step 1: Write the failing tests**

In the `voice.rs` test module. Use the existing `mk_poly()`/`mk()` helper and the `on(a, note, vel)` Cmd-capture helper. Add a `set_detune`/`set_width` capture helper if none exists (mirror `on`):

```rust
#[test]
fn poly_live_redetune_re_emits_held_lanes() {
    let mut a = mk_poly(); // pitch=NodeId(10), gate=NodeId(20), sum=NodeId(30)
    a.set_unison(3);
    {
        let mut sink = |_c: Cmd| {};
        a.set_detune(0.0, &mut sink); // no note yet → emits nothing (below)
    }
    on(&mut a, 69, 100); // 3 Held lanes
    // now re-detune the sounding note
    let mut cmds: std::vec::Vec<Cmd> = std::vec::Vec::new();
    {
        let mut sink = |c: Cmd| cmds.push(c);
        a.set_detune(20.0, &mut sink);
    }
    // 3 pitch SetParams on the pitch node, one per Held lane, new spread.
    let pitches: std::vec::Vec<(u8, f32)> = cmds.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(10), param, value } => Some((*param, *value)),
        _ => None,
    }).collect();
    assert_eq!(pitches.len(), 3, "3 held lanes re-detuned");
    // values are note(69)-69=0 + unison_offset(u,3,20) for u=0,1,2 → -0.2, 0, +0.2
    let mut vals: std::vec::Vec<f32> = pitches.iter().map(|(_, v)| *v).collect();
    vals.sort_by(|x, y| x.partial_cmp(y).unwrap());
    assert!((vals[0] - (-0.2)).abs() < 1e-6 && vals[1].abs() < 1e-6 && (vals[2] - 0.2).abs() < 1e-6);
}

#[test]
fn poly_live_rewidth_re_emits_and_recenters() {
    let mut a = mk_poly();
    a.set_unison(3);
    on(&mut a, 69, 100);
    // re-width to 1.0
    let mut c1: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| c1.push(c); a.set_width(1.0, &mut s); }
    let pans1 = c1.iter().filter(|cmd| matches!(cmd, Cmd::SetParam { node: NodeId(30), .. })).count();
    assert_eq!(pans1, 3, "3 held lanes re-panned");
    // re-width back to 0.0 → re-emits pan-0 (re-center), NOT skipped
    let mut c2: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| c2.push(c); a.set_width(0.0, &mut s); }
    let pans2: std::vec::Vec<f32> = c2.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(30), value, .. } => Some(*value), _ => None }).collect();
    assert_eq!(pans2.len(), 3, "re-center emits 3 pan SetParams");
    assert!(pans2.iter().all(|v| v.abs() < 1e-6), "all re-centered to 0");
}

#[test]
fn poly_live_setters_no_sounding_note_emit_nothing() {
    let mut a = mk_poly();
    a.set_unison(3);
    let mut cmds: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| cmds.push(c); a.set_detune(20.0, &mut s); a.set_width(1.0, &mut s); }
    assert_eq!(cmds.len(), 0, "no held voice → no re-emit (non-breaking)");
}

#[test]
fn poly_live_redetune_uses_per_lane_ctx() {
    let mut a = mk_poly();
    a.set_unison(2);
    on(&mut a, 60, 100); // note 60, 2 voices
    on(&mut a, 64, 100); // note 64, 2 voices → 4 Held lanes total
    let mut cmds: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| cmds.push(c); a.set_detune(10.0, &mut s); }
    let pitches: std::vec::Vec<f32> = cmds.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(10), value, .. } => Some(*value), _ => None }).collect();
    assert_eq!(pitches.len(), 4, "all 4 held lanes re-detuned");
    // note 60 → base -9 ± 0.1 ; note 64 → base -5 ± 0.1
    assert!(pitches.iter().any(|v| (v - (-9.1)).abs() < 1e-6));
    assert!(pitches.iter().any(|v| (v - (-8.9)).abs() < 1e-6));
    assert!(pitches.iter().any(|v| (v - (-5.1)).abs() < 1e-6));
    assert!(pitches.iter().any(|v| (v - (-4.9)).abs() < 1e-6));
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- poly_live`
Expected: FAIL (`set_detune`/`set_width` take 1 arg, no `lane_ctx`).

- [ ] **Step 3: Add `lane_ctx` + write it in `note_on`**

`VoiceAllocator` struct (after `width_amount: f32,`): add
```rust
    lane_ctx: [(u8, u8); VOICES], // (u-index, group size U) for each Held lane
```
`new` (in the struct literal, beside `lane_state`): add `lane_ctx: [(0, 0); VOICES],`.

In `note_on`, inside the `for u in 0..u_count` loop, right after `self.lane_state[lane] = LaneState::Held(note);`:
```rust
            self.lane_ctx[lane] = (u as u8, u_count as u8);
```

- [ ] **Step 4: Change `set_detune`/`set_width` to re-emit**

Replace the current one-line poly setters (voice.rs ~81-82):
```rust
pub fn set_detune(&mut self, cents: f32, emit: &mut impl FnMut(Cmd)) {
    self.detune_cents = cents;
    for lane in 0..VOICES {
        if let LaneState::Held(note) = self.lane_state[lane] {
            let (u, count) = self.lane_ctx[lane];
            let value = note as f32 - A440_NOTE
                + unison_offset(u as usize, count as usize, cents);
            emit(Cmd::SetParam { node: self.pitch_node, param: lane as u8, value });
        }
    }
}

pub fn set_width(&mut self, amount: f32, emit: &mut impl FnMut(Cmd)) {
    self.width_amount = amount;
    for lane in 0..VOICES {
        if let LaneState::Held(_) = self.lane_state[lane] {
            let (u, count) = self.lane_ctx[lane];
            emit(Cmd::SetParam {
                node: self.sum_node,
                param: (lane + 1) as u8,
                value: width_offset(u as usize, count as usize, amount),
            });
        }
    }
}
```

> `set_width` re-emits pan even for `amount == 0.0` (re-centers sounding voices) — this differs from `note_on`'s `width!=0` guard by design. Only `Held` lanes are touched.

- [ ] **Step 5: Fix the graph crate's own test call sites**

The signature change breaks any existing `voice.rs` test that calls `a.set_detune(x)` / `a.set_width(x)` (e.g. the Sy-5e/6a `poly_unison_*`/`poly_width_*` tests). Add the `emit` arg to each — a discarding sink is fine where the emit isn't asserted: `a.set_detune(10.0, &mut |_| {})`, `a.set_width(1.0, &mut |_| {})`. Do NOT change any assertions.

- [ ] **Step 6: Run + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- poly_live` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs (existing allocator/unison/width tests green with only the `emit`-arg edits). Expected: PASS.

> `deluge-wren-core` will NOT compile after this task (its `SynthAlloc::set_detune`/`set_width` still call the old 1-arg form) — fixed in Task 3. Expected interim per-crate state.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): poly live re-pan/re-pitch — VoiceAllocator re-emits held voices on set_detune/set_width"
```

---

### Task 2: Mono `MonoAllocator` — active-U memory + live re-emit

**Files:**
- Modify: `crates/deluge-audio-graph/src/voice.rs` (`MonoAllocator` struct, `new`, `note_on` ~line 239, `set_detune`/`set_width` ~lines 219-220)
- Test: `voice.rs` test module

**Interfaces:**
- Consumes: `unison_offset`/`width_offset`, `Cmd::SetParam`, `A440_NOTE`, `VOICES`.
- Produces: `MonoAllocator` gains `mono_active_u: usize`; **`set_detune(&mut self, cents: f32, emit: &mut impl FnMut(Cmd))`** and **`set_width(&mut self, amount: f32, emit: &mut impl FnMut(Cmd))`** (emit param appended). Used by Task 3.

- [ ] **Step 1: Write the failing tests**

Use the existing `mk_mono()` and `mon(m, note, vel)` helpers.

```rust
#[test]
fn mono_live_redetune_and_rewidth_re_emit() {
    let mut m = mk_mono(); // pitch=NodeId(10), slew=NodeId(20), gate=NodeId(30), sum=NodeId(40)
    m.set_unison(2);
    mon(&mut m, 69, 100); // from silence, lanes 0,1
    // live re-detune
    let mut cd: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| cd.push(c); m.set_detune(10.0, &mut s); }
    let pitches: std::vec::Vec<(u8, f32)> = cd.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(10), param, value } => Some((*param, *value)), _ => None }).collect();
    assert_eq!(pitches.len(), 2, "2 mono lanes re-detuned");
    // note 69 → base 0 ± 0.1 on lanes 0,1
    assert!(pitches.iter().any(|(p, v)| *p == 0 && (v - (-0.1)).abs() < 1e-6));
    assert!(pitches.iter().any(|(p, v)| *p == 1 && (v - 0.1).abs() < 1e-6));
    // live re-width
    let mut cw: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| cw.push(c); m.set_width(1.0, &mut s); }
    let pans: std::vec::Vec<u8> = cw.iter().filter_map(|cmd| match cmd {
        Cmd::SetParam { node: NodeId(40), param, .. } => Some(*param), _ => None }).collect();
    assert_eq!(pans, std::vec![1, 2], "pan on lanes 0,1 (param u+1)");
}

#[test]
fn mono_live_setters_silent_emit_nothing() {
    let mut m = mk_mono();
    m.set_unison(2);
    // no note sounding
    let mut cmds: std::vec::Vec<Cmd> = std::vec::Vec::new();
    { let mut s = |c: Cmd| cmds.push(c); m.set_detune(10.0, &mut s); m.set_width(1.0, &mut s); }
    assert_eq!(cmds.len(), 0, "no sounding note → no re-emit");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- mono_live`
Expected: FAIL.

- [ ] **Step 3: Add `mono_active_u` + write it in `note_on`**

`MonoAllocator` struct (after the unison/detune/width fields): add
```rust
    mono_active_u: usize, // the sounding note's U-at-play (unison is next-note)
```
`new` struct literal: add `mono_active_u: 0,`.

In `note_on`, right after `let u_count = self.unison.min(VOICES);`:
```rust
        self.mono_active_u = u_count;
```

- [ ] **Step 4: Change `set_detune`/`set_width` to re-emit**

Replace the mono one-line setters (voice.rs ~219-220):
```rust
pub fn set_detune(&mut self, cents: f32, emit: &mut impl FnMut(Cmd)) {
    self.detune_cents = cents;
    if self.len > 0 {
        let top = self.notes[self.len - 1];
        let u_count = self.mono_active_u;
        for u in 0..u_count {
            let value = top as f32 - A440_NOTE + unison_offset(u, u_count, cents);
            emit(Cmd::SetParam { node: self.pitch_node, param: u as u8, value });
        }
    }
}

pub fn set_width(&mut self, amount: f32, emit: &mut impl FnMut(Cmd)) {
    self.width_amount = amount;
    if self.len > 0 {
        let u_count = self.mono_active_u;
        for u in 0..u_count {
            emit(Cmd::SetParam {
                node: self.sum_node,
                param: (u + 1) as u8,
                value: width_offset(u, u_count, amount),
            });
        }
    }
}
```

- [ ] **Step 5: Fix the graph crate's own mono test call sites**

Add the `emit` arg to any existing mono test that calls `set_detune`/`set_width` (discarding sink where not asserted). No assertion changes.

- [ ] **Step 6: Run + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- mono_live` (+ `--features deluge-dsp-kernels/simd`).
Then FULL graph crate both configs. Expected: PASS. (`deluge-wren-core` still non-compiling until Task 3.)

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/voice.rs
git commit -m "feat(graph): mono live re-pan/re-pitch — MonoAllocator re-emits sounding note on set_detune/set_width"
```

---

### Task 3: Wren — `SynthAlloc` dispatch + impls pass the host emit sink

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`SynthAlloc::set_detune`/`set_width` ~lines 116-128; `synth_set_detune_impl`/`synth_set_width_impl` ~lines 1600-1614)

**Interfaces:**
- Consumes: `VoiceAllocator`/`MonoAllocator` `set_detune(cents, emit)`/`set_width(amount, emit)` (Tasks 1–2); the host emit idiom `&mut |c| crate::host::host().audio_cmd(c)` (already used by `synth_note_on` at bindings_audio.rs:1513).
- Produces: live `synth.detune`/`synth.width`. No new registration.

- [ ] **Step 1: Thread `emit` through `SynthAlloc` dispatch**

Replace `SynthAlloc::set_detune`/`set_width` (bindings_audio.rs ~116-128):
```rust
fn set_detune(&mut self, cents: f32, emit: &mut impl FnMut(deluge_audio_graph::Cmd)) {
    match self {
        SynthAlloc::Poly(a) => a.set_detune(cents, emit),
        SynthAlloc::Mono(m) => m.set_detune(cents, emit),
    }
}
fn set_width(&mut self, amount: f32, emit: &mut impl FnMut(deluge_audio_graph::Cmd)) {
    match self {
        SynthAlloc::Poly(a) => a.set_width(amount, emit),
        SynthAlloc::Mono(m) => m.set_width(amount, emit),
    }
}
```
(Match the exact `Cmd` path used by the neighboring `note_on`/`note_off` dispatch — `deluge_audio_graph::Cmd`.)

- [ ] **Step 2: Pass the host sink in the impls**

`synth_set_detune_impl` (bindings_audio.rs ~1600): change
```rust
    self_synth(vm).alloc.set_detune(cents);
```
to
```rust
    self_synth(vm).alloc.set_detune(cents, &mut |c| crate::host::host().audio_cmd(c));
```
`synth_set_width_impl` (~1612): same, `set_width(amount, &mut |c| crate::host::host().audio_cmd(c))`.

(`cents`/`amount` are already read via `vm.get_f(1)` in those impls — keep that. `synth_set_unison_impl` is unchanged.)

- [ ] **Step 3: Compile + full regression**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` (+ `--features deluge-dsp-kernels/simd`).
Gate: wren-core compiles clean + full existing suite green in BOTH configs (nothing exercises the LIVE path yet — Task 4 — but existing `detune`/`width`/`unison` e2e must still pass: they set the value before `noteOn`, so re-emit is a no-op). Expected: PASS both.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs
git commit -m "feat(wren): live synth.detune/synth.width — SynthAlloc dispatch passes host emit sink"
```

---

### Task 4: End-to-end

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: live `synth.detune`/`synth.width` (Task 3); `run_and_render(src, &mut [StereoFrame; N])`; `StereoFrame` (`.l`/`.r`).

- [ ] **Step 1: Write the failing e2e tests**

The render harness renders the whole script then reads the output buffer, so drive the live change as script statements AFTER `noteOn`, and compare against a baseline that never changes the value. (NEWLINE-separated; envelope required in the block.)

```rust
#[test]
fn synth_live_detune_moves_sounding_note() {
    // Baseline: detune set once, before noteOn.
    let base = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 4\ns.detune = 5\nOut.patch(s.out)\ns.noteOn(60,100)";
    // Live: same start, then re-detune the held note mid-render.
    let live = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 4\ns.detune = 5\nOut.patch(s.out)\ns.noteOn(60,100)\ns.detune = 40";
    let mut a = [StereoFrame::default(); 32];
    let mut b = [StereoFrame::default(); 32];
    run_and_render(base, &mut a);
    run_and_render(live, &mut b);
    assert!(a.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "base bounded");
    assert!(b.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "live bounded");
    assert!(b.iter().any(|f| f.l.abs() > 1e-3), "live sounds");
    // The live re-detune changed the held note's spread → output differs.
    assert!(a.iter().zip(b.iter()).any(|(x, y)| (x.l - y.l).abs() > 1e-4),
        "live detune moved the sounding note");
}

#[test]
fn synth_live_width_respreads_sounding_note() {
    let base = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 4\ns.detune = 12\nOut.patch(s.out)\ns.noteOn(60,100)";
    let live = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 4\ns.detune = 12\nOut.patch(s.out)\ns.noteOn(60,100)\ns.width = 1";
    let mut a = [StereoFrame::default(); 32];
    let mut b = [StereoFrame::default(); 32];
    run_and_render(base, &mut a);
    run_and_render(live, &mut b);
    // base has no width → dual-mono (L==R); live width=1 → a real stereo image.
    assert!(a.iter().all(|f| (f.l - f.r).abs() < 1e-9), "base is dual-mono");
    assert!(b.iter().any(|f| (f.l - f.r).abs() > 1e-4), "live width spread the held note");
}

#[test]
fn synth_live_mono_detune_moves_sounding_note() {
    let base = "var s = Synth.mono { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 3\ns.detune = 5\nOut.patch(s.out)\ns.noteOn(60,100)";
    let live = "var s = Synth.mono { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 3\ns.detune = 5\nOut.patch(s.out)\ns.noteOn(60,100)\ns.detune = 40";
    let mut a = [StereoFrame::default(); 32];
    let mut b = [StereoFrame::default(); 32];
    run_and_render(base, &mut a);
    run_and_render(live, &mut b);
    assert!(b.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0) && b.iter().any(|f| f.l.abs() > 1e-3), "mono live bounded+sounds");
    assert!(a.iter().zip(b.iter()).any(|(x, y)| (x.l - y.l).abs() > 1e-4), "mono live detune moved the note");
}

#[test]
fn synth_setter_before_noteon_is_unchanged() {
    // Setting detune/width BEFORE noteOn re-emits nothing → identical to a plain build.
    let with_pre = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 2\ns.detune = 0\nOut.patch(s.out)\ns.noteOn(60,100)";
    let plain    = "var s = Synth.new { |p| Osc.saw(p) * Env.adsr(0.005,0.5,0.9,0.3) }\ns.unison = 2\nOut.patch(s.out)\ns.noteOn(60,100)";
    let mut a = [StereoFrame::default(); 32];
    let mut b = [StereoFrame::default(); 32];
    run_and_render(with_pre, &mut a);
    run_and_render(plain, &mut b);
    for (x, y) in a.iter().zip(b.iter()) {
        assert_eq!(x.l, y.l, "pre-note detune=0 is byte-identical");
        assert_eq!(x.r, y.r);
    }
}
```

- [ ] **Step 2: Run to verify pass (setters already live from Task 3)**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- synth_live synth_setter_before` (+ `--features deluge-dsp-kernels/simd`).
These should PASS on a correct Tasks 1–3. If a `synth_live_*` test FAILS to see a difference, the re-emit isn't reaching the graph — a real bug; investigate (do NOT weaken the assertion). If `synth_setter_before_noteon_is_unchanged` fails, non-breaking is broken.
Expected: PASS both.

- [ ] **Step 3: Full regression**

Run the FULL wren-core suite both configs: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core` with and without `--features deluge-dsp-kernels/simd`.
Expected: PASS both.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): live re-pan/re-pitch e2e — detune/width move sounding notes, pre-note unchanged"
```

---

## Self-Review Notes (for the executor)

- **Non-breaking is the hard gate:** a setter call with NO sounding voice re-emits nothing (the `if let Held` / `if self.len > 0` guards), so `s.detune = …`/`s.width = …` BEFORE `noteOn` (the normal case) is byte-identical — proven by `synth_setter_before_noteon_is_unchanged` + the unchanged Sy-6a `width=0` proof. Existing allocator tests change ONLY by threading the `emit` arg (discarding sink), never assertions.
- **Per-lane ctx is the crux:** `lane_ctx[lane] = (u, U)` is written in `note_on` and read in `set_detune`/`set_width` for `Held` lanes only. Multiple held notes must not cross-contaminate — `poly_live_redetune_uses_per_lane_ctx` proves each lane re-computes from its own `(u, U, note)`.
- **`set_width` re-emits even at 0.0** (re-centers sounding voices) — intentionally different from `note_on`'s `width!=0` guard. Do not "optimize" it to skip 0.
- **Mono `mono_active_u`** captures U-at-play (unison is next-note); re-emit uses it, not the possibly-changed `self.unison`.
- **Interim non-compiling wren-core** between Tasks 1–2 (allocator signature change) and Task 3 (dispatch fix) is expected per-crate state — the graph crate itself stays green each task via its updated test call sites.
- **No new Wren registration** — `detune=(_)`/`width=(_)` already in both tables + prelude (Sy-5e/6a). Only the Rust impls change.
- **Deferred (do NOT implement):** live unison-count re-voicing; poly pitch/pan smoothing; re-centering reused Free/Releasing lanes on width→0; live glide; Tb303/Modal poly; f32x8 ADSR; Bus poly; MPE.
