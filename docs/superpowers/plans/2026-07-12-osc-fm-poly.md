# Poly FM Primitives Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the poly oscillators `PolyOsc` (sine/saw/square/tri) and `PolyWt` (wavetable) a per-voice phase-mod (`pm`) input and per-voice self-`feedback`, so the composable FM idiom `carrier.pm = modulator * index` works per-voice inside `Synth.new { }`.

**Architecture:** Mirror the proven mono `Osc` FM model (`pmod` added to a *read* phase; `feedback*0.5*(last+last2)` self-FM) onto `PolyOsc` and the shared `WtOsc` kernel. Wire `pm` as a per-voice poly edge (uniform port 2 across both poly kinds) and `feedback` as a scalar param (poly param 1). Reuse the existing `pm=`/`feedback=` Wren setters, made poly-aware via the `NodeObj.poly` flag. Un-patched `pm` + `feedback==0` is byte-identical to today.

**Tech Stack:** Rust `no_std`; `deluge-dsp-kernels` (kernels, scalar oracle + `#[cfg(feature="simd")] f32x8`), `deluge-audio-graph` (graph nodes), `deluge-wren-core` (Wren bindings). VOICES=8, voice-interleaved tiles `tile[i*VOICES+v]`.

## Global Constraints

- `no_std`, no heap, no panic on any input. VOICES == 8 (compile-time).
- Scalar path is the correctness oracle; `#[cfg(feature="simd")]` `f32x8` fast path must match within the crate's existing tolerance; tests pass in BOTH configs (`--features simd` for kernels; `--features deluge-dsp-kernels/simd` for graph/wren-core).
- MIT/Apache-2.0 only.
- **`pm` unconnected + `feedback == 0` MUST be byte-identical to today** in both the Cmd stream and the render (the non-breaking guarantee). This is the single most important invariant — every kernel change is written so the `pm=0, feedback=0` path reduces to the exact current code.
- No new Wren methods — reuse `pm=`/`feedback=` (routing becomes poly-aware).
- Test invocation is per-crate, NEVER `--workspace`; use `-- name1 name2`. LSP `armv7a … can't find crate for test` is environmental noise — ignore it.
- Reference model (mono `Osc`, osc.rs:231): `ph = phase + pmod.at(i) + feedback*0.5*(last+last2)`, wrap, sample → `y`, `last2=last; last=y`, `phase += freq*dt`. `set_feedback` clamps `[-1,1]`.

## Interfaces (locked indices — every task uses these verbatim)

- **Engine** `poly_in`/`poly_scratch` widen from **2 → 3** (max `poly_in_count` becomes 3). REQUIRED by PolyOsc's 3rd poly edge.
- **`pm` poly edge = port 2** for BOTH `PolyOsc` and `PolyWt` (uniform). `PolyOsc` poly ports: pitch=0, width=1, pm=2. `PolyWt` poly ports: pitch=0, **position=1**, pm=2.
- **`feedback` param = 1** for both poly kinds; **param 0** for mono `Osc` and mono `Wavetable`.
- **`position` (poly `PolyWt`) = poly port 1** (moved from the mono `ins[2]`=port 2 to make room for the unified pm=port 2). Mono `Wavetable` position stays port 2.
- `poly_in_count`: `PolyOsc` 2→**3**, `PolyWt` 1→**3**.
- **Setters route on `audio::poly_mode()`** (true inside `Synth.new{}`), mirroring the existing `node_set_width_impl`:
  - `pm=` → `set_input(id, if poly_mode() {2} else {1}, arg_input)`
  - `feedback=` → `set_param(id, if poly_mode() {1} else {0}, get_f)`
  - `position=` → `set_input(id, if poly_mode() {1} else {2}, arg_input)`
- Kernel signatures after this plan: `PolyOsc::process(&mut self, pitch: &[f32], width: &[f32], pm: &[f32], dt: f32, out: &mut [f32])` + `PolyOsc::set_feedback(f32)`; `WtOsc::set_feedback(f32)` (process signature unchanged — `pmod` already exists); `PolyWt::set_feedback(f32)`.

---

### Task 1: `PolyOsc` kernel — per-voice `pm` + `feedback`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolyOsc` struct + `new` + `process` + new `set_feedback`)
- Test: `crates/deluge-dsp-kernels/src/poly.rs` test module

**Interfaces:**
- Consumes: `wave_sample`/`wave_sample_x8`, `Wave`, `floorf`, `f32x8`, `VOICES`.
- Produces: `PolyOsc::process(&mut self, pitch: &[f32], width: &[f32], pm: &[f32], dt: f32, out: &mut [f32])`; `PolyOsc::set_feedback(&mut self, f: f32)`. Consumed by Task 3.

- [ ] **Step 1: Write the failing tests**

Add to the poly.rs test module (mirror the mono `pm_produces_sidebands`/`fm_and_feedback_stay_bounded` idioms; a helper builds a `VOICES*n` interleaved tile):

```rust
#[test]
fn polyosc_pm_zero_feedback_zero_is_identity() {
    // pm=0 & feedback=0 must reproduce the pre-change PolyOsc output bit-for-bit.
    let n = 64;
    let pitch = vec![440.0f32; VOICES * n];
    let width = vec![0.0f32; VOICES * n];
    let pm = vec![0.0f32; VOICES * n];
    let dt = 1.0 / 48000.0;
    let mut a = PolyOsc::new(); // new signature
    let mut out_new = vec![0.0f32; VOICES * n];
    a.process(&pitch, &width, &pm, dt, &mut out_new);
    // Reference: replicate the OLD math inline (phase accumulate + wave_sample(p)).
    let mut ref_phase = [0.0f32; VOICES];
    let mut out_ref = vec![0.0f32; VOICES * n];
    for i in 0..n {
        for v in 0..VOICES {
            let dtp = pitch[i * VOICES + v] * dt;
            let mut p = ref_phase[v] + dtp;
            p -= libm::floorf(p);
            ref_phase[v] = p;
            out_ref[i * VOICES + v] = wave_sample(Wave::Sine, p, dtp, width[i * VOICES + v]);
        }
    }
    assert_eq!(out_new, out_ref, "pm=0,fb=0 is bit-identical to the old PolyOsc");
}

#[test]
fn polyosc_pm_bends_a_lane() {
    // A non-zero pm on lane 0 changes lane 0's output but leaves an all-zero-pm
    // lane (lane 1) equal to the identity output.
    let n = 64;
    let pitch = vec![440.0f32; VOICES * n];
    let width = vec![0.0f32; VOICES * n];
    let mut pm = vec![0.0f32; VOICES * n];
    for i in 0..n { pm[i * VOICES + 0] = 0.25; } // constant phase offset on lane 0
    let dt = 1.0 / 48000.0;
    let mut a = PolyOsc::new();
    let mut out = vec![0.0f32; VOICES * n];
    a.process(&pitch, &width, &pm, dt, &mut out);
    let l0: f32 = (0..n).map(|i| out[i * VOICES + 0].abs()).sum();
    // lane 0 shifted vs a no-pm reference lane 0
    let mut b = PolyOsc::new();
    let zero = vec![0.0f32; VOICES * n];
    let mut out0 = vec![0.0f32; VOICES * n];
    b.process(&pitch, &width, &zero, dt, &mut out0);
    let mut differs = false;
    for i in 0..n { if (out[i*VOICES] - out0[i*VOICES]).abs() > 1e-6 { differs = true; } }
    assert!(differs, "pm on lane 0 changes its output");
    assert!(l0.is_finite());
}

#[test]
fn polyosc_feedback_stays_bounded() {
    let n = 256;
    let pitch = vec![440.0f32; VOICES * n];
    let width = vec![0.0f32; VOICES * n];
    let pm = vec![0.0f32; VOICES * n];
    let dt = 1.0 / 48000.0;
    let mut a = PolyOsc::new();
    a.set_feedback(1.0);
    let mut out = vec![0.0f32; VOICES * n];
    a.process(&pitch, &width, &pm, dt, &mut out);
    assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 2.0), "feedback bounded");
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- polyosc_pm polyosc_feedback`
Expected: FAIL to COMPILE (`process` takes 4 args, no `set_feedback`).

- [ ] **Step 3: Extend the `PolyOsc` struct + `new` + add `set_feedback`**

`crates/deluge-dsp-kernels/src/poly.rs`, `PolyOsc` (poly.rs:83):

```rust
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES], // [0,1) per voice
    last: [f32; VOICES],  // previous output sample per voice (feedback)
    last2: [f32; VOICES], // output two samples ago per voice (feedback)
    feedback: f32,        // self-FM depth, [-1,1]
    shape: Wave,
}
impl PolyOsc {
    pub fn new() -> PolyOsc {
        PolyOsc { phase: [0.0; VOICES], last: [0.0; VOICES], last2: [0.0; VOICES], feedback: 0.0, shape: Wave::Sine }
    }
    /// Self-FM depth, clamped to [-1, 1] (mirrors mono `Osc::set_feedback`).
    pub fn set_feedback(&mut self, f: f32) {
        self.feedback = f.clamp(-1.0, 1.0);
    }
    // ... set_shape unchanged ...
```

- [ ] **Step 4: Extend `process` — add `pm` and the feedback read-phase (both paths)**

Replace `PolyOsc::process` (poly.rs:105). The accumulator (`self.phase`) still advances by `dtp` and is read AFTER advancing — preserving today's phase; `pm` + feedback apply to a SEPARATE read phase `rp` so `pm=0,fb=0 ⇒ rp=p ⇒ identical`.

**Scalar path** (`#[cfg(not(feature = "simd"))]`):
```rust
let n = out.len() / VOICES;
for i in 0..n {
    for v in 0..VOICES {
        let f = pitch[i * VOICES + v];
        let dtp = f * dt;
        let mut p = self.phase[v] + dtp;
        p -= floorf(p); // wrap [0,1)
        self.phase[v] = p;
        let fb = self.feedback * 0.5 * (self.last[v] + self.last2[v]);
        let mut rp = p + pm[i * VOICES + v] + fb; // read phase
        rp -= floorf(rp);
        let y = wave_sample(self.shape, rp, dtp, width[i * VOICES + v]);
        self.last2[v] = self.last[v];
        self.last[v] = y;
        out[i * VOICES + v] = y;
    }
}
```
(When `pm=0` & `feedback=0`: `fb=0`, `rp = p` (p∈[0,1)⇒floorf(p)=0), `y = wave_sample(shape, p, dtp, width)` — byte-identical to the old line.)

**SIMD path** (`#[cfg(feature = "simd")]`): keep the existing accumulator wrap; add the read-phase vector:
```rust
use core::simd::prelude::*;
let n = out.len() / VOICES;
let one = f32x8::splat(1.0);
let dtv = f32x8::splat(dt);
let half = f32x8::splat(0.5);
let fbv = f32x8::splat(self.feedback);
let mut ph = f32x8::from_array(self.phase);
let mut last = f32x8::from_array(self.last);
let mut last2 = f32x8::from_array(self.last2);
for i in 0..n {
    let f = f32x8::from_slice(&pitch[i * VOICES..]);
    let w = f32x8::from_slice(&width[i * VOICES..]);
    let pm_v = f32x8::from_slice(&pm[i * VOICES..]);
    let dtp = f * dtv;
    let mut p = ph + dtp;
    let t: f32x8 = p.cast::<i32>().cast::<f32>();
    let fl = t.simd_gt(p).select(t - one, t);
    p -= fl;
    ph = p;
    // read phase = p + pm + feedback*0.5*(last+last2), wrapped like `p`
    let mut rp = p + pm_v + fbv * half * (last + last2);
    let tr: f32x8 = rp.cast::<i32>().cast::<f32>();
    let flr = tr.simd_gt(rp).select(tr - one, tr);
    rp -= flr;
    let y = wave_sample_x8(self.shape, rp, dtp, w);
    y.copy_to_slice(&mut out[i * VOICES..]);
    last2 = last;
    last = y;
}
self.phase = ph.to_array();
self.last = last.to_array();
self.last2 = last2.to_array();
```
(When `pm=0` & `feedback=0`: `rp = p` after wrap, `wave_sample_x8(shape, p, dtp, w)` — identical to the old copy_to_slice. `last`/`last2` update but don't affect output.)

Update the `process` signature to `(&mut self, pitch: &[f32], width: &[f32], pm: &[f32], dt: f32, out: &mut [f32])` and its doc comment.

- [ ] **Step 5: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- polyosc_pm polyosc_feedback` (and with `--features simd`).
Then the full crate both configs: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels` and `--features simd`.
Expected: PASS. (Any pre-existing `PolyOsc` kernel test that called `process` with 4 args must be updated to pass a zero `pm` tile — that's a byte-identical no-op.)

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(kernels): PolyOsc per-voice pm + self-feedback (pm=0,fb=0 byte-identical)"
```

---

### Task 2: `WtOsc` kernel `feedback` (shared) + `PolyWt::set_feedback`

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/wavetable.rs` (`WtOsc` struct + `new` + `process` + `process_morph` + new `set_feedback`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolyWt::set_feedback` fan-out)
- Test: both files' test modules

**Interfaces:**
- Consumes: existing `WtOsc::process(mips, freq, pmod, dt, out)` (already has `pmod`), `sample_one`/`sample_at_level`.
- Produces: `WtOsc::set_feedback(&mut self, f: f32)`; `PolyWt::set_feedback(&mut self, f: f32)`. Consumed by Task 3. Mono `Wavetable` also gains feedback (bonus).

- [ ] **Step 1: Write the failing tests**

`wavetable.rs` test module:
```rust
#[test]
fn wtosc_feedback_zero_is_identity() {
    // A fresh WtOsc with feedback=0 must render bit-identically to before.
    // (Compare a feedback=0 instance against an explicit second instance driven
    //  the same way — this locks the feedback==0 fast-path/general-path identity.)
    let mips = /* build the standard test MipSet as existing WtOsc tests do */;
    let dt = 1.0 / 48000.0;
    let n = 64;
    let mut a = WtOsc::new(); // feedback defaults 0
    let mut out_a = vec![0.0f32; n];
    a.process(mips, In::K(440.0), In::K(0.0), dt, &mut out_a);
    let mut b = WtOsc::new();
    b.set_feedback(0.0); // explicit 0 — must not change anything
    let mut out_b = vec![0.0f32; n];
    b.process(mips, In::K(440.0), In::K(0.0), dt, &mut out_b);
    assert_eq!(out_a, out_b, "feedback=0 is identity");
}

#[test]
fn wtosc_feedback_stays_bounded() {
    let mips = /* standard test MipSet */;
    let dt = 1.0 / 48000.0;
    let n = 256;
    let mut a = WtOsc::new();
    a.set_feedback(1.0);
    let mut out = vec![0.0f32; n];
    a.process(mips, In::K(440.0), In::K(0.0), dt, &mut out);
    assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 2.0), "feedback bounded");
}
```
> Build the `MipSet` exactly as the existing `WtOsc` tests in this file do (find one and copy its setup — do NOT invent a new table).

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- wtosc_feedback`
Expected: FAIL to compile (no `set_feedback`).

- [ ] **Step 3: Add feedback state + `set_feedback` to `WtOsc`**

`wavetable.rs` (`WtOsc`, wavetable.rs:93):
```rust
pub struct WtOsc {
    phase: f32,
    last: f32,
    last2: f32,
    feedback: f32,
}
impl WtOsc {
    pub fn new() -> WtOsc {
        WtOsc { phase: 0.0, last: 0.0, last2: 0.0, feedback: 0.0 }
    }
    pub fn set_feedback(&mut self, f: f32) {
        self.feedback = f.clamp(-1.0, 1.0);
    }
```

- [ ] **Step 4: Gate the const fast-path on `feedback==0`, add the feedback term to the general path**

In `WtOsc::process` (wavetable.rs:102): the const-freq/pmod fast path (line 112, `if let (Some(f), Some(pm)) = …`) hoists `mip_select` assuming a per-block-constant read phase. The feedback term changes the read phase every sample, so **only take the fast path when `feedback == 0.0`**:
```rust
// BEFORE
    if let (Some(f), Some(pm)) = (freq.as_const(), pmod.as_const()) {
// AFTER
    if self.feedback == 0.0 {
        if let (Some(f), Some(pm)) = (freq.as_const(), pmod.as_const()) {
            // ... entire existing fast-path body UNCHANGED (both simd + scalar sub-branches, the two `return;`s) ...
        }
    }
```
(Wrapping the existing block in `if self.feedback == 0.0 { … }` keeps every feedback=0 render on the exact current fast path. Do NOT edit the fast-path body.)

Then the general per-sample path (wavetable.rs:181) gains the feedback term + `last`/`last2` update:
```rust
// BEFORE
    for (i, s) in out.iter_mut().enumerate() {
        let dtp = freq.at(i) * dt;
        let mut ph = self.phase + pmod.at(i);
        ph -= floorf(ph);
        *s = sample_one(&mips, ph, dtp);
        self.phase += dtp;
        self.phase -= floorf(self.phase);
    }
// AFTER
    for (i, s) in out.iter_mut().enumerate() {
        let dtp = freq.at(i) * dt;
        let fb = self.feedback * 0.5 * (self.last + self.last2);
        let mut ph = self.phase + pmod.at(i) + fb;
        ph -= floorf(ph);
        let y = sample_one(&mips, ph, dtp);
        self.last2 = self.last;
        self.last = y;
        *s = y;
        self.phase += dtp;
        self.phase -= floorf(self.phase);
    }
```
(When `feedback==0`: `fb=0`, `ph` unchanged, `y=sample_one(...)` — identical; the `last` updates are dead. And feedback=0 takes the fast path anyway when freq/pmod are const, so this general-path change is only reached for audio-rate freq/pmod, where feedback=0 is still identical.)

Apply the **same** feedback term to `process_morph`'s per-sample read (find its `ph = self.phase + pmod.at(i)` line and add `+ fb` with the same `last`/`last2` update, gated identically if it has a const fast path). If `process_morph` has no const fast path, just add the feedback term to its per-sample loop.

- [ ] **Step 5: Add `PolyWt::set_feedback`**

`poly.rs`, `PolyWt` (poly.rs:676):
```rust
    pub fn set_feedback(&mut self, f: f32) {
        for w in &mut self.voices { w.set_feedback(f); }
    }
```

- [ ] **Step 6: Run tests, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels -- wtosc_feedback` (+ `--features simd`), then full crate both configs. Expected: PASS. Any existing `WtOsc`/`PolyWt` test must remain green (feedback defaults 0 → identical).

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-dsp-kernels/src/wavetable.rs crates/deluge-dsp-kernels/src/poly.rs
git commit -m "feat(kernels): WtOsc self-feedback (shared; mono Wavetable gains it), PolyWt::set_feedback; feedback=0 byte-identical"
```

---

### Task 3: Graph wiring — widen poly_in to 3, poly `pm`/`position` edges, feedback `set_param`

This task ALSO un-breaks `deluge-audio-graph` (broken since Task 1 changed `PolyOsc::process`'s arity).

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs` (widen `poly_in`/`poly_scratch` 2→3)
- Modify: `crates/deluge-audio-graph/src/node.rs` (`poly_in_count`, `poly_process` arms for `PolyOsc`+`PolyWt`, `set_param` arms, test-module assertions)
- Test: `node.rs` test module

**Interfaces:**
- Consumes: Task 1/2 kernel APIs (`PolyOsc::process(pitch,width,pm,dt,out)`, `PolyOsc/PolyWt/WtOsc::set_feedback`).
- Produces: `poly_in_count(PolyOsc)==3`, `poly_in_count(PolyWt)==3`; pm=poly port 2, PolyWt position=poly port 1; feedback params (PolyOsc/PolyWt param 1, mono Wt param 0). Consumed by Task 4.

- [ ] **Step 1: Write the failing tests**

`node.rs` test module:
```rust
#[test]
fn polyosc_polywt_pm_ports() {
    assert_eq!(Node::poly_in_count(Kind::PolyOsc), 3); // pitch,width,pm
    assert_eq!(Node::poly_in_count(Kind::PolyWt), 3);  // pitch,position,pm
}

#[test]
fn polyosc_feedback_setparam_reaches_kernel() {
    // set_param(1, x) is feedback (param 0 is shape); must build/drive without panic.
    let mut n = Node::new(Kind::PolyOsc, 0);
    n.set_param(1, 0.5); // feedback
    /* build poly_in tiles (pitch nonzero, width+pm zero) as the existing PolyOsc node
       test does (near node.rs:2008); call poly_process; assert all outputs finite */
}
```
> Copy the poly render-harness (poly_in tile construction, `poly_process` call) from the existing `PolyOsc` node test (near node.rs:2008) — note it must now provide THREE poly_in tiles. Defer PolyWt render coverage to Task 5's e2e if no pool-region node harness exists; note it.

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph -- polyosc_polywt_pm_ports polyosc_feedback_setparam`
Expected: FAIL to COMPILE (engine `poly_in` is `[_;2]`, `PolyOsc::process` arity, `poly_in_count` 2/1).

- [ ] **Step 3: Widen the engine poly arrays 2 → 3**

`engine.rs` (~line 246). The poly path builds `poly_in: [Option<&[f32]>; 2]` from a 2-slot `poly_scratch`; widen both to 3:
```rust
// BEFORE
let poly_in: [Option<&[f32]>; 2] = [
    if count > 0 { Some(poly_scratch[0].as_flattened()) } else { None },
    if count > 1 { Some(poly_scratch[1].as_flattened()) } else { None },
];
// AFTER
let poly_in: [Option<&[f32]>; 3] = [
    if count > 0 { Some(poly_scratch[0].as_flattened()) } else { None },
    if count > 1 { Some(poly_scratch[1].as_flattened()) } else { None },
    if count > 2 { Some(poly_scratch[2].as_flattened()) } else { None },
];
```
Find where `poly_scratch` is declared (a `[[f32; …]; 2]` or similar 2-element array of VOICES*BLOCK scratch buffers, and the resolve loop `for j in 0..Node::poly_in_count(kind)` at ~engine.rs:203 that fills it) and widen that array from 2 to 3 slots. `poly_process`'s signature already takes `poly_in: [Option<&[f32]>; N]` generically or as a fixed 2 — update its parameter type to `[Option<&[f32]>; 3]` in `node.rs` (the `poly_process` fn signature) to match. Grep for `[Option<&[f32]>; 2]` / `poly_scratch` across `engine.rs` + `node.rs` and bump every occurrence to 3.
> This widening is byte-neutral for existing nodes: `count <= 2` still yields `None` for the 3rd slot, and no current node reads `poly_in[2]` until Task 3's PolyOsc/PolyWt arms below.

- [ ] **Step 4: Bump `poly_in_count`**

`node.rs:293` `poly_in_count`: `PolyOsc` 2 → **3**; `PolyWt | PolyWtMorph` 1 → **3**. Update the assertion `poly_in_count(Kind::PolyOsc)==2` (node.rs:1839) → `== 3`, any PolyWt assertion → `== 3`.

- [ ] **Step 5: Thread `pm` (and PolyWt `position`) in the `poly_process` arms**

`PolyOsc` arm (node.rs:913) — `pm` is a whole tile (poly port 2), passed straight:
```rust
Kind::PolyOsc => {
    if let (State::PolyOsc(o), Some(pitch), Some(width), Some(pm)) =
        (&mut self.state, poly_in[0], poly_in[1], poly_in[2]) {
        o.process(pitch, width, pm, dt, out);
    }
}
```
(Un-patched `pm` (port 2) arrives as a broadcast zero tile — same mechanism as `width` — so byte-identical for existing patches.)

`PolyWt` arm (node.rs:1001) — it loops voices with per-lane `In` columns. Today: pitch from `poly_in[0]`, pmod from mono `ins[1]`, position (morph) from mono `ins[2]`. Change to: pm from **poly_in[2]**, position (morph) from **poly_in[1]** — both de-interleaved per lane:
```rust
if let (State::PolyWt(w), Some(pitch), Some(pos), Some(pm)) =
    (&mut self.state, poly_in[0], poly_in[1], poly_in[2]) {
    // ... existing per-frame region/levels resolution UNCHANGED ...
    let mut col = [0.0f32; MAX_BLOCK];
    let mut pmcol = [0.0f32; MAX_BLOCK];
    let mut poscol = [0.0f32; MAX_BLOCK]; // morph position, per lane
    let mut ocol = [0.0f32; MAX_BLOCK];
    for v in 0..VOICES {
        for i in 0..n {
            col[i] = pitch[i * VOICES + v];
            pmcol[i] = pm[i * VOICES + v];
            poscol[i] = pos[i * VOICES + v];
        }
        // single-cycle branch:
        w.process_voice(v, MipSet { levels: &levels }, In::A(&col[..n]), In::A(&pmcol[..n]), dt, &mut ocol[..n]);
        // morph branch (replaces ins[1]→pmcol, ins[2]→poscol):
        // w.process_voice_morph(v, region, frames, In::A(&col[..n]), In::A(&pmcol[..n]), In::A(&poscol[..n]), dt, &mut ocol[..n]);
        for i in 0..n { out[i * VOICES + v] = ocol[i]; }
    }
}
```
> Preserve the existing morph-vs-single-cycle branch structure; only swap the pmod arg (`ins[1]` → `In::A(&pmcol[..n])`) and the position arg (`ins[2]` → `In::A(&poscol[..n])`, morph only), and add the `poly_in[1]`/`poly_in[2]` bindings + de-interleave. Un-patched `pm`/`position` arrive as broadcast-zero tiles ⇒ `pmcol`/`poscol` all-zero. **Confirm the old mono defaults were also 0** (an unset `ins[1]`/`ins[2]` resolved to `Const(0.0)`): if so this is byte-identical; if the old position default was non-zero, match it by seeding `poscol` accordingly and NOTE it. (This drops PolyWt's old mono `ins[1]` pmod path; per the Osc-suite audit PolyWt had no pm setter and its mono pmod was not per-voice-reachable — grep to confirm nothing drove it.)

- [ ] **Step 6: Add feedback `set_param` arms**

`node.rs` set_param (`State::PolyOsc` near node.rs:480; mono `State::Osc` at node.rs:353):
- `PolyOsc`: extend its `match param` — add `1 => o.set_feedback(value)` (keep `0 => set_shape`).
- `PolyWt`: add `State::PolyWt(w) if param == 1 => w.set_feedback(value)`.
- mono `Wavetable`: add `State::Wt(o) if param == 0 => o.set_feedback(value)` (mirrors `State::Osc(o) if param == 0 => o.set_feedback(value)`). Confirm param 0 is unused by the mono wavetable node (position is input port 2, not a param).

- [ ] **Step 7: Run tests, both configs**

Targeted tests, then the full graph crate both configs (`cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` ± `--features deluge-dsp-kernels/simd`). Expected: PASS — existing PolyOsc/PolyWt/Wavetable render tests green (feedback defaults 0; pm/position broadcast zero).

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-audio-graph/src/engine.rs crates/deluge-audio-graph/src/node.rs
git commit -m "feat(graph): widen poly_in to 3, poly pm edge (port 2) + PolyWt position->poly port1, feedback set_param"
```

---

### Task 4: Wren `pm=`/`feedback=`/`position=` `poly_mode()`-aware routing

**Files:**
- Modify: `crates/deluge-wren-core/src/bindings_audio.rs` (`node_set_pm_impl`, `node_set_feedback_impl`, `node_set_position_impl`)
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `audio::poly_mode()` (the SAME flag `node_set_width_impl` uses), `audio::set_input`/`set_param`, `self_id`, `arg_input`, `get_f`. Port/param map from Task 3.
- Produces: `poly_mode()`-aware setters (no new Wren methods). Consumed by Task 5.

> **Pattern to copy verbatim:** `node_set_width_impl` (bindings_audio.rs:2474) already does `let port = if audio::poly_mode() { 1 } else { 2 }; audio::set_input(self_id(vm), port, v);`. All three setters below follow this exact shape — `poly_mode()` is true inside a `Synth.new{}` block (poly node), false at top level (mono).

- [ ] **Step 1: Write the failing tests**

`tests/audio_bindings.rs` (mirror the existing `osc_pm_emits_setinput_port1` / `osc_feedback_emits_setparam` command-capture tests — find them and copy their harness):
```rust
#[test]
fn poly_osc_pm_emits_setinput_port2() {
    // Inside Synth (poly_mode true), `c.pm = m` emits SetInput on port 2, not 1.
    /* build: Synth.new { |p| var m = Osc.sine(p); var c = Osc.sine(p); c.pm = m; c }
       assert a SetInput{port: 2} command was emitted for the carrier */
}
#[test]
fn poly_osc_feedback_emits_setparam_param1() {
    /* Synth.new { |p| var c = Osc.sine(p); c.feedback = 0.4; c }
       assert SetParam{param: 1} for the carrier */
}
#[test]
fn mono_osc_pm_feedback_unchanged() {
    // Top-level (poly_mode false): still port 1 / param 0 (non-regression).
    /* var c = Osc.sine(440); c.pm = Osc.sine(110); c.feedback = 0.4
       assert SetInput{port:1} and SetParam{param:0} */
}
```
> Use the exact command-capture assertions the existing `osc_pm_emits_setinput_port1` / `osc_feedback_emits_setparam` tests use. Use the Synth-body poly harness from the Sy-suite tests for the poly cases (so `poly_mode()` is true when the setter runs).

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- poly_osc_pm poly_osc_feedback mono_osc_pm`
Expected: the poly cases FAIL (setters still emit port 1 / param 0 unconditionally).

- [ ] **Step 3: Make the three setters `poly_mode()`-aware**

`bindings_audio.rs`, mirroring `node_set_width_impl`:
```rust
// node_set_pm_impl (bindings_audio.rs:2456)
pub(crate) fn node_set_pm_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    let port = if audio::poly_mode() { 2 } else { 1 }; // poly pm = port 2, mono = port 1
    audio::set_input(self_id(vm), port, v);
}
// node_set_feedback_impl (bindings_audio.rs:2495)
pub(crate) fn node_set_feedback_impl<S: SlotApi>(vm: &S) {
    let v = vm.get_f(1) as f32;
    let param = if audio::poly_mode() { 1 } else { 0 }; // poly feedback = param 1, mono = param 0
    audio::set_param(self_id(vm), param, v);
}
// node_set_position_impl (bindings_audio.rs:~2485) — poly PolyWt position = poly port 1
pub(crate) fn node_set_position_impl<S: SlotApi>(vm: &S) {
    let v = arg_input(vm, 1);
    let port = if audio::poly_mode() { 1 } else { 2 }; // poly position = port 1, mono = port 2
    audio::set_input(self_id(vm), port, v);
}
```
> If `audio::poly_mode()` isn't already re-exported for these fns, it is (node_set_width_impl calls it) — use the identical path. `self_id(vm)` reads the receiver NodeObj id at slot 0 (safe slot-0 receiver read, see [[wren-binding-safety]]).

- [ ] **Step 4: Run tests, both configs**

Targeted tests, then the full wren-core suite both configs. Expected: PASS — mono cases unchanged (pm port 1 / feedback param 0 / position port 2), poly cases now pm port 2 / feedback param 1 / position port 1. Existing mono `Osc.pm=`/`.feedback=` and mono `Wavetable.position=` tests stay green.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-wren-core/src/bindings_audio.rs crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "feat(wren): poly_mode()-aware pm=/feedback=/position= routing (mirrors width=)"
```

---

### Task 5: End-to-end poly FM

**Files:**
- Test: `crates/deluge-wren-core/tests/audio_bindings.rs`

**Interfaces:**
- Consumes: `Osc.sine`/`Osc.wavetable` inside `Synth.new`/`Synth.mono`, `.pm=`, `.feedback=`, `noteOn`, `run_and_render`/`run_script_ok`, `Wavetable.from`.

- [ ] **Step 1: Write the e2e tests**

`tests/audio_bindings.rs` (mirror the Sy/FM render harness — `run_and_render` returns a `StereoFrame` buffer):
```rust
#[test]
fn poly_sine_fm_plays_in_synth() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var s = Synth.new { |p|\n  var m = Osc.sine(p)\n  var c = Osc.sine(p)\n  c.pm = m * 3\n  c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)\ns.noteOn(64, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0), "bounded/finite");
    assert!(out.iter().any(|f| f.l.abs() > 1e-3), "poly FM sounds");
}

#[test]
fn poly_sine_fm_feedback_plays() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var s = Synth.new { |p|\n  var c = Osc.sine(p)\n  c.feedback = 0.6\n  c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite()) && out.iter().any(|f| f.l.abs() > 1e-3), "feedback sounds");
}

#[test]
fn poly_wavetable_fm_plays() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var wt = Wavetable.from([0.0, 0.7, 1.0, 0.7, 0.0, -0.7, -1.0, -0.7])\nvar s = Synth.new { |p|\n  var m = Osc.sine(p)\n  var c = Osc.wavetable(wt, p)\n  c.pm = m * 2\n  c * Env.adsr(0.001, 0.5, 1, 0.2)\n}\nOut.patch(s.out)\ns.noteOn(60, 100)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite()) && out.iter().any(|f| f.l.abs() > 1e-3), "poly wavetable FM sounds");
}

#[test]
fn mono_wavetable_feedback_plays() {
    let mut out = [StereoFrame::default(); 64];
    run_and_render(
        "var wt = Wavetable.from([0.0, 0.7, 1.0, 0.7, 0.0, -0.7, -1.0, -0.7])\nvar c = Osc.wavetable(wt, 220)\nc.feedback = 0.5\nOut.patch(c)",
        &mut out,
    );
    assert!(out.iter().all(|f| f.l.is_finite() && f.l.abs() <= 8.0) && out.iter().any(|f| f.l.abs() > 1e-3), "mono wavetable feedback bounded + sounds");
}
```
> Confirm the exact Wren surface against existing tests: `Osc.wavetable(wt, freq)`, `Env.adsr`, `Synth.new { |p| … }`, `s.out`, `s.noteOn`, `Wavetable.from`. Adapt names if the existing FM/Sy tests differ (e.g. `Env.ar`). If `Osc.wavetable` needs a poly context detail, mirror the existing poly wavetable synth test.

- [ ] **Step 2: Run + non-breaking check, both configs**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-wren-core -- poly_sine_fm poly_wavetable_fm mono_wavetable_feedback` (+ `--features deluge-dsp-kernels/simd`).
Then the FULL wren-core suite both configs. Expected: PASS, with all pre-existing poly-synth/oscillator tests green (the non-breaking proof: no existing test patches pm/feedback, and the kernel identity tests from Tasks 1–2 lock `pm=0,fb=0` byte-identity).

- [ ] **Step 3: Commit**

```bash
git add crates/deluge-wren-core/tests/audio_bindings.rs
git commit -m "test(wren): poly FM e2e — poly sine/wavetable FM + feedback render in Synth; mono wavetable feedback"
```

---

## Self-Review

**Spec coverage:** Every spec section maps to a task — PolyOsc kernel (T1), WtOsc feedback + PolyWt::set_feedback (T2), engine poly_in widen + graph poly_in_count/poly_process/set_param (T3), `poly_mode()`-aware Wren routing incl. `position=` (T4), e2e incl. mono Wavetable feedback + non-breaking (T5). The locked indices (pm=poly port 2, PolyWt position=poly port 1, feedback=param 1 poly / param 0 mono; poly_in_count PolyOsc 3 / PolyWt 3; engine poly_in widened 2→3) are identical across the Interfaces block and every task.

**Placeholder scan:** Kernel code (the risk) is complete and verbatim. The graph/binding/e2e steps that say "copy the existing harness" point at a NAMED existing test/fn to mirror (poly render harness near node.rs:2008; `osc_pm_emits_setinput_port1`; `node_set_width_impl` for the setter pattern; `run_and_render`) rather than leaving logic undefined — transcription guidance, not a design gap. The `MipSet` setup, `poly_scratch` array location, and exact Wren method names are "copy/grep from the existing code," because inventing them risks divergence.

**Type consistency:** `PolyOsc::process(pitch,width,pm,dt,out)` + `set_feedback` (T1) is consumed with the same signature in T3's poly_process arm. `WtOsc::set_feedback`/`PolyWt::set_feedback` (T2) → set_param arms (T3). T3 widens `poly_in` to `[Option<&[f32]>; 3]` (engine + `poly_process` signature must agree). Setter routing (T4) uses `audio::poly_mode()` — the SAME mechanism as `node_set_width_impl` — and its port/param map matches T3 exactly (pm poly 2 / mono 1; feedback poly 1 / mono 0; position poly 1 / mono 2). No `NodeObj` discriminator is used (poly_mode() supplies the mono-vs-poly split).

**Correction note:** an earlier draft used a "reserved PolyWt port 1" + `NodeObj.poly` routing; that collided with PolyWt's morph `position` (mono port 2) and mis-modeled the setter mechanism. Corrected here to: PolyWt position moves to poly port 1, pm unified at poly port 2, engine `poly_in` widened 2→3, and all setters route on `poly_mode()` (mirroring the existing `width=`).
