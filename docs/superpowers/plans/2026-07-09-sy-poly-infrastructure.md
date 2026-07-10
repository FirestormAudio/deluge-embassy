# Sy-1: Polyphony Infrastructure + VoiceSum Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Teach the audio graph to carry `VOICES = 8` voice-lanes through a poly region and collapse them to mono at a `VoiceSum` boundary, proven by `PolyCtrl → PolyOsc → VoiceSum`.

**Architecture:** Three new kernels (`PolyCtrl`, `PolyOsc`, `voice_sum`) operating on **voice-interleaved** flat tiles (`tile[i*VOICES + v]`). Poly nodes reserve `VOICES` contiguous arena rows (reusing `out_width`/`out_base`) and are dispatched through a new isolated engine path (`Node::poly_process`) — `process_resolved` and its ~30 callers are untouched. Task 4 adds the `f32x8` fast path for `PolyOsc` (the interleaved layout makes an `f32x8` load one SIMD step) behind the `simd` feature, null-tested against the scalar oracle.

**Tech Stack:** Rust `no_std` (`deluge-dsp-kernels`), the audio graph/engine (`deluge-audio-graph`), `proptest`.

## Global Constraints

- `no_std`, no heap, pure `f32`, deterministic. Poly nodes are mono-per-voice (no stereo×poly).
- **`VOICES = 8`**, defined once in `deluge_dsp_kernels::poly` and re-exported by `deluge-audio-graph`.
- **Voice-interleaved (sample-major) layout is mandatory:** every poly tile is a flat `&[f32]` of length `VOICES * n_samples`, indexed `tile[i * VOICES + v]` (voice `v` at sample `i`). Never index a poly buffer per-row-as-lane. This layout is what lets the voice loop vectorize to `f32x8` on NEON.
- Host tests run per-crate on x86: `cargo test --target x86_64-unknown-linux-gnu -p <crate>`. Never `--workspace` on host (firmware crates are ARM-only).
- LSP "can't find crate for `test`/`std`" armv7a diagnostics are noise — ignore.
- No Wren surface in Sy-1 (deferred to Sy-4). No panics on the audio path — a dangling/short poly edge resolves to silence.

---

### Task 1: `poly.rs` kernels — `VOICES`, `PolyCtrl`, `PolyOsc`, `voice_sum`

**Files:**
- Create: `crates/deluge-dsp-kernels/src/poly.rs`
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `pub mod poly;`)

**Interfaces:**
- Consumes: crate-root `crate::floorf`, `crate::fast_sin`.
- Produces:
  - `pub const VOICES: usize = 8;`
  - `pub struct PolyCtrl` — `new()`, `set_voice(&mut self, v: usize, x: f32)`, `process(&mut self, out: &mut [f32])`.
  - `pub struct PolyOsc` — `new()`, `process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32])`.
  - `pub fn voice_sum(tile: &[f32], out: &mut [f32])`.
  - All tiles voice-interleaved, length `VOICES * out_samples`.

- [ ] **Step 1: Create `poly.rs` with the kernels + tests**

Create `crates/deluge-dsp-kernels/src/poly.rs`:

```rust
//! Polyphony kernels: a per-voice settable source (`PolyCtrl`), a poly
//! oscillator (`PolyOsc`), and the voice→mono collapse (`voice_sum`). All poly
//! buffers are voice-interleaved (sample-major): `tile[i * VOICES + v]` is voice
//! `v` at sample `i`, so the voice loop vectorizes to `f32x8` on NEON.

use crate::{fast_sin, floorf};

/// Voices processed in parallel per poly node. Fixed at compile time.
pub const VOICES: usize = 8;

/// A per-voice settable scalar source: lane `v` outputs `values[v]`. No input.
/// The allocator's write-target (Sy-3). Output tile is voice-interleaved.
#[derive(Clone, Copy)]
pub struct PolyCtrl {
    values: [f32; VOICES],
}
impl PolyCtrl {
    pub fn new() -> PolyCtrl {
        PolyCtrl { values: [0.0; VOICES] }
    }
    pub fn set_voice(&mut self, v: usize, x: f32) {
        if v < VOICES {
            self.values[v] = x;
        }
    }
    /// `out` is voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                out[i * VOICES + v] = self.values[v];
            }
        }
    }
}
impl Default for PolyCtrl {
    fn default() -> Self {
        Self::new()
    }
}

/// A poly oscillator. `pitch` is a voice-interleaved tile of per-voice Hz;
/// writes a voice-interleaved audio tile. SoA phase; voice loop is the inner
/// (vectorizable) dimension. Raw sine shape (band-limiting is a later concern).
#[derive(Clone, Copy)]
pub struct PolyOsc {
    phase: [f32; VOICES], // [0,1) per voice
}
impl PolyOsc {
    pub fn new() -> PolyOsc {
        PolyOsc { phase: [0.0; VOICES] }
    }
    /// `pitch` and `out` are voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]) {
        let n = out.len() / VOICES;
        for i in 0..n {
            for v in 0..VOICES {
                let f = pitch[i * VOICES + v];
                let mut p = self.phase[v] + f * dt;
                p -= floorf(p); // wrap [0,1)
                self.phase[v] = p;
                out[i * VOICES + v] = fast_sin(p);
            }
        }
    }
}
impl Default for PolyOsc {
    fn default() -> Self {
        Self::new()
    }
}

/// Collapse a voice-interleaved tile to mono: `out[i] = Σ_v tile[i*VOICES + v]`.
/// `tile.len() == VOICES * out.len()`.
pub fn voice_sum(tile: &[f32], out: &mut [f32]) {
    for i in 0..out.len() {
        let mut s = 0.0;
        for v in 0..VOICES {
            s += tile[i * VOICES + v];
        }
        out[i] = s;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::{fast_sin, floorf}; // explicit so tests build under `--features simd` too

    #[test]
    fn polyctrl_fills_interleaved_lanes() {
        let mut c = PolyCtrl::new();
        for v in 0..VOICES {
            c.set_voice(v, v as f32 + 1.0); // 1..=8
        }
        let n = 5;
        let mut out = std::vec![0.0f32; VOICES * n];
        c.process(&mut out);
        for i in 0..n {
            for v in 0..VOICES {
                assert_eq!(out[i * VOICES + v], v as f32 + 1.0, "lane {v} sample {i}");
            }
        }
    }

    #[test]
    fn polyctrl_ignores_out_of_range_voice() {
        let mut c = PolyCtrl::new();
        c.set_voice(VOICES, 99.0); // no-op, no panic
        let mut out = std::vec![0.0f32; VOICES];
        c.process(&mut out);
        assert!(out.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn polyosc_renders_independent_per_voice_partials() {
        // Voice v runs at (v+1)*100 Hz. Each lane must equal a reference sine at
        // its own frequency, and lanes must not cross-contaminate.
        let dt = 1.0 / 48_000.0;
        let n = 480;
        let mut pitch = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                pitch[i * VOICES + v] = (v as f32 + 1.0) * 100.0;
            }
        }
        let mut osc = PolyOsc::new();
        let mut out = std::vec![0.0f32; VOICES * n];
        osc.process(&pitch, dt, &mut out);
        // Reference: independent phase accumulators per voice.
        let mut ph = [0.0f32; VOICES];
        for i in 0..n {
            for v in 0..VOICES {
                let f = (v as f32 + 1.0) * 100.0;
                ph[v] += f * dt;
                ph[v] -= floorf(ph[v]);
                let want = fast_sin(ph[v]);
                assert!((out[i * VOICES + v] - want).abs() < 1e-5, "voice {v} sample {i}");
            }
        }
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 1.0001));
    }

    #[test]
    fn voice_sum_adds_lanes_per_sample() {
        let n = 4;
        let mut tile = std::vec![0.0f32; VOICES * n];
        for i in 0..n {
            for v in 0..VOICES {
                tile[i * VOICES + v] = (i * VOICES + v) as f32; // distinct
            }
        }
        let mut out = std::vec![0.0f32; n];
        voice_sum(&tile, &mut out);
        for i in 0..n {
            let want: f32 = (0..VOICES).map(|v| (i * VOICES + v) as f32).sum();
            assert_eq!(out[i], want, "sample {i}");
        }
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn polyosc_bounded(f in 0.0f32..8000.0) {
            let dt = 1.0 / 48_000.0;
            let n = 64;
            let pitch = std::vec![f; VOICES * n];
            let mut osc = PolyOsc::new();
            let mut out = std::vec![0.0f32; VOICES * n];
            osc.process(&pitch, dt, &mut out);
            for &s in &out {
                prop_assert!(s.is_finite() && s.abs() <= 1.0001, "polyosc unbounded: {s}");
            }
        }
    }
}
```

- [ ] **Step 2: Register the module in `lib.rs`**

In `crates/deluge-dsp-kernels/src/lib.rs`, add `pub mod poly;` in alphabetical
order (after `pub mod osc;`, before `pub mod quant;`).

- [ ] **Step 3: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly`
Expected: PASS — `polyctrl_fills_interleaved_lanes`, `polyctrl_ignores_out_of_range_voice`,
`polyosc_renders_independent_per_voice_partials`, `voice_sum_adds_lanes_per_sample`,
`polyosc_bounded`. No warnings.

- [ ] **Step 4: Commit**

```bash
git add crates/deluge-dsp-kernels/src/poly.rs crates/deluge-dsp-kernels/src/lib.rs
git commit -m "feat(dsp-kernels): poly kernels — PolyCtrl / PolyOsc / voice_sum"
```

---

### Task 2: Graph node layer — poly kinds, widths, `poly_process` dispatch

**Files:**
- Modify: `crates/deluge-audio-graph/src/node.rs`
- Modify: `crates/deluge-audio-graph/src/lib.rs` (re-export `VOICES`)

**Interfaces:**
- Consumes: `deluge_dsp_kernels::poly::{PolyCtrl, PolyOsc, VOICES, voice_sum}` (Task 1).
- Produces on `Node`:
  - `Kind::PolyCtrl`, `Kind::PolyOsc`, `Kind::VoiceSum`.
  - `Node::out_width(Kind::PolyCtrl | Kind::PolyOsc) == VOICES`; `VoiceSum == 1`.
  - `pub fn is_poly(kind: Kind) -> bool` (true for the three poly kinds).
  - `pub fn has_poly_in(kind: Kind) -> bool` (true for `PolyOsc`, `VoiceSum`).
  - `pub fn poly_process(&mut self, poly_in: Option<&[f32]>, dt: f32, out: &mut [f32])`.
  - `set_param` on `PolyCtrl`: `param` = voice index.
- `deluge-audio-graph` re-exports `pub use deluge_dsp_kernels::poly::VOICES;`.

- [ ] **Step 1: Add imports and the `VOICES` re-export**

In `node.rs`, add to the `use deluge_dsp_kernels::{ ... }` block (next to
`quant::{...}`):

```rust
    poly::{voice_sum, PolyCtrl, PolyOsc, VOICES},
```

In `crates/deluge-audio-graph/src/lib.rs`, add near the other `pub use`s:

```rust
pub use deluge_dsp_kernels::poly::VOICES;
```

- [ ] **Step 2: Add `Kind` and `State` variants**

In `pub enum Kind`, after `Ctrl,` (the last entry) add:

```rust
    PolyCtrl,
    PolyOsc,
    VoiceSum,
```

In `enum State`, after the last kernel variant (before `Stateless,`) add:

```rust
    PolyCtrl(PolyCtrl),
    PolyOsc(PolyOsc),
```

(`VoiceSum` is stateless — no `State` variant.)

- [ ] **Step 3: Construction, width, and poly predicates**

In `Node::new`, add `Kind::VoiceSum` to the `State::Stateless` arm
(`Kind::Mul | ... | Kind::Curve | Kind::VoiceSum => State::Stateless,`) and add:

```rust
            Kind::PolyCtrl => State::PolyCtrl(PolyCtrl::new()),
            Kind::PolyOsc => State::PolyOsc(PolyOsc::new()),
```

Change `out_width` to give poly sources `VOICES` rows:

```rust
    pub fn out_width(kind: Kind) -> usize {
        match kind {
            Kind::Split2 | Kind::Pan | Kind::Chorus | Kind::Flanger | Kind::Room | Kind::Hall | Kind::Plate => 2,
            Kind::PolyCtrl | Kind::PolyOsc => VOICES,
            _ => 1,
        }
    }
```

Add the two predicates (next to `out_width`):

```rust
    /// A poly node carries `VOICES` voice-lanes and is dispatched via
    /// `poly_process`, not `process_resolved`.
    pub fn is_poly(kind: Kind) -> bool {
        matches!(kind, Kind::PolyCtrl | Kind::PolyOsc | Kind::VoiceSum)
    }

    /// True if the node consumes a poly input tile on port 0.
    pub fn has_poly_in(kind: Kind) -> bool {
        matches!(kind, Kind::PolyOsc | Kind::VoiceSum)
    }
```

- [ ] **Step 4: `set_param` for `PolyCtrl` and the `poly_process` dispatch**

In `set_param`, before the final `_ => {}`, add:

```rust
            State::PolyCtrl(c) => c.set_voice(param as usize, value),
```

Add the poly dispatch method (near `process_resolved`):

```rust
    /// Dispatch a poly node. `poly_in` is the voice-interleaved input tile
    /// (`Some` when `has_poly_in`), `out` is the writable region: a
    /// `VOICES * BLOCK` tile for poly-output kinds, or `BLOCK` for `VoiceSum`.
    pub fn poly_process(&mut self, poly_in: Option<&[f32]>, dt: f32, out: &mut [f32]) {
        match self.kind {
            Kind::PolyCtrl => {
                if let State::PolyCtrl(c) = &mut self.state {
                    c.process(out);
                }
            }
            Kind::PolyOsc => {
                if let (State::PolyOsc(o), Some(pin)) = (&mut self.state, poly_in) {
                    o.process(pin, dt, out);
                }
            }
            Kind::VoiceSum => {
                if let Some(pin) = poly_in {
                    voice_sum(pin, out);
                }
            }
            _ => {}
        }
    }
```

- [ ] **Step 5: Write the node-layer tests**

In `node.rs`'s `#[cfg(test)] mod tests`, after the last node test, add:

```rust
    #[test]
    fn poly_widths_and_predicates() {
        assert_eq!(Node::out_width(Kind::PolyCtrl), VOICES);
        assert_eq!(Node::out_width(Kind::PolyOsc), VOICES);
        assert_eq!(Node::out_width(Kind::VoiceSum), 1);
        assert!(Node::is_poly(Kind::PolyCtrl) && Node::is_poly(Kind::PolyOsc) && Node::is_poly(Kind::VoiceSum));
        assert!(!Node::is_poly(Kind::Saw));
        assert!(Node::has_poly_in(Kind::PolyOsc) && Node::has_poly_in(Kind::VoiceSum));
        assert!(!Node::has_poly_in(Kind::PolyCtrl));
    }

    #[test]
    fn polyctrl_node_fills_and_voicesum_collapses() {
        // PolyCtrl(set voices) → tile → VoiceSum → mono sum.
        let n = 4;
        let mut ctrl = Node::new(Kind::PolyCtrl, 0);
        for v in 0..VOICES {
            ctrl.set_param(v as u8, (v + 1) as f32); // 1..=8
        }
        let mut tile = [0.0f32; VOICES * 4];
        ctrl.poly_process(None, 1.0 / 48_000.0, &mut tile);
        for i in 0..n {
            for v in 0..VOICES {
                assert_eq!(tile[i * VOICES + v], (v + 1) as f32);
            }
        }
        let mut sum = Node::new(Kind::VoiceSum, 0);
        let mut mono = [0.0f32; 4];
        sum.poly_process(Some(&tile), 1.0 / 48_000.0, &mut mono);
        let want: f32 = (1..=VOICES).map(|x| x as f32).sum(); // 36
        assert!(mono.iter().all(|&s| (s - want).abs() < 1e-4), "each sample sums to {want}");
    }
```

- [ ] **Step 6: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph poly`
Expected: PASS — `poly_widths_and_predicates`, `polyctrl_node_fills_and_voicesum_collapses`.
Then run the full crate to confirm no regressions:
`cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph` — all pass, no warnings from `node.rs`.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/node.rs crates/deluge-audio-graph/src/lib.rs
git commit -m "feat(audio-graph): PolyCtrl / PolyOsc / VoiceSum node layer + poly_process"
```

---

### Task 3: Engine integration — poly-edge resolution + dispatch

**Files:**
- Modify: `crates/deluge-audio-graph/src/engine.rs`

**Interfaces:**
- Consumes: `Node::is_poly`, `Node::has_poly_in`, `Node::poly_process`,
  `Node::out_width` (Task 2); `deluge_dsp_kernels::poly::VOICES`.
- Produces: `render_block` evaluates poly nodes — poly sources write their
  `VOICES`-row tile; poly consumers read the referenced source tile into a
  voice-interleaved `poly_scratch` and dispatch via `poly_process`. Mono/stereo
  nodes keep the existing `process_resolved` path.

- [ ] **Step 1: Import `VOICES`**

In `engine.rs`, add near the other imports:

```rust
use deluge_dsp_kernels::poly::VOICES;
```

- [ ] **Step 2: Fetch `kind` in the per-node tuple**

In `render_block`, the tuple that snapshots node info currently reads:

```rust
            let (base, width, inputs, table_src) = {
                let n = self.arena.node(id).expect("eval-order node exists");
                (n.out_base as usize, Node::out_width(n.kind), n.inputs_snapshot(), n.table_src())
            };
```

Change it to also capture `kind`:

```rust
            let (base, kind, width, inputs, table_src) = {
                let n = self.arena.node(id).expect("eval-order node exists");
                (n.out_base as usize, n.kind, Node::out_width(n.kind), n.inputs_snapshot(), n.table_src())
            };
```

- [ ] **Step 3: Declare `poly_scratch` and fill it in the read block**

Immediately after the `let mut scratch = [[0.0f32; BLOCK]; MAX_INPUTS];` line, add:

```rust
            let mut poly_scratch = [[0.0f32; BLOCK]; VOICES];
```

Inside the existing immutable-read block (the `{ let arr = unsafe { &*self.outs.get() }; ... }` that fills `scratch`), after the `for (p, row) in scratch.iter_mut()...` loop but still inside the block, add the poly-input resolution:

```rust
                if Node::has_poly_in(kind) {
                    // Port 0 references the poly source; copy its VOICES rows
                    // (which store the interleaved tile) verbatim — row-wise copy
                    // preserves the flat interleaved layout. Dangling/short → silence.
                    match inputs[0] {
                        Input::Node { node, .. } => match self.arena.out_base(node) {
                            Some(sbase) if sbase + VOICES <= OUTS => {
                                for v in 0..VOICES {
                                    poly_scratch[v] = arr[sbase + v];
                                }
                            }
                            _ => {
                                for v in 0..VOICES {
                                    poly_scratch[v] = [0.0; BLOCK];
                                }
                            }
                        },
                        _ => {
                            for v in 0..VOICES {
                                poly_scratch[v] = [0.0; BLOCK];
                            }
                        }
                    }
                }
```

- [ ] **Step 4: Branch the dispatch (poly vs mono)**

Replace the dispatch tail — currently:

```rust
            let arr = unsafe { &mut *self.outs.get() };
            let mut view = OutView::from_arena::<OUTS, BLOCK>(arr, base, width);
            let pool_region: Option<&mut [f32]> = match table_src {
                Some(crate::node::TableSrc::Pooled(h)) => Some(self.pool.slice_mut(h)),
                _ => None,
            };
            if let Some(n) = self.arena.node_mut(id) {
                n.process_resolved(&ins, self.dt, &mut view, pool_region);
            }
```

with a poly branch around it:

```rust
            let arr = unsafe { &mut *self.outs.get() };
            if Node::is_poly(kind) {
                // Poly path: voice-interleaved tile in/out, isolated dispatch.
                let poly_in: Option<&[f32]> =
                    if Node::has_poly_in(kind) { Some(poly_scratch.as_flattened()) } else { None };
                let out = arr[base..base + width].as_flattened_mut(); // width*BLOCK
                if let Some(n) = self.arena.node_mut(id) {
                    n.poly_process(poly_in, self.dt, out);
                }
            } else {
                let mut view = OutView::from_arena::<OUTS, BLOCK>(arr, base, width);
                let pool_region: Option<&mut [f32]> = match table_src {
                    Some(crate::node::TableSrc::Pooled(h)) => Some(self.pool.slice_mut(h)),
                    _ => None,
                };
                if let Some(n) = self.arena.node_mut(id) {
                    n.process_resolved(&ins, self.dt, &mut view, pool_region);
                }
            }
```

- [ ] **Step 5: Write the end-to-end engine tests**

In `engine.rs`'s `#[cfg(test)] mod tests`, add (OUTS must fit `PolyCtrl` 8 +
`PolyOsc` 8 + `VoiceSum` 1 = 17 rows, so use OUTS = 32):

```rust
    #[test]
    fn poly_chain_sums_eight_voices_in_phase() {
        // PolyCtrl(all voices = same freq) → PolyOsc → VoiceSum.
        // All 8 lanes are identical (phase starts 0), so the sum peaks near 8×.
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: 440.0 });
        }
        e.create(NodeId(1), Kind::PolyOsc);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        let peak = out.iter().cloned().fold(0.0f32, |m, s| m.max(s.abs()));
        assert!(out.iter().all(|s| s.is_finite()), "finite");
        assert!(peak > 7.0 && peak <= 8.001, "8 in-phase voices sum near 8×: peak {peak}");
    }

    #[test]
    fn poly_distinct_voices_partially_cancel() {
        // Distinct per-voice frequencies → lanes drift out of phase → the sum's
        // peak stays well below 8× (proving voices are independent, not cloned).
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(0), Kind::PolyCtrl);
        for v in 0..VOICES {
            e.apply(Cmd::SetParam { node: NodeId(0), param: v as u8, value: (v as f32 + 1.0) * 300.0 });
        }
        e.create(NodeId(1), Kind::PolyOsc);
        *e.node_input_mut(NodeId(1), 0).unwrap() = Input::Node { node: NodeId(0), port: 0 };
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(1), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        let peak = out.iter().cloned().fold(0.0f32, |m, s| m.max(s.abs()));
        assert!(out.iter().all(|s| s.is_finite() && s.abs() <= 8.001));
        assert!(peak < 7.0, "distinct voices don't all align: peak {peak}");
    }

    #[test]
    fn poly_dangling_input_is_silent() {
        // A VoiceSum whose poly input references a non-existent node → silence.
        type PE = Engine<64, 8, 32, 4, 45056, 2048>;
        let mut e = PE::new(48_000.0);
        e.create(NodeId(2), Kind::VoiceSum);
        *e.node_input_mut(NodeId(2), 0).unwrap() = Input::Node { node: NodeId(5), port: 0 };
        e.render_block();
        let out = e.node_output(NodeId(2), 0);
        assert!(out.iter().all(|&s| s == 0.0), "dangling poly edge → silence, no panic");
    }
```

- [ ] **Step 6: Run the tests**

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-audio-graph`
Expected: PASS — the full crate suite plus `poly_chain_sums_eight_voices_in_phase`,
`poly_distinct_voices_partially_cancel`, `poly_dangling_input_is_silent`. No warnings.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-audio-graph/src/engine.rs
git commit -m "feat(audio-graph): engine poly-edge resolution + poly_process dispatch"
```

---

### Task 4: `f32x8` fast path for `PolyOsc` (behind the `simd` feature)

**Files:**
- Modify: `crates/deluge-dsp-kernels/src/lib.rs` (add `fast_sin_x8`)
- Modify: `crates/deluge-dsp-kernels/src/poly.rs` (`PolyOsc::process` SIMD path, guard, null test)

**Interfaces:**
- Consumes: `crate::fast_sin` (scalar oracle), `core::simd::f32x8` (nightly `portable_simd`, already enabled by the `simd` feature).
- Produces: `#[cfg(feature = "simd")] pub fn fast_sin_x8(p: f32x8) -> f32x8`; a
  `PolyOsc::process` that uses `f32x8` when `--features simd`, scalar otherwise.

**Context:** The default toolchain is nightly; `cargo test --features simd`
builds `portable_simd` on the host (→ SSE) and on device (→ NEON, 8 voices = 2×
`float32x4_t`). The crate convention (`math.rs`) is a scalar path + a
`#[cfg(feature = "simd")]` fast path that must agree — the scalar path is the
correctness oracle. Only `PolyOsc` is hand-vectorized: `PolyCtrl` (broadcast)
and `voice_sum` (reduction) auto-vectorize, so hand-SIMD there is YAGNI.

- [ ] **Step 1: Add `fast_sin_x8` to `lib.rs`**

After the scalar `fast_sin` in `crates/deluge-dsp-kernels/src/lib.rs`, add the
branchless 8-lane counterpart (identical polynomial, so it matches lane-for-lane):

```rust
/// SIMD counterpart of [`fast_sin`]: 8 phases at once, branchless (the `x > π`
/// wrap becomes a lanewise `select`). Matches `fast_sin` to f32 rounding.
/// On NEON this is 2× `float32x4_t`.
#[cfg(feature = "simd")]
#[inline]
pub fn fast_sin_x8(p: core::simd::f32x8) -> core::simd::f32x8 {
    use core::f32::consts::PI;
    use core::simd::prelude::*;
    let pi = f32x8::splat(PI);
    let two_pi = f32x8::splat(2.0 * PI);
    let mut x = f32x8::splat(2.0 * PI) * p;
    x = x.simd_gt(pi).select(x - two_pi, x); // if x > π { x -= 2π }
    let b = f32x8::splat(4.0 / PI);
    let c = f32x8::splat(-4.0 / (PI * PI));
    let y = b * x + c * x * x.abs();
    f32x8::splat(0.225) * (y * y.abs() - y) + y
}
```

- [ ] **Step 2: Gate the scalar imports and split `PolyOsc::process`**

In `poly.rs`, change the module import so the scalar helpers are only pulled in
for the non-SIMD build (they are unused on the SIMD path):

```rust
#[cfg(not(feature = "simd"))]
use crate::{fast_sin, floorf};
```

Add a compile-time guard that the SIMD path's `f32x8` matches `VOICES` (put it
just above the `PolyOsc` struct):

```rust
// The f32x8 poly path assumes exactly 8 voices. Changing VOICES requires
// revisiting the SIMD width (e.g. f32x16 or 2× f32x8).
#[cfg(feature = "simd")]
const _: () = assert!(VOICES == 8);
```

Replace `PolyOsc::process`'s body with a feature-split (the scalar arm is the
Task-1 body verbatim; the SIMD arm keeps phase register-resident as one `f32x8`):

```rust
    /// `pitch` and `out` are voice-interleaved, length `VOICES * n_samples`.
    pub fn process(&mut self, pitch: &[f32], dt: f32, out: &mut [f32]) {
        #[cfg(feature = "simd")]
        {
            use core::simd::prelude::*;
            let n = out.len() / VOICES;
            let one = f32x8::splat(1.0);
            let dtv = f32x8::splat(dt);
            let mut ph = f32x8::from_array(self.phase);
            for i in 0..n {
                let f = f32x8::from_slice(&pitch[i * VOICES..]);
                let mut p = ph + f * dtv;
                // wrap: p -= trunc-floor(p), matching scalar `floorf`
                let t: f32x8 = p.cast::<i32>().cast::<f32>();
                let fl = t.simd_gt(p).select(t - one, t);
                p -= fl;
                ph = p;
                crate::fast_sin_x8(p).copy_to_slice(&mut out[i * VOICES..]);
            }
            self.phase = ph.to_array();
        }
        #[cfg(not(feature = "simd"))]
        {
            let n = out.len() / VOICES;
            for i in 0..n {
                for v in 0..VOICES {
                    let f = pitch[i * VOICES + v];
                    let mut p = self.phase[v] + f * dt;
                    p -= floorf(p);
                    self.phase[v] = p;
                    out[i * VOICES + v] = fast_sin(p);
                }
            }
        }
    }
```

- [ ] **Step 3: Add a SIMD null test (only compiles under `--features simd`)**

In `poly.rs`'s `#[cfg(test)] mod tests`, add:

```rust
    #[cfg(feature = "simd")]
    #[test]
    fn fast_sin_x8_matches_scalar() {
        use core::simd::f32x8;
        // Sweep phases across two full cycles; every lane must match scalar fast_sin.
        for base in 0..250 {
            let ps: [f32; 8] = core::array::from_fn(|k| (base as f32 * 8.0 + k as f32) / 1000.0);
            let v = crate::fast_sin_x8(f32x8::from_array(ps)).to_array();
            for k in 0..8 {
                assert!((v[k] - crate::fast_sin(ps[k])).abs() < 1e-6, "phase {} lane {k}", ps[k]);
            }
        }
    }
```

- [ ] **Step 4: Run BOTH feature configs**

The Task-1 reference tests (`polyosc_renders_independent_per_voice_partials`,
`polyosc_bounded`) use scalar `fast_sin` as an independent oracle, so they pin
the SIMD path too. Run both:

Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels poly`
Expected: PASS (scalar build).
Run: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels --features simd poly`
Expected: PASS — all Task-1 poly tests PLUS `fast_sin_x8_matches_scalar`. No warnings in either config.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-dsp-kernels/src/lib.rs crates/deluge-dsp-kernels/src/poly.rs
git commit -m "perf(dsp-kernels): f32x8 PolyOsc + fast_sin_x8 behind the simd feature"
```

---

## Notes for the implementer

- The whole feature is additive; no existing behavior changes. `process_resolved`
  and its ~30 call sites are deliberately untouched — poly nodes get their own
  `poly_process` path.
- **Voice-interleaved is load-bearing:** every poly buffer is flat `VOICES *
  BLOCK`, indexed `[i * VOICES + v]`. `poly_scratch[v] = arr[sbase + v]` is a
  row-wise *copy* that preserves that flat layout; `.as_flattened()` /
  `.as_flattened_mut()` then expose the contiguous tile.
- `as_flattened`/`as_flattened_mut` are stable slice methods (Rust ≥1.80). If
  the toolchain rejects them, replace with a manual flat copy loop over
  `VOICES * BLOCK` — the layout is identical.
- If an anchor line has moved, grep for the named symbol (`Kind::Ctrl`,
  `out_width`, the `scratch` decl, the `process_resolved` dispatch) — the
  relationship (add next to it / branch around it) is what matters, not line
  numbers.
