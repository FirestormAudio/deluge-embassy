# Sy-6a: Stereo Spread — Design Spec

**Date:** 2026-07-11
**Suite:** Sy (synth/voice), sub-project Sy-6a — first of the Sy-6 spatial /
voice-management suite (the follow-on to the completed Sy-5 expressiveness suite:
5a velocity, 5b mono-glide, 5c multi-env, 5d release-tail, 5e unison).
**Status:** Approved — ready for implementation plan

## Goal

`synth.width = amount` (0..1) fans a note's `U` unison voices symmetrically
across the stereo field for a wide, spacious sound. Works in BOTH poly
(`Synth.new`) and mono (`Synth.mono`) modes. `width == 0` is byte-identical to
today's mono output.

## Background

The graph is **already stereo end-to-end below the voice sum.** The master bus
is a true stereo accumulator (`bus_l`/`bus_r`, `engine.rs`), there are already 7
width-2 stereo `Kind`s (`Split2`, `Pan`, `Chorus`, `Flanger`, `Room`, `Hall`,
`Plate` — `node.rs:244-254`, port0 = L, port1 = R), and routing is width-aware:
`write_source_to_bus` (`bindings_audio.rs`) sends a width-2 node's `port0→L (1,0)`
and `port1→R (0,1)`, while a mono node gets a center `(1,1)` dual-mono write. A
width-2 node is marked with `return_node_w(vm, id, 2)`.

Today a `Synth` collapses its `VOICES = 8` lanes through a mono `VoiceSum` node
(`out_width == 1`), whose single row is center-written to both bus sides
(dual-mono). Unison (Sy-5e) already spreads `U` detuned voices across lanes and
already normalizes level via the `VoiceSum` gain (`set_param(0) = 1/√U`). Stereo
spread reuses that per-voice lane structure: each of a note's `U` voices gets a
**pan position** (like it already gets a **detune offset**), and the sum node
mixes the lanes into L and R instead of a single mono row.

Because the stereo plumbing already exists, this is mostly an allocator +
one-kernel + Wren job, not a signal-path overhaul.

## Scope (Sy-6a)

A new `StereoVoiceSum` node (VOICES → L/R with per-lane pan), a shared
`width_offset` pan-spread helper, per-lane pan emission in both allocators, and a
`synth.width` setter. The Synth **always** uses `StereoVoiceSum` (replacing
`VoiceSum` in the build path); `width == 0` is byte-identical to today via the
unity-center pan law.

### Explicitly out of scope / deferred

- **Group-partitioned poly-unison** — Sy-6b, the sequenced next sub-project
  (fixed contiguous lane groups per note, replacing mark-and-pick + the 5d
  `LaneState` release-tail model). Orthogonal to stereo spread; built next.
- **Per-oscillator `Osc(...).spread=` interaction** — the existing oscillator
  stereo-width param (param 3) is a separate concept on a separate object; this
  spec does not touch or unify it.
- **Live re-pan on `width` change** — `width` takes effect on the next note-on
  (standard, matches `detune`/`unison`), not on currently-sounding voices.
- **Re-centering on `width → 0`** — because the per-lane pan `SetParam` is emitted
  only when `width_amount != 0` (to keep the `width == 0` Cmd stream byte-identical,
  §3), a live tweak from `width > 0` back to exactly `0` leaves stale pan on lanes a
  new note reuses (they were last written non-zero). Minor live-tweak wart; a
  future `set_width(0.0)` could re-emit per-lane zeros to fix it. Deferred.
- **Stereo global FX rework, per-lane pan as a modulatable signal, MPE.**

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/control paths.
  `VOICES == 8`.
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** `width == 0` is byte-identical to today (poly AND mono), in
  BOTH the emitted Cmd stream and the rendered audio. `StereoVoiceSum` default gain
  is 1.0 and default pan is 0.0 (center) on every lane, so an all-center sum gives
  `L = R = gain · Σ lane` — exactly the mono `voice_sum` value, deposited
  identically on both bus sides (width-2 `(1,0)/(0,1)` routing == mono `(1,1)`
  center write). The per-lane pan `SetParam` is emitted **only when
  `width_amount != 0`** (§3), so a synth that never calls `width` emits the exact
  same Cmd stream as today — every existing allocator test (including the Sy-5e
  `*_one_is_byte_identical` and `*_emits_two_cmds` count assertions) passes
  unchanged. The unison `1/√U` gain (Sy-5e, `set_param(0)`) is on the same slot and
  keeps working.
- **Two registration tables:** the new `width=(_)` foreign setter goes in BOTH
  `register_audio` (`bindings_audio.rs`) AND the `wren-sys-backend` `METHODS`
  table (`bindings.rs`), plus a `foreign` decl in the prelude `Synth` class.
- **Test invocation (per-crate, never `--workspace`; both configs; use `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — Shared pan-spread helper (`crates/deluge-audio-graph/src/voice.rs`)

A free fn, exact sibling of Sy-5e's `unison_offset`:

```rust
/// Pan position for unison voice `u` of `count`, spread symmetrically and evenly
/// over ±`amount` of the stereo field. `count <= 1` ⇒ 0.0 (center, no spread).
/// Result is in [-1.0, +1.0]: -1 = hard left, 0 = center, +1 = hard right.
fn width_offset(u: usize, count: usize, amount: f32) -> f32 {
    if count <= 1 { return 0.0; }
    let t = -1.0 + 2.0 * (u as f32) / ((count - 1) as f32); // -1..+1
    t * amount
}
```

`amount` is a dimensionless 0..1 width knob (unlike detune's cents). At
`amount == 1` the outer voices hard-pan (±1); the center voice (odd `U`) stays at
0. Values are not clamped here — the caller passes `amount` in 0..1; the pan law
(§2) clamps the resulting gains.

### §2 — `StereoVoiceSum` node (`crates/deluge-dsp-kernels/src/poly.rs` + `node.rs`)

**Kernel** — a stereo sibling of `voice_sum`:

```rust
/// Collapse the VOICES lanes of a voice-interleaved tile into a stereo pair,
/// applying per-lane constant-balance pan and an overall gain (the 1/√U unison
/// normalization). `pan[v]` in [-1,1]; unity-center law keeps pan 0 at (1,1) so
/// an all-center sum is bit-identical to `voice_sum`.
pub fn voice_sum_stereo(
    tile: &[f32], out_l: &mut [f32], out_r: &mut [f32], gain: f32, pan: &[f32; VOICES],
) {
    // Per-lane L/R gains, computed once (pan is per-note, block-constant).
    let mut gl = [0.0f32; VOICES];
    let mut gr = [0.0f32; VOICES];
    for v in 0..VOICES {
        gl[v] = (1.0 - pan[v]).clamp(0.0, 1.0); // p=0→1, p=+1→0, p=-1→1
        gr[v] = (1.0 + pan[v]).clamp(0.0, 1.0); // p=0→1, p=+1→1, p=-1→0
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

**Unity-center balance law:** `gl = clamp(1 - p, 0, 1)`, `gr = clamp(1 + p, 0, 1)`.
`p = 0 → (1, 1)` (dual-mono unity — the non-breaking baseline); `p = +1 → (0, 1)`
(hard right at unity); `p = -1 → (1, 0)` (hard left at unity). A pinned-side
balance law (the fully-panned side stays at unity, the other attenuates to 0).

An optional `#[cfg(feature = "simd")]` f32x8 path may follow (two masked lane
reductions), null-tested lane-for-lane against the scalar oracle ≤ 1e-4, as with
`voice_sum`. Scalar is the correctness oracle.

**Graph node** — `Kind::StereoVoiceSum`:
- `out_width(Kind::StereoVoiceSum) == 2` (port0 = L, port1 = R — uses the
  existing width-2 disjoint-slice output path, `node.rs` `outs.port(0)/port(1)`).
- `is_poly == true`, `poly_in_count == 1` (the voice tile is the one poly input).
- `State::StereoVoiceSum { gain: f32, pan: [f32; VOICES] }`, constructed as
  `{ gain: 1.0, pan: [0.0; VOICES] }` (center, unity).
- `set_param`:
  - `param == 0` ⇒ `gain = value` (**same slot as `VoiceSum` — the Sy-5e unison
    `set_param(out_node, 0, 1/√N)` keeps working unchanged**).
  - `1 <= param <= VOICES` ⇒ `pan[param - 1] = value`.
- `poly_process`: `if let (State::StereoVoiceSum { gain, pan }, Some(pin)) = … {
  voice_sum_stereo(pin, out.port(0), out.port(1), *gain, pan); }`.

`VoiceSum` is unchanged and remains in the codebase (it is simply no longer used
by the `Synth` build path).

### §3 — Poly unison spread (`VoiceAllocator`)

Add `sum_node: NodeId` (the `StereoVoiceSum` id, passed at construction) +
`width_amount: f32` (default 0.0) + `set_width(&mut self, amount: f32)`.

In `note_on`, inside the existing `for u in 0..U` lane loop, emit one extra pan
`SetParam` per lane — **only when `width_amount != 0.0`** (mirrors the Sy-5a
velocity-emit-when-present pattern; keeps the `width == 0` Cmd stream
byte-identical) — right beside the pitch/detune emit:

```rust
if self.width_amount != 0.0 {
    emit(Cmd::SetParam {
        node: self.sum_node,
        param: (lane + 1) as u8,          // param 0 is the gain slot
        value: width_offset(u, u_count, self.width_amount),
    });
}
```

`width == 0` ⇒ no pan emit ⇒ the note-on Cmd stream is byte-identical to today
(and the node's default pan is already center). `U == 1` ⇒ single voice,
`width_offset == 0` even at `width > 0` ⇒ center. Release/steal (5d) unaffected —
pan is per-lane node state, overwritten on each note-on when width is engaged.

### §4 — Mono unison spread (`MonoAllocator`)

The same `sum_node` + `width_amount` + `set_width`. In `note_on`, inside the
`for u in 0..u_count` loop over lanes `0..U`, emit the same per-lane pan
`SetParam(sum_node, u + 1, width_offset(u, u_count, width_amount))` — again
**only when `width_amount != 0.0`**. `U == 1` (or `width == 0`) reproduces today's
lane-0-only, center behavior exactly (byte-identical Cmd stream).

### §5 — Wren surface (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`)

- **Build path:** `node_poly_end_impl` / `node_mono_end_impl` build
  `Kind::StereoVoiceSum` instead of `Kind::VoiceSum` for the `sum` node, mark it
  width-2 via `return_node_w(vm, sum, 2)`, and pass `sum` to the allocator
  constructor as `sum_node`. `SynthObj.out_node` is that `StereoVoiceSum` id;
  `synth.out` now returns a width-2 node → `Out.patch(synth.out)` routes stereo
  through the existing width-aware `write_source_to_bus` (no new plumbing).
- **`synth.width = amount`:** `SynthAlloc::set_width(amount)` dispatch (match
  Poly/Mono, like `set_unison`/`set_detune`); `synth_set_width_impl` reads the
  amount via `vm.get_f(1) as f32` and calls `self_synth(vm).alloc.set_width(amount)`.
  **No immediate `SetParam`** —
  pan rides the next note-on (unlike `unison`, which sets the gain immediately).
- **`SynthAlloc`** gains `set_width(&mut self, amount: f32)` dispatch.
- Register `width=(_)` as a `Synth` instance setter in BOTH tables + a
  `foreign width=(amount)` decl in the prelude `Synth` class. No Wren
  wrapper/guard needed (width works in both mono and poly).

## Data Flow

```
synth.width = a → SynthAlloc.set_width(a)   (stored; no immediate emit)

poly noteOn(note): for u in 0..U: lane=pick_lane(); mark Held(note);
    SetParam(pitch, lane, note-69 + detune_offset[u])
    + [vel]
    + [width!=0: SetParam(sum_node, lane+1, width_offset[u])]   ← new: per-lane pan
    + gate_all(lane, on)

mono noteOn(note) from silence: for u in 0..U:
    SetParam(pitch, u, note-69 + detune_offset[u])
    + [vel] + [width!=0: SetParam(sum_node, u+1, width_offset[u])]   ← new
    + TriggerVoice(slew, u) + gate_all(u, on)

StereoVoiceSum: out_L = gain·Σ gl[v]·lane[v],  out_R = gain·Σ gr[v]·lane[v]
    gl = clamp(1-pan, 0, 1), gr = clamp(1+pan, 0, 1)
    → width-2 node → write_source_to_bus port0→L(1,0), port1→R(0,1)
```

## Error Handling

`width` is a total setter (no clamp needed at the boundary; the pan law clamps
the resulting gains, and `width_offset` is bounded for `amount` in 0..1). No new
panics, no heap. `width == 0` is the byte-identical baseline. Param indices
`1..=VOICES` are bounded by the `VOICES`-lane loop; `set_param` ignores
out-of-range params (matches existing `set_param` arms).

## Testing

Both feature configs, per-crate.

1. **`width_offset`** (voice.rs unit): `count=1 → 0.0`; `count=2 @ amount=1 → ∓1,±1`
   (i.e. `-1, +1`); `count=3 @ amount=0.5 → -0.5, 0.0, +0.5` — symmetric, evenly
   spaced, scaled by `amount`.
2. **`StereoVoiceSum` kernel** (poly.rs): all-center pan (0.0) ⇒ `out_l == out_r`
   AND both bit-identical to `voice_sum` (the non-breaking oracle); a lane panned
   `+1` contributes only to R (`gl=0, gr=1`); gain scales both rows.
3. **`StereoVoiceSum` node** (node.rs): `out_width == 2`, `is_poly`,
   `poly_in_count == 1`; `set_param(0)` sets gain, `set_param(lane+1)` sets that
   lane's pan; default is unity center; poly_process writes both L and R rows.
4. **Poly `VoiceAllocator`** (voice.rs): `set_width(1.0)` + `set_unison(3)`,
   note-on emits a per-lane `SetParam(sum_node, lane+1, …)` for each of the 3
   lanes with the 3 symmetric pan values; **`width == 0` emits NO pan `SetParam`**
   (the note-on Cmd stream is byte-identical to a no-width synth — assert the
   sum_node SetParam count is 0); `sum_node` id is threaded correctly.
5. **Mono `MonoAllocator`** (voice.rs): `set_width(1.0)` + `set_unison(2)`,
   from-silence emits pan `SetParam(sum_node, u+1, …)` on lanes 0 and 1;
   **`width == 0` emits none**.
6. **Wren e2e** (`tests/audio_bindings.rs`): `Synth.new { |p| Osc.saw(p) *
   Env.adsr(…) }` + `s.unison = 4` + `s.width = 1` builds + renders
   finite/bounded/non-silent AND **L ≠ R** on some frame (a real stereo image);
   `Synth.mono` + unison + width renders stereo; **`s.width = 0` renders L == R
   AND byte-identical to the same patch with no `width` call** (non-breaking);
   existing 1-voice synths unchanged.

## Success Criteria

- `Synth.new { |p| Osc.saw(p) * Env.adsr(…) }` with `s.unison = 7; s.detune = 15;
  s.width = 0.8` — a fat, detuned, level-normalized poly voice spread wide across
  the stereo field; `Synth.mono` + unison + width — a wide mono lead/bass.
- `synth.width` fans the `U` voices symmetrically L↔R; `width == 0` collapses to
  center (dual-mono).
- `width == 0` builds/renders byte-identically (poly and mono); the Sy-5e unison
  `1/√U` gain is unchanged on the shared `set_param(0)` slot.
- Unison, detune, velocity, glide, multi-env, and release-tail all compose with
  width.
- Both feature configs green, per-crate.
