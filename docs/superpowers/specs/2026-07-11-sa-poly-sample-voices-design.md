# Sa-2: Poly Sample Voices + Basic Keymap — Design Spec

**Date:** 2026-07-11
**Suite:** Sa (samples), sub-project Sa-2 — poly sampler voices with a basic
keymap, after Sa-1 (in-memory buffer + mono player). See [[sa-suite-samples]],
[[sy-suite-voice-model]].
**Status:** Approved — ready for implementation plan

## Goal

Play samples as polyphonic voices inside `Synth.new { |p| … }`, pitched by MIDI
note relative to a root, with a basic keymap (note zones → different samples).
`Sample.new(p, x)` (x = a `SampleBuffer` or a `Keymap`) is a poly voice source
used like `Osc.saw(p)`.

## Background

The poly voice infrastructure and the pooled-poly-source pattern already exist;
Sa-2 mirrors `PolyWt` (the poly wavetable — `[WtOsc; VOICES]` scalar-per-voice
reading one shared pooled region, de-interleave/re-interleave in `poly_process`).
Reusable, from the exploration:
- **`PolyWt` template** (`poly.rs` `[WtOsc; VOICES]`; `node.rs` `Kind::PolyWt` arm
  ~989: `out_width=VOICES`, `is_poly`, `poly_in_count=1`, resolves ONE shared
  `pool_region`, loops per voice de-interleaving `poly_in[0]` and re-interleaving
  the output). A `PolySamplePlayer` maps 1:1.
- **Per-voice trigger** — `Cmd::TriggerVoice` → `Node::trigger_voice` (dispatches
  `PolyAr`/`PolyAdsr`/`PolySlew`; add a `PolySamplePlayer` arm).
- **Per-voice pitch tile** — `PolyCtrl`(semitones)→`PolyMtof`(Hz); the builder's
  `p` is the PolyMtof Hz id, consumed as `poly_in[0]` (same port oscillators use).
- **Sa-1** — `SamplePlayer` kernel (Hermite, one-shot/loop, `trigger`),
  `Kind::SamplePlayer`, `SampleBuffer.from`/`Player.new` (Player aborts in poly —
  the labeled Sa-2 hook), `pool_set` raw-PCM upload (now works on device too).

**Missing** (built here): a `PolySamplePlayer` kernel/`Kind`; note→rate-relative-
to-root; a basic keymap (concatenated multi-sample region + zone table); a poly
Wren factory taking a PCM handle + poly pitch; and a **per-lane source trigger in
the allocators** (today the poly `note_on` emits only pitch `SetParam` + envelope
`GateVoice` — nothing re-triggers a source).

## Scope (Sa-2)

Poly (and mono-Synth) single-sample voices + a basic note-zone keymap, pitched by
note relative to a per-zone root, one-shot or looped.

### Explicitly out of scope / deferred

- **Velocity zones/layers** (round-robin, vel-crossfade) — Sa-2 keymap is note-
  range only. **Crossfade zones.**
- **SR-aware root** — `rate = hz / mtof(root)` assumes the sample is at the engine
  SR; no per-sample SR metadata (Sa-3).
- **SD streaming** (Sa-3), **granular** (Sa-4), reverse, gate-to-stop, per-zone
  loop points (Sa-2 loops the whole zone).

## Global Constraints

- **Device-first:** `no_std`, no heap, no panics in DSP/upload paths. `libm`
  (`exp2f`/`log2f` for note↔rate) — NO std float methods. Bounded loops. `VOICES == 8`.
  Pool never panics on exhaustion (`None` → graceful silence).
- **Licensing:** MIT / Apache-2.0 only. No GPL.
- **Non-breaking:** additive — new kernel, new `Kind`, new Wren classes, an
  additive `triggers` fan-out in the allocators (a note_on that registers no
  triggers behaves exactly as today — every existing Synth/allocator test
  unchanged), a `trigger_voice` arm. Sa-1 mono `Player`/`SampleBuffer` unchanged.
- **Two registration tables:** new foreigns in BOTH tables + prelude; names
  collision-checked in the plan (`Sample`/`Keymap` etc.).
- **Test invocation (per-crate, never `--workspace`; both configs; `-- name1 name2`):**
  - Kernels: `cargo test --target x86_64-unknown-linux-gnu -p deluge-dsp-kernels <name>` (+ `--features simd`).
  - Graph: `… -p deluge-audio-graph <name>` (+ `--features deluge-dsp-kernels/simd`).
  - Wren: `… -p deluge-wren-core <name>` (+ `--features deluge-dsp-kernels/simd`).
  - LSP `armv7a … can't find crate for test` is environmental noise — ignore it.

## Architecture

### §1 — Keymap model (concatenated region + zone table)

A poly node gets ONE bound pool region from the engine. So a keymap **concatenates
all zone samples into one pool region** + a **zone table**
`[Zone; MAX_ZONES]` where `Zone { offset, len, low_note, high_note, root_note }`
(offset/len in samples, relative to the region start). `MAX_ZONES = 8` (a crate
const). A single sample is a **1-zone** keymap (full range 0–127, root default 60)
that binds the `SampleBuffer`'s existing region directly (no concatenation).

The zone table lives in the node's `State` (params, §2), NOT in the audio region
(region = pure concatenated PCM). Zones are matched first-fit by note; a note in
no zone → that voice stays silent.

### §2 — `PolySamplePlayer` kernel (`crates/deluge-dsp-kernels/src/sampler.rs`, ADD)

Scalar-per-voice, mirroring `PolyWt`. Per-voice state
`Voice { pos: f32, rate: f32, offset: u32, len: u32, playing: bool, latch: bool }`
(latch = "select zone on next process"). Shared params: the zone table + loop_mode.

- **`trigger_voice(v)`**: `voice[v] = { pos: 0, playing: true, latch: true, … }`
  (zone selected at the next `process`, when the pitch tile is available).
- **`process_voice(&mut self, v, pcm: &[f32], hz: In, zones: &[Zone], n_zones, loop_mode, dt, out)`**
  (the graph calls this per lane, de-interleaving the Hz tile, like PolyWt):
  - On `latch` (first process after trigger): `let h = hz.at(0); let note = round(69
    + 12·log2f(h/440));` find the first zone with `low ≤ note ≤ high`; if found,
    latch `offset,len` and `rate = h / mtof(root)` (`mtof(n) = 440·2^((n−69)/12)`);
    else `playing = false` (unmapped note). Clear `latch`.
  - Per sample: if `!playing` → 0; else 4-point Hermite read at `offset + pos`
    within `[offset, offset+len)` (reuse Sa-1's `hermite` + the clamp/wrap edge
    logic, but bounded to the zone slice); `pos += rate`; one-shot → stop at
    `pos ≥ len`; loop → wrap `pos` in `[0, len)`.

Reuse Sa-1's `hermite` fn and the read/wrap helper (factor the Sa-1 per-sample read
into a shared helper the mono `SamplePlayer` and the per-voice path both call).
`#[derive(Clone, Copy)]`. No panics; empty region / unmapped note → silence.

### §3 — Graph node (`crates/deluge-audio-graph/src/node.rs`)

`Kind::PolySamplePlayer` — `out_width == VOICES`, `is_poly == true`,
`poly_in_count == 1` (port 0 = the Hz pitch tile). `State::PolySamplePlayer` holds
`[SamplePlayerVoice; VOICES]` + the zone table (`[Zone; MAX_ZONES]`, `n_zones`) +
`loop_mode`. `set_param` scheme: `0 = n_zones`, `1 = loop_mode`, then zone fields at
`2 + z*5 + f` (`f`: 0=offset,1=len,2=low,3=high,4=root). `trigger_voice(v)` arm →
the voice's trigger. `poly_process` arm mirrors the `Kind::PolyWt` arm: resolve the
shared `pool_region` (pure PCM), then for each `v`: de-interleave `poly_in[0]` lane
`v` into a scratch column, call the per-voice process (reading the shared PCM +
zones), re-interleave into `out`.

### §4 — Per-lane source trigger in the allocators (`voice.rs`)

`VoiceAllocator` and `MonoAllocator` gain a `triggers: [NodeId; MAX_TRIGGERS]` +
`n_triggers` (like `gates`). `note_on` fans `Cmd::TriggerVoice{node: triggers[i],
voice: lane}` to each registered trigger on the allocated lane(s), alongside the
existing pitch `SetParam` + envelope `GateVoice`. **A synth that registers no
triggers emits exactly today's Cmds** (non-breaking). Note-**off** does NOT stop the
sample — the amp env (the `Env.adsr` the builder multiplies in) releases the voice;
the sample plays out / loops under it. (`MAX_TRIGGERS` small, e.g. 4.)

### §5 — Wren surface (`bindings_audio.rs`, `bindings.rs`, `prelude.wren`, `audio.rs`)

- **`Keymap.from([[samples], low, high, root], …)`** — for each zone, append its
  PCM to a growing pool region (one `alloc_buffer(total)` after summing lengths,
  then `pool_set` each sample verbatim, tracking per-zone offset); build the zone
  list. Returns a `KeymapObj { handle, zones: [(offset,len,low,high,root); …],
  n_zones }` foreign.
- **`Sample.new(p, x)`** — `x` is a `SampleBuffer` (Sa-1) or a `Keymap`:
  - `SampleBuffer` → a 1-zone keymap: bind the buffer's `handle`, one zone
    `(0, len, 0, 127, root)` (default root 60; `.root=` overrides — restrict to
    the single-sample form).
  - `Keymap` → bind its concatenated `handle`, copy its zone table.
  - Factory `polysampleplayer_(pitch, x)`: `new_node(Kind::PolySamplePlayer,
    [pitch, …])` + `BindTable{Pooled(handle)}` + the zone `set_param`s + register
    the node as a per-lane trigger via `poly_record_trigger(id)` (mirror
    `poly_record_gate`). `return_poly_node`.
  - Routing: `Sample.new` requires poly context (`polyMode_ == 1`, inside a Synth)
    — abort outside with a clear message (it needs the poly pitch + voice
    allocator; top-level one-shot playback is Sa-1's `Player`).
- **`PolyCtx.triggers`** (like `gates`), reset in `poly_begin`/`mono_begin`;
  `poly_record_trigger` bounded-push; `poly_end`/`mono_end` return the trigger list;
  the `SynthAlloc` construction passes it to `VoiceAllocator::new`/`MonoAllocator::new`.

## Data Flow

```
Keymap.from([[pcm],lo,hi,root],…) → concat PCM into one pool region + zone table
Sample.new(p, x) [poly ctx] → polysampleplayer_(p=PolyMtof Hz, x)
    → NewNode(Kind::PolySamplePlayer,[p]) + BindTable{Pooled(handle)}
    + SetParam(0,n_zones)+SetParam(1,loop)+zone fields
    + poly_record_trigger(id)   → VoiceAllocator.triggers

note_on(note): lane=pick; SetParam(pitch=PolyCtrl, lane, note-69)+[vel]+[pan]
    + gate_all(GateVoice envelopes) + TriggerVoice(sample_node, lane)   ← new

per block, per voice v (poly_process, shared PCM region):
    hz = PolyMtof tile lane v
    on trigger-latch: note=round(69+12log2(hz/440)); zone by note; rate=hz/mtof(root)
    out[v] = hermite(pcm[zone.offset + pos]); pos += rate; one-shot stop | loop wrap
→ * Env.adsr(…) → StereoVoiceSum → Out
```

## Error Handling

Pool exhaustion → `None` handle → silent voice (no panic). Unmapped note → that
voice silent. Empty PCM / degenerate zone → silence. `log2f`/`exp2f` bounded
(hz>0 guarded by an EPS). Zone indices bounded by `MAX_ZONES`; per-voice reads
clamped/wrapped within the zone slice. `n_triggers`/`n_zones` saturating-bounded.
No heap, no panics, `no_std`.

## Testing

Both feature configs, per-crate. Oracle-driven.

1. **Per-voice pitch = hz/root** (kernel): a 1-zone player, root 60; drive lane 0
   at hz=mtof(60) → rate 1 (verbatim); lane 1 at hz=mtof(72) → rate 2 (octave up,
   reads every other). Distinct lanes play independently.
2. **Zone selection** (kernel): 2 zones (note 0–59 → sample A, 60–127 → sample B,
   distinct content/root); trigger lane at note 48 → plays A's content; note 72 →
   B's; note in no zone → silent.
3. **Trigger latch + retrigger** (kernel): `trigger_voice(v)` restarts lane `v`'s
   `pos` and re-latches its zone from the current pitch; independent per lane.
4. **Node** (node.rs): `Kind::PolySamplePlayer` `out_width == VOICES`, `is_poly`;
   `poly_process` reads the shared PCM at per-lane rates from the Hz tile;
   `trigger_voice` works; `set_param` zone scheme populates the table.
5. **Allocator trigger fan-out** (voice.rs): `note_on` with a registered trigger
   emits `TriggerVoice{node, lane}` on the allocated lane, in addition to the
   pitch/gate Cmds; NO registered trigger ⇒ byte-identical to today.
6. **Keymap upload** (wren, via the real-pool test host): `Keymap.from(…)`
   concatenates PCM (each zone's slice matches its input) + builds the zone table.
7. **Wren e2e** (`tests/audio_bindings.rs`): `Synth.new { |p| Sample.new(p, buf) *
   Env.adsr(…) }` + `s.noteOn(60,100)` renders finite/bounded/non-silent (the
   sample plays as a voice); a 2-zone `Keymap` + notes in different zones render;
   `s.noteOn` at two notes plays two voices (poly); `Sample.new` OUTSIDE a Synth
   aborts. Existing synths/effects/Sa-1 Player unchanged.

## Success Criteria

- `Synth.new { |p| Sample.new(p, keymap) * Env.adsr(…) }` is a polyphonic sampler:
  each note picks its zone, plays that sample pitched relative to the zone root,
  one-shot or looped, under the amp env; polyphony/stealing via the existing
  allocator.
- A single `SampleBuffer` works as a full-range 1-zone instrument with a `.root=`.
- Additive: existing Synth/allocator/effect/Sa-1 behavior byte-unchanged.
- Both feature configs green, per-crate.
