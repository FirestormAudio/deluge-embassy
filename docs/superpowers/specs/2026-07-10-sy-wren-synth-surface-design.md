# Sy-4: Wren `Synth` surface + MIDI — design & spec

The fourth **Sy (synth/voice)** sub-project — the Wren scripting face of
polyphony. Sy-1–3 built the poly voice machinery (poly kinds, per-voice gate,
`VoiceAllocator`) at the graph/engine level. Sy-4 makes it usable from a Wren
patch with the **single-name ergonomics** designed earlier: a `Synth.new { |pitch|
… }` voice-builder where the plain factories (`Osc.sine`/`.lpf`/`Env.ar`/`*`)
transparently emit the **poly** kinds because they run in a *poly context*; the
`Synth` owns a `VoiceAllocator`; and MIDI note events drive it.

> **Status:** design proposal. Depends on merged Sy-1–3 (poly kinds,
> `VoiceAllocator`, `PolyMtof`) and the wren-core bindings/prelude/Midi seam.
> MIT/Apache. `no_std`, device-first.

**Sy roadmap:** Sy-1 poly infra ✓ → Sy-2 poly voice channel ✓ → Sy-3 voice
allocation ✓ → **Sy-4 Wren Synth surface + MIDI (this)** → Sy-2b poly breadth →
Sy-5 expressiveness.

---

## 1. Goals & non-goals

**Goals**
- **Poly context.** A Rust flag (`poly_mode`) in the audio state. While set, the
  poly-capable prelude factories emit the poly `Kind` instead of the mono one.
  Building-time only (not audio-rate).
- **`Synth.new(builder)`** — a prelude class. `builder` is a `Fn { |pitch| … }`
  returning the voice's output node:
  1. `polyBegin_` — set `poly_mode`; create `PolyCtrl`(pitch source) → `PolyMtof`;
     record the `PolyCtrl` id; return the `PolyMtof` node as `pitch`.
  2. `builder.call(pitch)` — Wren runs the voice; poly factories active.
  3. `polyEnd_(out)` — clear `poly_mode`; require **exactly one** `Env.ar`
     (the amp gate) was created; wrap `out` in a `VoiceSum`; build a
     `VoiceAllocator(pitch_ctrl, gate_ar)` inside a **`Synth` foreign object**;
     return it. `synth.out` = the mono `VoiceSum` node (routed explicitly).
- **Poly factories (new Wren foreign methods)** for the poly-capable kinds
  (which had no Wren surface in Sy-1–3): `polyosc_(pitch)` → `PolyOsc`,
  `polysvf_(audio, cutoff, res)` → `PolySvf`, `polyar_(attack, release)` →
  `PolyAr` (records the gate), `polymul_(a, b)` → `PolyMul`. Plus `polyMode_`
  (flag getter), `polyBegin_`, `polyEnd_`.
- **Prelude routing** (Wren, `Fiber.abort` for the honest errors):
  - `Osc.sine(f)` → poly ? `polyosc_(f)` : `src_(0,f)`. `Osc.saw/square/tri`,
    `Noise.*`, `Osc.sync*`, `Osc.wavetable`, and `+`/`-` → `Fiber.abort("… not
    usable in a Synth yet")` while `poly_mode`.
  - `.lpf(c)` → poly ? `polysvf_(this, c, DEFAULT_RES)` : `lpf_(this, c)`.
  - `Env.ar(a,r)` → poly ? `polyar_(a,r)` : `env_(a,r)`.
  - `Node`/`Port` `*` → poly ? `polymul_(this,o)` : `binop_(0,this,o)`. In poly
    mode `*` is the VCA — **both operands must be poly nodes** (`filter *
    Env.ar`); `polymul_` aborts if `o` is a bare number ("multiply by a constant
    inside a Synth isn't supported yet — the amp comes from Env.ar"), since
    poly×scalar broadcast gain is a different op (deferred).
- **`Synth` foreign object** holding the `VoiceAllocator` + the `VoiceSum` node
  id. Methods: `noteOn(note, vel)`, `noteOff(note)` (drive the allocator, emit
  via `audio::audio_cmd`), `out` (the mono node for routing), `bindMidi()`
  (wire `Midi.onNoteOn`/`onNoteOff` to this synth).
- **QA-proven:** a Wren `Synth` renders a gated polyphonic sine voice; `noteOn`
  sounds at the right pitch, `noteOff` silences; two notes → two voices; MIDI
  (`bindMidi` + an incoming note) drives it; the error cases (`Osc.saw` in a
  Synth, zero or ≥2 `Env.ar`) abort with clear messages.

**Non-goals (deferred)**
- **Poly waveshapes beyond sine, poly Noise/Sync/Wavetable/other filters,
  `PolyAdd`** — Sy-2b breadth (the abort messages point there).
- **Velocity → amplitude, multiple envelopes per voice, per-voice modulation,
  multi-Synth MIDI channel routing** — Sy-5 / later.
- **Auto-patching** — a `Synth` exposes `.out`; the user routes it
  (`Out.patch(synth.out)` or through mono FX), consistent with the graph model.

---

## 2. Poly context (Rust audio state)

Add to the audio state (alongside the node-id allocator):

```text
poly_mode: bool           // set between polyBegin_ and polyEnd_
poly_pitch_ctrl: NodeId   // the PolyCtrl created by polyBegin_
poly_gate_ar: NodeId      // the PolyAr recorded by polyar_ (the amp gate)
poly_gate_count: u8       // number of Env.ar created this build (enforce == 1)
```

- `polyMode_` foreign getter returns `poly_mode` (a bool) — the prelude branches
  on it. (Building-time; overhead irrelevant.)
- `polyBegin_`: `poly_mode = true`; `poly_gate_count = 0`; create `PolyCtrl`
  (`poly_pitch_ctrl`) and `PolyMtof` (input = the PolyCtrl); return the
  `PolyMtof` NodeObj.
- `polyar_`: on creating a `PolyAr`, set `poly_gate_ar` and
  `poly_gate_count += 1`.
- `polyEnd_(out)`: `poly_mode = false`; if `poly_gate_count != 1` abort the
  fiber ("a Synth voice needs exactly one Env.ar (the amp gate)"); create
  `VoiceSum(out)`; construct `VoiceAllocator::new(poly_pitch_ctrl, poly_gate_ar)`
  into a `Synth` foreign object carrying the `VoiceSum` node id; return it.

Nested `Synth.new` is unsupported (a single `poly_mode` flag) — `polyBegin_`
while already in poly mode aborts ("nested Synth not supported").

---

## 3. `Synth` foreign object (`deluge-wren-core`)

```text
struct SynthObj {           // WrenForeign; not used as a node input (no shared tag)
    alloc: VoiceAllocator,  // from deluge-audio-graph (Sy-3)
    out_node: u16,          // the VoiceSum NodeId for `.out`
}
```

Foreign methods (registered in both binding tables, wren-sys + register_audio):
- `synth_note_on_(note, vel)` → `self.alloc.note_on(note as u8, vel as u8,
  &mut |c| audio::audio_cmd(c))`.
- `synth_note_off_(note)` → `self.alloc.note_off(note as u8, &mut |c| …)`.
- `synth_out_` → return a `NodeObj` for `out_node` (mono, width 1).

Prelude:
```wren
foreign class Synth {
  foreign noteOn(note, vel)
  foreign noteOff(note)
  foreign out
  bindMidi() {
    Midi.onNoteOn = Fn.new { |ch, note, vel| this.noteOn(note, vel) }
    Midi.onNoteOff = Fn.new { |ch, note, vel| this.noteOff(note) }
  }
  static new(builder) {
    var pitch = Node.polyBegin_()
    var out = builder.call(pitch)
    return Node.polyEnd_(out)
  }
}
```

`bindMidi`'s closures capture `this`, keeping the `Synth` alive (a GC root via
the Midi handle) while bound. One active synth per `Midi.onNoteOn` (last
`bindMidi` wins); channel routing is deferred.

---

## 4. Usage

```wren
var bass = Synth.new { |pitch|
  Osc.sine(pitch).lpf(1200) * Env.ar(0.01, 0.3)
}
Out.patch(bass.out)   // or Out.patch(Reverb.room(bass.out))
bass.bindMidi()       // play from DIN MIDI

// or drive directly:
bass.noteOn(69, 100)  // A4
bass.noteOff(69)
```

Inside the builder, `Osc.sine(pitch)` is a `PolyOsc` at the per-voice pitch,
`.lpf` a `PolySvf`, `Env.ar` the `PolyAr` amp gate, `*` a `PolyMul` (the VCA).
`polyEnd_` sums the 8 voices and hands back `bass.out`.

---

## 5. QA acceptance & testing (`deluge-wren-core`, host)

- **Builds the poly graph:** `Synth.new { |p| Osc.sine(p).lpf(1200) *
  Env.ar(0.01,0.3) }` emits `NewNode`s for `PolyCtrl`, `PolyMtof`, `PolyOsc`,
  `PolySvf`, `PolyMul`, `PolyAr`, `VoiceSum` (capture `Cmd`s) — the poly kinds,
  not the mono ones.
- **Sounds / silences:** patch `bass.out`; `bass.noteOn(69,100)` → render → the
  output is non-silent and bounded; `bass.noteOff(69)` + enough render → silence.
  (A lower-level host harness renders across note-on/off; `run_and_render` for
  the simple non-silent check.)
- **Right pitch:** `noteOn(69,100)` → dominant partial ≈ 440 Hz (spectrum /
  zero-crossing over a longer render).
- **MIDI:** `bass.bindMidi()`, then an incoming DIN note-on (via `midi_rx`) →
  the voice sounds; note-off silences.
- **Errors abort with clear messages:** `Osc.saw(p)` inside a `Synth` aborts;
  a builder with **no** `Env.ar` aborts; a builder with **two** `Env.ar` aborts;
  a `Noise.new()`/`+` inside a `Synth` aborts. (Assert the abort / error string.)
- **Poly mode is scoped:** after `Synth.new` returns, `Osc.saw(110)` outside
  builds a normal mono `Kind::Saw` (the flag was cleared).
- **Both feature configs** (`--features deluge-dsp-kernels/simd`) for the render.

**Determinism:** graph-build is deterministic; the allocator is integer logic.

---

## 6. Deferred / follow-ups

- **Sy-2b poly breadth** — poly saw/square/tri, Noise, Sync, Wavetable, other
  filters, `PolyAdd`; the Sy-4 abort messages become working factories.
- **Velocity → amp, multiple envelopes, per-voice mod, unison, portamento,
  sustain** — Sy-5.
- **Multi-Synth MIDI channel routing, Synth as a first-class graph node** (so a
  Synth composes like any node) — later.
- **Nested/reentrant `Synth.new`** — needs a poly-context stack rather than a
  single flag.
