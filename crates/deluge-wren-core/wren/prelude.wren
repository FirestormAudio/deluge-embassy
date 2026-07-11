// Deluge Wren prelude.
//
// Compiled into the `main` module at boot (before any user script / REPL line),
// so these classes and the `output` / `gate` accessors are always available.
// Each `foreign` member is bound to a native Rust method in `src/bindings.rs`.
//
// Embedded into the firmware via `include_str!` — edit freely, then rebuild.

// ── CV / Gate ────────────────────────────────────────────────────────────────

// Control-voltage output. `output[1]` and `output[2]` are the two CV jacks.
//   output[1].volts = 5.0     // set (immediately, or ramped if slew > 0)
//   output[1].volts           // read the current (post-slew) voltage
//   output[1].slew = 0.5      // seconds to ramp to future targets
foreign class Output {
  construct new(ch) {}
  foreign volts
  foreign volts=(v)
  foreign slew=(v)
}

// Gate output. `gate[1]`..`gate[4]` are the four gate jacks.
//   gate[1].on = true
foreign class Gate {
  construct new(ch) {}
  foreign on=(v)
}

// ── Timing ───────────────────────────────────────────────────────────────────

// Periodic timer. The callback receives an incrementing `stage` (1, 2, 3, …).
//   var m = Metro.new()
//   m.start(Fn.new { |stage| output[1].volts = stage % 2 == 0 ? 5 : 0 }, 0.25)
//   m.time = 0.5              // change interval while running
//   m.stop()
foreign class Metro {
  construct new() {}
  foreign start(fn, seconds)
  foreign stop()
  foreign time=(v)
}

// ── MIDI (DIN) ───────────────────────────────────────────────────────────────

// Channels are 1..16. Receive handlers get (channel, data1, data2).
//   Midi.onNoteOn = Fn.new { |ch, note, vel| output[1].volts = note / 12 }
//   Midi.noteOn(1, 60, 100)
foreign class Midi {
  foreign static noteOn(ch, note, vel)
  foreign static noteOff(ch, note, vel)
  foreign static cc(ch, num, val)
  foreign static send(a, b, c)
  foreign static onNoteOn=(fn)
  foreign static onNoteOff=(fn)
  foreign static onCC=(fn)
}

// ── Surface: pads, buttons, encoders ─────────────────────────────────────────

// 18x8 RGB pad grid. Press/release handlers get the pad's (x, y).
//   Pads.onPress = Fn.new { |x, y| Led.on(0) }
foreign class Pads {
  foreign static onPress=(fn)
  foreign static onRelease=(fn)
}

// Front-panel buttons. Handlers get the button id (0..35).
foreign class Buttons {
  foreign static onPress=(fn)
  foreign static onRelease=(fn)
}

// Rotary encoders (index 0..5). Handler gets (index, delta) in detents.
//   Enc.onTurn = Fn.new { |i, d| output[1].volts = output[1].volts + d * 0.1 }
foreign class Enc {
  foreign static onTurn=(fn)
}

// ── Surface: LEDs + OLED ─────────────────────────────────────────────────────

// Indicator LEDs by button id.
foreign class Led {
  foreign static on(id)
  foreign static off(id)
}

// 128x48 monochrome OLED. Draw into the buffer, then `show()` to render.
//   Oled.clear()
//   Oled.text(0, 0, "hello deluge")
//   Oled.show()
foreign class Oled {
  foreign static clear()
  foreign static text(x, y, s)
  foreign static pixel(x, y, on)
  foreign static show()
}

// ── Audio: native DSP graph ──────────────────────────────────────────────────
//
// Build a signal graph and patch it to the output. DSP runs natively at 44.1 kHz;
// these objects are lightweight handles to native nodes. Numbers OR nodes may be
// used wherever a value is expected (so params can be modulated by other nodes).
//   var env = Env.ar(0.01, 0.4)
//   Out.patch(Osc.saw(110) * env)
//   env.trigger()                        // one-shot AR
//   Out.patch(Osc.saw(110).lpf(800))     // filtered
//   var lfo = Osc.sine(5); var o = Osc.sine(440); o.freq = lfo   // vibrato
//   Out.reset()                          // clear the graph
foreign class Node {
  foreign static src_(kind, freq)
  foreign static sync_(wave, master, slave)
  foreign static env_(attack, release)
  foreign static adsr_(a, d, s, r)
  foreign static noise_()
  foreign static pink_()
  foreign static brown_()
  foreign static binop_(op, a, b)
  foreign static lpf_(input, cutoff)
  foreign static svf_(input, cutoff, res, resp)
  foreign static moog_(input, cutoff, res, poles)
  foreign static ms20_(input, cutoff, res, resp)
  foreign static modal_(input, freq, damping)
  foreign drive=(v)
  foreign static tb303_(input, cutoff, res)
  foreign static patch_(node)
  foreign static reset_()
  foreign static split_(input)
  foreign static pan_(input, position)
  foreign static wavetable_(table, freq)
  foreign static wavetable_pooled_(wt, freq)
  foreign static delay_(input, time, feedback)
  foreign mix=(v)
  foreign damp=(v)     // Delay feedback-path damping (NOT damping= — that's the Resonator's)
  foreign static chorus_(input, rate, depth, mix)
  foreign static flanger_(input, rate, depth, feedback, mix)
  foreign static room_(input, roomsize, damp, mix)
  foreign static hall_(input, size, damp, mix)
  foreign static plate_(input, size, damp, mix)
  foreign static drive_(input, drive, tone, mix, shape)
  foreign tone=(v)
  foreign wet=(v)      // Drive dry/wet (NOT mix= — that's set_param 0, = drive here)
  foreign static eq_(input, freq, gain, q, type)
  foreign hz=(v)       // EQ centre/corner frequency (NOT freq= — that's the Osc's)
  foreign gain=(v)     // EQ band gain in dB
  foreign q=(v)        // EQ Q / bandwidth
  foreign static lfo_(rate, shape)
  foreign phase=(v)    // LFO start/retrigger phase [0,1)
  foreign static sh_(input, clock)
  foreign static slew_(input, time)
  foreign static steps_(values, clock)
  foreign static curve_(input, k)
  foreign static ctrl_(value)
  foreign static qstep_(input, n)
  foreign static qpitch_(input, mask, root)
  foreign static mtof_(input, ref)
  foreign static polyMode_
  foreign static polyGateCount_
  foreign static polyBegin_()
  foreign static polyVelBegin_()
  foreign static monoBegin_()
  foreign static monoEnd_(out)
  foreign static polyosc_(pitch, shape)
  foreign static polysvf_(audio, cutoff, res)
  foreign static polymoog_(audio, cutoff, res, poles)
  foreign static polyms20_(audio, cutoff, res, resp)
  foreign static polyar_(attack, release)
  foreign static polyadsr_(a, d, s, r)
  foreign static polymul_(a, b)
  foreign static polyadd_(a, b)
  foreign static polynoise_()
  foreign isPoly_               // true if this Node wraps a per-voice audio signal (Sy-2d)
  foreign static polypink_()
  foreign static polybrown_()
  foreign static polysync_(wave, master, slave)
  foreign static polywt_(table, freq)
  foreign static polywt_pooled_(wt, freq)
  foreign static polyEnd_(out)
  foreign value=(v)     // Ctrl (Macro) held value
  foreign size=(v)
  foreign spread=(v)     // Room stereo width (NOT width= — that's the Osc's PWM)
  foreign rate=(v)
  foreign depth=(v)
  foreign regen=(v)     // Flanger feedback (NOT feedback= — that's the Osc's)
  foreign freq=(v)
  foreign cutoff=(v)
  foreign res=(v)
  foreign pitch=(v)     // Resonator freq (port 1) — do not use freq= (port 0 = exciter)
  foreign damping=(v)   // Resonator damping (port 2)
  foreign pm=(v)
  foreign width=(v)
  foreign position=(v)
  foreign feedback=(v)
  foreign structure=(v)
  foreign brightness=(v)
  foreign strike=(v)     // resonator strike position (position= is the wavetable's)
  foreign gate(on)
  foreign trigger()
  foreign out(p)
  foreign free()
  // `isPoly_` distinguishes a real per-voice AUDIO signal (an oscillator,
  // filter, noise, sync, wavetable, or a `*`/`+` combining one — amp must
  // come from Env.ar, so a scalar Num is refused) from a control-rate Node
  // (the voice `pitch`, a mono LFO/Ctrl, or a chain built purely from those —
  // e.g. `LFO.sine(4).to(0.2, 0.8)` for PWM, or `p * 1.5` for a sync ratio):
  // a scalar operand on THOSE is folded in via a broadcast `Ctrl` node (Sy-2d
  // §0) rather than refused.
  *(o) {
    if (Node.polyMode_ == 1) {
      if (o is Num) {
        if (this.isPoly_ == 1) Fiber.abort("multiply by a constant inside a Synth isn't supported yet — the amp comes from Env.ar")
        return Node.polymul_(this, Node.ctrl_(o))
      }
      return Node.polymul_(this, o)
    }
    return Node.binop_(0, this, o)
  }
  +(o) {
    if (Node.polyMode_ == 1) {
      if (o is Num) {
        if (this.isPoly_ == 1) Fiber.abort("`+` a constant inside a Synth isn't supported yet")
        return Node.polyadd_(this, Node.ctrl_(o))
      }
      return Node.polyadd_(this, o)
    }
    return Node.binop_(1, this, o)
  }
  -(o) {
    if (Node.polyMode_ == 1) Fiber.abort("`-` inside a Synth isn't supported yet")
    return Node.binop_(2, this, o)
  }
  lpf(cutoff) {
    if (Node.polyMode_ == 1) return Node.polysvf_(this, cutoff, 0.2)
    return Node.lpf_(this, cutoff)
  }
  to(lo, hi) { this * ((hi - lo) / 2) + ((hi + lo) / 2) }
  atten(k)       { this * k }
  offset(c)      { this + c }
  invert()       { this * -1 }
  unipolar()     { this * 0.5 + 0.5 }   // [-1,1] → [0,1]
  bipolar()      { this * 2 - 1 }       // [0,1] → [-1,1]
  scale(m, a)    { this * m + a }
  curve(k) {
    if (Node.polyMode_ == 1) Fiber.abort("curve not usable in a Synth yet (Sy-2c)")
    return Node.curve_(this, k)
  }
  steps(n) {
    if (Node.polyMode_ == 1) Fiber.abort("steps not usable in a Synth yet (Sy-2c)")
    return Node.qstep_(this, n)
  }
  quantize(s, r) {
    if (Node.polyMode_ == 1) Fiber.abort("quantize not usable in a Synth yet (Sy-2c)")
    return Node.qpitch_(this, s, r)
  }
  hz(ref) {
    if (Node.polyMode_ == 1) Fiber.abort("hz not usable in a Synth yet (Sy-2c)")
    return Node.mtof_(this, ref)
  }
}

// A polyphonic instrument. Build a voice once with a `{ |pitch| … }` closure;
// the plain factories emit poly nodes inside it. Route `.out` and play it:
//   var bass = Synth.new { |pitch| Osc.sine(pitch).lpf(1200) * Env.ar(0.01, 0.3) }
//   Out.patch(bass.out)
//   bass.bindMidi()
foreign class Synth {
  foreign noteOn(note, vel)
  foreign noteOff(note)
  foreign out
  foreign isMono_               // true if built via Synth.mono (has a PolySlew node)
  foreign setGlide_(seconds)    // native glide set (renamed; guarded by `glide=` below)
  // Guard the M1 poly-glide footgun: `Synth.new` (poly) has no per-voice slew
  // node, so `glide=` on a poly synth would otherwise be a silent no-op.
  // Mirrors the Sy-2e `Bus.write_`/`write` guard pattern above.
  glide=(seconds) {
    if (isMono_ != 1) Fiber.abort("glide has no meaning on a poly Synth — use Synth.mono")
    setGlide_(seconds)
  }
  bindMidi() {
    Midi.onNoteOn = Fn.new { |ch, note, vel| this.noteOn(note, vel) }
    Midi.onNoteOff = Fn.new { |ch, note, vel| this.noteOff(note) }
  }
  static new(builder) {
    if (Node.polyMode_ == 1) Fiber.abort("nested Synth not supported")
    var pitch = Node.polyBegin_()
    var out
    if (builder.arity >= 2) {
      var vel = Node.polyVelBegin_()
      out = builder.call(pitch, vel)
    } else {
      out = builder.call(pitch)
    }
    if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
    if (Node.polyGateCount_ > 4) Fiber.abort("more than 4 envelopes per voice isn't supported")
    return Node.polyEnd_(out)
  }
  // A single mono/legato voice with true glide between overlapping notes:
  // the last-note-priority allocator gates ON only from silence and glides
  // the pitch (via `.glide = seconds`) on legato note-ons instead of
  // re-triggering the envelope. `.glide` ABORTS on `Synth.new` (poly) — there
  // is no per-voice slew node to affect (M1).
  //   var s = Synth.mono { |p| Osc.saw(p).lpf(1500) * Env.adsr(0.005,0.1,0.7,0.2) }
  //   s.glide = 0.08
  //   Out.patch(s.out)
  static mono(builder) {
    if (Node.polyMode_ == 1) Fiber.abort("nested Synth not supported")
    var pitch = Node.monoBegin_()
    var out
    if (builder.arity >= 2) {
      var vel = Node.polyVelBegin_()
      out = builder.call(pitch, vel)
    } else {
      out = builder.call(pitch)
    }
    if (Node.polyGateCount_ == 0) Fiber.abort("a Synth voice needs an Env.ar (the amp gate)")
    if (Node.polyGateCount_ > 4) Fiber.abort("more than 4 envelopes per voice isn't supported")
    return Node.monoEnd_(out)
  }
}

// A multi-output port: `node.out(p)` returns a handle to output port `p` of
// a multi-output node (e.g. `Split`), usable anywhere a Node/number is
// (including as the left operand of the arithmetic operators below).
foreign class Port {
  *(o) {
    if (Node.polyMode_ == 1) {
      if (o is Num) Fiber.abort("multiply by a constant inside a Synth isn't supported yet — the amp comes from Env.ar")
      return Node.polymul_(this, o)
    }
    return Node.binop_(0, this, o)
  }
  +(o) {
    if (Node.polyMode_ == 1) {
      if (o is Num) Fiber.abort("`+` a constant inside a Synth isn't supported yet")
      return Node.polyadd_(this, o)
    }
    return Node.binop_(1, this, o)
  }
  -(o) {
    if (Node.polyMode_ == 1) Fiber.abort("`-` inside a Synth isn't supported yet")
    return Node.binop_(2, this, o)
  }
  lpf(cutoff) {
    if (Node.polyMode_ == 1) return Node.polysvf_(this, cutoff, 0.2)
    return Node.lpf_(this, cutoff)
  }
  to(lo, hi) { this * ((hi - lo) / 2) + ((hi + lo) / 2) }
  atten(k)       { this * k }
  offset(c)      { this + c }
  invert()       { this * -1 }
  unipolar()     { this * 0.5 + 0.5 }   // [-1,1] → [0,1]
  bipolar()      { this * 2 - 1 }       // [0,1] → [-1,1]
  scale(m, a)    { this * m + a }
  curve(k) {
    if (Node.polyMode_ == 1) Fiber.abort("curve not usable in a Synth yet (Sy-2c)")
    return Node.curve_(this, k)
  }
  steps(n) {
    if (Node.polyMode_ == 1) Fiber.abort("steps not usable in a Synth yet (Sy-2c)")
    return Node.qstep_(this, n)
  }
  quantize(s, r) {
    if (Node.polyMode_ == 1) Fiber.abort("quantize not usable in a Synth yet (Sy-2c)")
    return Node.qpitch_(this, s, r)
  }
  hz(ref) {
    if (Node.polyMode_ == 1) Fiber.abort("hz not usable in a Synth yet (Sy-2c)")
    return Node.mtof_(this, ref)
  }
}

// A user-supplied dynamic wavetable, uploaded from a Wren list of samples
// (one base cycle; a band-limited mip pyramid is built natively on upload):
//   var w = Wavetable.from([-1, -0.5, 0, 0.5, 1, 0.5, 0, -0.5])
//   Out.patch(Osc.wavetable(w, 220))
// Distinct from the static `WT` ids below (baked-in tables, e.g. `WT.Saw`) —
// `Osc.wavetable` accepts either.
//
// Lifetime: the table's pool memory is node-scoped, not owned by this Wren
// object. It is released when the node bound to it (via `Osc.wavetable`) is
// freed — not by this object's GC. Bind a Wavetable to a node and free that
// node when done; don't rely on GC to reclaim it, and don't free a node
// while another node still shares the same Wavetable (that frees the table
// out from under the survivor). Misuse degrades gracefully (silence, or a
// leak until the pool is exhausted) — never undefined behavior.
//
// `Wavetable.from2d([[frame0...], [frame1...], ...])` uploads a *multi-frame*
// table (a nested list: one inner list per frame, each one base cycle) and
// builds one pyramid per frame. A node built from it (`Osc.wavetable`) morphs
// continuously across frames by `.position` (0 = frame 0, 1 = the last frame):
//   var w = Wavetable.from2d([[-1, 0, 1, 0], [-1, -1, 1, 1]])
//   var o = Osc.wavetable(w, 220)
//   o.position = 0.5   // halfway between frame 0 and frame 1
// Same lifetime contract as `from` above.
foreign class Wavetable {
  foreign static from(samples)
  foreign static from2d(frames)
}

class Osc {
  static sine(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 0)
    return Node.src_(0, f)
  }
  static saw(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 1)
    return Node.src_(1, f)
  }
  static square(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 2)
    return Node.src_(2, f)
  }
  static tri(f) {
    if (Node.polyMode_ == 1) return Node.polyosc_(f, 3)
    return Node.src_(3, f)
  }
  // Hard sync: `master` resets `slave`'s phase each cycle, locking the
  // slave's pitch to the master's (a classic sync-lead timbre). `slave`
  // is the audible waveform; `master` sets the fundamental.
  static syncSine(master, slave) {
    if (Node.polyMode_ == 1) return Node.polysync_(0, master, slave)
    return Node.sync_(0, master, slave)
  }
  static syncSaw(master, slave) {
    if (Node.polyMode_ == 1) return Node.polysync_(1, master, slave)
    return Node.sync_(1, master, slave)
  }
  static syncSquare(master, slave) {
    if (Node.polyMode_ == 1) return Node.polysync_(2, master, slave)
    return Node.sync_(2, master, slave)
  }
  static syncTri(master, slave) {
    if (Node.polyMode_ == 1) return Node.polysync_(3, master, slave)
    return Node.sync_(3, master, slave)
  }
  static wavetable(t, f) {
    if (Node.polyMode_ == 1) {
      if (t is Wavetable) return Node.polywt_pooled_(t, f)
      return Node.polywt_(t, f) // WT.x numeric id (static table)
    }
    if (t is Wavetable) return Node.wavetable_pooled_(t, f)
    return Node.wavetable_(t, f) // WT.x numeric id (static table)
  }
}

// Low-frequency modulator. Bipolar [-1,1]; scale with `.to(min, max)`:
//   var lfo = LFO.tri(0.5)
//   filter.cutoff = lfo.to(200, 2000)
//   var o = Osc.saw(110); o.width = LFO.sine(4).to(0.1, 0.9)   // PWM
//   lfo.freq = 2      // rate is port 0
//   lfo.trigger()     // retrigger (sync to a note)
class LFO {
  static sine(rate)       { Node.lfo_(rate, 0) }
  static tri(rate)        { Node.lfo_(rate, 1) }
  static saw(rate)        { Node.lfo_(rate, 2) }
  static square(rate)     { Node.lfo_(rate, 3) }
  static sampleHold(rate) { Node.lfo_(rate, 4) }
  static random(rate)     { Node.lfo_(rate, 5) }
}

// Sample & hold: latch `input` on each rising edge of `clock` (any signal — an
// LFO, a square, a metro):
//   var rnd = SampleHold.new(Noise.pink(), Osc.square(4))   // stepped random
class SampleHold {
  static new(input, clock) {
    if (Node.polyMode_ == 1) Fiber.abort("SampleHold not usable in a Synth yet (Sy-2c)")
    return Node.sh_(input, clock)
  }
}

// Slew / glide: one-pole lag over `time` seconds. Smooths steps / portamento:
//   osc.freq = Slew.new(pitchSeq, 0.02)
class Slew {
  static new(input, time) {
    if (Node.polyMode_ == 1) Fiber.abort("Slew not usable in a Synth yet (Sy-2c)")
    return Node.slew_(input, time)
  }
}

// Step sequencer: cycles a list of values, one per rising `clock` edge (up to 16
// steps; the first clock plays step 0):
//   var seq = Steps.new([0, 7, 5, 12], Osc.square(2))
//   osc.freq = seq.to(110, 880)
class Steps {
  static new(values, clock) {
    if (Node.polyMode_ == 1) Fiber.abort("Steps not usable in a Synth yet (Sy-2c)")
    return Node.steps_(values, clock)
  }
}

// Non-linear response curve (Schlick bias), odd-symmetric on [-1,1].
//   var shaped = env.curve(0.6)          // ease-in
//   Out.patch(Osc.saw(110) * env.curve(-0.4))
class Curve {
  static new(sig, k) {
    if (Node.polyMode_ == 1) Fiber.abort("Curve not usable in a Synth yet (Sy-2c)")
    return Node.curve_(sig, k)
  }
  static exp(sig) {
    if (Node.polyMode_ == 1) Fiber.abort("Curve not usable in a Synth yet (Sy-2c)")
    return Node.curve_(sig, 0.6)
  }
  static log(sig) {
    if (Node.polyMode_ == 1) Fiber.abort("Curve not usable in a Synth yet (Sy-2c)")
    return Node.curve_(sig, -0.6)
  }
}

// A macro control: one settable value that fans out to many destinations
// (each via its own `* depth`/`.to(...)`). Drive it live from an encoder:
//   var m = Macro.new(0.5)
//   filter.cutoff = m.to(200, 2000)
//   osc.width = m * 0.3 + 0.5
//   Enc.onTurn = Fn.new { |i, d| m.value = (m.value + d * 0.05) }
class Macro {
  static new(v) { Node.ctrl_(v) }
}

// Musical scales as 12-bit pitch-class masks (bit i ⇒ pc i allowed, relative to
// the root passed to `.quantize`). Use with a semitone signal:
//   var note = seq.to(0, 24).quantize(Scale.Minor, 0)
//   osc.freq = note.hz(220)
class Scale {
  static Chromatic        { 4095 }  // 0xFFF, {0..11}
  static Major            { 2741 }  // {0,2,4,5,7,9,11}
  static Minor            { 1453 }  // {0,2,3,5,7,8,10}  (natural)
  static HarmonicMinor    { 2477 }  // {0,2,3,5,7,8,11}
  static Dorian           { 1709 }  // {0,2,3,5,7,9,10}
  static Mixolydian       { 1717 }  // {0,2,4,5,7,9,10}
  static MajorPentatonic  { 661 }   // {0,2,4,7,9}
  static MinorPentatonic  { 1193 }  // {0,3,5,7,10}
  static WholeTone        { 1365 }  // {0,2,4,6,8,10}
  static Blues            { 1257 }  // {0,3,5,6,7,10}
}

// State-variable filter: LP/HP/BP/notch from one topology, resonant and
// audio-rate-modulatable. Each factory returns a Node you can modulate:
//   var f = Svf.lp(Osc.saw(110), 1200, 0.6)
//   f.cutoff = Osc.sine(3) * 400 + 1200   // wobble
//   f.res = 0.9
class Svf {
  static lp(input, cutoff, res) {
    if (Node.polyMode_ == 1) Fiber.abort("Svf inside a Synth isn't poly yet — use .lpf(cutoff) for a poly lowpass (Sy-2c)")
    return Node.svf_(input, cutoff, res, 0)
  }
  static hp(input, cutoff, res) {
    if (Node.polyMode_ == 1) Fiber.abort("Svf inside a Synth isn't poly yet — use .lpf(cutoff) for a poly lowpass (Sy-2c)")
    return Node.svf_(input, cutoff, res, 1)
  }
  static bp(input, cutoff, res) {
    if (Node.polyMode_ == 1) Fiber.abort("Svf inside a Synth isn't poly yet — use .lpf(cutoff) for a poly lowpass (Sy-2c)")
    return Node.svf_(input, cutoff, res, 2)
  }
  static notch(input, cutoff, res) {
    if (Node.polyMode_ == 1) Fiber.abort("Svf inside a Synth isn't poly yet — use .lpf(cutoff) for a poly lowpass (Sy-2c)")
    return Node.svf_(input, cutoff, res, 3)
  }
}

// TB-303 diode-ladder filter — the acid-bass lowpass. Resonant and
// audio-rate-modulatable; sweep cutoff with an envelope for the classic squelch:
//   var f = Tb303.lp(Osc.saw(55), 400, 0.9)
//   f.cutoff = Env.ar(0.0, 0.3) * 1500 + 200
class Tb303 {
  static lp(input, cutoff, res) {
    if (Node.polyMode_ == 1) Fiber.abort("Tb303 not usable in a Synth yet (Sy-2c)")
    return Node.tb303_(input, cutoff, res)
  }
}

// Moog transistor-ladder — the warm, self-oscillating classic. 24 dB (lp) or 12 dB (lp2);
// `drive` overdrives the ladder for growl:
//   var f = Moog.lp(Osc.saw(55), 800, 0.85)
//   f.cutoff = Env.ar(0.0, 0.4) * 4000 + 200
//   f.drive = 3
class Moog {
  static lp(input, cutoff, res)  {
    if (Node.polyMode_ == 1) return Node.polymoog_(input, cutoff, res, 4)
    return Node.moog_(input, cutoff, res, 4)
  }
  static lp2(input, cutoff, res) {
    if (Node.polyMode_ == 1) return Node.polymoog_(input, cutoff, res, 2)
    return Node.moog_(input, cutoff, res, 2)
  }
}

// Korg MS-20 (Korg35) Sallen-Key — the screaming, diode-clipped 2-pole. LP or HP;
// crank `res` for the self-oscillating scream, `drive` for grit:
//   var f = Ms20.hp(Osc.saw(110), 1200, 0.9)
//   f.drive = 4
class Ms20 {
  static lp(input, cutoff, res) {
    if (Node.polyMode_ == 1) return Node.polyms20_(input, cutoff, res, 0)
    return Node.ms20_(input, cutoff, res, 0)
  }
  static hp(input, cutoff, res) {
    if (Node.polyMode_ == 1) return Node.polyms20_(input, cutoff, res, 1)
    return Node.ms20_(input, cutoff, res, 1)
  }
}

// Modal resonator — a struck/plucked bank of tuned modes (strings, bells, plates).
// Excite it with an input; ring it with pitch/damping; shape it with structure/brightness/strike:
//   var body = Resonator.new(Noise.pink() * Env.ar(0.0, 0.02), 220, 0.4)
//   body.structure = 0.3    // toward inharmonic
//   body.brightness = 0.8
//   body.strike = 0.2       // strike position (comb)
//   body.pitch = 330        // retune (NOT freq= — that targets the exciter input)
//   body.damping = 0.6      // shorter ring
class Resonator {
  static new(input, freq, damping) {
    if (Node.polyMode_ == 1) Fiber.abort("Resonator not usable in a Synth yet (Sy-2c)")
    return Node.modal_(input, freq, damping)
  }
}

// Constant-power stereo pan: a mono input placed in the stereo field.
// `position` -1 = hard left, 0 = center (-3 dB), +1 = hard right; it is a
// port, so it can be modulated (auto-pan):
//   Out.patch(Pan.new(Osc.saw(110), -0.3))
//   var p = Pan.new(pad, 0)
//   p.freq = ...            // (position is arg 2 at construction; patch via a node)
// Pan is a stereo (width-2) node: Out.patch / a bus routes its L/R to the
// stereo output.
class Pan {
  static new(input, position) {
    if (Node.polyMode_ == 1) Fiber.abort("Pan is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.pan_(input, position)
  }
}

// Feedback delay — echoes with damped repeats. `time` in seconds (up to ~1 s),
// `feedback` [0, 0.98], `mix` dry/wet, `damp` darkens the repeats:
//   var d = Delay.new(Osc.saw(110) * Env.ar(0.0, 0.2), 0.25, 0.5)
//   d.mix = 0.4
//   d.damp = 0.3
//   Out.patch(d)
// The ring buffer is node-scoped pool memory (like a pooled Wavetable): it is
// released when the node is freed. `time`/`feedback` can be modulated (they are
// ports); `mix`/`damp` are control params.
class Delay {
  static new(input, time, feedback) {
    if (Node.polyMode_ == 1) Fiber.abort("Delay is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.delay_(input, time, feedback)
  }
}

// Chorus — a lush multi-voice stereo modulated delay. `rate` LFO Hz, `depth`
// [0,1] sweep, `mix` dry/wet. Stereo (width-2); patch it straight out:
//   Out.patch(Chorus.new(Osc.saw(110), 0.5, 0.4, 0.5))
//   var c = Chorus.new(pad, 0.3, 0.6, 0.5); c.rate = 0.8; c.depth = 0.7
class Chorus {
  static new(input, rate, depth, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Chorus is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.chorus_(input, rate, depth, mix)
  }
}

// Flanger — a swept single-voice comb with feedback (`regen`). Short delay,
// jet-sweep. `rate`/`depth` as chorus; `feedback` [0,0.9] is the resonance:
//   Out.patch(Flanger.new(Osc.saw(110), 0.3, 0.7, 0.6, 0.5))
//   var f = Flanger.new(pad, 0.2, 0.8, 0.7, 0.5); f.regen = 0.8
class Flanger {
  static new(input, rate, depth, feedback, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Flanger is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.flanger_(input, rate, depth, feedback, mix)
  }
}

// Room reverb (Schroeder-Moorer). `roomsize` [0,1] decay/size, `damp` [0,1]
// high-frequency absorption, `mix` dry/wet. Stereo (width-2):
//   Out.patch(Room.new(Osc.saw(110), 0.7, 0.4, 0.4))
//   var r = Room.new(pad, 0.8, 0.3, 0.5); r.size = 0.9; r.damp = 0.6; r.spread = 0.8
class Room {
  static new(input, roomsize, damp, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Room is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.room_(input, roomsize, damp, mix)
  }
}

// Hall reverb (8-line modulated FDN) — a dense, smooth, lush tail. `size` [0,1]
// decay/length, `damp` [0,1] HF absorption, `mix` dry/wet. Stereo (width-2);
// reuses the Room controls (size=/damp=/spread=):
//   Out.patch(Hall.new(Osc.saw(110), 0.85, 0.4, 0.4))
//   var h = Hall.new(pad, 0.9, 0.3, 0.5); h.size = 0.95; h.spread = 0.8
class Hall {
  static new(input, size, damp, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Hall is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.hall_(input, size, damp, mix)
  }
}

// Plate reverb (Dattorro) — a bright, dense, metallic-smooth plate. `size` [0,1]
// decay/length, `damp` [0,1] HF absorption, `mix` dry/wet. Stereo (width-2);
// reuses the Room controls (size=/damp=/spread=):
//   Out.patch(Plate.new(Osc.saw(110), 0.85, 0.4, 0.4))
//   var p = Plate.new(pad, 0.9, 0.3, 0.5); p.size = 0.95; p.spread = 0.8
class Plate {
  static new(input, size, damp, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Plate is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.plate_(input, size, damp, mix)
  }
}

// Waveshaper / distortion. `drive` [0,1] amount, `tone` [0,1] brightness, `mix`
// dry/wet. Four characters; 4× oversampled:
//   Out.patch(Drive.hard(Osc.saw(110), 0.8, 0.6, 1.0))
//   var d = Drive.tube(pad, 0.6, 0.7, 0.5); d.drive = 0.9; d.tone = 0.4; d.wet = 0.8
class Drive {
  static soft(input, drive, tone, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Drive is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.drive_(input, drive, tone, mix, 0)
  }
  static hard(input, drive, tone, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Drive is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.drive_(input, drive, tone, mix, 1)
  }
  static fold(input, drive, tone, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Drive is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.drive_(input, drive, tone, mix, 2)
  }
  static tube(input, drive, tone, mix) {
    if (Node.polyMode_ == 1) Fiber.abort("Drive is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.drive_(input, drive, tone, mix, 3)
  }
}

// Parametric EQ band (RBJ). `freq` Hz, `gain` dB, `q` bandwidth. Three types:
//   Out.patch(EQ.peak(Osc.saw(110), 800, 6, 1.5))
//   var e = EQ.lowShelf(pad, 200, -4, 0.707); e.hz = 250; e.gain = -6; e.q = 0.8
// (LP/HP/BP/notch live on `Svf` — this adds the gain-shaping bands.)
class EQ {
  static peak(input, freq, gain, q) {
    if (Node.polyMode_ == 1) Fiber.abort("EQ is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.eq_(input, freq, gain, q, 0)
  }
  static lowShelf(input, freq, gain, q) {
    if (Node.polyMode_ == 1) Fiber.abort("EQ is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.eq_(input, freq, gain, q, 1)
  }
  static highShelf(input, freq, gain, q) {
    if (Node.polyMode_ == 1) Fiber.abort("EQ is an effect — apply it after the Synth's .out, not inside the voice")
    return Node.eq_(input, freq, gain, q, 2)
  }
}

// Named static wavetable ids, in the generated `TABLES` registry order
// (deluge-dsp-kernels' `wavetables_generated.rs`). Ids 0-5 are single-cycle
// tables (one frame; `Osc.wavetable(WT.x, f)` plays them directly). Ids 6-7
// are named *2D* (multi-frame) morph banks, built at `gen_tables` time (no
// runtime build): bind them the same way, then sweep `.position` (0 = first
// frame, 1 = last frame) to morph, e.g.:
//   var o = Osc.wavetable(WT.HarmonicSweep, 220)
//   o.position = lfo   // sweeps near-sine -> bright saw
//   Out.patch(o)
class WT {
  static Saw { 0 }
  static Square { 1 }
  static Sine { 2 }
  static Tri { 3 }
  static Organ { 4 }
  static Formant { 5 }
  static HarmonicSweep { 6 }
  static FormantMorph { 7 }
}

class Env {
  static ar(attack, release) {
    if (Node.polyMode_ == 1) return Node.polyar_(attack, release)
    return Node.env_(attack, release)
  }
  static adsr(attack, decay, sustain, release) {
    if (Node.polyMode_ == 1) return Node.polyadsr_(attack, decay, sustain, release)
    return Node.adsr_(attack, decay, sustain, release)
  }
}

class Noise {
  static new() {
    if (Node.polyMode_ == 1) return Node.polynoise_()
    return Node.noise_()
  }
  static pink() {
    if (Node.polyMode_ == 1) return Node.polypink_()
    return Node.pink_()
  }
  static brown() {
    if (Node.polyMode_ == 1) return Node.polybrown_()
    return Node.brown_()
  }
}

// A width-2 test node: routes its input to both output ports 0 and 1.
//   var s = Split.new(osc); s.out(0); s.out(1)
class Split {
  static new(input) {
    if (Node.polyMode_ == 1) Fiber.abort("Split not usable in a Synth yet (Sy-2c)")
    return Node.split_(input)
  }
}

class Out {
  static patch(node) { Node.patch_(node) }
  static reset() { Node.reset_() }
}

// A bus is a mix/render target: `.write(src)` accumulates a signal into it,
// and `Out.patch(bus)` sets it as the render root directly (bypassing the
// master-bus sugar used for Node/Port).
//   var m = Bus.new()
//   m.write(Osc.saw(110))
//   Out.patch(m)
foreign class Bus {
  foreign static new_()      // returns a fresh Bus foreign in slot 0
  foreign write_(src)        // native bus write (renamed; guarded by `write` below)
  static new() { new_() }    // the public Bus.new() from the spec
  // Guard the Sy-2e silent-mono footgun: a poly voice signal written to a
  // (mono) bus inside a Synth bypasses the VoiceSum that `.out` inserts and is
  // read as a single interleaved row. A poly source is always a `Node`
  // (`return_poly_node`); Ports come only from Split etc., which abort in poly
  // mode, so `src is Node` fully covers it. A Num/mono write is safe.
  write(src) {
    if (Node.polyMode_ == 1 && (src is Node) && src.isPoly_ == 1) {
      Fiber.abort("can't write a poly voice to a Bus inside a Synth — route the Synth's .out to a bus instead (Sy-2e)")
    }
    write_(src)
  }
}

// ── Accessors ────────────────────────────────────────────────────────────────
// Index 0 is left null so jacks read 1-based (output[1] = first CV jack).

var output = [null, Output.new(0), Output.new(1)]
var gate = [null, Gate.new(0), Gate.new(1), Gate.new(2), Gate.new(3)]
