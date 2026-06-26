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
  foreign static env_(attack, release)
  foreign static noise_()
  foreign static binop_(op, a, b)
  foreign static lpf_(input, cutoff)
  foreign static patch_(node)
  foreign static reset_()
  foreign freq=(v)
  foreign cutoff=(v)
  foreign gate(on)
  foreign trigger()
  *(o) { Node.binop_(0, this, o) }
  +(o) { Node.binop_(1, this, o) }
  -(o) { Node.binop_(2, this, o) }
  lpf(cutoff) { Node.lpf_(this, cutoff) }
}

class Osc {
  static sine(f) { Node.src_(0, f) }
  static saw(f) { Node.src_(1, f) }
  static square(f) { Node.src_(2, f) }
  static tri(f) { Node.src_(3, f) }
}

class Env {
  static ar(attack, release) { Node.env_(attack, release) }
}

class Noise {
  static new() { Node.noise_() }
}

class Out {
  static patch(node) { Node.patch_(node) }
  static reset() { Node.reset_() }
}

// ── Accessors ────────────────────────────────────────────────────────────────
// Index 0 is left null so jacks read 1-based (output[1] = first CV jack).

var output = [null, Output.new(0), Output.new(1)]
var gate = [null, Gate.new(0), Gate.new(1), Gate.new(2), Gate.new(3)]
