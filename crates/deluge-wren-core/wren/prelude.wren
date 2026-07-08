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
  foreign static noise_()
  foreign static pink_()
  foreign static brown_()
  foreign static binop_(op, a, b)
  foreign static lpf_(input, cutoff)
  foreign static svf_(input, cutoff, res, resp)
  foreign static patch_(node)
  foreign static reset_()
  foreign static split_(input)
  foreign static wavetable_(table, freq)
  foreign static wavetable_pooled_(wt, freq)
  foreign freq=(v)
  foreign cutoff=(v)
  foreign res=(v)
  foreign pm=(v)
  foreign width=(v)
  foreign position=(v)
  foreign feedback=(v)
  foreign gate(on)
  foreign trigger()
  foreign out(p)
  foreign free()
  *(o) { Node.binop_(0, this, o) }
  +(o) { Node.binop_(1, this, o) }
  -(o) { Node.binop_(2, this, o) }
  lpf(cutoff) { Node.lpf_(this, cutoff) }
}

// A multi-output port: `node.out(p)` returns a handle to output port `p` of
// a multi-output node (e.g. `Split`), usable anywhere a Node/number is
// (including as the left operand of the arithmetic operators below).
foreign class Port {
  *(o) { Node.binop_(0, this, o) }
  +(o) { Node.binop_(1, this, o) }
  -(o) { Node.binop_(2, this, o) }
  lpf(cutoff) { Node.lpf_(this, cutoff) }
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
  static sine(f) { Node.src_(0, f) }
  static saw(f) { Node.src_(1, f) }
  static square(f) { Node.src_(2, f) }
  static tri(f) { Node.src_(3, f) }
  // Hard sync: `master` resets `slave`'s phase each cycle, locking the
  // slave's pitch to the master's (a classic sync-lead timbre). `slave`
  // is the audible waveform; `master` sets the fundamental.
  static syncSine(master, slave) { Node.sync_(0, master, slave) }
  static syncSaw(master, slave) { Node.sync_(1, master, slave) }
  static syncSquare(master, slave) { Node.sync_(2, master, slave) }
  static syncTri(master, slave) { Node.sync_(3, master, slave) }
  static wavetable(t, f) {
    if (t is Wavetable) return Node.wavetable_pooled_(t, f)
    return Node.wavetable_(t, f) // WT.x numeric id (static table)
  }
  static pink() { Node.pink_() }
  static brown() { Node.brown_() }
}

// State-variable filter: LP/HP/BP/notch from one topology, resonant and
// audio-rate-modulatable. Each factory returns a Node you can modulate:
//   var f = Svf.lp(Osc.saw(110), 1200, 0.6)
//   f.cutoff = Osc.sine(3) * 400 + 1200   // wobble
//   f.res = 0.9
class Svf {
  static lp(input, cutoff, res) { Node.svf_(input, cutoff, res, 0) }
  static hp(input, cutoff, res) { Node.svf_(input, cutoff, res, 1) }
  static bp(input, cutoff, res) { Node.svf_(input, cutoff, res, 2) }
  static notch(input, cutoff, res) { Node.svf_(input, cutoff, res, 3) }
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
  static ar(attack, release) { Node.env_(attack, release) }
}

class Noise {
  static new() { Node.noise_() }
}

// A width-2 test node: routes its input to both output ports 0 and 1.
//   var s = Split.new(osc); s.out(0); s.out(1)
class Split {
  static new(input) { Node.split_(input) }
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
  foreign write(src)
  static new() { new_() }    // the public Bus.new() from the spec
}

// ── Accessors ────────────────────────────────────────────────────────────────
// Index 0 is left null so jacks read 1-based (output[1] = first CV jack).

var output = [null, Output.new(0), Output.new(1)]
var gate = [null, Gate.new(0), Gate.new(1), Gate.new(2), Gate.new(3)]
