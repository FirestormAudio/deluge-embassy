// Starter projects, mirroring wren-firmware/examples. Most are one file; the last
// demonstrates multi-file imports.
export interface Example {
  name: string;
  entry: string;
  files: Record<string, string>;
}

const one = (name: string, source: string): Example => ({ name, entry: "main.wren", files: { "main.wren": source } });

export const EXAMPLES: Example[] = [
  one(
    "drone",
    `// A continuous filtered-saw drone with a slow vibrato LFO. Press Run to hear it.
var lfo = Osc.sine(5)
var voice = Osc.saw(110)
voice.freq = lfo * 4 + 110   // Node first: 110 + lfo would be Num + Node (undefined)
Out.patch(voice.lpf(900))
System.print("drone patched - you should hear a tone")
`,
  ),
  one(
    "hello oled",
    `// Draw to the 128x48 OLED.
Oled.clear()
Oled.text(8, 8, "WREN DELUGE")
Oled.text(8, 22, "LIVE CODING")
Oled.show()
System.print("drew to oled")
`,
  ),
  one(
    "midi synth",
    `// Monophonic synth: MIDI notes drive a filtered saw + an envelope,
// and mirror pitch to CV jack 1 and gate 1.
var pitch = Osc.saw(110)
var filt  = pitch.lpf(1200)
var amp   = Env.ar(0.005, 0.3)
Out.patch(filt * amp)

var noteToHz = Fn.new { |n| 440.0 * (2.0).pow((n - 69) / 12.0) }

Midi.onNoteOn = Fn.new { |ch, note, vel|
  pitch.freq = noteToHz.call(note)
  amp.gate(true)
  output[1].volts = note / 12.0
  gate[1].on = true
  Oled.clear()
  Oled.text(8, 16, "NOTE %(note)")
  Oled.show()
}
Midi.onNoteOff = Fn.new { |ch, note, vel|
  amp.gate(false)
  gate[1].on = false
}
System.print("midi synth ready - play the keyboard")
`,
  ),
  one(
    "pad paint",
    `// Light a pad when you press it.
Pads.onPress = Fn.new { |x, y|
  Oled.clear()
  Oled.text(8, 16, "PAD %(x) %(y)")
  Oled.show()
  System.print("pad %(x),%(y)")
}
System.print("press the pads")
`,
  ),
  one(
    "cv lfo",
    `// A metro sweeps CV jack 1 as a stepped LFO.
output[1].slew = 0.08
var m = Metro.new()
m.start(Fn.new { |stage|
  var v = (stage % 8) * 0.6
  output[1].volts = v
  gate[1].on = stage % 2 == 0
  Oled.clear()
  Oled.text(8, 16, "CV %(v)")
  Oled.show()
}, 0.2)
System.print("lfo running")
`,
  ),
  {
    name: "imports (multi-file)",
    entry: "main.wren",
    files: {
      "main.wren": `// A multi-file project: main imports a Voice from lib/voice.wren.
import "lib/voice" for Voice

var v = Voice.new(110)
Out.patch(v.out)

Midi.onNoteOn = Fn.new { |ch, note, vel|
  v.play(note)
  Oled.clear()
  Oled.text(8, 16, "NOTE %(note)")
  Oled.show()
}
Midi.onNoteOff = Fn.new { |ch, note, vel| v.release() }
System.print("imports demo - play the keyboard")
`,
      "lib/voice.wren": `// A reusable voice: filtered saw + amplitude envelope, exposed via \`out\`.
class Voice {
  construct new(hz) {
    _osc = Osc.saw(hz)
    _amp = Env.ar(0.005, 0.3)
    _out = _osc.lpf(1200) * _amp
  }
  out { _out }
  play(note) {
    _osc.freq = 440.0 * (2.0).pow((note - 69) / 12.0)
    _amp.gate(true)
  }
  release() { _amp.gate(false) }
}
`,
    },
  },
];
