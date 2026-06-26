// Starter scripts, mirroring wren-firmware/examples. Each is self-contained and
// drives a different part of the simulated surface.
export interface Example {
  name: string;
  source: string;
}

export const EXAMPLES: Example[] = [
  {
    name: "hello oled",
    source: `// Draw to the 128x48 OLED.
Oled.clear()
Oled.text(8, 8, "WREN DELUGE")
Oled.text(8, 22, "LIVE CODING")
Oled.show()
System.print("drew to oled")
`,
  },
  {
    name: "midi synth",
    source: `// Monophonic synth: MIDI notes drive a filtered saw + an envelope,
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
  },
  {
    name: "pad paint",
    source: `// Light a pad when you press it.
Pads.onPress = Fn.new { |x, y|
  Oled.clear()
  Oled.text(8, 16, "PAD %(x) %(y)")
  Oled.show()
  System.print("pad %(x),%(y)")
}
System.print("press the pads")
`,
  },
  {
    name: "cv lfo",
    source: `// A metro sweeps CV jack 1 as a stepped LFO.
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
  },
];
