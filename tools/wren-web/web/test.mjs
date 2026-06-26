// Smoke test for the wren-web wasm core, run under Node's WASI.
//   node --experimental-wasi-unstable-preview1 web/test.mjs <path-to.wasm>
import { readFileSync } from "node:fs";
import { WASI } from "node:wasi";
import { loadSim } from "./loader.mjs";

const wasmPath = process.argv[2] ?? "target/wasm32-unknown-unknown/release/wren_web.wasm";
const wasi = new WASI({ version: "preview1", returnOnExit: true });
const sim = await loadSim(readFileSync(wasmPath), wasi.wasiImport, (inst) => wasi.initialize(inst));

let failures = 0;
const check = (name, cond, detail = "") => {
  console.log(`${cond ? "ok  " : "FAIL"}  ${name}${detail ? ` — ${detail}` : ""}`);
  if (!cond) failures++;
};

// 1. Load a script exercising OLED, MIDI handler, CV, and audio patch.
const res = sim.load(`
Oled.clear()
Oled.text(0, 0, "HELLO")
Oled.show()
Midi.onNoteOn = Fn.new { |ch, note, vel|
  output[1].volts = note / 12.0
  System.print("note %(note)")
}
Out.patch(Osc.saw(110))
System.print("loaded")
`);
check("script loads without error", res.ok, `code=${res.code} ${res.error}`);
check("System.print captured", res.output.includes("loaded"), JSON.stringify(res.output));

// 2. OLED text rendered some pixels.
const lit = sim.oled().px.reduce((n, v) => n + (v ? 1 : 0), 0);
check("OLED 'HELLO' rasterised pixels", lit > 20, `${lit} lit pixels`);

// 3. MIDI note-on drives the handler → CV + print. CV flushes to the host on the
//    next tick (same as the firmware's continuous vm_task tick).
sim.clearOutput();
sim.midiIn(0x90, 60, 100); // note 60 → output[1] = 60/12 = 5.0 V
sim.tick(10, 0.01);        // flush control-rate CV
check("MIDI handler set CV jack 1 to 5.0 V", Math.abs(sim.cv(0) - 5.0) < 1e-4, `cv0=${sim.cv(0)}`);
check("MIDI handler print fired", sim.output().includes("note 60"), JSON.stringify(sim.output()));

// 4. Metro fires on tick after its interval elapses. A value set inside the
//    callback flushes on the following tick (render runs before metros fire).
const metro = sim.load(`
var m = Metro.new()
m.start(Fn.new { |stage|
  gate[1].on = true
}, 0.1)
`);
check("metro script loads", metro.ok, `code=${metro.code} ${metro.error}`);
sim.tick(0, 0.0);     // arm at t=0
sim.tick(200, 0.2);   // 200 ms later: the 100 ms metro is due → callback runs
sim.tick(210, 0.01);  // flush the gate the callback set
check("metro fired → gate 1 on", (sim.gateBits() & 1) !== 0, `gateBits=${sim.gateBits()}`);

// 5. Compile error is reported with a line number.
const bad = sim.load("this is not valid wren @@@");
check("compile error reported", bad.code === 1 && bad.error.length > 0, `code=${bad.code} line=${bad.errorLine}`);

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
