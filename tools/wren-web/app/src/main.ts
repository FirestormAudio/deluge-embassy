import "./style.css";
import { loadSim } from "./sim";
import { createEditor, setErrorMarker, setAnalyzerMarkers, analyzerMarkers, registerIntelligence } from "./editor";
import { Panel } from "./panel";
import { Audio } from "./audio";
import { Analyzer } from "./analyzer";
import { EXAMPLES } from "./examples";

// Served from public/ at the app's base URL; fetched + instantiated in sim.ts.
const wasmUrl = `${import.meta.env.BASE_URL}wren_web.wasm`;

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

async function boot() {
  const status = $("#status");
  const consoleEl = $("#console");

  const { editor } = createEditor($("#editor"), EXAMPLES[0].source);

  // Examples menu.
  const select = $<HTMLSelectElement>("#examples");
  for (const ex of EXAMPLES) {
    const opt = document.createElement("option");
    opt.value = ex.name;
    opt.textContent = ex.name;
    select.appendChild(opt);
  }
  select.value = EXAMPLES[0].name;
  select.addEventListener("change", () => {
    const ex = EXAMPLES.find((e) => e.name === select.value);
    if (ex) editor.setValue(ex.source);
  });

  const sim = await loadSim(wasmUrl);
  status.textContent = "booted";
  status.classList.add("ok");

  const panel = new Panel(
    $<HTMLCanvasElement>("#oled"),
    $("#pad-grid"),
    $("#cv-row"),
    $("#keyboard"),
    sim,
  );

  const log = (text: string, kind = "out") => {
    if (!text) return;
    for (const line of text.replace(/\n$/, "").split("\n")) {
      const row = document.createElement("div");
      row.className = `line ${kind}`;
      row.textContent = line;
      consoleEl.appendChild(row);
    }
    consoleEl.scrollTop = consoleEl.scrollHeight;
  };

  const workletUrl = `${import.meta.env.BASE_URL}wren-dsp.js`;
  const audio = new Audio(sim, wasmUrl, workletUrl);
  const audioState = $("#audio-state");

  // Live static analysis (off-thread). Squiggles update as you type, separate
  // from the VM's run-time errors (which appear on Run).
  const analyzer = new Analyzer(`${import.meta.env.BASE_URL}wren-analyzer.wasm`);
  registerIntelligence(analyzer); // hover, go-to-def, symbol completion
  let editVersion = 0;
  analyzer.onDiagnostics = (version, diags) => {
    if (version === editVersion) setAnalyzerMarkers(editor.getModel()!, diags);
  };
  const reanalyze = () => analyzer.analyze(editor.getValue(), ++editVersion);
  editor.onDidChangeModelContent(reanalyze);
  const scope = $<HTMLCanvasElement>("#scope");
  const scopeCtx = scope.getContext("2d")!;

  const run = () => {
    const res = sim.run(editor.getValue()); // fresh VM each run
    const model = editor.getModel()!;
    if (res.ok) {
      setErrorMarker(model, -1, "");
      log(res.output, "out");
    } else {
      setErrorMarker(model, res.errorLine, res.error.split("\n")[0] ?? "error");
      log(res.output, "out");
      log(res.error, "err");
    }
  };

  // Running is a user gesture, so it's also where we (re)start audio.
  const runWithAudio = async () => {
    run();
    await audio.start();
    audioState.textContent = "live";
    audioState.classList.add("on");
  };

  $("#run").addEventListener("click", runWithAudio);
  // Cmd/Ctrl-Enter to run.
  editor.addCommand(2048 | 3 /* KeyMod.CtrlCmd | KeyCode.Enter */, runWithAudio);

  // Audio scope — rendered from the main engine (audio itself plays in the
  // worklet). One block per frame; advancing the main engine for the waveform is
  // cheap and independent of the worklet's render.
  scope.width = 388;
  scope.height = 64;
  const scopeBuf = new Float32Array(1024);
  const drawScope = () => {
    sim.render(scopeBuf, scopeBuf.length);
    const w = scope.width, h = scope.height, mid = h / 2;
    const data = scopeBuf;
    scopeCtx.clearRect(0, 0, w, h);
    scopeCtx.strokeStyle = "#23323a";
    scopeCtx.beginPath(); scopeCtx.moveTo(0, mid); scopeCtx.lineTo(w, mid); scopeCtx.stroke();
    scopeCtx.strokeStyle = "#8fe9ff";
    scopeCtx.lineWidth = 1.25;
    scopeCtx.shadowColor = "#8fe9ff";
    scopeCtx.shadowBlur = 5;
    scopeCtx.beginPath();
    const step = data.length / w;
    for (let x = 0; x < w; x++) {
      const y = mid - data[Math.floor(x * step)] * mid * 0.92;
      x === 0 ? scopeCtx.moveTo(x, y) : scopeCtx.lineTo(x, y);
    }
    scopeCtx.stroke();
    scopeCtx.shadowBlur = 0;
  };

  // Per-frame: advance control-rate state, capture any callback output, repaint.
  let last = performance.now();
  const tick = (now: number) => {
    sim.clearOutput();
    sim.tick(now, (now - last) / 1000);
    last = now;
    log(sim.output(), "out");
    audio.forward(); // ship queued graph commands to the worklet engine
    if (audio.running) {
      audioState.textContent = audio.peak > 0.001 ? "live" : "idle";
    }
    panel.frame();
    drawScope();
    requestAnimationFrame(tick);
  };

  run();
  reanalyze(); // initial diagnostics for the starting script
  requestAnimationFrame(tick);

  // Small inspection hook (handy in the console / for verification).
  (window as unknown as { wren: unknown }).wren = {
    setSource: (s: string) => editor.setValue(s),
    markers: () => analyzerMarkers(editor.getModel()!),
    hover: (off: number) => analyzer.hover(editor.getValue(), off),
    definition: (off: number) => analyzer.definition(editor.getValue(), off),
    completions: () => analyzer.completions(editor.getValue()),
  };
}

boot().catch((e) => {
  const status = document.querySelector("#status");
  if (status) {
    status.textContent = "boot failed";
    (status as HTMLElement).classList.add("err");
  }
  console.error(e);
});
