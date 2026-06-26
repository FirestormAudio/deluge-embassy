import "./style.css";
import { loadSim } from "./sim";
import { createEditor, setErrorMarker } from "./editor";
import { Panel } from "./panel";
import { EXAMPLES } from "./examples";

// Served from public/ at the app's base URL; fetched + instantiated in sim.ts.
const wasmUrl = `${import.meta.env.BASE_URL}wren_web.wasm`;

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

async function boot() {
  const status = $("#status");
  const consoleEl = $("#console");

  const { editor } = createEditor($("#editor"), EXAMPLES[1].source);

  // Examples menu.
  const select = $<HTMLSelectElement>("#examples");
  for (const ex of EXAMPLES) {
    const opt = document.createElement("option");
    opt.value = ex.name;
    opt.textContent = ex.name;
    select.appendChild(opt);
  }
  select.value = EXAMPLES[1].name;
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

  const run = () => {
    sim.clearOutput();
    const res = sim.load(editor.getValue());
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

  $("#run").addEventListener("click", run);
  // Cmd/Ctrl-Enter to run.
  editor.addCommand(
    // KeyMod.CtrlCmd | KeyCode.Enter
    2048 | 3,
    run,
  );

  // Per-frame: advance control-rate state, capture any callback output, repaint.
  let last = performance.now();
  const tick = (now: number) => {
    sim.clearOutput();
    sim.tick(now, (now - last) / 1000);
    last = now;
    log(sim.output(), "out");
    panel.frame();
    requestAnimationFrame(tick);
  };

  run();
  requestAnimationFrame(tick);
}

boot().catch((e) => {
  const status = document.querySelector("#status");
  if (status) {
    status.textContent = "boot failed";
    (status as HTMLElement).classList.add("err");
  }
  console.error(e);
});
