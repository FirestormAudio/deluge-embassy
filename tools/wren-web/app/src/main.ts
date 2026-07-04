import "./fonts";
import "./style.css";
import { loadSim } from "./sim";
import { createEditor, setErrorMarker, setAnalyzerMarkers, analyzerMarkers, registerIntelligence } from "./editor";
import { Panel } from "./panel";
import { Audio } from "./audio";
import { Analyzer } from "./analyzer";
import { WebMidi } from "./midi";
import { ProjectStore, loadInitialProject, projectFromExample, moduleName } from "./project";
import { Tabs } from "./tabs";
import { FileBrowser } from "./filebrowser";
import { setupBreakpointGutter } from "./debug/gutter";
import { setupResizers } from "./resize";
import { DebugSession } from "./debug/session";
import { DebugToolbar } from "./debug/ui";
import { DebugSidebar } from "./debug/panels";
import { EXAMPLES } from "./examples";

// Served from public/ at the app's base URL; fetched + instantiated in sim.ts.
const wasmUrl = `${import.meta.env.BASE_URL}wren_web.wasm`;

// Task 3.3: headless test/debug hook. The threaded DebugController is imported
// only on demand (dynamic import), so it never touches the normal boot path or
// bundle. Phase 4 (Monaco UI) will consume this controller directly.
(window as unknown as { __wrenDebug: unknown }).__wrenDebug = {
  createController: () =>
    import("./debug/controller").then(
      (m) => new m.DebugController(`${import.meta.env.BASE_URL}wren-debug-threads.wasm`),
    ),
};

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

async function boot() {
  const status = $("#status");
  const consoleEl = $("#console");

  // Project: a virtual filesystem with tabs + a file tree. The active file's
  // Monaco model is shown in the shared editor (a #p= permalink wins over the
  // saved project, which wins over the default example).
  const { editor, monaco } = createEditor($("#editor"), "");
  const store = new ProjectStore(loadInitialProject());
  const tabs = new Tabs(editor, $("#tab-bar"), store);
  const browser = new FileBrowser($("#file-browser"), store);
  // Breakpoint gutter: glyph-margin clicks toggle amber breakpoints, stored in
  // the project (persisted, and read by 4.2's debug launch).
  const gutter = setupBreakpointGutter(monaco, editor, tabs, store);
  tabs.render();
  browser.render();
  setupResizers(); // drag-resizable left panel + console (persisted)

  // Examples menu loads a whole project.
  const select = $<HTMLSelectElement>("#examples");
  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = "load…";
  placeholder.hidden = true;
  select.appendChild(placeholder);
  for (const ex of EXAMPLES) {
    const opt = document.createElement("option");
    opt.value = ex.name;
    opt.textContent = ex.name;
    select.appendChild(opt);
  }
  select.value = "";
  select.addEventListener("change", () => {
    const ex = EXAMPLES.find((e) => e.name === select.value);
    if (ex) store.replace(projectFromExample(ex));
    select.value = "";
  });

  const sim = await loadSim(wasmUrl);
  status.textContent = "booted";
  status.classList.add("ok");

  const panel = new Panel(
    $<HTMLCanvasElement>("#oled"),
    $("#face-overlay"),
    $("#cv-row"),
    $("#keyboard"),
    sim,
  );

  // Faceplate theme (hardware art / dark chassis / emissive recreation).
  const faceplate = $("#faceplate");
  const faceTheme = $<HTMLSelectElement>("#face-theme");
  const savedTheme = localStorage.getItem("wren-deluge:face") ?? "hardware";
  faceplate.dataset.faceTheme = savedTheme;
  faceTheme.value = savedTheme;
  faceTheme.addEventListener("change", () => {
    faceplate.dataset.faceTheme = faceTheme.value;
    try { localStorage.setItem("wren-deluge:face", faceTheme.value); } catch { /* ignore */ }
  });

  // Web MIDI input: a real keyboard/controller drives the same path as the
  // on-screen keys (sim.midiIn + the monitor).
  const webmidi = new WebMidi();
  webmidi.onMessage = (s, d1, d2) => { sim.midiIn(s, d1, d2); panel.logMidi(s, d1, d2, "in"); };
  const midiEnable = $<HTMLButtonElement>("#midi-enable");
  const midiDevice = $<HTMLSelectElement>("#midi-device");
  if (!webmidi.supported) {
    midiEnable.textContent = "no web midi";
    midiEnable.disabled = true;
  } else {
    midiEnable.addEventListener("click", async () => {
      try {
        const inputs = await webmidi.enable();
        if (inputs.length === 0) { midiEnable.textContent = "no devices"; return; }
        midiDevice.innerHTML = "";
        for (const inp of inputs) {
          const opt = document.createElement("option");
          opt.value = inp.id;
          opt.textContent = inp.name ?? inp.id;
          midiDevice.appendChild(opt);
        }
        midiEnable.hidden = true;
        midiDevice.hidden = false;
        webmidi.select(inputs[0].id);
        midiDevice.addEventListener("change", () => webmidi.select(midiDevice.value));
      } catch {
        midiEnable.textContent = "access denied";
      }
    });
  }

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
  // Per-file analysis version so multi-file re-analysis can drop stale results
  // for each file independently.
  const analyzeVersions = new Map<string, number>();
  analyzer.onDiagnostics = (version, diags, path) => {
    if (analyzeVersions.get(path) === version && store.project.files[path] != null) {
      setAnalyzerMarkers(tabs.model(path), diags);
    }
  };

  // Analyze one file with the *other* project files as modules (so cross-file
  // imports resolve), and mark that file's own model. The active file uses the
  // live editor buffer; others use their stored (autosaved) content.
  const analyzeFile = (path: string) => {
    if (store.project.files[path] == null) return;
    const modules: Record<string, string> = {};
    for (const [p, c] of Object.entries(store.project.files)) {
      if (p !== path) modules[moduleName(p)] = c;
    }
    const src = path === store.project.active ? editor.getValue() : store.project.files[path];
    const v = (analyzeVersions.get(path) ?? 0) + 1;
    analyzeVersions.set(path, v);
    analyzer.analyze(src, v, path, modules);
  };

  // Re-analyze the active file plus every file that imports it — so editing a
  // module refreshes the import markers of its dependents (no Run needed) —
  // coalesced on a short pause so it only runs when you stop typing.
  const importsModule = (content: string, mod: string) =>
    new RegExp(`import\\s+"${mod.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}"`).test(content);
  let analyzeTimer = 0;
  const scheduleReanalyze = () => {
    clearTimeout(analyzeTimer);
    analyzeTimer = window.setTimeout(() => {
      const active = store.project.active;
      analyzeFile(active);
      const activeMod = moduleName(active);
      for (const [path, content] of Object.entries(store.project.files)) {
        if (path !== active && importsModule(content, activeMod)) analyzeFile(path);
      }
    }, 300);
  };

  // ── Background compile/run check (the full compiler surface, off-thread) ─────
  // A throwaway sim VM in a Worker compiles+runs the whole project through the
  // REAL wren compiler on a debounce, surfacing errors the static analyzer can't
  // (undefined module vars, arity, top-level runtime). It works IN CONJUNCTION
  // with the analyzer, never against it: if the analyzer already flags any file,
  // the check defers (the analyzer's per-file diagnostics are more precise); the
  // check only speaks when static analysis is clean but the compiler/runtime
  // still fails. Owner "wren-check" keeps it separate from analyzer/vm markers.
  const checkWasmUrl = new URL(wasmUrl, location.href).href;
  let checkWorker: Worker;
  let checkSeq = 0;
  let checkTimeout = 0;
  const applyCheckResult = (ok: boolean, error: string, errorLine: number) => {
    const entryModel = tabs.model(store.project.entry);
    const clear = () => monaco.editor.setModelMarkers(entryModel, "wren-check", []);
    if (ok || !error) return clear();
    const line = Math.max(1, Math.min(errorLine, entryModel.getLineCount()));
    // Contention is per-ISSUE, not per-file: defer only if the analyzer already
    // marks this exact line (the same problem) — so the check still surfaces an
    // error the analyzer misses even when the analyzer flags something elsewhere.
    const analyzerCoversLine = analyzerMarkers(entryModel).some(
      (m) => m.severity === monaco.MarkerSeverity.Error && m.startLineNumber <= line && line <= m.endLineNumber,
    );
    if (analyzerCoversLine) return clear();
    const lineText = entryModel.getLineContent(line);
    monaco.editor.setModelMarkers(entryModel, "wren-check", [
      {
        severity: monaco.MarkerSeverity.Error,
        message: error.split("\n")[0] ?? "error",
        startLineNumber: line,
        startColumn: 1,
        endLineNumber: line,
        endColumn: lineText.length + 1,
      },
    ]);
  };
  const spawnCheckWorker = () => {
    checkWorker?.terminate();
    checkWorker = new Worker(new URL("./check-worker.ts", import.meta.url), { type: "module" });
    checkWorker.onmessage = (e) => {
      const m = e.data;
      if (m.type === "result" && m.seq === checkSeq) {
        clearTimeout(checkTimeout);
        applyCheckResult(m.ok, m.error, m.errorLine);
      }
    };
    checkWorker.postMessage({ type: "init", wasmUrl: checkWasmUrl });
  };
  spawnCheckWorker();
  let checkDebounce = 0;
  const scheduleCheck = () => {
    clearTimeout(checkDebounce);
    checkDebounce = window.setTimeout(() => {
      const seq = ++checkSeq;
      checkWorker.postMessage({ type: "check", seq, files: { ...store.project.files }, entry: store.project.entry });
      // Kill + respawn a check that hangs (e.g. a top-level infinite loop) so a
      // stuck VM never blocks future checks; the editor itself never blocked.
      clearTimeout(checkTimeout);
      checkTimeout = window.setTimeout(spawnCheckWorker, 4000);
    }, 600);
  };

  // A Run writes a `wren-vm` error marker on the entry that otherwise lingers
  // until the *next* Run — so a fixed error keeps squiggling. Clear it on any
  // edit; the live analyzer + background check now carry current errors.
  const clearRunError = () => setErrorMarker(tabs.model(store.project.entry), -1, "");
  editor.onDidChangeModelContent(() => {
    store.writeQuiet(store.project.active, editor.getValue()); // mirror + autosave
    clearRunError();
    scheduleReanalyze();
    scheduleCheck();
  });
  // Structural changes (open/close/create/rename/delete/load) re-render the tree
  // + tabs and re-analyze the (possibly new) active file + its dependents.
  store.onChange = () => {
    browser.render();
    tabs.render();
    gutter.renderActive(); // repaint the active file's breakpoints after a tab/model switch
    clearRunError();
    scheduleReanalyze();
    scheduleCheck();
  };

  // Share: copy a permalink whose hash encodes the whole project.
  const shareBtn = $("#share");
  shareBtn.addEventListener("click", async () => {
    const url = store.permalink();
    try {
      await navigator.clipboard.writeText(url);
      const prev = shareBtn.textContent;
      shareBtn.textContent = "Copied!";
      setTimeout(() => (shareBtn.textContent = prev), 1200);
    } catch {
      location.hash = url.slice(url.indexOf("#")); // fall back to updating the URL
    }
  });
  const scope = $<HTMLCanvasElement>("#scope");
  const scopeCtx = scope.getContext("2d")!;

  const run = () => {
    // Run the project's entry file; imports resolve from the other files.
    const res = sim.runProject(store.project.files, store.project.entry);
    const entryModel = tabs.model(store.project.entry);
    setErrorMarker(entryModel, res.ok ? -1 : res.errorLine, res.ok ? "" : res.error.split("\n")[0] ?? "error");
    log(res.output, "out");
    if (!res.ok) log(res.error, "err");
  };

  // Running is a user gesture, so it's also where we (re)start audio.
  const runWithAudio = async () => {
    run();
    await audio.start();
    audioState.textContent = "live";
    audioState.classList.add("on");
  };

  const runBtn = $("#run");
  runBtn.addEventListener("click", runWithAudio);
  // Cmd/Ctrl-Enter to run.
  editor.addCommand(2048 | 3 /* KeyMod.CtrlCmd | KeyCode.Enter */, runWithAudio);

  // Shared debug session: owns the DebugController lifecycle, state machine,
  // output routing, isolation gate + compile pre-flight. Both the transport
  // toolbar and the left-sidebar debug view observe this single source of truth.
  const debugSession = new DebugSession({
    wasmUrl: `${import.meta.env.BASE_URL}wren-debug-threads.wasm`,
    log,
    // Pre-flight: if the analyzer already flags the entry as broken, surface the
    // error verbatim and abort before spawning a worker (the debug core doesn't
    // report compile errors yet — see the task report).
    preflight: () => {
      const entryModel = tabs.model(store.project.entry);
      const errs = analyzerMarkers(entryModel).filter((m) => m.severity === monaco.MarkerSeverity.Error);
      if (!errs.length) return null;
      const e = errs[0];
      return { line: e.startLineNumber, message: `${store.project.entry}:${e.startLineNumber}: ${e.message}` };
    },
  });

  // Debug transport toolbar (topbar, left of Run). Drives the session; the
  // paused-line highlight lives here too.
  const debugToolbar = new DebugToolbar({ monaco, editor, tabs, store, session: debugSession });
  runBtn.parentElement!.insertBefore(debugToolbar.root, runBtn);

  // Left-sidebar debug view: [ files · debug ] switch + CALL STACK panel. Auto-
  // activates the debug segment while a session is live; returns to the last
  // shown pane when the run ends.
  const debugSidebar = new DebugSidebar($("#file-browser"), debugSession);

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
  scheduleReanalyze(); // initial diagnostics for the starting script
  scheduleCheck(); // initial full-compiler check
  requestAnimationFrame(tick);

  // Small inspection hook (handy in the console / for verification).
  (window as unknown as { wren: unknown }).wren = {
    setSource: (s: string) => editor.setValue(s),
    getSource: () => editor.getValue(),
    permalink: () => store.permalink(),
    audioUsingSab: () => audio.usingSab,
    markers: () => analyzerMarkers(editor.getModel()!),
    hover: (off: number) => analyzer.hover(editor.getValue(), off),
    definition: (off: number) => analyzer.definition(editor.getValue(), off),
    completions: () => analyzer.completions(editor.getValue()),
    // Project helpers for testing.
    files: () => ({ ...store.project.files }),
    entry: () => store.project.entry,
    open: () => [...store.project.open],
    active: () => store.project.active,
    createFile: (p: string, c = "") => store.create(p, c),
    activate: (p: string) => store.activate(p),
    setEntry: (p: string) => store.setEntry(p),
    run: () => run(),
    // Breakpoint inspection (used by tests).
    breakpoints: (p: string) => store.breakpointsFor(p),
    // Analyzer markers for any file's model (not just the active one).
    markersOf: (p: string) => analyzerMarkers(tabs.model(p)).map((m) => m.message),
    // Background-check markers on the entry (owner "wren-check"), for tests.
    checkMarkers: () =>
      monaco.editor
        .getModelMarkers({ owner: "wren-check", resource: tabs.model(store.project.entry).uri })
        .map((m) => m.message),
    // Debug session inspection (used by tests): current state + selected frame.
    debugState: () => debugSession.state,
    selectedFrame: () => debugSession.selectedFrameId,
    debugPane: () => debugSidebar.shown,
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
