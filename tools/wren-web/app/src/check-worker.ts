// Background compile/run check: hosts a throwaway sim VM in a Worker and runs
// the whole project through the REAL wren compiler on demand, reporting the
// first error (the authoritative surface the static analyzer can't reach:
// undefined module vars, arity, top-level runtime errors). Running off the main
// thread means a heavy or infinite-looping top-level can't freeze the editor —
// the main thread just times out and respawns this worker. Side effects (audio,
// CV, print) stay in this instance and are discarded.
import { loadSim, type Sim } from "./sim";

// The sim logs `System.print` output via console.log; silence it here so a
// background check never spams the console with the script's output.
console.log = () => {};
console.warn = () => {};

interface InitMsg {
  type: "init";
  wasmUrl: string;
}
interface CheckMsg {
  type: "check";
  seq: number;
  files: Record<string, string>;
  entry: string;
}

let ready: Promise<Sim> | null = null;
const post = (m: unknown) => (self as unknown as Worker).postMessage(m);

self.onmessage = async (e: MessageEvent<InitMsg | CheckMsg>) => {
  const msg = e.data;
  if (msg.type === "init") {
    ready = loadSim(msg.wasmUrl);
    await ready;
    post({ type: "ready" });
    return;
  }
  // check
  if (!ready) return;
  const sim = await ready;
  try {
    const res = sim.runProject(msg.files, msg.entry);
    post({ type: "result", seq: msg.seq, ok: res.ok, error: res.error, errorLine: res.errorLine });
  } catch (err) {
    // A wasm trap during the check — report it (line unknown) rather than crash.
    post({ type: "result", seq: msg.seq, ok: false, error: `check crashed: ${String(err)}`, errorLine: 1 });
  }
};
