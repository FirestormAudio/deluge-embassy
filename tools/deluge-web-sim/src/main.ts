import { Panel } from "./panel";
import { PanelClient } from "./panel-client";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

// Until Task 6 adds the WebSocket, input is discarded and frames arrive only via
// the __replay test hook. The connect button is wired in Task 6.
const client = new PanelClient(() => {});
const panel = new Panel($<HTMLCanvasElement>("#oled"), $("#face-overlay"), client);

// e2e hook: feed [type, ...data] message bodies straight into the client.
(window as unknown as { __replay: (frames: number[][]) => void }).__replay = (frames) => {
  for (const f of frames) client.onMessage(Uint8Array.from(f));
};

function tick() {
  panel.frame();
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
$("#status").textContent = "ready";
