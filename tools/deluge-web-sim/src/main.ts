import { Panel } from "./panel";
import { PanelClient } from "./panel-client";
import { Connection } from "./connection";

const $ = <T extends HTMLElement>(sel: string) => document.querySelector(sel) as T;

const client = new PanelClient(() => {});
const panel = new Panel($<HTMLCanvasElement>("#oled"), $("#face-overlay"), client);
const statusEl = $("#status");
const conn = new Connection(client, (s) => { statusEl.textContent = s; });

$("#connect").addEventListener("click", () => {
  conn.connect($<HTMLInputElement>("#ws-url").value);
});

// e2e hook retained for the M1 render test.
(window as unknown as { __replay: (frames: number[][]) => void }).__replay = (frames) => {
  for (const f of frames) client.onMessage(Uint8Array.from(f));
};

function tick() { panel.frame(); requestAnimationFrame(tick); }
requestAnimationFrame(tick);
statusEl.textContent = "ready";
