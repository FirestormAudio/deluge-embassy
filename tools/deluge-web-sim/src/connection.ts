// Owns the WebSocket and bridges it to a PanelClient: inbound binary messages →
// client.onMessage; client outbound bytes → socket. One binary message carries
// one [type, ...data] frame body.
import type { PanelClient } from "./panel-client";

export class Connection {
  private ws: WebSocket | null = null;

  constructor(
    private client: PanelClient,
    private onStatus: (s: "connecting" | "open" | "closed") => void,
  ) {}

  connect(url: string) {
    this.disconnect();
    this.onStatus("connecting");
    const ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";
    this.ws = ws;
    this.client.setSend((bytes) => {
      if (ws.readyState === WebSocket.OPEN) ws.send(bytes);
    });
    ws.onopen = () => { this.onStatus("open"); this.client.sendReady(); };
    ws.onmessage = (e) => this.client.onMessage(new Uint8Array(e.data as ArrayBuffer));
    ws.onclose = () => { this.onStatus("closed"); this.client.setSend(() => {}); };
    ws.onerror = () => ws.close();
  }

  disconnect() {
    if (this.ws) { this.ws.onclose = null; this.ws.close(); this.ws = null; }
    this.client.setSend(() => {});
  }
}
