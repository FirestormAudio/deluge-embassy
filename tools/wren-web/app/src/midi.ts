// Web MIDI input: connect a hardware keyboard/controller so the synth examples
// are playable for real. Uses the standard lib.dom Web MIDI types.

export class WebMidi {
  private access: MIDIAccess | null = null;
  private current: MIDIInput | null = null;
  /// Called for each channel-voice message from the selected input.
  onMessage: (status: number, d1: number, d2: number) => void = () => {};

  get supported(): boolean {
    return typeof navigator.requestMIDIAccess === "function";
  }

  async enable(): Promise<MIDIInput[]> {
    this.access = await navigator.requestMIDIAccess();
    return this.inputs();
  }

  inputs(): MIDIInput[] {
    return this.access ? [...this.access.inputs.values()] : [];
  }

  /// Route a single input by id to `onMessage` (channel-voice only).
  select(id: string) {
    if (this.current) this.current.onmidimessage = null;
    this.current = this.inputs().find((i) => i.id === id) ?? null;
    if (this.current) {
      this.current.onmidimessage = (e) => {
        const data = e.data;
        if (!data) return;
        const status = data[0];
        if (status >= 0x80 && status < 0xf0) this.onMessage(status, data[1] ?? 0, data[2] ?? 0);
      };
    }
  }
}
