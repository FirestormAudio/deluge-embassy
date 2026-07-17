//! Hardware mirror: a **physical Deluge** running `controller-firmware` attached
//! over USB-CDC serial as an *additional* control surface for the in-process
//! brain (`cargo deluge sim --hardware <port>`).
//!
//! The physical Deluge is hardwired as the [`deluge_protocol`] **device**: it
//! emits [`FromDeluge`] input and consumes [`ToDeluge`] illumination. So — unlike
//! the simulator's [`crate::link::PanelLink`], which plays the *panel* role — this
//! endpoint plays the **brain** role toward the hardware: it consumes the device's
//! input and emits illumination at it.
//!
//! It is orthogonal to [`crate::link::LinkKind`]: the on-screen GUI and the
//! physical Deluge are *peers*, both mirroring the same [`SharedPanel`]. Input from
//! the device is fed into the brain exactly like a GUI click (via `app::send`), and
//! the brain's illumination is pushed back out so the real OLED/pads/LEDs/knob
//! rings/CV-gate track the screen.
//!
//! Two background threads own the serial halves; the GUI talks to them over
//! channels and drives both directions on its 16 ms tick:
//!   - rx: decode frames → [`FromDeluge`] → [`HardwareMirror::poll_input`]
//!   - tx: framed [`ToDeluge`] bytes → the port

use std::io::{ErrorKind, Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError, channel};
use std::thread;
use std::time::Duration;

use deluge_protocol::{FrameDecoder, FromDeluge, ToDeluge};
use deluge_sim_link::{ALL_PADS_BYTES, LED_COUNT, PAD_COLS, PAD_ROWS, SharedPanel};
use log::{debug, info, warn};

/// USB identity of the controller-firmware (see
/// `firmwares/controller-firmware/src/main.rs`). Used by [`HardwareMirror::detect`].
const DELUGE_VID: u16 = 0x16D0;
const DELUGE_PID: u16 = 0x0EDA;

/// A physical Deluge attached over serial, mirroring the in-process brain.
pub struct HardwareMirror {
    /// The brain's shared illumination/input state (a clone of the GUI's panel).
    panel: SharedPanel,
    /// Decoded device input, drained each tick by [`poll_input`](Self::poll_input).
    inbound: Receiver<FromDeluge>,
    /// Framed [`ToDeluge`] bytes for the writer thread.
    outbound: Sender<Vec<u8>>,
    /// Cleared by either I/O thread when it exits (EOF / unplug / write error).
    alive: Arc<AtomicBool>,
    /// `false` until the device has spoken (*any* valid frame — its greeting, a
    /// `Pong`, or a control event). The firmware only drains its OUT endpoint once
    /// its session is up, so we must not send illumination before then or the
    /// writes time out; any decoded frame proves the session is live. The early
    /// greeting/PONG can be corrupted by the connect transient, so we don't insist
    /// on a specific frame — and [`retry_handshake`](Self::retry_handshake)
    /// re-pings until something decodes.
    handshaked: bool,
    /// Ticks since the last handshake PING retransmission (see `retry_handshake`).
    handshake_ticks: u32,

    // Per-category change generations already pushed to the device. Start at
    // `u64::MAX` so the first `mirror()` pass force-sends the full current state to
    // a freshly-connected device (which boots blank).
    seen_display: u64,
    seen_pads: u64,
    seen_controls: u64,
    seen_cv: u64,
    seen_gate: u64,
    /// Last LED state pushed, so `mirror()` sends only the LEDs that changed
    /// (the protocol has no "set all LEDs", only per-index `SetLed`).
    prev_leds: [bool; LED_COUNT],
}

impl HardwareMirror {
    /// Open `port`, spawn the rx/tx threads, and mirror `panel`. The baud rate is
    /// irrelevant for USB-CDC ACM (the firmware ignores it), but a value is required.
    pub fn connect(port: &str, panel: SharedPanel) -> serialport::Result<Self> {
        let mut reader = serialport::new(port, 115_200)
            // Generous timeout: writes can stall while the device pushes a frame
            // to its (slow) PIC, and a stall must not be mistaken for a hangup.
            // The read loop just re-loops on timeout, so a long value is harmless.
            .timeout(Duration::from_millis(2000))
            // No flow control: the wire is a raw byte protocol whose bytes include
            // 0x11/0x13 (XON/XOFF), which must not be intercepted. serialport opens
            // the tty in raw mode (cfmakeraw); make the no-flow-control explicit.
            .flow_control(serialport::FlowControl::None)
            .open()?;
        info!("hw: opened {port} (115200 8N1, raw, no flow control)");
        // Drive a DTR **edge** (drop, then raise). The firmware's `cdc_task` blocks
        // on embassy-usb's `wait_connection()`, which waits for a *change* in the
        // control-line state and then checks DTR. Linux's cdc-acm already asserts
        // DTR when the port is opened, so simply setting it `true` here is a no-op
        // (no SET_CONTROL_LINE_STATE is sent, no edge), and the device waits
        // forever. Toggling low→high guarantees the edge the device is waiting for.
        let _ = reader.write_data_terminal_ready(false);
        let _ = reader.write_request_to_send(false);
        thread::sleep(Duration::from_millis(60));
        match reader.write_data_terminal_ready(true) {
            Ok(()) => info!("hw: cycled DTR low→high to trigger device connection"),
            Err(e) => warn!("hw: could not assert DTR: {e}"),
        }
        let _ = reader.write_request_to_send(true);
        // Give the device time to register the edge and bring its rx loop up
        // before we touch the data endpoints.
        thread::sleep(Duration::from_millis(150));
        let mut writer = reader.try_clone()?;

        let (in_tx, in_rx) = channel::<FromDeluge>();
        let (out_tx, out_rx) = channel::<Vec<u8>>();
        let alive = Arc::new(AtomicBool::new(true));

        // Spawn the reader FIRST. The firmware's `cdc_task` sends a READY+VERSION
        // greeting on connect and only starts its OUT-draining `rx_loop` once that
        // `write_packet` completes — which needs the host to read the IN endpoint.
        // If we flood OUT before draining IN, the device never starts reading and
        // our writes time out. Drain IN immediately so its `rx_loop` comes up.
        let rx_alive = alive.clone();
        thread::Builder::new()
            .name("hw-link-rx".into())
            .spawn(move || {
                reader_loop(reader, in_tx);
                rx_alive.store(false, Ordering::Release);
            })?;
        // Now complete the handshake: the device answers PING with PONG. Sending
        // it after the reader is live means the greeting is being consumed, so the
        // device's `rx_loop` is up to receive this and the illumination that follows.
        match writer.write_all(&ToDeluge::Ping.to_frame()) {
            Ok(()) => info!("hw: sent handshake PING; waiting for device greeting/PONG…"),
            Err(e) => warn!("hw: handshake PING write failed: {e}"),
        }
        let tx_alive = alive.clone();
        thread::Builder::new()
            .name("hw-link-tx".into())
            .spawn(move || {
                writer_loop(writer, out_rx);
                tx_alive.store(false, Ordering::Release);
            })?;

        Ok(Self::with_io(panel, in_rx, out_tx, alive))
    }

    /// Assemble a mirror around already-wired I/O channels. `seen_*` start at
    /// `u64::MAX` so the first [`mirror`](Self::mirror) pass force-sends the full
    /// current state to a freshly-connected (blank) device.
    fn with_io(
        panel: SharedPanel,
        inbound: Receiver<FromDeluge>,
        outbound: Sender<Vec<u8>>,
        alive: Arc<AtomicBool>,
    ) -> Self {
        Self {
            panel,
            inbound,
            outbound,
            alive,
            handshaked: false,
            handshake_ticks: 0,
            seen_display: u64::MAX,
            seen_pads: u64::MAX,
            seen_controls: u64::MAX,
            seen_cv: u64::MAX,
            seen_gate: u64::MAX,
            prev_leds: [false; LED_COUNT],
        }
    }

    /// Auto-detect a connected Deluge by its USB VID/PID, returning its port name.
    pub fn detect() -> Option<String> {
        serialport::available_ports()
            .ok()?
            .into_iter()
            .find_map(|p| match p.port_type {
                serialport::SerialPortType::UsbPort(info)
                    if info.vid == DELUGE_VID && info.pid == DELUGE_PID =>
                {
                    Some(p.port_name)
                }
                _ => None,
            })
    }

    /// Whether both I/O threads are still running (false once the device is
    /// unplugged or the port errors). The app drops the mirror when this goes false.
    pub fn is_alive(&self) -> bool {
        self.alive.load(Ordering::Acquire)
    }

    /// Drain all input the device has sent since the last tick. The caller feeds
    /// each [`FromDeluge`] into the brain via `app::send` (same path as GUI clicks).
    pub fn poll_input(&mut self) -> Vec<FromDeluge> {
        let mut out = Vec::new();
        loop {
            match self.inbound.try_recv() {
                Ok(msg) => {
                    // Any decoded frame proves the device's session — and thus its
                    // OUT-draining `rx_loop` — is up, so it's safe to start
                    // mirroring. Arm the gate on the first one and push full state.
                    if !self.handshaked {
                        info!("hw: handshake complete (first frame {msg:?}); pushing full state");
                        self.handshaked = true;
                        self.force_full_resend();
                    }
                    // Handshake/greeting frames aren't control input; forward the rest.
                    match msg {
                        FromDeluge::Ready | FromDeluge::Version { .. } | FromDeluge::Pong => {}
                        input => out.push(input),
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }
        out
    }

    /// While not yet handshaked, retransmit a `PING` every ~half second so the
    /// device answers with a `PONG` even if the user never touches a control (and
    /// in case the greeting/PONG was lost to the connect transient). Call once per
    /// GUI tick; a no-op once the handshake has armed.
    pub fn retry_handshake(&mut self) {
        if self.handshaked {
            return;
        }
        // 16 ms tick → ~30 ticks ≈ 0.5 s. Ping immediately on the first tick too.
        if self.handshake_ticks % 30 == 0 {
            debug!("hw: re-sending handshake PING (awaiting first frame)");
            self.send(ToDeluge::Ping);
        }
        self.handshake_ticks = self.handshake_ticks.wrapping_add(1);
    }

    /// Reset the per-category seen generations so the next [`mirror`](Self::mirror)
    /// re-sends the entire current illumination state to the device.
    fn force_full_resend(&mut self) {
        self.seen_display = u64::MAX;
        self.seen_pads = u64::MAX;
        self.seen_controls = u64::MAX;
        self.seen_cv = u64::MAX;
        self.seen_gate = u64::MAX;
        self.prev_leds = [false; LED_COUNT];
    }

    /// Push the brain's illumination to the device: for each category whose
    /// change-generation advanced past what we last sent, encode a [`ToDeluge`]
    /// frame and queue it. Mirrors the structure of `app::apply_shared_panel`, but
    /// emits frames instead of painting the canvas.
    pub fn mirror(&mut self) {
        // Don't send anything until the device has proven its session is up (see
        // `handshaked`); otherwise the burst races the device's rx loop and the
        // writes time out.
        if !self.handshaked {
            return;
        }
        let panel = self.panel.clone();

        let gen_d = panel.display_gen();
        if gen_d != self.seen_display {
            let buf = panel.display_snapshot();
            if !self.send(ToDeluge::UpdateDisplay(&buf)) {
                return;
            }
            self.seen_display = gen_d;
        }

        let gen_p = panel.pads_gen();
        if gen_p != self.seen_pads {
            // Pack the col-major `[r,g,b]` grid into a `SetAllPads` buffer (same
            // layout: offset `(col*ROWS + row)*3`).
            let pads = panel.pads_snapshot();
            let mut buf = [0u8; ALL_PADS_BYTES];
            for col in 0..PAD_COLS {
                for row in 0..PAD_ROWS {
                    let o = (col * PAD_ROWS + row) * 3;
                    buf[o..o + 3].copy_from_slice(&pads[col][row]);
                }
            }
            if !self.send(ToDeluge::SetAllPads(&buf)) {
                return;
            }
            self.seen_pads = gen_p;
        }

        let gen_c = panel.controls_gen();
        if gen_c != self.seen_controls {
            let leds = panel.leds_snapshot();
            for (index, &on) in leds.iter().enumerate() {
                if on != self.prev_leds[index] {
                    if !self.send(ToDeluge::SetLed {
                        index: index as u8,
                        on,
                    }) {
                        return;
                    }
                    self.prev_leds[index] = on;
                }
            }
            let knobs = panel.knobs_snapshot();
            for which in 0..2u8 {
                if !self.send(ToDeluge::SetKnobIndicator {
                    which,
                    levels: knobs[which as usize],
                }) {
                    return;
                }
            }
            if !self.send(ToDeluge::SetSyncedLed(panel.synced_led())) {
                return;
            }
            self.seen_controls = gen_c;
        }

        let gen_cv = panel.cv_gen();
        if gen_cv != self.seen_cv {
            let cv = panel.cv_snapshot();
            for (channel, &value) in cv.iter().enumerate() {
                if !self.send(ToDeluge::SetCv {
                    channel: channel as u8,
                    value,
                }) {
                    return;
                }
            }
            self.seen_cv = gen_cv;
        }

        let gen_gate = panel.gate_gen();
        if gen_gate != self.seen_gate {
            let gate = panel.gate_snapshot();
            for (channel, &on) in gate.iter().enumerate() {
                if !self.send(ToDeluge::SetGate {
                    channel: channel as u8,
                    on,
                }) {
                    return;
                }
            }
            self.seen_gate = gen_gate;
        }
    }

    /// Frame a [`ToDeluge`] and queue it for the writer thread. Returns `false` if
    /// the writer has gone (device unplugged) — callers stop the current pass.
    fn send(&self, msg: ToDeluge) -> bool {
        self.outbound.send(msg.to_frame()).is_ok()
    }
}

/// Read bytes, reassemble frames, decode device input, forward [`FromDeluge`].
/// Exits on EOF/error or when the GUI drops the receiver. A read timeout (no data)
/// is normal and loops; only a real error or EOF ends the loop.
///
/// The firmware's greeting can arrive partial/garbled during the connect transient
/// (the DTR edge / USB re-enumeration), but steady-state frames are clean.
/// [`FrameDecoder`] is the auto-discard: an out-of-range length prefix makes it
/// drop its buffer and resynchronise, so a mangled greeting costs at most one
/// `undecodable` log before the stream realigns onto the next valid frame.
fn reader_loop(mut stream: Box<dyn serialport::SerialPort>, tx: Sender<FromDeluge>) {
    let mut dec = FrameDecoder::<4096>::new();
    let mut buf = [0u8; 1024];
    let mut payload = [0u8; 1024];
    let mut idle_timeouts = 0u32;
    loop {
        match stream.read(&mut buf) {
            Ok(0) => {
                warn!("hw rx: read returned EOF (device closed)");
                break;
            }
            Ok(n) => {
                idle_timeouts = 0;
                let head = n.min(24);
                debug!(
                    "hw rx: {n} bytes {:02x?}{}",
                    &buf[..head],
                    if n > head { " …" } else { "" }
                );
                dec.push(&buf[..n]);
                while let Some((type_byte, len)) = dec.pop_frame(&mut payload) {
                    let len = len.min(payload.len());
                    match FromDeluge::decode(type_byte, &payload[..len]) {
                        Some(msg) => {
                            debug!("hw rx: frame type=0x{type_byte:02x} → {msg:?}");
                            if tx.send(msg).is_err() {
                                return; // GUI gone
                            }
                        }
                        None => {
                            warn!("hw rx: undecodable frame type=0x{type_byte:02x} len={len}");
                        }
                    }
                }
            }
            Err(ref e) if e.kind() == ErrorKind::TimedOut => {
                idle_timeouts += 1;
                debug!("hw rx: idle (no data, timeout #{idle_timeouts})");
                continue;
            }
            Err(ref e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) => {
                warn!("hw rx: read error: {e} (kind={:?})", e.kind());
                break;
            }
        }
    }
}

/// Frame-write outbound illumination until the GUI drops the sender or the port
/// errors (device unplugged).
fn writer_loop(mut stream: Box<dyn serialport::SerialPort>, rx: Receiver<Vec<u8>>) {
    while let Ok(frame) = rx.recv() {
        if let Err(e) = stream.write_all(&frame) {
            warn!("hw tx: write error: {e} (kind={:?})", e.kind());
            break;
        }
        // No flush(): serialport's flush is a blocking `tcdrain` that, at the
        // 60 fps display-update rate, can starve the read path. `write_all` has
        // already handed the bytes to cdc-acm, which transmits them immediately.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A mirror wired to plain channels (no serial threads) for testing the
    /// encode/drain logic in-process.
    fn test_mirror(panel: SharedPanel) -> (HardwareMirror, Receiver<Vec<u8>>, Sender<FromDeluge>) {
        let (in_tx, in_rx) = channel::<FromDeluge>();
        let (out_tx, out_rx) = channel::<Vec<u8>>();
        let alive = Arc::new(AtomicBool::new(true));
        (
            HardwareMirror::with_io(panel, in_rx, out_tx, alive),
            out_rx,
            in_tx,
        )
    }

    /// Arm the handshake gate the way a real device does — by speaking first.
    fn handshake(hw: &mut HardwareMirror, in_tx: &Sender<FromDeluge>) {
        in_tx.send(FromDeluge::Ready).unwrap();
        hw.poll_input();
    }

    /// Decode every queued outbound frame into owned `(type, payload)` pairs.
    fn drain_frames(rx: &Receiver<Vec<u8>>) -> Vec<(u8, Vec<u8>)> {
        let mut dec = FrameDecoder::<2048>::new();
        let mut payload = [0u8; 1024];
        let mut out = Vec::new();
        while let Ok(frame) = rx.try_recv() {
            dec.push(&frame);
            while let Some((t, n)) = dec.pop_frame(&mut payload) {
                out.push((t, payload[..n].to_vec()));
            }
        }
        out
    }

    #[test]
    fn mirror_silent_before_handshake() {
        let panel = SharedPanel::new();
        let (mut hw, out_rx, _in_tx) = test_mirror(panel.clone());
        // The device hasn't spoken yet, so nothing must be sent even though there
        // is state to mirror.
        panel.set_led(1, true);
        hw.mirror();
        assert_eq!(
            drain_frames(&out_rx).len(),
            0,
            "must stay silent until handshake"
        );
    }

    #[test]
    fn mirror_emits_changed_illumination() {
        let panel = SharedPanel::new();
        let (mut hw, out_rx, in_tx) = test_mirror(panel.clone());
        handshake(&mut hw, &in_tx);
        let _ = drain_frames(&out_rx); // discard the post-handshake full push

        // A specific LED on, plus the implicit first-pass full-state push.
        panel.set_led(5, true);
        hw.mirror();

        let frames = drain_frames(&out_rx);
        // First pass force-sends display + all pads.
        assert!(
            frames
                .iter()
                .any(|(t, _)| *t == deluge_protocol::to::UPDATE_DISPLAY)
        );
        assert!(
            frames
                .iter()
                .any(|(t, _)| *t == deluge_protocol::to::SET_ALL_PADS)
        );
        // The lit LED is encoded as SetLed{index:5,on:true}.
        let set_led = frames
            .iter()
            .filter_map(|(t, p)| ToDeluge::decode(*t, p))
            .find(|m| matches!(m, ToDeluge::SetLed { index: 5, on: true }));
        assert!(
            set_led.is_some(),
            "expected SetLed{{index:5,on:true}}, got {frames:?}"
        );

        // A second pass with no further changes emits nothing.
        let n = drain_frames(&out_rx).len();
        hw.mirror();
        assert_eq!(
            drain_frames(&out_rx).len(),
            0,
            "idle mirror should be silent (was {n})"
        );
    }

    #[test]
    fn mirror_diffs_leds() {
        let panel = SharedPanel::new();
        let (mut hw, out_rx, in_tx) = test_mirror(panel.clone());
        handshake(&mut hw, &in_tx);
        hw.mirror(); // consume the first-pass burst
        let _ = drain_frames(&out_rx);

        // Toggle one LED; only that index should be re-sent.
        panel.set_led(7, true);
        hw.mirror();
        let leds: Vec<_> = drain_frames(&out_rx)
            .iter()
            .filter_map(|(t, p)| ToDeluge::decode(*t, p))
            .filter_map(|m| match m {
                ToDeluge::SetLed { index, on } => Some((index, on)),
                _ => None,
            })
            .collect();
        assert_eq!(leds, vec![(7, true)]);
    }

    #[test]
    fn ready_greeting_triggers_full_resend() {
        let panel = SharedPanel::new();
        let (mut hw, out_rx, in_tx) = test_mirror(panel.clone());
        hw.mirror(); // first-pass burst
        let _ = drain_frames(&out_rx);

        // Idle: nothing to send.
        hw.poll_input();
        hw.mirror();
        assert_eq!(drain_frames(&out_rx).len(), 0);

        // The device announces itself → the next mirror re-sends the full state
        // (so a blit dropped during the connect race is recovered).
        in_tx.send(FromDeluge::Ready).unwrap();
        hw.poll_input();
        hw.mirror();
        let frames = drain_frames(&out_rx);
        assert!(
            frames
                .iter()
                .any(|(t, _)| *t == deluge_protocol::to::UPDATE_DISPLAY)
        );
        assert!(
            frames
                .iter()
                .any(|(t, _)| *t == deluge_protocol::to::SET_ALL_PADS)
        );
    }

    #[test]
    fn poll_input_drains_decoded_events() {
        let panel = SharedPanel::new();
        let (mut hw, _out_rx, in_tx) = test_mirror(panel);
        in_tx
            .send(FromDeluge::PadPressed { col: 3, row: 4 })
            .unwrap();
        in_tx.send(FromDeluge::ButtonPressed { id: 150 }).unwrap();
        let got = hw.poll_input();
        assert_eq!(got.len(), 2);
        assert!(matches!(got[0], FromDeluge::PadPressed { col: 3, row: 4 }));
        assert!(hw.poll_input().is_empty());
    }
}
