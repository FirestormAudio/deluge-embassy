//! Panel link: the simulator's connection to a Deluge "brain" over a byte stream.
//!
//! The simulator plays the **panel/device** role of the [`deluge_protocol`] contract: it
//! receives [`ToDeluge`] illumination frames (OLED, pad LEDs, indicator LEDs) and emits
//! [`FromDeluge`] input. The brain on the other end is the DelugeFirmware C build's
//! `deluge_host` (its `host_link` bridge listens on the stream); the same simulator could
//! later drive real hardware or an SDK app, since all speak this wire.
//!
//! The transport is decided from the connect target ([`Target::parse`]): a `host:port`
//! (or `:port` / `tcp://…`) connects over **TCP loopback**, anything else is treated as a
//! **Unix domain socket** path (Unix only). The loops are generic over [`Read`]/[`Write`],
//! so the only platform difference is whether the Unix branch is compiled in.
//!
//! Two background threads own the stream halves; the GUI talks to them over channels:
//!   - inbound: decoded `(type, payload)` frames → [`PanelLink::inbound`]
//!   - outbound: [`FromDeluge`] input → framed onto the stream
//!
//! This module also owns the wire-id ↔ control-enum glue (ported from spark's
//! `protocol.rs`): the firmware addresses LEDs by raw PIC index and buttons by matrix id,
//! which the renderer needs mapped onto [`HardwareLED`] / [`HardwareButton`].

use std::io::{Read, Write};
use std::net::TcpStream;
#[cfg(unix)]
use std::os::unix::net::UnixStream;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread;

use deluge_protocol::{FrameDecoder, FromDeluge};

use crate::hardware::{HardwareButton, HardwareEncoder, HardwareLED};

/// A live connection to the brain. Drop to disconnect (the threads exit when the socket
/// closes or the channels hang up).
pub struct PanelLink {
    /// Decoded inbound frames `(type_byte, payload)` — feed to `ToDeluge::decode`.
    pub inbound: Receiver<(u8, Vec<u8>)>,
    /// Input to push to the brain.
    pub outbound: Sender<FromDeluge>,
}

impl PanelLink {
    /// Connect to the brain at `target` and spawn the reader/writer threads. `target` is a
    /// TCP address (`host:port`, `:port`, or `tcp://…`) or, on Unix, a domain-socket path —
    /// see [`Target::parse`].
    pub fn connect(target: &str) -> std::io::Result<Self> {
        let (inbound, outbound) = match Target::parse(target) {
            Target::Tcp(addr) => {
                let stream = TcpStream::connect(&addr)?;
                let _ = stream.set_nodelay(true); // frames are tiny and latency-sensitive
                let writer = stream.try_clone()?;
                spawn_io(stream, writer)?
            }
            #[cfg(unix)]
            Target::Unix(path) => {
                let stream = UnixStream::connect(&path)?;
                let writer = stream.try_clone()?;
                spawn_io(stream, writer)?
            }
        };
        Ok(Self { inbound, outbound })
    }
}

/// Which transport [`PanelLink::connect`] should use for a given target string.
enum Target {
    /// A resolved TCP address to hand to [`TcpStream::connect`].
    Tcp(String),
    /// A Unix domain socket path (Unix builds only).
    #[cfg(unix)]
    Unix(String),
}

impl Target {
    /// Classify a connect target: anything that looks like a TCP endpoint becomes
    /// [`Target::Tcp`], otherwise it's a Unix socket path. Recognised TCP forms:
    ///   - `tcp://host:port` — explicit scheme (host optional → loopback)
    ///   - `:port` — shorthand for `127.0.0.1:port`
    ///   - `host:port` — when the part after the last `:` parses as a port number
    ///
    /// On non-Unix targets there is no Unix-socket fallback, so an unrecognised string is
    /// passed through as a TCP address (and will surface a connect error if it isn't one).
    fn parse(s: &str) -> Self {
        if let Some(addr) = as_tcp_addr(s) {
            return Target::Tcp(addr);
        }
        #[cfg(unix)]
        {
            Target::Unix(s.to_string())
        }
        #[cfg(not(unix))]
        {
            Target::Tcp(s.to_string())
        }
    }
}

/// Recognise the TCP-endpoint spellings, normalising the loopback shorthand. Returns the
/// string to feed [`TcpStream::connect`], or `None` if `s` doesn't look like a TCP address.
fn as_tcp_addr(s: &str) -> Option<String> {
    // `tcp://…` is an explicit request; a bare leading `:` means loopback.
    let normalize = |rest: &str| match rest.strip_prefix(':') {
        Some(port) => format!("127.0.0.1:{port}"),
        None => rest.to_string(),
    };
    if let Some(rest) = s.strip_prefix("tcp://") {
        return Some(normalize(rest));
    }
    if let Some(port) = s.strip_prefix(':') {
        return port
            .parse::<u16>()
            .is_ok()
            .then(|| format!("127.0.0.1:{port}"));
    }
    // `host:port` — only when the trailing component is a real port number, so Unix paths
    // (and Windows drive paths like `C:\…`) aren't mistaken for TCP endpoints.
    let (_, port) = s.rsplit_once(':')?;
    port.parse::<u16>().is_ok().then(|| s.to_string())
}

/// Wire up the channels and spawn the reader/writer threads over an already-connected
/// stream split into read/write halves.
fn spawn_io<R, W>(
    reader: R,
    writer: W,
) -> std::io::Result<(Receiver<(u8, Vec<u8>)>, Sender<FromDeluge>)>
where
    R: Read + Send + 'static,
    W: Write + Send + 'static,
{
    let (in_tx, in_rx) = channel::<(u8, Vec<u8>)>();
    let (out_tx, out_rx) = channel::<FromDeluge>();

    thread::Builder::new()
        .name("panel-link-rx".into())
        .spawn(move || reader_loop(reader, in_tx))?;
    thread::Builder::new()
        .name("panel-link-tx".into())
        .spawn(move || writer_loop(writer, out_rx))?;

    Ok((in_rx, out_tx))
}

/// Read bytes, reassemble frames, forward `(type, payload)`. Exits on EOF/error or when
/// the GUI has dropped the receiver.
fn reader_loop<R: Read>(mut stream: R, tx: Sender<(u8, Vec<u8>)>) {
    let mut dec = FrameDecoder::<4096>::new();
    let mut buf = [0u8; 4096];
    let mut payload = [0u8; 1024]; // largest inbound payload is the 768-byte display blit
    loop {
        match stream.read(&mut buf) {
            Ok(0) => break, // brain closed the socket
            Ok(n) => {
                dec.push(&buf[..n]);
                while let Some((type_byte, len)) = dec.pop_frame(&mut payload) {
                    let len = len.min(payload.len());
                    if tx.send((type_byte, payload[..len].to_vec())).is_err() {
                        return; // GUI gone
                    }
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(_) => break,
        }
    }
}

/// Frame and write outbound input until the GUI drops the sender or the stream errors.
fn writer_loop<W: Write>(mut stream: W, rx: Receiver<FromDeluge>) {
    while let Ok(msg) = rx.recv() {
        let frame = msg.to_frame();
        if stream.write_all(&frame).is_err() {
            break;
        }
    }
}

// ── In-process link (an SDK app running in the same process) ──────────────────

use deluge_sim_link::{InputEvent, SharedPanel};

/// How the panel is driven: a wire `PanelLink` to an external brain, a shared
/// in-memory [`SharedPanel`] for an SDK app in the same process, or nothing.
pub enum LinkKind {
    /// No brain connected — a passive, blank panel.
    None,
    /// An external brain over `deluge-protocol` (e.g. DelugeFirmware's `deluge_host`).
    Protocol(PanelLink),
    /// An SDK app in this process (`cargo deluge sim`).
    InProcess(InProcessLink),
}

/// The panel side of the in-process link: read illumination from the shared
/// panel (tracking change generations to skip redundant repaints) and push input
/// straight into it. No serialization — the same [`FromDeluge`]/[`ToDeluge`]
/// vocabulary, passed as owned state.
pub struct InProcessLink {
    pub panel: SharedPanel,
    pub seen_display: u64,
    pub seen_pads: u64,
    pub seen_controls: u64,
    pub seen_cv: u64,
    pub seen_gate: u64,
    pub seen_midi_in: u64,
    pub seen_midi_out: u64,
}

impl InProcessLink {
    pub fn new(panel: SharedPanel) -> Self {
        Self {
            panel,
            seen_display: 0,
            seen_pads: 0,
            seen_controls: 0,
            seen_cv: 0,
            seen_gate: 0,
            seen_midi_in: 0,
            seen_midi_out: 0,
        }
    }

    /// Translate a panel input event into the SDK-native event and enqueue it.
    /// Buttons carry the wire id (raw + [`deluge_protocol::BUTTON_ID_BASE`]); the
    /// SDK expects the raw id, so subtract the base here.
    pub fn send_input(&self, msg: FromDeluge) {
        let ev = match msg {
            FromDeluge::PadPressed { col, row } => InputEvent::Pad {
                x: col,
                y: row,
                pressed: true,
            },
            FromDeluge::PadReleased { col, row } => InputEvent::Pad {
                x: col,
                y: row,
                pressed: false,
            },
            FromDeluge::ButtonPressed { id } => InputEvent::Button {
                id: id.wrapping_sub(deluge_protocol::BUTTON_ID_BASE),
                pressed: true,
            },
            FromDeluge::ButtonReleased { id } => InputEvent::Button {
                id: id.wrapping_sub(deluge_protocol::BUTTON_ID_BASE),
                pressed: false,
            },
            FromDeluge::EncoderRotated { id, delta } => InputEvent::Encoder { index: id, delta },
            _ => return,
        };
        self.panel.push_event(ev);
    }
}

// ── Wire-id ↔ control-enum mappings (from spark's protocol.rs) ────────────────

/// Map a [`HardwareButton`] to its wire button id (`9*(y+16)+x` per `hid/button.h`), for
/// encoding outbound presses. Encoder-push buttons have their own ids and are sent via
/// [`encoder_push_id`]; this covers the matrix buttons the faceplate exposes.
pub fn button_to_id(b: HardwareButton) -> u8 {
    use HardwareButton::*;
    match b {
        // Zmod column 1 (x=1) / column 2 (x=2)
        EncoderFunction1 => 145,
        EncoderFunction5 => 146,
        Scope => 147,
        Time => 149,
        Scale => 150,
        Copy => 151,
        Shift => 152,
        EncoderFunction2 => 154,
        EncoderFunction6 => 155,
        Session => 156,
        Quantize => 158,
        Load => 159,
        Back => 160,
        Select => 161,
        EncoderFunction3 => 163,
        EncoderFunction7 => 164,
        Clip => 165,
        Automation => 167,
        Loop => 168,
        Fill => 169,
        Record => 170,
        EncoderFunction4 => 172,
        EncoderFunction8 => 173,
        Keyboard => 174,
        Transform => 176,
        Save => 177,
        TapTempo => 178,
        Play => 179,
    }
}

/// Wire id for pressing an encoder's push-button (encoders double as buttons).
pub fn encoder_push_id(e: HardwareEncoder) -> Option<u8> {
    use HardwareEncoder::*;
    Some(match e {
        VerticalEncoder => 144,
        HorizontalEncoder => 153,
        Tempo => 157,
        LowerGold => 162,
        UpperGold => 171,
        Select => 175,
        Volume => return None, // not part of the control-surface matrix
    })
}

/// Wire encoder-rotation id (`0=SCROLL_X, 1=TEMPO, 2=MOD_0, 3=MOD_1, 4=SCROLL_Y, 5=SELECT`).
pub fn encoder_to_id(e: HardwareEncoder) -> Option<u8> {
    use HardwareEncoder::*;
    Some(match e {
        HorizontalEncoder => 0,
        Tempo => 1,
        LowerGold => 2,
        UpperGold => 3,
        VerticalEncoder => 4,
        Select => 5,
        Volume => return None,
    })
}

/// Map a raw PIC LED index (from an inbound `SetLed`) to a [`HardwareLED`], so the renderer
/// can light the right control. Inverse of spark's `led_to_index` (`index = x + 9*y`).
/// Gold-knob segments / Synced arrive via `SetKnobIndicator` / `SetSyncedLed`, not here.
pub fn led_index_to_led(index: u8) -> Option<HardwareLED> {
    use HardwareLED::*;
    Some(match index {
        1 => EncoderFunction1,
        10 => EncoderFunction2,
        19 => EncoderFunction3,
        28 => EncoderFunction4,
        2 => EncoderFunction5,
        11 => EncoderFunction6,
        20 => EncoderFunction7,
        29 => EncoderFunction8,
        3 => Scope,
        5 => Time,
        6 => Scale,
        7 => Copy,
        8 => Shift,
        12 => Session,
        14 => Quantize,
        15 => Load,
        16 => Back,
        17 => Select,
        21 => Clip,
        23 => Automation,
        24 => Loop,
        25 => Fill,
        26 => Record,
        30 => Keyboard,
        32 => Transform,
        33 => Save,
        34 => TapTempo,
        35 => Play,
        _ => return None,
    })
}
