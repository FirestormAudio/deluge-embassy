//! Host MIDI bridge: virtual MIDI ports wired to the in-process panel.
//!
//! Creates two virtual ports (ALSA on Linux, CoreMIDI on macOS):
//!   - **Deluge Sim In** — connect a keyboard / DAW *to* this port; incoming
//!     bytes are pushed to the app via [`SharedPanel::push_midi_in`], so the
//!     app's `midi().recv()` receives them (and the rack's MIDI-IN dot flashes).
//!   - **Deluge Sim Out** — the app's `midi().send()` bytes are drained from the
//!     panel and forwarded here, so a DAW connected to this port hears them.
//!
//! Virtual ports mean no device picking: the user wires their gear to the
//! sim's ports. On platforms without virtual-port support the bridge is a no-op.

use deluge_sim_link::SharedPanel;

/// Keeps the MIDI connections alive for the simulator's lifetime (dropping the
/// input connection closes the virtual port; the output thread is detached).
pub struct MidiBridge {
    _input: Option<midir::MidiInputConnection<()>>,
}

/// Start the bridge, creating the virtual ports. Failures are logged and the
/// bridge degrades to a no-op so the simulator still runs.
#[cfg(unix)]
pub fn start(panel: SharedPanel) -> MidiBridge {
    use midir::os::unix::{VirtualInput, VirtualOutput};

    // Input: a virtual port the user's gear connects to; forward bytes to the app.
    let input = match midir::MidiInput::new("deluge-sim") {
        Ok(mi) => {
            let panel = panel.clone();
            match mi.create_virtual(
                "Deluge Sim In",
                move |_ts, msg, _| panel.push_midi_in(msg),
                (),
            ) {
                Ok(conn) => {
                    log::info!("MIDI bridge: virtual input port 'Deluge Sim In' ready");
                    Some(conn)
                }
                Err(e) => {
                    log::warn!("MIDI bridge: no input port ({e})");
                    None
                }
            }
        }
        Err(e) => {
            log::warn!("MIDI bridge: MIDI input unavailable ({e})");
            None
        }
    };

    // Output: drain the app's MIDI-out and forward to a virtual output port.
    let out = midir::MidiOutput::new("deluge-sim")
        .map_err(|e| e.to_string())
        .and_then(|mo| {
            mo.create_virtual("Deluge Sim Out")
                .map_err(|e| e.to_string())
        });
    match out {
        Ok(mut conn) => {
            log::info!("MIDI bridge: virtual output port 'Deluge Sim Out' ready");
            std::thread::Builder::new()
                .name("deluge-midi-out".into())
                .spawn(move || {
                    loop {
                        let bytes = panel.drain_midi_out();
                        if !bytes.is_empty() {
                            let _ = conn.send(&bytes);
                        }
                        std::thread::sleep(std::time::Duration::from_millis(1));
                    }
                })
                .ok();
        }
        Err(e) => log::warn!("MIDI bridge: no output port ({e})"),
    }

    MidiBridge { _input: input }
}

/// No-op bridge on platforms without virtual-port support.
#[cfg(not(unix))]
pub fn start(_panel: SharedPanel) -> MidiBridge {
    log::info!("MIDI bridge: virtual ports are not supported on this platform");
    MidiBridge { _input: None }
}
