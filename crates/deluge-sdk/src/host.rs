//! Host (desktop-simulator) backend, active when an app is built for the host
//! triple by `cargo deluge sim` (`cfg(not(target_os = "none"))`).
//!
//! On the device the capability modules talk to `deluge-bsp` peripherals; on the
//! host they talk to a [`SharedPanel`] instead — the in-memory link the simulator
//! GUI renders from. This module owns the process-wide handles (the panel and the
//! app side of the audio bridge), set up once by [`crate::__rt::host::run`] before
//! any app code runs, mirroring the device's global-static peripheral pattern.

use std::sync::{Mutex, OnceLock};

use deluge_sim_link::SharedPanel;
use deluge_sim_link::audio::BrainEnds;

static PANEL: OnceLock<SharedPanel> = OnceLock::new();
static AUDIO: Mutex<Option<BrainEnds>> = Mutex::new(None);

/// Install the shared panel and the app-side audio endpoints. Called once by the
/// host runtime before the executor starts.
pub(crate) fn init(panel: SharedPanel, audio: BrainEnds) {
    let _ = PANEL.set(panel);
    *AUDIO.lock().unwrap() = Some(audio);
}

/// The process-wide shared panel. Panics if called before [`init`].
pub(crate) fn panel() -> &'static SharedPanel {
    PANEL
        .get()
        .expect("host panel not initialised (run via `cargo deluge sim`)")
}

/// Take the app-side audio endpoints (once, by `Audio::process`).
pub(crate) fn take_audio() -> Option<BrainEnds> {
    AUDIO.lock().unwrap().take()
}
