//! Deluge desktop front-panel simulator.
//!
//! Renders the Deluge's OLED, RGB pad grid, buttons, encoders and LEDs and feeds
//! input back to a "brain" that owns all UI logic. Two front-ends share the same
//! rendering:
//!
//! - [`run_connected`] — drive an external brain over the [`deluge_protocol`]
//!   wire (TCP / Unix socket), e.g. DelugeFirmware's native `deluge_host`.
//! - [`run_in_process`] — drive an SDK app running in *this* process, sharing
//!   state through [`deluge_sim_link`] with no serialization. This is what
//!   `cargo deluge sim` uses.

mod app;
mod audio;
mod display;
mod headless;
mod hardware;
mod hardware_link;
mod hardware_state;
mod link;
mod midi;
mod pad_grid;
#[cfg(feature = "rack")]
mod rack;
mod renderer;
mod rgb;
#[cfg(feature = "rack")]
mod scope;

pub use headless::run_headless;

use app::DelugeSimulator;
use deluge_sim_link::SharedPanel;
use deluge_sim_link::audio::{GuiEnds, HeapCons};
use hardware_link::HardwareMirror;
use link::{InProcessLink, LinkKind, PanelLink};

/// The faceplate art, rasterised at launch.
const DELUGE_SVG: &[u8] = include_bytes!("../assets/Deluge.svg");

/// Rasterise the faceplate SVG to an iced image handle (half native resolution —
/// iced scales it to the window anyway).
fn render_svg() -> Option<iced::widget::image::Handle> {
    const WIDTH: u32 = 1089;
    const HEIGHT: u32 = 741;
    const SCALE: f32 = 0.5;
    let tree = usvg::Tree::from_data(DELUGE_SVG, &usvg::Options::default()).ok()?;
    let mut pixmap = tiny_skia::Pixmap::new(WIDTH, HEIGHT)?;
    resvg::render(&tree, tiny_skia::Transform::from_scale(SCALE, SCALE), &mut pixmap.as_mut());
    Some(iced::widget::image::Handle::from_rgba(WIDTH, HEIGHT, pixmap.data().to_vec()))
}

/// Launch the iced window with the given link and optional audio-scope monitor.
/// Blocks until the window closes.
fn run_app(
    link: LinkKind,
    audio_monitor: Option<HeapCons<[f32; 2]>>,
    volume: audio::Volume,
    hardware: Option<HardwareMirror>,
) -> iced::Result {
    let svg = render_svg();
    // iced's init closure must be `Fn` (called once); hand the link + svg +
    // monitor + volume + hardware through a Mutex<Option<_>> so the first call can
    // `take()` them.
    let init = std::sync::Mutex::new(Some((link, svg, audio_monitor, volume, hardware)));
    iced::application(
        move || {
            let (link, svg, mon, vol, hw) = init.lock().unwrap().take().expect("init called once");
            (DelugeSimulator::new(link, svg, mon, vol, hw), iced::Task::none())
        },
        DelugeSimulator::update,
        DelugeSimulator::view,
    )
    .title(|s: &DelugeSimulator| s.title())
    // The CV/gate rack strip above + the faceplate (half the SVG's 2178×1482).
    // Not resizable: a tiling Wayland compositor would otherwise tile (and
    // letterbox) the window, and Wayland blocks client-side resize/reposition
    // anyway — so collapsing the rack hides its contents rather than resizing.
    .window_size(iced::Size::new(app::WINDOW_WIDTH, app::WINDOW_HEIGHT))
    .resizable(false)
    .theme(|s: &DelugeSimulator| s.theme())
    .subscription(|s: &DelugeSimulator| s.subscription())
    .run()
}

/// Run the panel against an external brain over `deluge-protocol`. `target` is a
/// connect string (`host:port`, `:port`, `tcp://…`, or a Unix socket path); pass
/// `None` for a passive, blank panel.
pub fn run_connected(target: Option<&str>) -> Result<(), Box<dyn std::error::Error>> {
    let link = match target {
        Some(t) => {
            println!("deluge-simulator: connecting to brain at {t} ...");
            LinkKind::Protocol(PanelLink::connect(t)?)
        }
        None => {
            println!("deluge-simulator: no --connect target; opening a passive (blank) panel.");
            LinkKind::None
        }
    };
    // No in-process audio over the protocol link, so no scope monitor; the
    // volume knob is inert (nothing to attenuate). The `--hardware` mirror is
    // in-process only (it re-encodes the shared panel), so none here.
    run_app(link, None, audio::new_volume(), None)?;
    Ok(())
}

/// Run the panel against an SDK app in the same process: start host audio, then
/// hand the main thread to the GUI. Returns when the window closes.
///
/// Called by the SDK's host runtime (`cargo deluge sim`). The `panel` carries
/// illumination + input; `gui_audio` carries the audio rings to the host's
/// speakers/mic.
pub fn run_in_process(panel: SharedPanel, gui_audio: GuiEnds) {
    // Master volume shared between the faceplate Volume knob and the audio
    // output callback.
    let volume = audio::new_volume();
    // Kept alive until the window closes (dropping the streams stops audio and
    // finalises any --audio-out recording). `monitor` is a stereo tap of the
    // output for the rack's audio oscilloscopes.
    let (_audio, monitor) =
        audio::start(gui_audio, volume.clone(), audio::AudioConfig::from_env());
    // Host MIDI bridge: virtual ports ⇄ the panel (kept alive for the session).
    let _midi = midi::start(panel.clone());
    // Optional physical Deluge control surface (`cargo deluge sim --hardware`,
    // forwarded as `DELUGE_SIM_HARDWARE`). An empty value or `auto` auto-detects
    // by USB VID/PID. A connect failure is non-fatal: warn and run GUI-only.
    let hardware = connect_hardware(panel.clone());
    let link = LinkKind::InProcess(InProcessLink::new(panel));
    if let Err(e) = run_app(link, Some(monitor), volume, hardware) {
        eprintln!("deluge-simulator: {e}");
    }
}

/// Build the optional hardware control-surface mirror from `DELUGE_SIM_HARDWARE`.
/// Returns `None` (and warns) when unset or when the device can't be opened.
fn connect_hardware(panel: SharedPanel) -> Option<HardwareMirror> {
    let spec = std::env::var("DELUGE_SIM_HARDWARE").ok()?;
    let port = if spec.is_empty() || spec == "auto" {
        match HardwareMirror::detect() {
            Some(p) => p,
            None => {
                log::warn!("--hardware: no Deluge found by USB VID/PID");
                return None;
            }
        }
    } else {
        spec
    };
    match HardwareMirror::connect(&port, panel) {
        Ok(hw) => {
            log::info!("hardware control surface on {port}");
            Some(hw)
        }
        Err(e) => {
            log::warn!("--hardware: could not open {port}: {e}");
            None
        }
    }
}
