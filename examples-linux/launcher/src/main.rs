//! Reference Linux appliance: the app launcher.
//!
//! Lists `/sd/LINUX/APPS/` on the OLED, launches the selected app with the select
//! encoder, and always returns here on SHIFT+TRIPLETS+LEARN held ~1s.

mod apps;
mod killwatch;
mod model;
mod supervisor;
mod ui;

use deluge_hal_linux::{Deluge, Event};
use deluge_linux_ui::{controls, OledTarget};
use killwatch::KillWatch;
use model::{Model, Screen};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant};
use ui::View;

const FRAME: Duration = Duration::from_millis(33);
const HOLD_MS: u64 = 1000;

// Event kinds (mirror deluge-linux / include/deluge/input.h).
const EV_BUTTON: u8 = 1;
const EV_ENCODER: u8 = 2;

enum Msg {
    Input(Event),
    ChildExited(supervisor::Exit),
}

fn main() {
    let mut dlg = match Deluge::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("launcher: deluge_open failed: {e}");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel::<Msg>();

    // Input: the SDK-owned thread forwards every event onto the channel.
    let input_tx = tx.clone();
    if let Err(e) = dlg.input_start(move |ev| {
        let _ = input_tx.send(Msg::Input(ev));
    }) {
        eprintln!("launcher: input_start failed: {e}");
        std::process::exit(1);
    }

    let mut target = OledTarget::new();
    let mut view = View::new();
    let mut model = Model::new(apps::scan(&apps::apps_dir()));
    let mut kw = KillWatch::new(HOLD_MS);

    // While a child runs: its pid (process group) and whether we initiated a kill.
    let mut running_pid: Option<i32> = None;
    let mut killed_by_user = false;

    // SHIFT is a modifier for chords (SHIFT+SELECT opens SETTINGS).
    let mut shift_held = false;

    let start = Instant::now();
    let now_ms = || start.elapsed().as_millis() as u64;

    // Initial paint.
    view.render(&model, &mut target);
    let _ = target.flush(&mut dlg);

    let mut last_tick = Instant::now();

    loop {
        match rx.recv_timeout(FRAME) {
            Ok(Msg::Input(ev)) => {
                let t = now_ms();
                if ev.kind == EV_BUTTON {
                    let pressed = ev.value == 1;
                    // Track SHIFT for chords (SHIFT+SELECT → SETTINGS).
                    if ev.id == controls::button::SHIFT {
                        shift_held = pressed;
                    }
                    // Kill-chord watch runs in every mode.
                    if kw.on_button(ev.id, pressed, t) {
                        if let Some(pid) = running_pid {
                            killed_by_user = true;
                            supervisor::kill_group(pid);
                        }
                    }
                    // Screen-aware clicks, only while browsing.
                    if model.is_browsing() && pressed {
                        if ev.id == controls::encoder_button::SELECT {
                            match model.screen() {
                                Screen::Apps => {
                                    if shift_held {
                                        model.open_settings();
                                    } else {
                                        launch_selected(
                                            &mut model,
                                            &tx,
                                            &mut running_pid,
                                            &mut killed_by_user,
                                            t,
                                        );
                                    }
                                }
                                Screen::Settings => model.activate_settings_item(),
                                Screen::ConfirmShutdown => model.confirm_shutdown(),
                            }
                        } else if ev.id == controls::button::BACK {
                            model.back();
                        }
                    }
                } else if ev.kind == EV_ENCODER
                    && ev.id == controls::encoder::SELECT
                    && model.is_browsing()
                {
                    match model.screen() {
                        Screen::Apps => model.move_selection(ev.value as i32),
                        Screen::Settings => model.move_settings_selection(ev.value as i32),
                        Screen::ConfirmShutdown => {}
                    }
                }
            }
            Ok(Msg::ChildExited(exit)) => {
                running_pid = None;
                model.on_child_exit(exit, killed_by_user, now_ms());
                killed_by_user = false;
                kw.reset();
                model.set_entries(apps::scan(&apps::apps_dir())); // rescan on return
                view.render(&model, &mut target);
                let _ = target.flush(&mut dlg);
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }

        // Frame cadence: poll the chord timer, expire toasts, animate + repaint
        // (only while browsing — a running child owns the OLED).
        let t = now_ms();
        if kw.poll(t) {
            if let Some(pid) = running_pid {
                killed_by_user = true;
                supervisor::kill_group(pid);
            }
        }
        model.tick(t);

        // A confirmed shutdown. This board can't cut its own power, so a clean
        // shutdown means: make the SD safe, blank the panel, then park the CPU
        // with a "SAFE TO POWER OFF" message the OLED holds on its own.
        //
        //   1. "SHUTTING DOWN" while `prepare` syncs + unmounts /sd.
        //   2. only once the card is safe: blank the LEDs/pads and draw the
        //      final message (so we never claim "safe" before it is).
        //   3. `stop` halts; on hardware it never returns. If it does (an
        //      off-device stub, or a prepare failure), fall back to the list.
        if model.take_shutdown_request() {
            view.render_shutting_down(&mut target);
            let _ = target.flush(&mut dlg);

            if run_shutdown("prepare").map(|s| s.success()).unwrap_or(false) {
                blank_panel(&mut dlg);
                view.render_safe_to_power_off(&mut target);
                let _ = target.flush(&mut dlg);
                // Both the OLED and pad framebuffers are deferred-io: their
                // blits land ~1 frame (33ms) later on a workqueue, and that
                // work stops the instant `stop` disables interrupts. Give the
                // blits time to reach the panels before we halt.
                std::thread::sleep(Duration::from_millis(200));
                let _ = run_shutdown("stop");
            }
            // prepare failed, or stop returned without halting (off-device).
            model.return_to_apps();
            model.show_toast("SHUTDOWN FAILED".into(), now_ms());
        }

        if model.is_browsing() {
            let delta = last_tick.elapsed().as_millis() as u32;
            last_tick = Instant::now();
            view.tick(&model, delta);
            view.render(&model, &mut target);
            let _ = target.flush(&mut dlg);
        } else {
            last_tick = Instant::now();
        }
    }
}

/// Launch the highlighted app: spawn it, mark the model running, and start a
/// reaper thread that reports the exit status back onto the channel.
fn launch_selected(
    model: &mut Model,
    tx: &mpsc::Sender<Msg>,
    running_pid: &mut Option<i32>,
    killed_by_user: &mut bool,
    now_ms: u64,
) {
    let Some(path) = model.selected_path() else {
        return;
    };
    let name = model
        .entries
        .get(model.selected)
        .map(|e| e.display_name.clone())
        .unwrap_or_default();

    match supervisor::launch(&path) {
        Ok(supervisor::Launched { pid, mut child }) => {
            *running_pid = Some(pid);
            *killed_by_user = false;
            model.begin_running(name);
            let reap_tx = tx.clone();
            std::thread::spawn(move || {
                let status = child.wait();
                let exit = match status {
                    Ok(s) => supervisor::classify(s),
                    Err(_) => supervisor::Exit::Nonzero(-1),
                };
                let _ = reap_tx.send(Msg::ChildExited(exit));
            });
        }
        Err(_) => {
            // exec failed — the app never started; show a toast and stay in the list.
            model.show_toast(format!("CAN'T RUN {name}"), now_ms);
        }
    }
}

/// Run a step of the SD-safe shutdown sequence. `verb` is `prepare` (sync +
/// unmount /sd, then return) or `stop` (halt the CPU). Defaults to
/// `/sbin/deluge-shutdown`; overridable via `LAUNCHER_SHUTDOWN_CMD` so running
/// the launcher off-device (on a workstation) cannot accidentally halt it.
fn run_shutdown(verb: &str) -> std::io::Result<std::process::ExitStatus> {
    let cmd = std::env::var("LAUNCHER_SHUTDOWN_CMD")
        .unwrap_or_else(|_| "/sbin/deluge-shutdown".to_string());
    std::process::Command::new(cmd).arg(verb).status()
}

/// Turn off every front-panel light so only the OLED message remains lit: the
/// 36 button indicators, both gold-knob LED columns, and the RGB pad grid. The
/// PIC latches the indicator/knob state; the pad grid is a deferred-io fb (its
/// blit lands with the OLED's, before the halt). Errors are ignored — a failed
/// blank must never block the shutdown itself.
fn blank_panel(dlg: &mut Deluge) {
    for id in 0..36 {
        let _ = dlg.leds_indicator(id, false);
    }
    for col in 0..2 {
        for i in 0..4 {
            let _ = dlg.leds_gold(col, i, 0);
        }
    }
    // DELUGE_PADS_BYTES: 18x8 RGB, 24bpp → 432 bytes, all zero = all dark.
    let _ = dlg.pads_write(&[0u8; 432]);
}
