//! Headless run mode: drive an SDK app from a script and dump golden-frame
//! snapshots, with no GUI — for CI / `#[test]`-style integration testing.
//!
//! Selected by the `DELUGE_HEADLESS` env var (set by `cargo deluge sim
//! --headless`). The SDK host runtime hands us the same `SharedPanel` + audio
//! bridge it would give the GUI; we run a tiny driver instead of `iced`:
//!   - a **null audio pacer** keeps an audio app's DSP loop running without a
//!     real device (drains output, feeds silence);
//!   - a **script** (`DELUGE_SIM_SCRIPT`) of timed input events + snapshots is
//!     replayed against the panel;
//!   - **snapshots** write the OLED to a PNG and the pad/LED/CV/gate state to a
//!     diffable `.state` text file under `DELUGE_SIM_OUT` (default `.`).
//!
//! Script format — one step per line, `#` starts a comment:
//! ```text
//! # <at_ms> <command> <args…>
//! 0    button 25 down
//! 120  encoder 4 +1
//! 150  pad 3 5 down
//! 300  snapshot after-edit
//! 600  quit
//! ```

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use deluge_sim_link::audio::{Consumer, GuiEnds, Producer};
use deluge_sim_link::{DISPLAY_BYTES, InputEvent, LED_COUNT, PAD_COLS, PAD_ROWS, SharedPanel};

/// Run the app headlessly: replay the script, dump snapshots, then return (the
/// SDK host runtime exits the process afterwards). The app is already running on
/// the brain thread by the time this is called.
pub fn run_headless(panel: SharedPanel, gui_audio: GuiEnds) {
    start_null_pacer(gui_audio);

    let out_dir = std::env::var_os("DELUGE_SIM_OUT")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    if let Err(e) = std::fs::create_dir_all(&out_dir) {
        eprintln!("deluge-sim headless: cannot create {out_dir:?}: {e}");
        return;
    }

    let steps = match std::env::var_os("DELUGE_SIM_SCRIPT") {
        Some(path) => match parse_script(Path::new(&path)) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("deluge-sim headless: {e}");
                return;
            }
        },
        // No script: let the app render, snapshot once, quit.
        None => vec![
            Step {
                at_ms: 500,
                action: Action::Snapshot("frame".into()),
            },
            Step {
                at_ms: 500,
                action: Action::Quit,
            },
        ],
    };

    let start = Instant::now();
    for step in steps {
        let target = Duration::from_millis(step.at_ms);
        if let Some(wait) = target.checked_sub(start.elapsed()) {
            std::thread::sleep(wait);
        }
        match step.action {
            Action::Input(ev) => panel.push_event(ev),
            Action::Snapshot(name) => dump_snapshot(&panel, &out_dir, &name),
            Action::Quit => break,
        }
    }
}

/// Drain the app's audio output and feed it silence at a steady cadence, so an
/// audio app's `process` loop keeps running without a real output device.
fn start_null_pacer(gui_audio: GuiEnds) {
    let GuiEnds { mut out, mut in_ } = gui_audio;
    std::thread::Builder::new()
        .name("deluge-null-audio".into())
        .spawn(move || {
            loop {
                while out.try_pop().is_some() {}
                while in_.try_push([0.0, 0.0]).is_ok() {}
                std::thread::sleep(Duration::from_millis(3));
            }
        })
        .ok();
}

struct Step {
    at_ms: u64,
    action: Action,
}

enum Action {
    Input(InputEvent),
    Snapshot(String),
    Quit,
}

fn parse_script(path: &Path) -> Result<Vec<Step>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("read {path:?}: {e}"))?;
    let mut steps = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let n = i + 1;
        let mut t = line.split_whitespace();
        let at_ms: u64 = t
            .next()
            .unwrap()
            .parse()
            .map_err(|_| format!("line {n}: expected a millisecond timestamp"))?;
        let cmd = t
            .next()
            .ok_or_else(|| format!("line {n}: missing command"))?;
        let action = match cmd {
            "button" => Action::Input(InputEvent::Button {
                id: int(&mut t, n, "button id")? as u8,
                pressed: press(&mut t, n)?,
            }),
            "pad" => Action::Input(InputEvent::Pad {
                x: int(&mut t, n, "pad x")? as u8,
                y: int(&mut t, n, "pad y")? as u8,
                pressed: press(&mut t, n)?,
            }),
            "encoder" => Action::Input(InputEvent::Encoder {
                index: int(&mut t, n, "encoder index")? as u8,
                delta: int(&mut t, n, "encoder delta")? as i8,
            }),
            "snapshot" => Action::Snapshot(t.next().unwrap_or("frame").to_string()),
            "quit" => Action::Quit,
            other => return Err(format!("line {n}: unknown command {other:?}")),
        };
        steps.push(Step { at_ms, action });
    }
    Ok(steps)
}

fn int<'a>(t: &mut impl Iterator<Item = &'a str>, n: usize, what: &str) -> Result<i64, String> {
    let s = t
        .next()
        .ok_or_else(|| format!("line {n}: missing {what}"))?;
    s.trim_start_matches('+')
        .parse()
        .map_err(|_| format!("line {n}: bad {what} {s:?}"))
}

fn press<'a>(t: &mut impl Iterator<Item = &'a str>, n: usize) -> Result<bool, String> {
    match t.next() {
        Some("down" | "press" | "1" | "true") => Ok(true),
        Some("up" | "release" | "0" | "false") => Ok(false),
        Some(other) => Err(format!("line {n}: expected down/up, got {other:?}")),
        None => Err(format!("line {n}: missing down/up")),
    }
}

/// Dump the OLED (PNG) and the pad/LED/CV/gate state (text) under `dir`.
fn dump_snapshot(panel: &SharedPanel, dir: &Path, name: &str) {
    let png = dir.join(format!("{name}.png"));
    if let Err(e) = write_oled_png(&png, &panel.display_snapshot()) {
        eprintln!("deluge-sim headless: writing {png:?}: {e}");
    }
    let state = dir.join(format!("{name}.state"));
    if let Err(e) = std::fs::write(&state, state_text(panel)) {
        eprintln!("deluge-sim headless: writing {state:?}: {e}");
    }
    eprintln!("deluge-sim headless: snapshot {name:?} → {png:?}, {state:?}");
}

/// OLED frame dimensions (the panel framebuffer is page-major: 6 pages × 8 rows).
const OLED_W: usize = 128;
const OLED_H: usize = (DISPLAY_BYTES / OLED_W) * 8; // 6 pages × 8 = 48 rows
const PNG_SCALE: usize = 4;

/// Render the 1-bpp OLED framebuffer to a scaled grayscale PNG (lit = white).
fn write_oled_png(path: &Path, fb: &[u8; DISPLAY_BYTES]) -> std::io::Result<()> {
    let (w, h) = (OLED_W * PNG_SCALE, OLED_H * PNG_SCALE);
    let mut img = vec![0u8; w * h];
    for page in 0..(DISPLAY_BYTES / OLED_W) {
        for col in 0..OLED_W {
            let byte = fb[page * OLED_W + col];
            for bit in 0..8 {
                if byte & (1 << bit) != 0 {
                    let (x0, y0) = (col * PNG_SCALE, (page * 8 + bit) * PNG_SCALE);
                    for dy in 0..PNG_SCALE {
                        for dx in 0..PNG_SCALE {
                            img[(y0 + dy) * w + (x0 + dx)] = 0xFF;
                        }
                    }
                }
            }
        }
    }
    let file = std::fs::File::create(path)?;
    let mut enc = png::Encoder::new(std::io::BufWriter::new(file), w as u32, h as u32);
    enc.set_color(png::ColorType::Grayscale);
    enc.set_depth(png::BitDepth::Eight);
    let mut writer = enc.write_header().map_err(std::io::Error::other)?;
    writer.write_image_data(&img).map_err(std::io::Error::other)
}

/// A diffable text dump of the non-default panel state for golden comparison.
fn state_text(panel: &SharedPanel) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let cv = panel.cv_snapshot();
    let _ = writeln!(s, "cv {} {} {} {}", cv[0], cv[1], cv[2], cv[3]);
    let g = panel.gate_snapshot();
    let _ = writeln!(
        s,
        "gate {} {} {} {}",
        g[0] as u8, g[1] as u8, g[2] as u8, g[3] as u8
    );
    let _ = writeln!(s, "synced {}", panel.synced_led() as u8);
    let leds = panel.leds_snapshot();
    for (i, on) in leds.iter().enumerate().take(LED_COUNT) {
        if *on {
            let _ = writeln!(s, "led {i}");
        }
    }
    let pads = panel.pads_snapshot();
    for col in 0..PAD_COLS {
        for row in 0..PAD_ROWS {
            let [r, gr, b] = pads[col][row];
            if r | gr | b != 0 {
                let _ = writeln!(s, "pad {col} {row} {r} {gr} {b}");
            }
        }
    }
    s
}
