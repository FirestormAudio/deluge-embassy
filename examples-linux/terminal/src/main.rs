//! Terminal: an interactive BusyBox shell on the OLED, typed on the pad grid.
//! Launched by the launcher; the SHIFT+TRIPLETS+LEARN kill-chord exits.

mod keys;
mod pty;
mod render;
mod term;

use deluge_hal_linux::{Deluge, Event};
use deluge_linux_ui::deluge_grid_toolkit::imode::{GridUi, PadInput};
use deluge_linux_ui::deluge_grid_toolkit::{KeyPress, KeyboardLayout, TextKeyboardComponent};
use deluge_linux_ui::{pads, OledTarget, PadTarget};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};
use term::Term;

enum Msg {
    Input(Event),
    TermChanged,
    ShellExited(i32),
}

/// A running shell: the pty, the shared screen model, and its reader thread.
struct Session {
    pty: Arc<pty::Pty>,
    term: Arc<Mutex<Term>>,
}

impl Session {
    fn start(tx: &Sender<Msg>) -> Session {
        let pty = Arc::new(pty::spawn(term::COLS as u16, term::ROWS as u16).expect("spawn shell"));
        let term = Arc::new(Mutex::new(Term::new()));
        let (rp, rt, txr) = (pty.clone(), term.clone(), tx.clone());
        std::thread::spawn(move || {
            let mut buf = [0u8; 1024];
            loop {
                match rp.read(&mut buf) {
                    Ok(0) => {
                        let st = rp.try_reap().unwrap_or(0);
                        let _ = txr.send(Msg::ShellExited(st));
                        break;
                    }
                    Ok(n) => {
                        rt.lock().unwrap().feed(&buf[..n]);
                        let _ = txr.send(Msg::TermChanged);
                    }
                    Err(_) => {
                        let st = rp.try_reap().unwrap_or(0);
                        let _ = txr.send(Msg::ShellExited(st));
                        break;
                    }
                }
            }
        });
        Session { pty, term }
    }
}

fn now_ms() -> u32 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: writing a timespec we own; CLOCK_MONOTONIC always exists.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64 * 1000 + ts.tv_nsec as u64 / 1_000_000) as u32
}

fn relight_pads(
    ui: &mut GridUi,
    keyboard: &TextKeyboardComponent,
    shift: bool,
    input: PadInput,
    pad_target: &mut PadTarget,
    dlg: &mut Deluge,
) -> Option<KeyPress> {
    let key = ui.run(now_ms(), input, |f| keyboard.show(f, shift)).painted().flatten();
    pad_target.blit(ui.grid());
    let _ = pad_target.flush(dlg);
    key
}

fn main() {
    let mut dlg = match Deluge::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("terminal: deluge_open failed: {e}");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel::<Msg>();
    let tx_in = tx.clone();
    if let Err(e) = dlg.input_start(move |ev| {
        let _ = tx_in.send(Msg::Input(ev));
    }) {
        eprintln!("terminal: input_start failed: {e}");
        std::process::exit(1);
    }

    pty::ensure_devpts();
    let mut session = Session::start(&tx);

    let mut oled = OledTarget::new();
    let mut pad_target = PadTarget::new();
    let keyboard = TextKeyboardComponent::new(KeyboardLayout::Qwerty);
    let mut ui = GridUi::new();
    let mut shift = false;
    let mut exited: Option<i32> = None;

    // Initial paint: light the keyboard on the pads, draw the empty screen.
    relight_pads(&mut ui, &keyboard, shift, PadInput::new(), &mut pad_target, &mut dlg);
    render::render(&session.term.lock().unwrap().screen, &mut oled);
    let _ = oled.flush(&mut dlg);

    for msg in rx {
        match msg {
            Msg::Input(ev) if pads::pad_from_event(&ev).is_some() => {
                let input = pads::pad_input(&ev);
                let key = relight_pads(&mut ui, &keyboard, shift, input, &mut pad_target, &mut dlg);
                let Some(k) = key else { continue };
                match k {
                    KeyPress::Shift => shift = !shift,
                    KeyPress::Enter if exited.is_some() => {
                        session = Session::start(&tx);
                        exited = None;
                        render::render(&session.term.lock().unwrap().screen, &mut oled);
                        let _ = oled.flush(&mut dlg);
                    }
                    _ if exited.is_some() => {} // ignore other keys once exited
                    other => {
                        if let Some(bytes) = keys::keypress_bytes(other) {
                            let _ = session.pty.write_all(&bytes);
                        }
                        shift = false; // sticky one-shot
                    }
                }
            }
            Msg::Input(ev) if ev.kind == keys::EV_BUTTON && ev.value == 1 => {
                if ev.id == deluge_linux_ui::controls::button::SHIFT {
                    shift = !shift;
                    // reflect the new shift state on the pad keyboard immediately
                    ui.request_repaint();
                    relight_pads(&mut ui, &keyboard, shift, PadInput::new(), &mut pad_target, &mut dlg);
                } else if let Some(b) = keys::button_bytes(ev.id) {
                    let _ = session.pty.write_all(b);
                }
            }
            Msg::Input(ev) if ev.kind == keys::EV_ENCODER => {
                let b = keys::encoder_bytes(ev.id, ev.value);
                if !b.is_empty() {
                    let _ = session.pty.write_all(&b);
                }
            }
            Msg::Input(_) => {}
            Msg::TermChanged => {
                if exited.is_none() {
                    render::render(&session.term.lock().unwrap().screen, &mut oled);
                    let _ = oled.flush(&mut dlg);
                }
            }
            Msg::ShellExited(status) => {
                exited = Some(status);
                render::render_exited(&session.term.lock().unwrap().screen, status, &mut oled);
                let _ = oled.flush(&mut dlg);
            }
        }
    }
}
