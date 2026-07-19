//! Snake on the Deluge pad grid — a reference non-appliance (`/LINUX/APPS/`)
//! app. Steer with the tempo encoder; the launcher's kill-chord exits.

mod game;
mod render_pads;
mod tick;
mod ui;

use deluge_hal_linux::{Deluge, Event};
use deluge_linux_ui::{controls, OledTarget};
use game::{Game, State};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::Instant;
use tick::Ticker;

// Event kinds (mirror deluge-linux / include/deluge/input.h).
const EV_BUTTON: u8 = 1;
const EV_ENCODER: u8 = 2;

/// Seed the RNG from the monotonic clock (no `rand` crate).
fn seed() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: writing a valid timespec we own; CLOCK_MONOTONIC always exists.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (((ts.tv_sec as u64) << 32) ^ (ts.tv_nsec as u64)) | 1
}

fn repaint(game: &Game, dlg: &mut Deluge, oled: &mut OledTarget) {
    let _ = dlg.pads_write(&render_pads::frame(game));
    ui::render(game, oled);
    let _ = oled.flush(dlg);
}

fn main() {
    let mut dlg = match Deluge::open() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("snake: deluge_open failed: {e}");
            std::process::exit(1);
        }
    };

    let (tx, rx) = mpsc::channel::<Event>();
    if let Err(e) = dlg.input_start(move |ev| {
        let _ = tx.send(ev);
    }) {
        eprintln!("snake: input_start failed: {e}");
        std::process::exit(1);
    }

    let mut game = Game::new(seed());
    let mut oled = OledTarget::new();
    repaint(&game, &mut dlg, &mut oled);

    // Drive ticks off an absolute deadline: an incoming event shortens the wait
    // but never moves the tick, so turning a knob (a stream of encoder events)
    // can't starve `step()` and slow the game. See `tick::Ticker`.
    let mut ticker = Ticker::new(Instant::now(), game.tick_period());

    loop {
        let mut dirty = false;
        match rx.recv_timeout(ticker.wait(Instant::now())) {
            Ok(ev) => {
                if ev.kind == EV_ENCODER
                    && ev.id == controls::encoder::TEMPO
                    && game.state() == State::Playing
                {
                    game.turn(ev.value);
                    dirty = true;
                } else if ev.kind == EV_BUTTON
                    && ev.id == controls::encoder_button::TEMPO
                    && ev.value == 1
                    && game.state() == State::GameOver
                {
                    game.restart();
                    // A fresh game restores the start tick period; realign the
                    // deadline so the first new tick is a full period away.
                    ticker = Ticker::new(Instant::now(), game.tick_period());
                    dirty = true;
                }
            }
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }
        // Advance the game whenever the deadline has arrived, independent of why
        // the wait returned (event or timeout).
        if ticker.poll(Instant::now(), game.tick_period()) {
            game.step();
            dirty = true;
        }
        if dirty {
            repaint(&game, &mut dlg, &mut oled);
        }
    }
}
