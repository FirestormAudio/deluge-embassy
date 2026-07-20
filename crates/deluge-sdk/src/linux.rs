//! Native Linux backend, active under `feature = "linux"` — the capability
//! modules drive real hardware through `libdeluge` (`deluge-hal-linux`) instead
//! of the simulator panel. Owns the process-wide handle, set up once by
//! [`crate::__rt::linux::run`] before any app code runs.
use std::sync::{Mutex, MutexGuard, OnceLock};

use deluge_hal_linux::Deluge;

static DELUGE: OnceLock<Mutex<Deluge>> = OnceLock::new();

pub(crate) fn init(d: Deluge) {
    let _ = DELUGE.set(Mutex::new(d));
}
pub(crate) fn dev() -> MutexGuard<'static, Deluge> {
    DELUGE
        .get()
        .expect("libdeluge not opened (run via `cargo deluge linux`)")
        .lock()
        .unwrap()
}
