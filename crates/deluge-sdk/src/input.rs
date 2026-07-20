//! Unified async input event stream (pads, buttons, encoders).
//!
//! Two hardware sources are merged into one queue:
//! - **pads & buttons** arrive over the PIC RX stream — [`crate::pic_service`]'s
//!   pump decodes them and calls [`route_pic_event`];
//! - **encoders** are wired to RZ/A1L GPIO interrupts — an encoder-pump task
//!   (started by [`crate::plat::input_start_pump`]) wakes on the encoder IRQ,
//!   drains the detent deltas, and enqueues them.
//!
//! Apps drain the queue via [`Input::next`].

#[cfg(target_os = "none")]
use deluge_bsp::pic;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// A decoded input event.
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A grid pad changed state. `x` 0–17, `y` 0–7.
    Pad { x: u8, y: u8, pressed: bool },
    /// A button changed state. `id` is the raw PIC button id (0–35).
    Button { id: u8, pressed: bool },
    /// An encoder turned. `index` 0–5; `delta` is signed detents since the last
    /// event (positive = clockwise).
    Encoder { index: u8, delta: i8 },
}

/// Bounded event queue. If an app stops draining, the oldest events are dropped
/// (producers use `try_send`) so input never stalls the PIC pump or an ISR-fed
/// task.
pub(crate) static EVENTS: Channel<CriticalSectionRawMutex, Event, 32> = Channel::new();

/// The input event stream, taken once from [`Deluge::input`](crate::Deluge::input).
pub struct Input {
    _not_send: crate::NotSend,
}

impl Input {
    pub(crate) fn new() -> Self {
        Self {
            _not_send: crate::NOT_SEND,
        }
    }

    /// Await the next input event.
    #[inline]
    pub async fn next(&self) -> Event {
        EVENTS.receive().await
    }

    /// Return the next event if one is queued, without awaiting.
    #[inline]
    pub fn try_next(&self) -> Option<Event> {
        EVENTS.try_receive().ok()
    }
}

/// Map a PIC pad/button event into an [`Event`] and enqueue it.
///
/// Called by the PIC pump for every non-OLED event; OLED chip-select echoes are
/// handled by the pump itself. Drops the event if the queue is full.
#[cfg(target_os = "none")]
pub(crate) fn route_pic_event(ev: pic::Event) {
    let mapped = match ev {
        pic::Event::PadPress { id } => {
            let (x, y) = pic::pad_coords(id);
            Event::Pad {
                x,
                y,
                pressed: true,
            }
        }
        pic::Event::PadRelease { id } => {
            let (x, y) = pic::pad_coords(id);
            Event::Pad {
                x,
                y,
                pressed: false,
            }
        }
        pic::Event::ButtonPress { id } => Event::Button { id, pressed: true },
        pic::Event::ButtonRelease { id } => Event::Button { id, pressed: false },
        // FirmwareVersion / NoPresses / future variants are not input events.
        _ => return,
    };
    let _ = EVENTS.try_send(mapped);
}

/// Start the backend's input-event producer (idempotent; the actual pump is
/// guarded and only spawned once per process). On device this brings up the
/// encoder-IRQ pump; on host it spawns the GUI-input pump. See
/// [`plat::input_start_pump`](crate::plat::input_start_pump).
pub(crate) fn ensure_started(spawner: Spawner) {
    crate::plat::input_start_pump(spawner);
}
