//! Hardware event bus — shared between input tasks and the CDC task.
//!
//! Input tasks (`pic_task`, `encoder_task`) push events here with
//! [`try_send`][embassy_sync::channel::Channel::try_send].  The CDC task
//! drains the channel and encodes each event as a `MessageFromDeluge` frame.
//!
//! Channel capacity is 32; overflow silently drops events, matching the
//! C firmware's rx-buffer-discard flow-control policy.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// A hardware input event forwarded to the USB CDC task.
#[derive(Clone, Copy)]
pub(crate) enum HardwareEvent {
    /// Pad pressed. `col` 0–17, `row` 0–7 (CDC wire format).
    PadPressed { col: u8, row: u8 },
    /// Pad released. `col` 0–17, `row` 0–7.
    PadReleased { col: u8, row: u8 },
    /// Button pressed. `id` uses the raw Deluge button-id space (`0..=35`),
    /// including encoder shaft clicks.
    ButtonPressed { id: u8 },
    /// Button released. `id` uses the raw Deluge button-id space (`0..=35`).
    ButtonReleased { id: u8 },
    /// Encoder rotated. `id` uses `deluge_bsp::controls::encoder::*`; `delta` is
    /// signed (positive = clockwise).
    EncoderRotated { id: u8, delta: i8 },
}

/// Channel capacity — must be a power of two.
const CAPACITY: usize = 32;

/// Broadcast channel from hardware tasks → CDC task.
pub(crate) static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, HardwareEvent, CAPACITY> =
    Channel::new();
