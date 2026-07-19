//! Device backend ops (deluge-bsp peripherals). Bodies moved verbatim from the
//! capability modules' `#[cfg(target_os = "none")]` arms.
use core::convert::Infallible;
use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use deluge_bsp::jacks::{self, Jack};
use deluge_bsp::oled::{self, FrameBuffer};
use deluge_bsp::pic;
use deluge_bsp::rgb::PadLeds;
use deluge_bsp::trigger_clock;
use embassy_time::Instant;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use rza1l_hal::gpio::{Output, Pin};

/// The SYNC LED is wired to port 6, pin 7.
type SyncLedPin = Pin<6, 7, Output>;

pub(crate) async fn oled_init_panel() {
    oled::init().await;
}
pub(crate) async fn oled_flush(fb: &FrameBuffer) {
    oled::send_frame(fb).await;
}

pub(crate) async fn pads_flush(leds: &mut PadLeds) {
    leds.flush().await;
}
pub(crate) async fn pads_set_brightness_interval(interval: u8) {
    deluge_bsp::pic::set_refresh_time(interval).await;
}

pub(crate) async fn leds_set(id: u8, on: bool) {
    if on {
        pic::led_on(id).await;
    } else {
        pic::led_off(id).await;
    }
}
pub(crate) async fn leds_clear() {
    for id in 0..crate::leds::Leds::NUM_INDICATOR_LEDS {
        pic::led_off(id).await;
    }
}
pub(crate) async fn leds_gold_knob(knob: u8, brightness: [u8; 4]) {
    pic::set_gold_knob_indicators(knob, brightness).await;
}

pub(crate) fn sync_led_init() -> SyncLedPin {
    // SAFETY: the take-once guard in `Deluge::sync_led` ensures this runs once
    // and nothing else owns P6_7; clocks are up by the time an app runs.
    unsafe { SyncLedPin::into_output() }
}
pub(crate) fn sync_led_set_high(pin: &mut SyncLedPin) -> Result<(), Infallible> {
    pin.set_high()
}
pub(crate) fn sync_led_set_low(pin: &mut SyncLedPin) -> Result<(), Infallible> {
    pin.set_low()
}
pub(crate) fn sync_led_toggle(pin: &mut SyncLedPin) -> Result<(), Infallible> {
    StatefulOutputPin::toggle(pin)
}
pub(crate) fn sync_led_is_set_high(pin: &mut SyncLedPin) -> Result<bool, Infallible> {
    pin.is_set_high()
}
pub(crate) fn sync_led_is_set_low(pin: &mut SyncLedPin) -> Result<bool, Infallible> {
    pin.is_set_low()
}

pub(crate) fn cv_gate_init() {
    // SAFETY: runs once. Configures GPIO + RSPI0 and runs the DAC's ~10 ms
    // linearity init (poll-based delays). Acquire CV/gate before entering a
    // loop that also drives the OLED, so this one-time RSPI0 setup can't race an
    // in-flight OLED transfer (see docs/advanced-guide.md §7).
    unsafe { deluge_bsp::cv_gate::init() };
}
pub(crate) async fn cv_set(ch: u8, code: u16) {
    deluge_bsp::cv_gate::cv_set(ch, code).await;
}
pub(crate) fn gate_set(ch: u8, on: bool) {
    // SAFETY: GPIO write to a gate line we own; pins configured by init.
    unsafe { deluge_bsp::cv_gate::gate_set(ch, on) };
}

/// MIDI DIN baud rate.
const MIDI_BAUD: u32 = 31_250;

pub(crate) fn midi_init() {
    // SAFETY: runs once. Sets up SCIF0 with DMA RX and registers its TX handler
    // before the source is enabled, so it is safe at runtime.
    unsafe { deluge_bsp::uart::init_midi(MIDI_BAUD) };
}
pub(crate) async fn midi_send(data: &[u8]) {
    deluge_bsp::uart::write_midi(data).await;
}
pub(crate) async fn midi_recv() -> u8 {
    deluge_bsp::uart::read_midi_byte().await
}
pub(crate) fn midi_try_recv() -> Option<u8> {
    deluge_bsp::uart::try_read_midi()
}

pub(crate) fn clock_in_init() {
    // SAFETY: runs once. Registers the P1_14/IRQ6 handler and enables the
    // GIC line. Registering lazily here (after global IRQ enable) matches
    // the proven `input()`/encoder precedent.
    unsafe { trigger_clock::irq_init() };
}
pub(crate) async fn clock_in_wait_edge() -> u64 {
    let start = trigger_clock::EDGE_COUNT.load(Ordering::Relaxed);
    poll_fn(|cx| {
        trigger_clock::EDGE_WAKER.register(cx.waker());
        if trigger_clock::EDGE_COUNT.load(Ordering::Relaxed) != start {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;
    trigger_clock::LAST_EDGE_TICKS.load(Ordering::Relaxed)
}
pub(crate) fn clock_in_count() -> u32 {
    trigger_clock::EDGE_COUNT.load(Ordering::Relaxed)
}
pub(crate) fn clock_in_last_edge() -> Option<Instant> {
    match trigger_clock::LAST_EDGE_TICKS.load(Ordering::Relaxed) {
        0 => None,
        t => Some(Instant::from_ticks(t)),
    }
}

pub(crate) fn jacks_init() {
    // SAFETY: runs once. Configures the five jack-detect inputs and the
    // speaker-enable output (left disabled).
    unsafe { jacks::init() };
}
pub(crate) fn jacks_headphone() -> bool {
    jacks::is_inserted(Jack::Headphone)
}
pub(crate) fn jacks_line_in() -> bool {
    jacks::is_inserted(Jack::LineIn)
}
pub(crate) fn jacks_mic() -> bool {
    jacks::is_inserted(Jack::Mic)
}
pub(crate) fn jacks_line_out_left() -> bool {
    jacks::is_inserted(Jack::LineOutL)
}
pub(crate) fn jacks_line_out_right() -> bool {
    jacks::is_inserted(Jack::LineOutR)
}
pub(crate) fn jacks_set_speaker(on: bool) {
    // SAFETY: GPIO write to the speaker-enable output configured by init.
    unsafe { jacks::set_speaker_enable(on) };
}
