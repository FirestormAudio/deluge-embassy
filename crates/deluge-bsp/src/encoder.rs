use core::sync::atomic::{AtomicI8, Ordering};

use embassy_sync::waitqueue::AtomicWaker;
#[cfg(target_os = "none")]
use log::info;

pub const NUM_ENCODERS: usize = 6;

/// Per-encoder signed delta accumulators, written by IRQ/TINT ISRs.
#[allow(clippy::declare_interior_mutable_const)]
pub static ENCODER_DELTAS: [AtomicI8; NUM_ENCODERS] = [
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
];

/// Wakes the firmware encoder task whenever any encoder delta becomes non-zero.
pub static ENCODER_WAKER: AtomicWaker = AtomicWaker::new();

/// Drain the accumulated edge delta for one encoder and convert it into whole detents.
///
/// Each encoder IRQ contributes ±1 per edge. Two accumulated edges in the same
/// direction produce one detent, matching the Deluge's physical click spacing.
#[inline]
pub fn take_detents(encoder_index: usize, edge_accumulator: &mut i8) -> i8 {
    let delta = ENCODER_DELTAS[encoder_index].swap(0, Ordering::Relaxed);
    // Pure edge→detent accumulation lives in `crate::encoder_detent` (host-tested).
    crate::encoder_detent::accumulate_detents(delta, edge_accumulator)
}

fn enc_irq_handler(enc_idx: usize, irq_pin: u8, companion: u8, irq_num: u8, invert: bool) {
    let pins = unsafe { rza1l_hal::gpio::read_port(1) };
    let irq_new = (pins >> irq_pin) & 1 != 0;
    let comp = (pins >> companion) & 1 != 0;
    let cw = if invert {
        irq_new != comp
    } else {
        irq_new == comp
    };

    ENCODER_DELTAS[enc_idx].fetch_add(if cw { 1 } else { -1 }, Ordering::Relaxed);
    unsafe { rza1l_hal::gic::clear_irq_pending(irq_num) };
    ENCODER_WAKER.wake();
}

/// One-time encoder GPIO + GIC edge-IRQ bring-up. Device only — the host has no
/// front-panel encoders; deltas simply never accumulate (see the host no-op).
///
/// This is board-specific GPIO and GIC setup for the six front-panel encoders.
/// The firmware task consumes [`ENCODER_DELTAS`] and [`ENCODER_WAKER`] to apply
/// product-specific behavior.
///
/// # Safety
/// Must be called before `cortex_ar::interrupt::enable()`.
#[cfg(target_os = "none")]
pub unsafe fn irq_init() {
    unsafe {
        const SETUP: [(u8, u8, u16, u8, bool); NUM_ENCODERS] = [
            (11, 12, 35, 3, false),
            (6, 7, 34, 2, true),
            (0, 15, 36, 4, false),
            (5, 4, 33, 1, false),
            (8, 10, 32, 0, false),
            // SELECT: P1_14 (trigger-clock input) owns IRQ6 (GIC 38), so SELECT uses
            // IRQ7 on P1_3 instead. A/B are swapped versus the polled wiring, so
            // `invert = true` to keep CW = positive direction.
            (3, 2, 39, 7, true),
        ];

        for &(irq_pin, comp_pin, _, _, _) in &SETUP {
            rza1l_hal::gpio::set_pin_mux(1, irq_pin, 2);
            rza1l_hal::gpio::enable_input_buffer(1, irq_pin);
            rza1l_hal::gpio::set_as_input(1, comp_pin);
        }

        for &(_, _, _, irq_num, _) in &SETUP {
            rza1l_hal::gic::set_irq_both_edges(irq_num);
        }

        rza1l_hal::gic::register(35, || enc_irq_handler(0, 11, 12, 3, false));
        rza1l_hal::gic::register(34, || enc_irq_handler(1, 6, 7, 2, true));
        rza1l_hal::gic::register(36, || enc_irq_handler(2, 0, 15, 4, false));
        rza1l_hal::gic::register(33, || enc_irq_handler(3, 5, 4, 1, false));
        rza1l_hal::gic::register(32, || enc_irq_handler(4, 8, 10, 0, false));
        rza1l_hal::gic::register(39, || enc_irq_handler(5, 3, 2, 7, true));

        for &(_, _, gic_id, _, _) in &SETUP {
            rza1l_hal::gic::set_priority(gic_id, 14);
            rza1l_hal::gic::enable(gic_id);
        }

        info!("encoder: interrupt-driven init complete (IRQ0/1/2/3/4/7 → GIC 32–36, 39)");
    }
}

/// Host: no front-panel hardware. Encoders are quiescent; `ENCODER_DELTAS`
/// stays zero and `ENCODER_WAKER` never fires, so any consumer parks cleanly.
///
/// # Safety
/// No-op; trivially safe. Signature matches the device variant so callers
/// don't need to branch on target.
#[cfg(not(target_os = "none"))]
pub unsafe fn irq_init() {}

#[cfg(all(test, not(target_os = "none")))]
mod host_tests {
    use super::*;
    use core::sync::atomic::Ordering;

    #[test]
    fn host_irq_init_is_a_safe_noop_and_encoders_are_quiescent() {
        // Must not touch hardware / must not panic on host.
        unsafe { irq_init() };
        assert_eq!(NUM_ENCODERS, 6);
        // Deltas start (and stay) zero with no ISRs firing.
        for d in &ENCODER_DELTAS {
            assert_eq!(d.load(Ordering::Relaxed), 0);
        }
        // take_detents on a zero delta yields no detents (pure-logic sanity).
        let mut acc = 0i8;
        assert_eq!(take_detents(0, &mut acc), 0);
    }
}
