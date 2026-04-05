use core::sync::atomic::{AtomicI8, Ordering};

use embassy_sync::waitqueue::AtomicWaker;
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
    if delta == 0 {
        return 0;
    }

    *edge_accumulator = edge_accumulator.saturating_add(delta);

    let mut detents = 0;
    while *edge_accumulator > 1 {
        *edge_accumulator -= 2;
        detents += 1;
    }
    while *edge_accumulator < -1 {
        *edge_accumulator += 2;
        detents -= 1;
    }

    detents
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

/// Initialise the Deluge's interrupt-driven quadrature encoder inputs.
///
/// This is board-specific GPIO and GIC setup for the six front-panel encoders.
/// The firmware task consumes [`ENCODER_DELTAS`] and [`ENCODER_WAKER`] to apply
/// product-specific behavior.
///
/// # Safety
/// Must be called before `cortex_ar::interrupt::enable()`.
pub unsafe fn irq_init() {
    unsafe {
        const SETUP: [(u8, u8, u16, u8, bool); NUM_ENCODERS] = [
            (11, 12, 35, 3, false),
            (6, 7, 34, 2, true),
            (0, 15, 36, 4, false),
            (5, 4, 33, 1, false),
            (8, 10, 32, 0, false),
            (2, 3, 38, 6, false),
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
        rza1l_hal::gic::register(38, || enc_irq_handler(5, 2, 3, 6, false));

        for &(_, _, gic_id, _, _) in &SETUP {
            rza1l_hal::gic::set_priority(gic_id, 14);
            rza1l_hal::gic::enable(gic_id);
        }

        info!("encoder: interrupt-driven init complete (IRQ0/1/2/3/4/6 → GIC 32–38)");
    }
}
