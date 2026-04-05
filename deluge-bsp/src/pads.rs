// ---------------------------------------------------------------------------
// Shared pad state — atomic bit fields (no mutex, no allocation)
// ---------------------------------------------------------------------------
//
// 144 pads packed into 5 × u32 (160 bits; high 16 bits of word 4 unused).
// AtomicU32::fetch_xor provides lock-free single-bit toggle.

use core::sync::atomic::{AtomicU32, Ordering};

pub static PAD_BITS: [AtomicU32; 5] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Return `true` if pad `id` (0–143) is currently lit.
#[inline]
pub fn pad_get(id: u8) -> bool {
    (PAD_BITS[id as usize / 32].load(Ordering::Relaxed) >> (id % 32)) & 1 != 0
}

/// Toggle pad `id`; return `true` if it is now lit.
#[inline]
pub fn pad_toggle(id: u8) -> bool {
    let old = PAD_BITS[id as usize / 32].fetch_xor(1 << (id % 32), Ordering::Relaxed);
    (old >> (id % 32)) & 1 == 0
}

/// Set every pad to `val` (true = all lit, false = all dark).
pub fn pad_set_all(val: bool) {
    let fill = if val { !0u32 } else { 0u32 };
    for slot in PAD_BITS[..4].iter() {
        slot.store(fill, Ordering::Relaxed);
    }
    PAD_BITS[4].store(fill & 0x0000_FFFF, Ordering::Relaxed);
}

/// Flip every pad's lit state.
pub fn pad_invert_all() {
    for slot in PAD_BITS[..4].iter() {
        slot.fetch_xor(!0u32, Ordering::Relaxed);
    }
    PAD_BITS[4].fetch_xor(0x0000_FFFF, Ordering::Relaxed);
}

/// Convert display grid position (x ∈ 0..18, y ∈ 0..8) to a pad ID 0–143.
///
/// `y` is in pad coordinates where y=0 is the bottom row (lower-left origin).
/// Inverse of [`crate::pic::pad_coords`].
#[inline]
pub fn pad_id_from_xy(x: u8, y: u8) -> u8 {
    if x.is_multiple_of(2) {
        y * 9 + x / 2
    } else {
        (y + 8) * 9 + (x - 1) / 2
    }
}
