//! Deluge SDK example: a line-in audio effect on the **per-block IRQ clock** (M5
//! v2). Identical to `audio_passthru` but built with `deluge/audio-irq`, so the
//! processing loop is driven by the RX DMA block interrupt (codec-locked,
//! drift-free, lower latency) instead of a timer poll. The `process()` API is
//! the same — only the cadence differs.
//!
//! **Requires a signal on the codec line-in.**

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![feature(impl_trait_in_assoc_type)]

use deluge::prelude::*;

/// Cubic soft saturator: `1.5x - 0.5x³`, output in [-1, 1].
#[inline]
fn soft_clip(x: f32) -> f32 {
    let x = x.clamp(-1.0, 1.0);
    1.5 * x - 0.5 * x * x * x
}

#[deluge::app]
async fn main(dlg: Deluge) {
    let drive = 2.5;
    dlg.audio()
        .process(move |block: &mut [StereoFrame]| {
            for f in block {
                f.l = soft_clip(f.l * drive);
                f.r = soft_clip(f.r * drive);
            }
        })
        .await
}
