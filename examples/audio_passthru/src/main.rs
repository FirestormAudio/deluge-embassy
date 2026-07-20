//! Deluge SDK example: a line-in audio effect.
//!
//! Runs a per-block DSP callback over the codec: each block arrives pre-loaded
//! with line-in, and we overwrite it with gain + a cubic soft-clip (a stateless,
//! audibly-obvious saturator). **Requires a signal on the codec line-in** — with
//! no input it outputs near-silence (just anti-auto-mute dither).
//!
//! For a clean passthrough, use an identity callback: `.process(|_| {})`.

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
