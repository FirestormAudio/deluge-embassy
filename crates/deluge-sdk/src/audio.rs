//! Audio DSP — a per-block callback over the codec.

#[cfg(target_os = "none")]
use core::sync::atomic::{AtomicBool, Ordering};

/// One stereo audio frame; samples in `[-1.0, 1.0]`. `l` = left, `r` = right.
#[cfg(target_os = "none")]
pub use deluge_bsp::audio_block::Frame as StereoFrame;

/// One stereo audio frame; samples in `[-1.0, 1.0]`. `l` = left, `r` = right.
/// Host simulator definition (mirrors `deluge_bsp::audio_block::Frame`).
#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct StereoFrame {
    pub l: f32,
    pub r: f32,
}

#[cfg(target_os = "none")]
fn ensure_init() {
    static DONE: AtomicBool = AtomicBool::new(false);
    if DONE.swap(true, Ordering::Relaxed) {
        return;
    }
    // SAFETY: runs once. Brings up the codec on the direct SSI TX+RX DMA path
    // (no SCUX — that's only for rate conversion); blocks ~5 ms internally.
    // Acquire `audio()` before the main loop. Owns the codec — incompatible with
    // the USB UAC2 device tasks.
    unsafe { deluge_bsp::audio::init() };
}

/// The codec audio path, taken once from [`Deluge::audio`](crate::Deluge::audio).
///
/// Run a DSP callback over every block with [`process`](Audio::process). The
/// block arrives pre-loaded with codec line-in; overwrite it with the output
/// sent to line-out — so the same API serves insert-effects and synths.
///
/// **Owns the codec path.** Do not also run a USB audio (UAC2) device stack; both
/// drive the same SSI/SCUX rings.
pub struct Audio {
    _private: (),
}

impl Audio {
    pub(crate) fn new() -> Self {
        #[cfg(target_os = "none")]
        ensure_init();
        Self { _private: () }
    }

    /// Run `f` over every audio block, forever.
    ///
    /// `f` receives a `BLOCK`-length slice pre-loaded with codec input; whatever
    /// it leaves in the slice is sent to the codec. Never returns.
    ///
    /// ```ignore
    /// dlg.audio().process(|block| {
    ///     for f in block { f.l *= 0.5; f.r *= 0.5; }
    /// }).await
    /// ```
    pub async fn process<F: FnMut(&mut [StereoFrame])>(self, f: F) -> ! {
        crate::plat::audio_run(f).await
    }
}
