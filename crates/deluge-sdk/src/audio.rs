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

/// Adapt libdeluge's split input/output buffers to [`Audio::process`]'s in-place
/// block contract.
///
/// libdeluge hands its callback two slices (input, output); the SDK's DSP
/// closure takes **one** slice pre-loaded with input, whose final contents are
/// sent to line-out. So seed `out` with the input and hand `out` to `f`.
///
/// Pure by design — no `libdeluge`, no hardware, no locks — so the Linux
/// backend's only interesting logic is unit-testable on the host. See
/// `plat::linux::audio_run`, its sole caller.
#[cfg(any(feature = "linux", test))]
#[inline]
pub(crate) fn adapt_block<F>(f: &mut F, inp: &[[f32; 2]], out: &mut [[f32; 2]])
where
    F: FnMut(&mut [StereoFrame]),
{
    // libdeluge passes the same period length for both (DELUGE_PERIOD). The
    // `copy_from_slice` below also panics on mismatch in release, which is the
    // correct failure for a broken ABI.
    debug_assert_eq!(inp.len(), out.len());
    out.copy_from_slice(inp);
    // SAFETY: `StereoFrame` is `#[repr(C)] { l: f32, r: f32 }`, layout-identical
    // to `[f32; 2]`. Asserted at compile time in `plat::linux::audio_run` and at
    // run time by `stereoframe_is_layout_compatible_with_f32_pair`.
    f(unsafe { core::mem::transmute::<&mut [[f32; 2]], &mut [StereoFrame]>(out) });
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

#[cfg(test)]
mod tests {
    use super::{adapt_block, StereoFrame};

    /// The closure must observe the *input* buffer, not `out`'s prior contents.
    #[test]
    fn input_reaches_the_closure() {
        let inp = [[1.0, 2.0], [3.0, 4.0]];
        let mut out = [[-9.0, -9.0]; 2];
        let mut seen = Vec::new();
        adapt_block(
            &mut |b: &mut [StereoFrame]| seen.extend(b.iter().map(|f| (f.l, f.r))),
            &inp,
            &mut out,
        );
        assert_eq!(seen, vec![(1.0, 2.0), (3.0, 4.0)]);
    }

    /// Whatever the closure leaves in the block is what libdeluge sends out.
    #[test]
    fn closure_writes_reach_out() {
        let inp = [[1.0, 2.0], [3.0, 4.0]];
        let mut out = [[0.0, 0.0]; 2];
        adapt_block(
            &mut |b: &mut [StereoFrame]| {
                for f in b {
                    f.l *= 10.0;
                    f.r *= 100.0;
                }
            },
            &inp,
            &mut out,
        );
        assert_eq!(out, [[10.0, 200.0], [30.0, 400.0]]);
    }

    /// A no-op closure is bit-exact passthrough — the `audio_passthru` contract.
    #[test]
    fn noop_closure_is_bit_exact_passthrough() {
        let inp = [[0.5, -0.25], [f32::MIN_POSITIVE, -0.0], [1.0, -1.0]];
        let mut out = [[9.9, 9.9]; 3];
        adapt_block(&mut |_: &mut [StereoFrame]| {}, &inp, &mut out);
        assert_eq!(out, inp);
        // -0.0 must survive as -0.0, not collapse to 0.0.
        assert!(out[1][1].is_sign_negative());
    }

    /// Degenerate but legal: no frames, no work, no panic.
    #[test]
    fn empty_block_is_a_noop() {
        let inp: [[f32; 2]; 0] = [];
        let mut out: [[f32; 2]; 0] = [];
        let mut called = false;
        adapt_block(
            &mut |b: &mut [StereoFrame]| {
                called = true;
                assert!(b.is_empty());
            },
            &inp,
            &mut out,
        );
        assert!(called);
    }

    /// The transmute in `adapt_block` is sound only under layout equality.
    #[test]
    fn stereoframe_is_layout_compatible_with_f32_pair() {
        assert_eq!(
            core::mem::size_of::<StereoFrame>(),
            core::mem::size_of::<[f32; 2]>()
        );
        assert_eq!(
            core::mem::align_of::<StereoFrame>(),
            core::mem::align_of::<[f32; 2]>()
        );
    }
}
