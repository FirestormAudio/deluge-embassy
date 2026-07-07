//! CPU-cost harness. Host wall-clock is noisy, so this is for *comparison and
//! regression*, never an absolute gate: median-of-N per-block timing, plus a
//! ratio helper for A/B (e.g. band-limited vs naïve). The absolute per-block
//! budget gate lives on-device (deferred — see the QA spec §5).

use std::time::Instant;

/// Result of a timing run.
pub struct CostReport {
    /// Median wall-clock nanoseconds for one `render_block` call.
    pub ns_per_block: f64,
    /// Cost relative to real time: `ns_per_block / (block_len/sample_rate in ns)`.
    /// < 1.0 means faster than real time for that block.
    pub times_realtime: f64,
}

/// Warm up, then time `iters` calls of `render_block` and take the median.
pub fn measure(
    sample_rate: f32,
    block_len: usize,
    iters: usize,
    mut render_block: impl FnMut(),
) -> CostReport {
    assert!(iters > 0, "iters must be > 0");
    // Warmup (fill caches / let the CPU ramp) — not measured.
    for _ in 0..(iters / 10).max(1) {
        render_block();
    }
    let mut samples = Vec::with_capacity(iters);
    for _ in 0..iters {
        let t = Instant::now();
        render_block();
        samples.push(t.elapsed().as_nanos() as f64);
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let ns_per_block = samples[samples.len() / 2];

    let block_ns = (block_len as f64 / sample_rate as f64) * 1e9;
    CostReport {
        ns_per_block,
        times_realtime: ns_per_block / block_ns,
    }
}

/// Ratio of two reports' per-block cost. `> 1.0` means `a` is costlier than `b`.
pub fn compare(a: &CostReport, b: &CostReport) -> f64 {
    a.ns_per_block / b.ns_per_block.max(f64::MIN_POSITIVE)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::hint::black_box;

    fn work(iters: usize) {
        let mut acc = 0.0f32;
        for i in 0..iters {
            acc += (i as f32).sin();
        }
        black_box(acc);
    }

    #[test]
    fn measure_reports_positive_and_realtime_ratio() {
        let r = measure(48_000.0, 32, 200, || work(1_000));
        assert!(r.ns_per_block > 0.0);
        assert!(r.times_realtime > 0.0);
    }

    #[test]
    fn compare_tracks_relative_workload() {
        let a = measure(48_000.0, 32, 200, || work(2_000));
        let b = measure(48_000.0, 32, 200, || work(1_000));
        let ratio = compare(&a, &b);
        // Host timing is noisy — assert only that 2× work costs roughly 2×.
        assert!(ratio > 1.4 && ratio < 3.0, "ratio ~2, got {ratio}");
    }
}
