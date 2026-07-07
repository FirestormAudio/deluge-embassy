//! Numeric guards + null tests — the reviewed home for the finite/bounded/
//! denormal/agreement checks the suites would otherwise copy-paste.

/// Assert every sample is finite and within `[-max_abs, max_abs]`.
pub fn assert_finite_bounded(buf: &[f32], max_abs: f32) {
    for (i, &x) in buf.iter().enumerate() {
        assert!(x.is_finite(), "sample {i} is not finite: {x}");
        assert!(x.abs() <= max_abs, "sample {i} = {x} exceeds |{max_abs}|");
    }
}

/// Assert no sample is a subnormal (nonzero with magnitude below the smallest
/// normal float) — denormals wreck real-time performance on some FPUs.
pub fn assert_no_denormals(buf: &[f32]) {
    for (i, &x) in buf.iter().enumerate() {
        assert!(
            x == 0.0 || x.abs() >= f32::MIN_POSITIVE,
            "sample {i} = {x} is a subnormal"
        );
    }
}

/// Largest absolute difference between two equal-length buffers.
pub fn max_abs_diff(a: &[f32], b: &[f32]) -> f32 {
    assert_eq!(a.len(), b.len(), "buffers differ in length");
    a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f32::max)
}

/// Assert two buffers agree within `tol` (a null test).
pub fn null(a: &[f32], b: &[f32], tol: f32) {
    let d = max_abs_diff(a, b);
    assert!(d <= tol, "buffers differ by {d} > tol {tol}");
}

/// Root-mean-square level of a buffer.
pub fn rms(buf: &[f32]) -> f32 {
    if buf.is_empty() {
        return 0.0;
    }
    let sum_sq: f32 = buf.iter().map(|x| x * x).sum();
    (sum_sq / buf.len() as f32).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finite_bounded_passes_clean_and_catches_nan() {
        assert_finite_bounded(&[0.0, 0.5, -1.0], 1.0); // clean
        let r = std::panic::catch_unwind(|| assert_finite_bounded(&[0.0, f32::NAN], 1.0));
        assert!(r.is_err(), "NaN must trip the guard");
        let r = std::panic::catch_unwind(|| assert_finite_bounded(&[2.0], 1.0));
        assert!(r.is_err(), "out-of-bounds must trip the guard");
    }

    #[test]
    fn denormals_are_caught() {
        assert_no_denormals(&[0.0, 1.0, -0.5]); // zero + normals OK
        let sub = f32::MIN_POSITIVE / 2.0; // a subnormal
        let r = std::panic::catch_unwind(|| assert_no_denormals(&[sub]));
        assert!(r.is_err(), "subnormal must trip the guard");
    }

    #[test]
    fn diff_null_and_rms() {
        let a = [1.0, -1.0, 1.0, -1.0];
        assert_eq!(max_abs_diff(&a, &a), 0.0);
        null(&a, &a, 0.0);
        assert!((max_abs_diff(&[0.0, 0.0], &[0.1, -0.2]) - 0.2).abs() < 1e-6);
        assert!((rms(&a) - 1.0).abs() < 1e-6);
    }
}
