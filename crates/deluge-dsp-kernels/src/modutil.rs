//! Modulation utilities: sample & hold, one-pole slew (glide), and an inline
//! step sequencer. Tiny per-sample mono kernels — no heap, no buffer.

use crate::In;

/// Rising-edge detector: the clock crossed 0 upward.
#[inline]
fn rising(prev: f32, cur: f32) -> bool {
    prev <= 0.0 && cur > 0.0
}

/// Sample & hold: latch `input` on each rising clock edge. Ports 0=input, 1=clock.
#[derive(Clone, Copy)]
pub struct SampleHold {
    held: f32,
    prev_clock: f32,
}
impl SampleHold {
    pub fn new() -> SampleHold {
        SampleHold { held: 0.0, prev_clock: 0.0 }
    }
    pub fn process(&mut self, input: In, clock: In, out: &mut [f32]) {
        for i in 0..out.len() {
            let c = clock.at(i);
            if rising(self.prev_clock, c) {
                self.held = input.at(i);
            }
            out[i] = self.held;
            self.prev_clock = c;
        }
    }
}
impl Default for SampleHold {
    fn default() -> Self {
        Self::new()
    }
}

/// One-pole slew / lag (glide). Ports 0=input, 1=time (seconds).
#[derive(Clone, Copy)]
pub struct Slew {
    z: f32,
}
impl Slew {
    pub fn new() -> Slew {
        Slew { z: 0.0 }
    }
    pub fn process(&mut self, input: In, time: In, dt: f32, out: &mut [f32]) {
        for i in 0..out.len() {
            let c = (dt / time.at(i).max(dt)).min(1.0);
            self.z += (input.at(i) - self.z) * c;
            out[i] = self.z;
        }
    }
}
impl Default for Slew {
    fn default() -> Self {
        Self::new()
    }
}

/// Step sequencer over an inline value array, clocked by port 0.
pub const MAX_STEPS: usize = 16;
#[derive(Clone, Copy)]
pub struct Steps {
    values: [f32; MAX_STEPS],
    len: usize,
    idx: usize,
    prev_clock: f32,
    first: bool,
}
impl Steps {
    pub fn new() -> Steps {
        Steps { values: [0.0; MAX_STEPS], len: 1, idx: 0, prev_clock: 0.0, first: true }
    }
    pub fn set_len(&mut self, n: u8) {
        self.len = (n as usize).clamp(1, MAX_STEPS);
    }
    /// `i` is the 1-based param index (`set_param` k → `values[k−1]`).
    pub fn set_value(&mut self, i: usize, v: f32) {
        if (1..=MAX_STEPS).contains(&i) {
            self.values[i - 1] = v;
        }
    }
    pub fn process(&mut self, clock: In, out: &mut [f32]) {
        for i in 0..out.len() {
            let c = clock.at(i);
            if rising(self.prev_clock, c) {
                if self.first {
                    self.first = false;
                } else {
                    self.idx = (self.idx + 1) % self.len.max(1);
                }
            }
            out[i] = self.values[self.idx.min(MAX_STEPS - 1)];
            self.prev_clock = c;
        }
    }
}
impl Default for Steps {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::In;

    // A clock that is −1 everywhere except +1 on the given sample indices' runs.
    fn clock_with_edges(n: usize, edges: &[usize]) -> std::vec::Vec<f32> {
        let mut c = std::vec![-1.0f32; n];
        // Hold +1 from each edge for 3 samples so the rising transition is clean.
        for &e in edges {
            for s in e..(e + 3).min(n) {
                c[s] = 1.0;
            }
        }
        c
    }

    #[test]
    fn sample_hold_latches_and_holds() {
        let n = 40;
        let input: std::vec::Vec<f32> = (0..n).map(|i| i as f32 * 0.01).collect();
        let clock = clock_with_edges(n, &[10, 20]);
        let mut sh = SampleHold::new();
        let mut out = std::vec![0.0f32; n];
        sh.process(In::A(&input), In::A(&clock), &mut out);
        assert!(out[5].abs() < 1e-6, "before any edge: initial held 0");
        assert!((out[15] - 0.10).abs() < 1e-6, "latched input at edge 10 (0.10)");
        assert!((out[19] - 0.10).abs() < 1e-6, "holds between edges");
        assert!((out[25] - 0.20).abs() < 1e-6, "latched input at edge 20 (0.20)");
    }

    #[test]
    fn slew_converges_and_passes_dc() {
        let dt = 1.0 / 48_000.0;
        let n = 2_000;
        let input = std::vec![1.0f32; n]; // step 0→1
        let time = std::vec![0.005f32; n]; // 5 ms
        let mut s = Slew::new();
        let mut out = std::vec![0.0f32; n];
        s.process(In::A(&input), In::A(&time), dt, &mut out);
        assert!(out[0] > 0.0 && out[0] < 0.1, "starts rising from 0: {}", out[0]);
        assert!(out[1] > out[0], "monotonic rise");
        assert!(out[n - 1] > 0.99, "converges to the step: {}", out[n - 1]);
    }

    #[test]
    fn steps_walks_values_wrapping() {
        let n = 50;
        let clock = clock_with_edges(n, &[10, 20, 30, 40]);
        let mut st = Steps::new();
        st.set_len(3);
        st.set_value(1, 10.0);
        st.set_value(2, 20.0);
        st.set_value(3, 30.0);
        let mut out = std::vec![0.0f32; n];
        st.process(In::A(&clock), &mut out);
        assert!((out[5] - 10.0).abs() < 1e-6, "step 0 before any advance");
        assert!((out[15] - 10.0).abs() < 1e-6, "first edge keeps step 0");
        assert!((out[25] - 20.0).abs() < 1e-6, "2nd edge → step 1");
        assert!((out[35] - 30.0).abs() < 1e-6, "3rd edge → step 2");
        assert!((out[45] - 10.0).abs() < 1e-6, "4th edge → wrap to step 0");
    }

    use proptest::prelude::*;
    proptest! {
        #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]
        #[test]
        fn slew_stays_bounded(amp in -1.0f32..1.0, time_ms in 0.1f32..100.0) {
            let dt = 1.0 / 48_000.0;
            let input = std::vec![amp; 2000];
            let time = std::vec![time_ms * 1e-3; 2000];
            let mut s = Slew::new();
            let mut out = std::vec![0.0f32; 2000];
            s.process(In::A(&input), In::A(&time), dt, &mut out);
            for &v in &out {
                prop_assert!(v.is_finite() && v.abs() <= 1.0001, "slew unbounded: {v}");
            }
        }
    }
}
