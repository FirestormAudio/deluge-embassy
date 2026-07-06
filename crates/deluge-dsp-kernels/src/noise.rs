//! White noise via xorshift32 → [-1, 1). Serial recurrence → scalar.

/// xorshift32 noise generator. Seed with a nonzero value.
#[derive(Clone, Copy)]
pub struct Noise {
    rng: u32,
}

impl Noise {
    pub fn seeded(seed: u32) -> Noise {
        Noise {
            rng: if seed == 0 { 0x2545_F491 } else { seed },
        }
    }

    pub fn process(&mut self, out: &mut [f32]) {
        for s in out.iter_mut() {
            let mut r = self.rng;
            r ^= r << 13;
            r ^= r >> 17;
            r ^= r << 5;
            self.rng = r;
            *s = (r as i32 as f32) / (i32::MAX as f32);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noise_is_bounded_and_deterministic_per_seed() {
        let mut a = Noise::seeded(0x2545_F491);
        let mut b = Noise::seeded(0x2545_F491);
        let mut oa = [0.0f32; 32];
        let mut ob = [0.0f32; 32];
        a.process(&mut oa);
        b.process(&mut ob);
        assert_eq!(oa, ob); // same seed → same stream
        assert!(oa.iter().all(|s| s.abs() <= 1.0));
        assert!(oa.iter().any(|s| *s != oa[0])); // not a constant
    }
}
