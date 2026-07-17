//! Persistent buffer pool for state that outlives a block (delay lines, reverb,
//! sample data). Fixed `[f32; CAP]` arena carved into `CHUNK`-sized chunks with a
//! bitmap free-list; allocations round up to whole chunks. Distinct from the
//! per-block output arena. Exhaustion returns `None` (caller degrades) — no panic.

/// A claim on a contiguous region of the pool. Returned to the pool on `free`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PoolHandle {
    off: u32, // start index in f32 units
    len: u32, // requested length in f32 units
}

pub struct Pool<const CAP: usize, const CHUNK: usize> {
    buf: [f32; CAP],
    // `used[c]` marks chunk c as allocated. CAP/CHUNK chunks.
    used: [bool; CAP],
}

impl<const CAP: usize, const CHUNK: usize> Pool<CAP, CHUNK> {
    pub fn new() -> Self {
        Pool {
            buf: [0.0; CAP],
            used: [false; CAP],
        }
    }

    #[inline]
    fn chunk_count() -> usize {
        CAP / CHUNK
    }

    /// Allocate `len` f32s as a contiguous run of chunks. First-fit.
    pub fn alloc(&mut self, len: usize) -> Option<PoolHandle> {
        if len == 0 {
            return None;
        }
        let need = (len + CHUNK - 1) / CHUNK; // chunks
        let total = Self::chunk_count();
        let mut start = 0;
        while start + need <= total {
            if (start..start + need).all(|c| !self.used[c]) {
                for c in start..start + need {
                    self.used[c] = true;
                }
                return Some(PoolHandle {
                    off: (start * CHUNK) as u32,
                    len: len as u32,
                });
            }
            start += 1;
        }
        None
    }

    pub fn free(&mut self, h: PoolHandle) {
        let need = ((h.len as usize) + CHUNK - 1) / CHUNK;
        let start = (h.off as usize) / CHUNK;
        for c in start..start + need {
            self.used[c] = false;
        }
    }

    pub fn slice(&self, h: PoolHandle) -> &[f32] {
        let o = h.off as usize;
        &self.buf[o..o + h.len as usize]
    }

    pub fn slice_mut(&mut self, h: PoolHandle) -> &mut [f32] {
        let o = h.off as usize;
        &mut self.buf[o..o + h.len as usize]
    }
}

impl<const CAP: usize, const CHUNK: usize> Default for Pool<CAP, CHUNK> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 64 f32 capacity, 8-f32 chunks → 8 chunks.
    type P = Pool<64, 8>;

    #[test]
    fn alloc_returns_usable_distinct_regions() {
        let mut p = P::new();
        let a = p.alloc(8).unwrap();
        let b = p.alloc(8).unwrap();
        p.slice_mut(a).fill(1.0);
        p.slice_mut(b).fill(2.0);
        assert!(p.slice(a).iter().all(|x| *x == 1.0));
        assert!(p.slice(b).iter().all(|x| *x == 2.0));
        assert_eq!(p.slice(a).len(), 8);
    }

    #[test]
    fn exhaustion_returns_none_not_panic() {
        let mut p = P::new();
        let mut hs = heapless_like();
        for _ in 0..8 {
            hs.push(p.alloc(8).expect("should fit"));
        }
        assert!(p.alloc(1).is_none()); // full
    }

    #[test]
    fn free_reclaims_for_reuse() {
        let mut p = P::new();
        let a = p.alloc(16).unwrap(); // 2 chunks
        p.free(a);
        // All chunks free again → a 64-wide alloc (8 chunks) now succeeds.
        assert!(p.alloc(64).is_some());
    }

    // Tiny fixed vec so the test needs no std collections.
    fn heapless_like() -> TestVec {
        TestVec {
            n: 0,
            items: [PoolHandle { off: 0, len: 0 }; 8],
        }
    }
    struct TestVec {
        n: usize,
        items: [PoolHandle; 8],
    }
    impl TestVec {
        fn push(&mut self, h: PoolHandle) {
            self.items[self.n] = h;
            self.n += 1;
        }
    }
}
