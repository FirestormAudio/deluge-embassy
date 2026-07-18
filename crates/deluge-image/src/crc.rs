//! CRC-32 (IEEE 802.3) — the single shared implementation both ends of the
//! dev-upload protocol and the on-flash settings record agree on.
//!
//! Table-free, reflected, `0xEDB88320` polynomial, `0xFFFFFFFF` init and final
//! XOR — i.e. the same CRC `gzip`/`zlib`/`PNG` use, so the host tool can compute
//! it with any off-the-shelf routine and get the same value.  Kept tiny and
//! `const`-friendly so it lives in this `no_std`, host-testable crate and is
//! reused verbatim on the device (settings record + USB upload framing).

/// Incremental CRC-32 (IEEE) accumulator, so a stream can be checksummed without
/// buffering it: `Crc32::new().update(a).update(b).finalize()` equals
/// `crc32(a followed by b)`.
#[derive(Clone, Copy)]
pub struct Crc32(u32);

impl Crc32 {
    /// Start a fresh CRC accumulation.
    pub fn new() -> Self {
        Crc32(0xFFFF_FFFF)
    }

    /// Fold `data` into the running CRC.
    pub fn update(&mut self, data: &[u8]) {
        let mut crc = self.0;
        for &byte in data {
            crc ^= byte as u32;
            let mut bit = 0;
            while bit < 8 {
                // Branch-free: `mask` is all-ones when the low bit is set.
                let mask = (crc & 1).wrapping_neg();
                crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
                bit += 1;
            }
        }
        self.0 = crc;
    }

    /// Finish and return the CRC-32 value (does not consume the accumulator).
    pub fn finalize(&self) -> u32 {
        !self.0
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute the CRC-32 (IEEE) of `data`.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(data);
    c.finalize()
}

#[cfg(test)]
mod tests {
    use super::crc32;

    #[test]
    fn known_vectors() {
        // Standard CRC-32/ISO-HDLC check values (cross-checked against zlib).
        assert_eq!(crc32(b""), 0x0000_0000);
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(
            crc32(b"The quick brown fox jumps over the lazy dog"),
            0x414F_A339
        );
    }

    #[test]
    fn single_byte_difference_changes_crc() {
        assert_ne!(crc32(b"DLUP\x01"), crc32(b"DLUP\x00"));
    }

    #[test]
    fn streaming_matches_one_shot() {
        use super::Crc32;
        let data = b"The quick brown fox jumps over the lazy dog";
        let mut c = Crc32::new();
        c.update(&data[..10]);
        c.update(&data[10..20]);
        c.update(&data[20..]);
        assert_eq!(c.finalize(), crc32(data));
        assert_eq!(c.finalize(), 0x414F_A339);
        assert_eq!(Crc32::new().finalize(), 0x0000_0000);
    }
}
