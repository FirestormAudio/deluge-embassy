//! SD High-Speed bench: throughput at P1/4 vs P1/2, then a CRC soak.
//!
//! All results go to the RTT log.  Reads are non-destructive (LBA 0 upward);
//! the optional write soak targets a scratch region near the end of the card
//! and is off by default — enable [`WRITE_SOAK`] on dev cards only.

use deluge_bsp::sd;
use embassy_time::Instant;
use log::{error, info, warn};
use rza1l_hal::sdhi;

/// Sectors per read chunk (matches the BSP DMA bounce buffer size).
const CHUNK_SECTORS: u32 = 128; // 64 KiB
/// Data volume per throughput measurement, in MiB.
const BENCH_MIB: u32 = 64;
/// CRC soak passes over the bench region.
const SOAK_PASSES: u32 = 8;
/// Enable the destructive write soak (dev cards only!).
pub const WRITE_SOAK: bool = false;
/// Distance of the write scratch region from the end of the card, in sectors.
const SCRATCH_FROM_END: u32 = 64 * 1024; // 32 MiB clearance

/// Transfer bounce buffer (BSS).  Access is serialised: one bench task.
static mut BUF: [u8; (CHUNK_SECTORS as usize) * 512] = [0; (CHUNK_SECTORS as usize) * 512];

/// FNV-1a 64-bit checksum.
fn checksum(buf: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in buf {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Read `sectors` sectors starting at `start`, returning a combined checksum.
async fn read_region(start: u32, sectors: u32) -> Result<u64, sd::SdError> {
    let mut h: u64 = 0;
    let mut lba = start;
    let end = start + sectors;
    while lba < end {
        let n = (end - lba).min(CHUNK_SECTORS);
        #[allow(static_mut_refs)]
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
        sd::read_sectors(lba, n, &mut buf[..(n as usize) * 512]).await?;
        h ^= checksum(&buf[..(n as usize) * 512]);
        lba += n;
    }
    Ok(h)
}

/// Timed sequential read of [`BENCH_MIB`] MiB from LBA 0.
async fn measure_read(label: &str) -> Result<u64, sd::SdError> {
    let sectors = BENCH_MIB * 2048; // MiB → 512-byte sectors
    let t0 = Instant::now();
    let h = read_region(0, sectors).await?;
    let elapsed_ms = t0.elapsed().as_millis() as u32;
    let kib = sectors / 2;
    info!(
        "bench[{}]: {} KiB in {} ms = {} KiB/s (csum {:016x})",
        label,
        kib,
        elapsed_ms,
        // kib ≤ 65536 → kib*1000 ≤ 6.6e7: no u32 overflow.
        (kib * 1000) / elapsed_ms.max(1),
        h
    );
    Ok(h)
}

#[embassy_executor::task]
pub async fn bench_task() {
    info!("sd-bench: initialising card");
    if let Err(e) = sd::init().await {
        error!("sd-bench: init failed: {:?}", e);
        return;
    }
    info!(
        "sd-bench: card ready — {} sectors, HC={}, HS={}",
        sd::total_sectors(),
        sd::is_hc(),
        sd::is_hs()
    );

    // ---- Throughput: forced default-speed clock (P1/4 = 16.7 MHz) ----
    // Safe regardless of card mode: a High-Speed card also runs at 16.7 MHz.
    // SDHI port 1 = Deluge SD slot
    unsafe { sdhi::set_clock_div(1, sdhi::CLK_DIV_4) };
    if let Err(e) = measure_read("P1/4 16.7MHz").await {
        error!("bench[P1/4 16.7MHz]: FAILED: {:?}", e);
    }

    // ---- Throughput: high-speed clock (P1/2 = 33.3 MHz) ----
    if sd::is_hs() {
        // SDHI port 1 = Deluge SD slot
        unsafe { sdhi::set_clock_div(1, sdhi::CLK_DIV_2) };
        if let Err(e) = measure_read("P1/2 33.3MHz").await {
            error!("bench[P1/2 33.3MHz]: FAILED: {:?}", e);
        }
    } else {
        warn!("sd-bench: card not in High-Speed mode; skipping 33.3 MHz run");
    }

    // ---- CRC soak at the final configured clock ----
    let sectors = BENCH_MIB * 2048;
    let mut reference: Option<u64> = None;
    let mut failures = 0u32;
    for pass in 1..=SOAK_PASSES {
        match read_region(0, sectors).await {
            Ok(h) => match reference {
                None => {
                    reference = Some(h);
                    info!(
                        "soak: pass {}/{}: csum {:016x} (reference)",
                        pass, SOAK_PASSES, h
                    );
                }
                Some(r) if r == h => info!("soak: pass {}/{}: OK", pass, SOAK_PASSES),
                Some(r) => {
                    failures += 1;
                    error!(
                        "soak: pass {}/{}: MISMATCH {:016x} != {:016x}",
                        pass, SOAK_PASSES, h, r
                    );
                }
            },
            Err(e) => {
                failures += 1;
                error!("soak: pass {}/{}: read error {:?}", pass, SOAK_PASSES, e);
            }
        }
    }
    if failures == 0 {
        info!("soak: PASS — {} passes, 0 errors", SOAK_PASSES);
    } else {
        error!(
            "soak: FAIL — {} error(s) in {} passes",
            failures, SOAK_PASSES
        );
    }

    if WRITE_SOAK {
        write_soak().await;
    }

    info!("sd-bench: done");
}

/// Destructive write/read-back verify against a scratch region near the end
/// of the card.  Guarded by [`WRITE_SOAK`].
async fn write_soak() {
    let total = sd::total_sectors();
    if total < SCRATCH_FROM_END + CHUNK_SECTORS {
        error!("write soak: card too small ({} sectors)", total);
        return;
    }
    let base = total - SCRATCH_FROM_END;
    info!(
        "write soak: scratch @ LBA {} ({} sectors) — DESTRUCTIVE",
        base, CHUNK_SECTORS
    );

    #[allow(static_mut_refs)]
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
    // Fill with an LCG pattern so stale data can't false-pass.
    let mut x: u32 = 0x1234_5678;
    for b in buf.iter_mut() {
        x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *b = (x >> 24) as u8;
    }
    let expect = checksum(&buf[..]);

    if let Err(e) = sd::write_sectors(base, CHUNK_SECTORS, buf).await {
        error!("write soak: write error {:?}", e);
        return;
    }
    buf.fill(0);
    match sd::read_sectors(base, CHUNK_SECTORS, buf).await {
        Ok(()) => {
            let got = checksum(&buf[..]);
            if got == expect {
                info!("write soak: verify OK ({:016x})", got);
            } else {
                error!("write soak: MISMATCH {:016x} != {:016x}", got, expect);
            }
        }
        Err(e) => error!("write soak: read-back error {:?}", e),
    }
}
