//! Separate heap allocators for on-chip SRAM and external SDRAM.
//!
//! Two [`CsHeap`] instances – [`SRAM`] and [`SDRAM`] – provide distinct
//! allocation arenas, each protected by a critical-section lock (IRQ-safe).
//!
//! ## Algorithm
//! Each arena is a [TLSF (Two-Level Segregated Fit)][1] allocator from the
//! [`rlsf`] crate.  TLSF gives **O(1) allocation *and* deallocation** with a
//! bounded worst case — important here because every alloc/dealloc runs with
//! interrupts disabled (see *Locking*), so the time spent inside the allocator
//! is directly added to interrupt latency.  Its good-fit segregation also keeps
//! fragmentation low over a long session, which matters for the 64 MB SDRAM
//! arena that churns audio/sample buffers.
//!
//! [1]: http://www.gii.upv.es/tlsf/
//!
//! ## Usage
//!
//! 1. Call [`CsHeap::init`] once per allocator during startup, before any
//!    allocation in that arena.
//! 2. Pass a reference to the allocator where a nightly `Allocator` is
//!    expected:
//!
//! ```rust,ignore
//! #![feature(allocator_api)]
//! use deluge_alloc::{SRAM, SDRAM};
//!
//! let buf: Box<[u8; 4096], _> = Box::new_in([0u8; 4096], &SDRAM);
//! let vec: Vec<u32, _>        = Vec::new_in(&SRAM);
//! ```
//!
//! ## Locking
//! Both allocators use [`critical_section::with`], which disables interrupts
//! for the duration of each allocation or deallocation.  This prevents
//! deadlocks caused by IRQ handlers that also allocate, at the cost of a
//! brief IRQ-latency bump.

#![cfg_attr(target_os = "none", no_std)]
#![feature(allocator_api)]

use core::alloc::{AllocError, Allocator, Layout};
use core::cell::UnsafeCell;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicUsize, Ordering};

use rlsf::Tlsf;

// ---------------------------------------------------------------------------
// TLSF configuration
// ---------------------------------------------------------------------------
//
// The `FLLEN`/`SLLEN` const parameters trade metadata size against the maximum
// pool size and the worst-case internal fragmentation:
//
//   * max pool size      = GRANULARITY << FLLEN
//   * worst-case waste   ≈ allocation_size / SLLEN
//
// where `GRANULARITY = size_of::<usize>() * 4` (16 bytes on the 32-bit device).
// Both arenas share one config sized for the larger SDRAM arena; the extra
// metadata for the small SRAM arena is a couple of KiB, negligible against the
// 3 MB of on-chip RAM.
//
//   * FLLEN = 23  →  max pool 16 << 23 = 128 MiB  (covers the 64 MiB SDRAM)
//   * SLLEN = 16  →  ≤ ~6 % worst-case internal fragmentation
//
// `FLBitmap` must hold ≥ FLLEN bits and `SLBitmap` ≥ SLLEN bits; `SLLEN` must be
// a power of two. Bump `SLLEN` to 32 (and `SlBitmap` to `u32`) to halve the
// worst-case fragmentation at the cost of doubling the per-arena metadata.
type FlBitmap = u32;
type SlBitmap = u16;
const FLLEN: usize = 23;
const SLLEN: usize = 16;

type Arena = Tlsf<'static, FlBitmap, SlBitmap, FLLEN, SLLEN>;

// ---------------------------------------------------------------------------
// CsHeap — critical-section-protected heap
// ---------------------------------------------------------------------------

/// Mutable state behind the critical-section lock.
struct HeapState {
    tlsf: Arena,
    /// Total bytes the arena manages (as accepted by the TLSF pool).
    size: usize,
    /// Sum of the `Layout::size()` of all live allocations.
    used: usize,
}

/// A heap allocator guarded by a `critical_section` lock.
///
/// Implements the nightly [`Allocator`] trait so it can be used with
/// `Box::new_in`, `Vec::new_in`, and similar APIs.
///
/// # Safety invariant
/// [`init`][Self::init] must be called exactly once before any allocation.
pub struct CsHeap {
    state: UnsafeCell<HeapState>,
    /// Base of the region passed to [`init`][Self::init], for [`contains`].
    /// Write-once at init, then read-only — so it lives outside the lock and is
    /// read without taking a critical section (see [`contains`][Self::contains]).
    region_start: AtomicUsize,
    /// Size of the region passed to [`init`][Self::init] (the raw request, not
    /// the post-trim TLSF [`size`][HeapState::size]). Write-once at init.
    region_size: AtomicUsize,
}

// SAFETY: All accesses go through `critical_section::with`, which disables
// IRQs on single-core targets, providing the required mutual exclusion. The
// `region_*` atomics are write-once at init and otherwise read-only.
unsafe impl Sync for CsHeap {}
unsafe impl Send for CsHeap {}

impl CsHeap {
    /// Creates a new, uninitialised heap.  Must be initialised with
    /// [`init`][Self::init] before any allocation attempt.
    pub const fn empty() -> Self {
        Self {
            state: UnsafeCell::new(HeapState {
                tlsf: Tlsf::new(),
                size: 0,
                used: 0,
            }),
            region_start: AtomicUsize::new(0),
            region_size: AtomicUsize::new(0),
        }
    }

    /// Initialises the heap to cover `start..start+size`.
    ///
    /// # Safety
    /// - `start..start+size` must be a valid, exclusively-owned, writable
    ///   memory region for the lifetime of the program.
    /// - Must be called exactly once per instance, before the first
    ///   allocation.
    pub unsafe fn init(&self, start: *mut u8, size: usize) {
        // Record the raw region bounds for `contains` before TLSF trims them.
        self.region_start.store(start as usize, Ordering::Relaxed);
        self.region_size.store(size, Ordering::Relaxed);
        critical_section::with(|_| {
            // SAFETY: exclusive via critical section; caller upholds the rest.
            let state = unsafe { &mut *self.state.get() };
            // SAFETY: caller guarantees `start` is non-null and the region is
            // valid, exclusively owned, and lives for the whole program — so a
            // `'static` block is sound.
            let block =
                NonNull::slice_from_raw_parts(unsafe { NonNull::new_unchecked(start) }, size);
            // SAFETY: the block is exclusively owned for `'static` per the above.
            let accepted = unsafe { state.tlsf.insert_free_block_ptr(block) };
            state.size = accepted.map_or(0, |n| n.get());
        });
    }

    /// Returns `true` if `ptr` points within the region this heap manages.
    ///
    /// Used to route a pointer back to the arena it came from (e.g. by
    /// [`Spill`]). Lock-free: reads the write-once region bounds, so it is safe
    /// to call concurrently with allocation/deallocation.
    pub fn contains(&self, ptr: NonNull<u8>) -> bool {
        let start = self.region_start.load(Ordering::Relaxed);
        let size = self.region_size.load(Ordering::Relaxed);
        let addr = ptr.as_ptr() as usize;
        addr >= start && addr < start + size
    }

    /// Returns the number of bytes currently in use (sum of live allocation
    /// sizes). Useful as a high-water diagnostic.
    pub fn used(&self) -> usize {
        critical_section::with(|_| unsafe { (*self.state.get()).used })
    }

    /// Returns the total number of bytes managed by this allocator.
    pub fn size(&self) -> usize {
        critical_section::with(|_| unsafe { (*self.state.get()).size })
    }

    /// Returns the number of free bytes remaining (`size - used`).
    ///
    /// Note: this is the *accounting* free total, not the largest allocatable
    /// block — it does not reflect fragmentation.
    pub fn free(&self) -> usize {
        critical_section::with(|_| unsafe {
            let s = &*self.state.get();
            s.size - s.used
        })
    }
}

unsafe impl Allocator for CsHeap {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        critical_section::with(|_| {
            // SAFETY: exclusive via critical section.
            let state = unsafe { &mut *self.state.get() };
            match state.tlsf.allocate(layout) {
                Some(ptr) => {
                    state.used += layout.size();
                    Ok(NonNull::slice_from_raw_parts(ptr, layout.size()))
                }
                None => Err(AllocError),
            }
        })
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        critical_section::with(|_| {
            // SAFETY: exclusive via critical section.
            let state = unsafe { &mut *self.state.get() };
            // SAFETY: caller guarantees `ptr` came from this allocator with a
            // matching layout, so `layout.align()` is the original alignment.
            unsafe { state.tlsf.deallocate(ptr, layout.align()) };
            state.used = state.used.saturating_sub(layout.size());
        });
    }
}

// ---------------------------------------------------------------------------
// Spill — tiered (fast-first, fall back to slow) allocator
// ---------------------------------------------------------------------------

/// A two-tier allocator that serves from a fast `primary` arena until it is
/// full, then spills to a larger `fallback` arena.
///
/// Built for the Deluge's split memory: allocate from on-chip [`SRAM`] first
/// (small, fast) and fall back to external [`SDRAM`] (large, slow), so hot/young
/// allocations land in fast RAM. Deallocation routes each pointer back to the
/// arena it came from by address ([`CsHeap::contains`]), which works because the
/// two arenas occupy disjoint, non-overlapping address ranges.
///
/// Both tiers must be [`init`][CsHeap::init]ialised before first use. Implements
/// the nightly [`Allocator`] trait; the default `grow`/`shrink` (allocate-copy-
/// free through this allocator) are correct, so they need no override.
///
/// ```rust,ignore
/// use deluge_alloc::{Spill, SRAM, SDRAM};
/// static HEAP: Spill = Spill::new(&SRAM, &SDRAM);
/// ```
///
/// Note: there is no budget cap — the primary fills completely before any spill.
/// A `with_budget` variant could be added if the primary gains other consumers
/// that need reserved headroom.
pub struct Spill {
    primary: &'static CsHeap,
    fallback: &'static CsHeap,
}

impl Spill {
    /// Creates a tiered allocator that prefers `primary` and spills to
    /// `fallback`. Both arenas must be `init`ialised before allocating.
    pub const fn new(primary: &'static CsHeap, fallback: &'static CsHeap) -> Self {
        Self { primary, fallback }
    }
}

unsafe impl Allocator for Spill {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        match self.primary.allocate(layout) {
            Ok(p) => Ok(p),
            Err(_) => self.fallback.allocate(layout),
        }
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        // Route by address: the arenas occupy disjoint ranges, so the pointer
        // tells us which one it came from.
        if self.primary.contains(ptr) {
            unsafe { self.primary.deallocate(ptr, layout) };
        } else {
            unsafe { self.fallback.deallocate(ptr, layout) };
        }
    }
}

// SAFETY: both fields are `&'static CsHeap`, which is `Sync`; `Spill` adds no
// interior mutability of its own.
unsafe impl Sync for Spill {}
unsafe impl Send for Spill {}

// ---------------------------------------------------------------------------
// Public allocator instances
// ---------------------------------------------------------------------------

/// Allocator backed by on-chip SRAM.
///
/// Must be initialised with
/// `unsafe { SRAM.init(heap_start, heap_size) }` before first use.
/// The canonical heap window is from the end of the firmware image
/// (`__sram_heap_start`) to just below the RTT/stack reservation
/// (`__sram_heap_end`).
pub static SRAM: CsHeap = CsHeap::empty();

/// Allocator backed by the external 64 MB SDRAM (CS3, 0x0C00_0000–0x0FFF_FFFF).
///
/// Must be initialised *after* the SDRAM controller has been brought up and the
/// SDRAM window is accessible:
/// ```rust,ignore
/// unsafe { SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) }
/// ```
///
/// **Calling `allocate` before `init` will return `AllocError`** (which causes
/// `Box::new_in` / `Vec::try_reserve` to panic). There is no silent UB, but
/// any allocation attempt before `init` will fail at runtime.
pub static SDRAM: CsHeap = CsHeap::empty();

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;
    use core::alloc::Layout;

    #[test]
    fn empty_heap_reports_zero() {
        let h = CsHeap::empty();
        assert_eq!(h.size(), 0);
        assert_eq!(h.used(), 0);
        assert_eq!(h.free(), 0);
    }

    #[test]
    fn init_then_allocate_and_free_tracks_usage() {
        // Back the arena with an owned, sufficiently-aligned buffer.
        let mut buf = vec![0u8; 8192];
        let start = buf.as_mut_ptr();
        let h = CsHeap::empty();
        unsafe { h.init(start, buf.len()) };

        // TLSF may trim a few bytes for alignment / its sentinel, so the
        // managed size is close to but not necessarily equal to the request.
        let size = h.size();
        assert!(size > 0 && size <= 8192, "managed size within the arena");
        assert_eq!(h.used(), 0);
        assert_eq!(h.free(), size);

        let layout = Layout::from_size_align(256, 8).unwrap();
        let p = h.allocate(layout).expect("allocation within an 8 KB arena");
        assert_eq!(h.used(), 256, "used grows by the requested size");
        assert_eq!(h.free(), size - 256);

        // Returned region is inside the arena.
        let addr = p.as_ptr() as *mut u8 as usize;
        let base = start as usize;
        assert!(addr >= base && addr < base + 8192);

        unsafe { h.deallocate(p.cast::<u8>(), layout) };
        assert_eq!(h.used(), 0, "freeing returns the arena to empty");
    }

    #[test]
    fn allocation_failure_when_arena_exhausted() {
        let mut buf = vec![0u8; 1024];
        let h = CsHeap::empty();
        unsafe { h.init(buf.as_mut_ptr(), buf.len()) };
        // Requesting far more than the arena holds must fail, not panic/UB.
        let too_big = Layout::from_size_align(64 * 1024, 8).unwrap();
        assert!(h.allocate(too_big).is_err());
    }

    #[test]
    fn contains_reports_region_membership() {
        let mut buf = vec![0u8; 8192];
        let start = buf.as_mut_ptr();
        let h = CsHeap::empty();
        unsafe { h.init(start, buf.len()) };

        let layout = Layout::from_size_align(64, 8).unwrap();
        let p = h.allocate(layout).expect("alloc").cast::<u8>();
        assert!(h.contains(p), "an allocation from this heap is contained");

        // A clearly-outside pointer is not contained.
        let outside = NonNull::new((start as usize).wrapping_sub(0x1000) as *mut u8).unwrap();
        assert!(!h.contains(outside));
        unsafe { h.deallocate(p, layout) };
    }

    // `Spill::new` takes `&'static CsHeap`, so back the test arenas with leaked
    // buffers and leaked heaps to obtain `'static` references.
    fn leak_heap(bytes: usize) -> &'static CsHeap {
        let buf = vec![0u8; bytes].leak();
        let h: &'static CsHeap = Box::leak(Box::new(CsHeap::empty()));
        unsafe { h.init(buf.as_mut_ptr(), buf.len()) };
        h
    }

    #[test]
    fn spill_fills_primary_then_falls_back() {
        // Tiny primary, roomy fallback.
        let primary = leak_heap(1024);
        let fallback = leak_heap(64 * 1024);
        let spill = Spill::new(primary, fallback);

        // First allocation fits the primary.
        let small = Layout::from_size_align(256, 8).unwrap();
        let a = spill.allocate(small).expect("primary alloc").cast::<u8>();
        assert!(primary.contains(a), "young allocation lands in the primary");
        assert_eq!(
            fallback.used(),
            0,
            "fallback untouched while primary has room"
        );

        // A request the primary can't satisfy spills to the fallback.
        let big = Layout::from_size_align(16 * 1024, 8).unwrap();
        let b = spill.allocate(big).expect("spill to fallback").cast::<u8>();
        assert!(
            fallback.contains(b),
            "oversized allocation spills to the fallback"
        );
        assert!(!primary.contains(b));

        // Each frees back to the arena it came from.
        unsafe { spill.deallocate(a, small) };
        unsafe { spill.deallocate(b, big) };
        assert_eq!(primary.used(), 0, "primary block freed to primary");
        assert_eq!(fallback.used(), 0, "fallback block freed to fallback");
    }
}
