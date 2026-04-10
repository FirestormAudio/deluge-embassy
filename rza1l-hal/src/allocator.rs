//! Separate heap allocators for on-chip SRAM and external SDRAM.
//!
//! Two [`CsHeap`] instances – [`SRAM`] and [`SDRAM`] – provide distinct
//! allocation arenas, each protected by a critical-section lock (IRQ-safe).
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
//! use rza1::allocator::{SRAM, SDRAM};
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

use core::alloc::{AllocError, Allocator, Layout};
use core::cell::UnsafeCell;
use core::ptr::NonNull;

use linked_list_allocator::Heap;

// ---------------------------------------------------------------------------
// CsHeap — critical-section-protected heap
// ---------------------------------------------------------------------------

/// A heap allocator guarded by a `critical_section` lock.
///
/// Implements the nightly [`Allocator`] trait so it can be used with
/// `Box::new_in`, `Vec::new_in`, and similar APIs.
///
/// # Safety invariant
/// [`init`][Self::init] must be called exactly once before any allocation.
pub struct CsHeap(UnsafeCell<Heap>);

// SAFETY: All accesses go through `critical_section::with`, which disables
// IRQs on single-core targets, providing the required mutual exclusion.
unsafe impl Sync for CsHeap {}
unsafe impl Send for CsHeap {}

impl CsHeap {
    /// Creates a new, uninitialised heap.  Must be initialised with
    /// [`init`][Self::init] before any allocation attempt.
    pub const fn empty() -> Self {
        Self(UnsafeCell::new(Heap::empty()))
    }

    /// Initialises the heap to cover `start..start+size`.
    ///
    /// # Safety
    /// - `start..start+size` must be a valid, exclusively-owned, writable
    ///   memory region for the lifetime of the program.
    /// - Must be called exactly once per instance, before the first
    ///   allocation.
    pub unsafe fn init(&self, start: *mut u8, size: usize) {
        unsafe {
            critical_section::with(|_| {
                // SAFETY: exclusive via critical section; caller upholds the rest.
                (*self.0.get()).init(start, size);
            });
        }
    }

    /// Returns the number of bytes currently in use.
    pub fn used(&self) -> usize {
        critical_section::with(|_| unsafe { (*self.0.get()).used() })
    }

    /// Returns the total number of bytes managed by this allocator.
    pub fn size(&self) -> usize {
        critical_section::with(|_| unsafe { (*self.0.get()).size() })
    }

    /// Returns the number of free bytes remaining.
    pub fn free(&self) -> usize {
        critical_section::with(|_| unsafe { (*self.0.get()).free() })
    }

    /// Returns the base address of this heap, or 0 if not yet initialised.
    /// Used by allocation-policy code to route `dealloc` to the correct arena.
    pub fn bottom(&self) -> usize {
        critical_section::with(|_| unsafe { (*self.0.get()).bottom() as usize })
    }
}

unsafe impl Allocator for CsHeap {
    fn allocate(&self, layout: Layout) -> Result<NonNull<[u8]>, AllocError> {
        critical_section::with(|_| {
            // SAFETY: exclusive via critical section.
            unsafe { &mut *self.0.get() }
                .allocate_first_fit(layout)
                .map(|ptr| NonNull::slice_from_raw_parts(ptr, layout.size()))
                .map_err(|_| AllocError)
        })
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, layout: Layout) {
        unsafe {
            critical_section::with(|_| {
                // SAFETY: exclusive via critical section; caller guarantees ptr.
                (*self.0.get()).deallocate(ptr, layout);
            });
        }
    }
}

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
/// Must be initialised *after* [`crate::sdram::init`] has completed and the
/// SDRAM window is accessible:
/// ```rust,ignore
/// unsafe { SDRAM.init(0x0C00_0000 as *mut u8, 64 * 1024 * 1024) }
/// ```
///
/// **Calling `allocate` before `init` will return `AllocError`** (which causes
/// `Box::new_in` / `Vec::try_reserve` to panic). There is no silent UB, but
/// any allocation attempt before `init` will fail at runtime.
pub static SDRAM: CsHeap = CsHeap::empty();
