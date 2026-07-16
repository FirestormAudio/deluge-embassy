//! `deluge_wren_core::SlotApi` over wren-core's `WrenSlotApi`.
//!
//! This is the second backend for the deluge binding bodies (the first being
//! `wren-sys`). A [`CoreSlots`] is constructed at each foreign-call boundary,
//! wrapping the `&dyn WrenSlotApi` that wren-core hands the foreign method, and
//! forwards deluge's slot operations to the equivalent wren-core calls.
//!
//! ## Handle / call surface
//! wren-core's [`WrenSlotApi`] trait now exposes a handle/call API
//! (`make_call_handle` / `call` / `get_slot_handle` / `set_slot_handle` /
//! `release_handle`, added alongside deluge Task 1.1b), mirroring `wren-sys`'s
//! `Vm` handle surface. This lets the callback-style bindings (`Metro.start`,
//! `Midi.on*`, `Pads/Buttons/Enc.on*`) work under wren-core too: a foreign
//! method captures a `Fn` via [`SlotApi::get_handle`], and the host loop
//! invokes it later via [`SlotApi::call`] — see `WrenSlotApi::call`'s doc for
//! the reentrancy constraint (host/top-level context only, never from inside a
//! foreign method).

use core::cell::RefCell;
use core::ffi::c_void;

use deluge_wren_core::{Handle, SlotApi, WrenForeign as DwcForeign, WrenType as DwcType};
use wren_core::foreign::{WrenSlotApi, WrenType as CoreType};
use wren_core::vm::ffi;

/// A deluge [`SlotApi`] backed by a wren-core [`WrenSlotApi`] for the duration
/// of a single foreign-method call.
pub struct CoreSlots {
    api: &'static dyn WrenSlotApi,
    /// Owns strings returned by [`SlotApi::get_str`] so we can hand out a
    /// borrowed `&str`. Fresh per call; entries are never removed, so the boxed
    /// allocations stay valid for the life of this `CoreSlots`.
    /// Bounded per foreign-method call (a new `CoreSlots`/`strings` is created
    /// at each call boundary and dropped at its end), NOT per-VM — this is not
    /// a latent unbounded-growth leak.
    strings: RefCell<Vec<Box<str>>>,
}

impl CoreSlots {
    /// Wrap the wren-core slot API for the current foreign call.
    pub fn new(api: &dyn WrenSlotApi) -> Self {
        // SAFETY: erase the borrow lifetime. A `CoreSlots` is created at a
        // foreign-call boundary and used only within that call, so `api`
        // (the wren-core `CSlotApi`) outlives every use of this wrapper.
        let api: &'static dyn WrenSlotApi = unsafe { core::mem::transmute(api) };
        Self {
            api,
            strings: RefCell::new(Vec::new()),
        }
    }
}

fn conv_type(t: CoreType) -> DwcType {
    match t {
        CoreType::Bool => DwcType::Bool,
        CoreType::Num => DwcType::Num,
        CoreType::Foreign => DwcType::Foreign,
        CoreType::List => DwcType::List,
        CoreType::Map => DwcType::Map,
        CoreType::Null => DwcType::Null,
        CoreType::String => DwcType::String,
        CoreType::Unknown => DwcType::Unknown,
    }
}

impl SlotApi for CoreSlots {
    fn ensure_slots(&self, n: i32) {
        self.api.ensure_slots(n.max(0) as usize);
    }

    fn slot_type(&self, slot: i32) -> DwcType {
        conv_type(self.api.get_slot_type(slot as usize))
    }

    fn get_f(&self, slot: i32) -> f64 {
        self.api.get_slot_double(slot as usize).unwrap_or(0.0)
    }

    fn set_f(&self, slot: i32, v: f64) {
        self.api.set_slot_double(slot as usize, v);
    }

    fn get_bool(&self, slot: i32) -> bool {
        self.api.get_slot_bool(slot as usize).unwrap_or(false)
    }

    fn get_str(&self, slot: i32) -> &str {
        let s = self
            .api
            .get_slot_string(slot as usize)
            .unwrap_or_default()
            .into_boxed_str();
        let ptr: *const str = &*s;
        self.strings.borrow_mut().push(s);
        // SAFETY: the boxed str is now owned by `self.strings` and never
        // removed, so its heap allocation is stable for the life of `self`.
        unsafe { &*ptr }
    }

    fn get_list_count(&self, slot: i32) -> i32 {
        self.api.get_list_count(slot as usize) as i32
    }

    fn get_list_element(&self, list_slot: i32, index: i32, elem_slot: i32) {
        self.api
            .get_list_element(list_slot as usize, index as usize, elem_slot as usize);
    }

    unsafe fn foreign_mut<T>(&self, slot: i32) -> &mut T {
        let ptr = unsafe { self.api.get_slot_foreign(slot as usize) } as *mut T;
        unsafe { &mut *ptr }
    }

    unsafe fn alloc_foreign<T>(&self, value: T) {
        // Foreign-class allocator convention: slot 0 already holds the class
        // (the receiver), and the new foreign is written back into slot 0.
        let data =
            unsafe { self.api.set_slot_new_foreign(0, 0, core::mem::size_of::<T>()) } as *mut T;
        if !data.is_null() {
            unsafe { core::ptr::write(data, value) };
        }
    }

    unsafe fn new_foreign_in<T: DwcForeign>(&self, slot: i32, value: T) {
        let slot = slot as usize;
        let class_slot = slot + 1;
        self.api.ensure_slots(class_slot + 1);
        self.api
            .get_variable(T::module_name(), T::class_name(), class_slot);
        let data = unsafe {
            self.api
                .set_slot_new_foreign(slot, class_slot, core::mem::size_of::<T>())
        } as *mut T;
        if !data.is_null() {
            unsafe { core::ptr::write(data, value) };
        }
    }

    // ── Handle / call surface: delegate to wren-core's handle API ──────────
    fn get_handle(&self, slot: i32) -> Handle {
        Handle(self.api.get_slot_handle(slot as usize) as *mut c_void)
    }

    fn set_handle(&self, slot: i32, h: Handle) {
        self.api.set_slot_handle(slot as usize, h.0 as *mut ffi::WrenHandle);
    }

    fn make_call_handle(&self, signature: &str) -> Handle {
        Handle(self.api.make_call_handle(signature) as *mut c_void)
    }

    fn call(&self, method: Handle) -> i32 {
        match self.api.call(method.0 as *mut ffi::WrenHandle) {
            Ok(()) => 0,
            Err(_) => 1,
        }
    }

    fn release_handle(&self, h: Handle) {
        self.api.release_handle(h.0 as *mut ffi::WrenHandle);
    }
}
