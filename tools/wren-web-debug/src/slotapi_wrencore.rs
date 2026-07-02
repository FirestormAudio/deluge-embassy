//! `deluge_wren_core::SlotApi` over wren-core's `WrenSlotApi`.
//!
//! This is the second backend for the deluge binding bodies (the first being
//! `wren-sys`). A [`CoreSlots`] is constructed at each foreign-call boundary,
//! wrapping the `&dyn WrenSlotApi` that wren-core hands the foreign method, and
//! forwards deluge's slot operations to the equivalent wren-core calls.
//!
//! ## Handle / call gap (IMPORTANT)
//! wren-core's [`WrenSlotApi`] trait exposes only value slots — it has **no**
//! persistent-handle or `wrenCall` surface (those C entry points exist in
//! wren-core's FFI, but the trait doesn't expose them and the concrete
//! `CSlotApi` passed to foreign methods is private, so we can't recover the raw
//! `WrenVM*`). The deluge `SlotApi` needs `get_handle` / `set_handle` /
//! `make_call_handle` / `call` / `release_handle` for the callback-style
//! bindings (`Metro.start`, `Midi.on*`, `Pads/Buttons/Enc.on*`). Those methods
//! are therefore **stubbed** here (null handle / no-op). Every value binding
//! (`Output`, `Gate`, `Node`/`Osc`/`Env`/`Noise`/`Out`, `Led`, `Oled`, and the
//! direct `Midi.noteOn/…` senders) maps cleanly and is fully functional; the
//! callback-registration bindings will silently no-op until wren-core grows a
//! handle API. See the task report for the escalation.

use core::cell::RefCell;

use deluge_wren_core::{Handle, SlotApi, WrenForeign as DwcForeign, WrenType as DwcType};
use wren_core::foreign::{WrenSlotApi, WrenType as CoreType};

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

    // ── Handle / call surface: unsupported under wren-core (see module docs) ──
    fn get_handle(&self, _slot: i32) -> Handle {
        Handle(core::ptr::null_mut())
    }
    fn set_handle(&self, _slot: i32, _h: Handle) {}
    fn make_call_handle(&self, _signature: &str) -> Handle {
        Handle(core::ptr::null_mut())
    }
    fn call(&self, _method: Handle) -> i32 {
        0
    }
    fn release_handle(&self, _h: Handle) {}
}
