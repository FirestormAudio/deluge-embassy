//! `SlotApi` over the stock C VM (`wren_sys::Vm`). Delegates to the existing
//! ergonomic wrappers; behavior is identical to the pre-refactor direct calls.
use crate::slotapi::{Handle, SlotApi, WrenForeign as DwcForeign, WrenType};
use wren_sys::Vm;

fn conv_type(t: wren_sys::WrenType) -> WrenType {
    match t {
        wren_sys::WrenType::Bool => WrenType::Bool,
        wren_sys::WrenType::Num => WrenType::Num,
        wren_sys::WrenType::Foreign => WrenType::Foreign,
        wren_sys::WrenType::List => WrenType::List,
        wren_sys::WrenType::Map => WrenType::Map,
        wren_sys::WrenType::Null => WrenType::Null,
        wren_sys::WrenType::String => WrenType::String,
        wren_sys::WrenType::Unknown => WrenType::Unknown,
    }
}

/// Mirrors [`wren_sys::Vm::new_foreign_in`], but generic over the dwc
/// [`DwcForeign`] marker instead of `wren_sys::WrenForeign` — so `SlotApi`
/// callers never need to know about the wren-sys-specific marker trait.
unsafe fn new_foreign_in_shim<T: DwcForeign>(vm: &Vm, slot: i32, value: T) {
    vm.ensure_slots(slot + 2);
    let class_slot = slot + 1;
    vm.load_class(T::module_name(), T::class_name(), class_slot);
    let data = unsafe { wren_sys::wrenSetSlotNewForeign(vm.0, slot, class_slot, core::mem::size_of::<T>()) };
    if !data.is_null() {
        unsafe { core::ptr::write(data as *mut T, value) };
    }
}

impl SlotApi for Vm {
    fn ensure_slots(&self, n: i32) {
        Vm::ensure_slots(self, n)
    }
    fn slot_type(&self, slot: i32) -> WrenType {
        conv_type(Vm::slot_type(self, slot))
    }
    fn get_f(&self, slot: i32) -> f64 {
        Vm::get_f64(self, slot)
    }
    fn set_f(&self, slot: i32, v: f64) {
        Vm::set_f64(self, slot, v)
    }
    fn get_bool(&self, slot: i32) -> bool {
        Vm::get_bool(self, slot)
    }
    fn get_str(&self, slot: i32) -> &str {
        Vm::get_str(self, slot)
    }
    fn get_list_count(&self, slot: i32) -> i32 {
        Vm::get_list_count(self, slot)
    }
    fn get_list_element(&self, list_slot: i32, index: i32, elem_slot: i32) {
        Vm::get_list_element(self, list_slot, index, elem_slot)
    }
    unsafe fn foreign_mut<T>(&self, slot: i32) -> &mut T {
        unsafe { Vm::foreign_mut::<T>(self, slot) }
    }
    unsafe fn alloc_foreign<T>(&self, value: T) {
        unsafe { Vm::alloc_foreign::<T>(self, value) }
    }
    unsafe fn new_foreign_in<T: DwcForeign>(&self, slot: i32, value: T) {
        // Bridge the dwc marker to wren-sys's WrenForeign via a local shim
        // that inlines `Vm::new_foreign_in`'s body. (Concrete binding types
        // will implement the dwc marker directly in Task 0.2.)
        unsafe { new_foreign_in_shim(self, slot, value) }
    }
    fn get_handle(&self, slot: i32) -> Handle {
        Handle(Vm::get_handle(self, slot) as *mut _)
    }
    fn set_handle(&self, slot: i32, h: Handle) {
        Vm::set_handle(self, slot, h.0 as *mut _)
    }
    fn make_call_handle(&self, sig: &str) -> Handle {
        Handle(Vm::make_call_handle(self, sig) as *mut _)
    }
    fn call(&self, method: Handle) -> i32 {
        Vm::call(self, method.0 as *mut _)
    }
    fn release_handle(&self, h: Handle) {
        Vm::release_handle(self, h.0 as *mut _)
    }
}
