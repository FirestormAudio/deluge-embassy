//! Backend-generic slot access. Binding bodies target this trait so one
//! implementation serves the stock-C-VM sim/device (`wren-sys`) and the
//! `wren-core` debug core. Generic methods => static dispatch only.
use core::ffi::c_void;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WrenType {
    Bool,
    Num,
    Foreign,
    List,
    Map,
    Null,
    String,
    Unknown,
}

/// A persistent handle to a wren value (callback `Fn`, etc.). Opaque; the
/// backend owns the real pointer.
#[derive(Clone, Copy)]
pub struct Handle(pub *mut c_void);

/// Marks a Rust type that has a declared foreign wren class.
pub trait WrenForeign {
    fn module_name() -> &'static str;
    fn class_name() -> &'static str;
}

pub trait SlotApi {
    fn ensure_slots(&self, n: i32);
    fn slot_type(&self, slot: i32) -> WrenType;
    fn get_f(&self, slot: i32) -> f64;
    fn set_f(&self, slot: i32, v: f64);
    fn get_bool(&self, slot: i32) -> bool;
    fn get_str(&self, slot: i32) -> &str;
    fn get_list_count(&self, slot: i32) -> i32;
    fn get_list_element(&self, list_slot: i32, index: i32, elem_slot: i32);
    /// # Safety: `slot` must hold a foreign of type `T`.
    unsafe fn foreign_mut<T>(&self, slot: i32) -> &mut T;
    /// # Safety: call only from a foreign-class allocator.
    unsafe fn alloc_foreign<T>(&self, value: T);
    /// # Safety: `T`'s foreign class must be declared in wren.
    unsafe fn new_foreign_in<T: WrenForeign>(&self, slot: i32, value: T);
    fn get_handle(&self, slot: i32) -> Handle;
    fn set_handle(&self, slot: i32, h: Handle);
    fn make_call_handle(&self, signature: &str) -> Handle;
    fn call(&self, method: Handle) -> i32;
    fn release_handle(&self, h: Handle);
}

/// List length if `slot` is actually a List, else 0. A subsequent `0..count`
/// walk is then always in-bounds (the wren-sys VM does NO bounds/type checking
/// — its C `ASSERT`s are compiled out).
pub(crate) fn checked_list_count<S: SlotApi>(vm: &S, slot: i32) -> usize {
    if vm.slot_type(slot) == WrenType::List {
        vm.get_list_count(slot).max(0) as usize
    } else {
        0
    }
}

/// `&str` if `slot` is a String, else `""`. Guards `get_str`, which the VM
/// backs with an unconditional `AS_STRING` deref (UB on a non-String slot).
pub(crate) fn checked_str<S: SlotApi>(vm: &S, slot: i32) -> &str {
    if vm.slot_type(slot) == WrenType::String {
        vm.get_str(slot)
    } else {
        ""
    }
}
