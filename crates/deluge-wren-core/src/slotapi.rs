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
