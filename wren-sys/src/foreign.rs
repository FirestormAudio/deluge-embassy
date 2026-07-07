//! Pure-Rust foreign objects for the upstream wren C VM.
//!
//! The C VM's foreign-method ABI is `WrenForeignMethodFn = void(*)(WrenVM*)`
//! (`wren.h`), which a Rust `unsafe extern "C" fn(*mut WrenVM)` implements
//! directly. Foreign object data is a Rust struct stored inline in the slot
//! returned by `wrenSetSlotNewForeign`. So `Osc`/`Output`/… are plain Rust
//! structs with plain Rust methods — no C glue, no trampolines.
//!
//! The host firmware describes its bindings with two `&'static` tables
//! ([`MethodEntry`] / [`ClassEntry`]) passed to [`crate::boot_with_foreign`].
//! The VM's `bindForeign*Fn` callbacks (installed here) scan those tables.
//!
//! Inside a method, wrap the raw `*mut WrenVM` in [`Vm`] for ergonomic slot
//! access; slot 0 is the receiver, slots 1.. are arguments, and the return
//! value is written to slot 0.

use core::ffi::{c_char, c_int, c_void};

use crate::{
    WrenHandle, WrenVM, wrenAbortFiber, wrenCall, wrenEnsureSlots, wrenGetListCount,
    wrenGetListElement, wrenGetSlotBool, wrenGetSlotBytes, wrenGetSlotDouble, wrenGetSlotForeign,
    wrenGetSlotHandle, wrenGetSlotType, wrenGetVariable, wrenMakeCallHandle, wrenReleaseHandle,
    wrenSetSlotBool, wrenSetSlotBytes, wrenSetSlotDouble, wrenSetSlotHandle, wrenSetSlotNewForeign,
    wrenSetSlotNull,
};

// ── Foreign ABI types (match wren.h) ────────────────────────────────────────

/// A foreign method / class allocator: `void(*)(WrenVM*)`.
pub type ForeignMethod = unsafe extern "C" fn(*mut WrenVM);
/// A foreign finalizer: `void(*)(void* data)`.
pub type Finalizer = unsafe extern "C" fn(*mut c_void);

/// One foreign-method binding. `signature` is the wren method signature, e.g.
/// `"volts=(_)"` (setter), `"volts"` (getter), `"slew(_)"`, `"new(_,_)"`.
pub struct MethodEntry {
    pub module: &'static str,
    pub class: &'static str,
    pub is_static: bool,
    pub signature: &'static str,
    pub func: ForeignMethod,
}

/// One foreign-class binding (allocator + optional finalizer).
pub struct ClassEntry {
    pub module: &'static str,
    pub class: &'static str,
    pub allocate: ForeignMethod,
    pub finalize: Option<Finalizer>,
}

/// `WrenForeignClassMethods` (returned by value from `bindForeignClassFn`).
#[repr(C)]
pub struct WrenForeignClassMethods {
    pub allocate: Option<ForeignMethod>,
    pub finalize: Option<Finalizer>,
}

// ── Registry (set once at boot, read-only after; single-core) ────────────────

static mut METHODS: &[MethodEntry] = &[];
static mut CLASSES: &[ClassEntry] = &[];

/// Install the foreign tables. Called by [`crate::boot_with_foreign`] before
/// `wrenNewVM`, so the bind callbacks see them.
pub(crate) fn set_registry(methods: &'static [MethodEntry], classes: &'static [ClassEntry]) {
    // SAFETY: single-core, called once during boot before any VM use.
    unsafe {
        METHODS = methods;
        CLASSES = classes;
    }
}

/// Compare a NUL-terminated C string to a Rust `&str` for equality.
unsafe fn cstr_eq(c: *const c_char, s: &str) -> bool {
    if c.is_null() {
        return s.is_empty();
    }
    let bytes = s.as_bytes();
    for (i, &b) in bytes.iter().enumerate() {
        // SAFETY: scanning a NUL-terminated string up to its length.
        if unsafe { *c.add(i) } as u8 != b {
            return false;
        }
    }
    // The C string must also end exactly where `s` does.
    unsafe { *c.add(bytes.len()) == 0 }
}

/// `bindForeignMethodFn`: scan the method table for a match.
pub(crate) unsafe extern "C" fn bind_method(
    _vm: *mut WrenVM,
    module: *const c_char,
    class: *const c_char,
    is_static: bool,
    signature: *const c_char,
) -> Option<ForeignMethod> {
    // SAFETY: registry is set once at boot and read-only thereafter.
    let methods = unsafe { METHODS };
    for e in methods {
        if e.is_static == is_static
            && unsafe { cstr_eq(module, e.module) }
            && unsafe { cstr_eq(class, e.class) }
            && unsafe { cstr_eq(signature, e.signature) }
        {
            return Some(e.func);
        }
    }
    None
}

/// `bindForeignClassFn`: scan the class table for a match.
pub(crate) unsafe extern "C" fn bind_class(
    _vm: *mut WrenVM,
    module: *const c_char,
    class: *const c_char,
) -> WrenForeignClassMethods {
    // SAFETY: registry is set once at boot and read-only thereafter.
    let classes = unsafe { CLASSES };
    for e in classes {
        if unsafe { cstr_eq(module, e.module) } && unsafe { cstr_eq(class, e.class) } {
            return WrenForeignClassMethods {
                allocate: Some(e.allocate),
                finalize: e.finalize,
            };
        }
    }
    WrenForeignClassMethods { allocate: None, finalize: None }
}

// ── Slot value types ─────────────────────────────────────────────────────────

/// Low-level slot type (matches `WrenType` in wren.h).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

impl WrenType {
    fn from_int(v: c_int) -> Self {
        match v {
            0 => Self::Bool,
            1 => Self::Num,
            2 => Self::Foreign,
            3 => Self::List,
            4 => Self::Map,
            5 => Self::Null,
            6 => Self::String,
            _ => Self::Unknown,
        }
    }
}

// ── Ergonomic slot wrapper ───────────────────────────────────────────────────

/// Borrowed handle to a VM during a foreign-method call, for slot access.
///
/// Slot 0 is the receiver (`this`); arguments are slots 1, 2, …; the method's
/// return value goes in slot 0.
#[derive(Clone, Copy)]
pub struct Vm(pub *mut WrenVM);

impl Vm {
    /// Number of slots in use (receiver + arguments).
    pub fn slot_count(&self) -> i32 {
        unsafe { wrenGetSlotCount(self.0) }
    }
    /// Ensure at least `n` slots exist.
    pub fn ensure_slots(&self, n: i32) {
        unsafe { wrenEnsureSlots(self.0, n) };
    }
    /// Low-level type of the value in `slot`.
    pub fn slot_type(&self, slot: i32) -> WrenType {
        WrenType::from_int(unsafe { wrenGetSlotType(self.0, slot) })
    }
    pub fn is_null(&self, slot: i32) -> bool {
        self.slot_type(slot) == WrenType::Null
    }

    // ── numbers / bools ──────────────────────────────────────────────────
    pub fn get_f64(&self, slot: i32) -> f64 {
        unsafe { wrenGetSlotDouble(self.0, slot) }
    }
    pub fn set_f64(&self, slot: i32, v: f64) {
        unsafe { wrenSetSlotDouble(self.0, slot, v) };
    }
    pub fn get_bool(&self, slot: i32) -> bool {
        unsafe { wrenGetSlotBool(self.0, slot) }
    }
    pub fn set_bool(&self, slot: i32, v: bool) {
        unsafe { wrenSetSlotBool(self.0, slot, v) };
    }
    pub fn set_null(&self, slot: i32) {
        unsafe { wrenSetSlotNull(self.0, slot) };
    }

    // ── lists ───────────────────────────────────────────────────────────
    pub fn get_list_count(&self, slot: i32) -> i32 {
        unsafe { wrenGetListCount(self.0, slot) }
    }
    pub fn get_list_element(&self, list_slot: i32, index: i32, element_slot: i32) {
        unsafe { wrenGetListElement(self.0, list_slot, index, element_slot) }
    }

    // ── strings (valid only during the call) ─────────────────────────────
    /// Read the bytes of a string/bytes slot. Borrowed from the VM; valid only
    /// for the duration of the current foreign call.
    pub fn get_bytes(&self, slot: i32) -> &[u8] {
        let mut len: c_int = 0;
        let p = unsafe { wrenGetSlotBytes(self.0, slot, &mut len) };
        if p.is_null() || len <= 0 {
            return &[];
        }
        unsafe { core::slice::from_raw_parts(p as *const u8, len as usize) }
    }
    /// Read a string slot as `&str` (lossy: empty on invalid UTF-8).
    pub fn get_str(&self, slot: i32) -> &str {
        core::str::from_utf8(self.get_bytes(slot)).unwrap_or("")
    }
    /// Write a string into `slot`.
    pub fn set_str(&self, slot: i32, s: &str) {
        unsafe { wrenSetSlotBytes(self.0, slot, s.as_ptr() as *const c_char, s.len()) };
    }

    // ── foreign objects ──────────────────────────────────────────────────
    /// Get a `&mut T` to the foreign data in `slot`.
    ///
    /// # Safety
    /// `slot` must hold a foreign instance whose data is a valid `T`.
    pub unsafe fn foreign_mut<T>(&self, slot: i32) -> &mut T {
        unsafe { &mut *(wrenGetSlotForeign(self.0, slot) as *mut T) }
    }

    /// Load a top-level class `module::name` into `slot` (needed before
    /// allocating a new foreign of that class).
    pub fn load_class(&self, module: &str, name: &str, slot: i32) {
        // wrenGetVariable needs NUL-terminated names; copy onto the stack.
        let mut mbuf = [0u8; 64];
        let mut nbuf = [0u8; 64];
        let m = nul_terminate(&mut mbuf, module);
        let n = nul_terminate(&mut nbuf, name);
        unsafe { wrenGetVariable(self.0, m, n, slot) };
    }

    /// For use **inside a foreign-class allocator**: wren has already put the
    /// class in slot 0 (the receiver), so allocate the foreign into slot 0 from
    /// the class in slot 0 and move `value` into it. Call exactly once.
    ///
    /// # Safety
    /// Must be called from the foreign-class `allocate` callback.
    pub unsafe fn alloc_foreign<T>(&self, value: T) {
        let data =
            unsafe { wrenSetSlotNewForeign(self.0, 0, 0, core::mem::size_of::<T>()) };
        if !data.is_null() {
            unsafe { core::ptr::write(data as *mut T, value) };
        }
    }

    /// Construct a foreign object of type `T` into `slot`, loading its class by
    /// name first — for **returning** a freshly-made foreign from an ordinary
    /// method (not the class's own allocator).
    ///
    /// # Safety
    /// `T`'s foreign class must be declared in wren as `T::module_name()::
    /// T::class_name()`.
    pub unsafe fn new_foreign_in<T: WrenForeign>(&self, slot: i32, value: T) {
        self.ensure_slots(slot + 2);
        let class_slot = slot + 1;
        self.load_class(T::module_name(), T::class_name(), class_slot);
        let data = unsafe {
            wrenSetSlotNewForeign(self.0, slot, class_slot, core::mem::size_of::<T>())
        };
        if !data.is_null() {
            unsafe { core::ptr::write(data as *mut T, value) };
        }
    }

    /// Abort the current fiber, using the message written into `slot`.
    pub fn abort(&self, slot: i32) {
        unsafe { wrenAbortFiber(self.0, slot) };
    }
    /// Convenience: abort the current fiber with `msg` as the error.
    pub fn abort_with(&self, msg: &str) {
        self.ensure_slots(1);
        self.set_str(0, msg);
        self.abort(0);
    }

    // ── calling wren from Rust (callbacks) ───────────────────────────────
    /// Take a persistent handle to the value in `slot` (e.g. a callback `Fn`).
    /// Keeps it alive across GC until [`release_handle`](Self::release_handle).
    pub fn get_handle(&self, slot: i32) -> *mut WrenHandle {
        unsafe { wrenGetSlotHandle(self.0, slot) }
    }
    /// Put a handle's value into `slot` (e.g. the receiver before [`call`]).
    pub fn set_handle(&self, slot: i32, h: *mut WrenHandle) {
        unsafe { wrenSetSlotHandle(self.0, slot, h) };
    }
    /// Make a reusable call handle for the method `signature` (e.g. `"call(_)"`).
    pub fn make_call_handle(&self, signature: &str) -> *mut WrenHandle {
        let mut buf = [0u8; 32];
        let s = nul_terminate(&mut buf, signature);
        unsafe { wrenMakeCallHandle(self.0, s) }
    }
    /// Invoke a previously-made call handle. The receiver is in slot 0 and
    /// arguments in slots 1.. (set them first). Returns a `WREN_RESULT_*` code.
    pub fn call(&self, method: *mut WrenHandle) -> i32 {
        unsafe { wrenCall(self.0, method) }
    }
    /// Release a handle (allow GC to reclaim its value).
    pub fn release_handle(&self, h: *mut WrenHandle) {
        unsafe { wrenReleaseHandle(self.0, h) };
    }
}

/// Copy `s` into `buf` with a trailing NUL and return a pointer to it. Truncates
/// to `buf.len() - 1`. Class/module names are short, so 64 bytes is ample.
fn nul_terminate(buf: &mut [u8], s: &str) -> *const c_char {
    let n = s.len().min(buf.len() - 1);
    buf[..n].copy_from_slice(&s.as_bytes()[..n]);
    buf[n] = 0;
    buf.as_ptr() as *const c_char
}

// ── WrenForeign trait ────────────────────────────────────────────────────────

/// A Rust type that can back a wren `foreign class`.
///
/// Implement (or, later, `#[derive(WrenForeign)]`) so [`Vm::new_foreign_in`] can
/// load the class and size the allocation.
pub trait WrenForeign: Sized {
    /// The wren class name (e.g. `"Osc"`).
    fn class_name() -> &'static str;
    /// The wren module the class is declared in. Defaults to `"main"`.
    fn module_name() -> &'static str {
        "main"
    }
}

// Re-export the one FFI fn that lives in lib.rs and isn't imported above.
use crate::wrenGetSlotCount;
