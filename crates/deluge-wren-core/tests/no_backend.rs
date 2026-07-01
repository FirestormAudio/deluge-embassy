//! Compile-only check that the backend-agnostic surface is namable with the
//! `wren-sys-backend` feature off — this is what Phase 1's `wren-web-debug`
//! crate needs (`default-features = false`), since it links a second upstream
//! C VM and must not also pull in `wren-sys`'s copy.
#![cfg(not(feature = "wren-sys-backend"))]

use deluge_wren_core::{Handle, SlotApi, WrenForeign, WrenType};

// Just needs to compile: `SlotApi`, `WrenType`, `WrenForeign`, and `Handle` all
// resolve without `wren-sys` anywhere in the dependency graph.
fn _requires_slotapi<S: SlotApi>(_vm: &S) {}
fn _uses_types(_t: WrenType, _h: Handle) {}

struct Marker;
impl WrenForeign for Marker {
    fn module_name() -> &'static str {
        "main"
    }
    fn class_name() -> &'static str {
        "Marker"
    }
}

#[test]
fn generic_surface_is_namable_without_backend() {
    let _t = WrenType::Num;
    let _h = Handle(core::ptr::null_mut());
    assert_eq!(Marker::class_name(), "Marker");
}
