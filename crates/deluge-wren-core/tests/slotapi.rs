// The trait must be object-safe-free (generic methods) but usable via a concrete
// type. This compile-level test asserts the wren-sys Vm implements SlotApi.
use deluge_wren_core::SlotApi;

fn assert_impl<S: SlotApi>() {}

#[test]
fn wren_sys_vm_is_slotapi() {
    assert_impl::<wren_sys::Vm>();
}

// Compile-level check that the list-read primitives exist on `SlotApi` with
// the expected signatures. Never called (no live VM here), just monomorphized
// via a function-pointer coercion so the body's method calls are typechecked.
// Behavioral coverage (a real Wren list driven through these) lands in Task 5
// (`Wavetable.from` round-trip).
fn uses_list_api<S: SlotApi>(s: &S) {
    let _count: i32 = s.get_list_count(0);
    s.get_list_element(0, 0, 1);
}

#[test]
fn slotapi_has_list_read() {
    let _f: fn(&wren_sys::Vm) = uses_list_api::<wren_sys::Vm>;
}
