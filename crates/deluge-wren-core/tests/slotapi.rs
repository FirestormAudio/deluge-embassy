// The trait must be object-safe-free (generic methods) but usable via a concrete
// type. This compile-level test asserts the wren-sys Vm implements SlotApi.
use deluge_wren_core::SlotApi;

fn assert_impl<S: SlotApi>() {}

#[test]
fn wren_sys_vm_is_slotapi() {
    assert_impl::<wren_sys::Vm>();
}
