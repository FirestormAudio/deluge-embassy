# Multi-Output Foundation (IO-4a) Plan — executed inline
### Task 1: OutputSrc map + fill_usb
- [ ] `ids.rs`: add `pub const USB_CHANNELS = 8` + `pub enum OutputSrc { Silent, BusL(BusId), BusR(BusId), Node{node,port} }` (derive Clone/Copy/Debug/PartialEq); re-export both in `lib.rs`.
- [ ] `cmd.rs`: `SetUsbOut { channel: u8, src: OutputSrc }` (+ import OutputSrc).
- [ ] `engine.rs`: `usb_out: [OutputSrc; USB_CHANNELS]` field + `new` init (Silent) + `Reset` clear + apply arm (bounds-guarded) + `pub fn fill_usb(&self, usb: &mut [[f32; BLOCK]; USB_CHANNELS])` (Silent→0; BusL/BusR→bus rows guarded; Node→out_base+port guarded, dangling→0).
- [ ] Tests: fill_usb_routes_bus_side_and_node; set_usb_out_of_range_is_noop; usb_dangling_node_is_silent; reset_clears_usb_map.
- [ ] Full graph crate green both configs; deluge-wren-core + wren-firmware armv7a build.
