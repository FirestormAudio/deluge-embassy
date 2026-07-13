# Post-Fader Sends (IO-2f) Implementation Plan

> Single-task engine reorder. Executed inline.

### Task 1: Swap per-bus gain before bus→bus sends
- [x] In `render`, move the per-bus-gain loop ABOVE the bus→bus-send loop; update the two seam comments (pre-fader → post-fader).
- [x] Add `bus_send_is_post_fader` engine test (gained bus's send carries post-gain signal).
- [x] Full graph crate green both configs (byte-identical for existing IO-2b/2c/2e tests); dependent crates build.
