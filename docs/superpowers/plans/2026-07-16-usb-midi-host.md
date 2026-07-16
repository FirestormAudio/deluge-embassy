# USB MIDI Host Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Host class-compliant USB MIDI 1.0 devices on the Deluge — controllers, synths, and multiple devices behind a hub — delivering raw 4-byte USB-MIDI packets to the firmware.

**Architecture:** A generic `MidiHost<'d, A: UsbHostAllocator>` class driver in `deluge-bsp/src/usb/host/`, bound to enumerated devices by a supervisor task that owns the `BusController`. Per-device pooled Embassy tasks own their pipes and pump a merged, source-tagged static channel. The final task adds transparent OUT-pipe multiplexing inside `rza1l-hal`'s `Rusb1Allocator`, lifting the device ceiling from ~2 to 6 by sharing one TX pipe across devices with software data-toggle save/restore.

**Tech Stack:** Rust (nightly), `embassy-usb-host` 0.1.0, `embassy-usb-driver` 0.2.2, `embassy-executor` 0.10, `embassy-sync` 0.8, `rza1l-hal` (RZ/A1L RUSB2), QEMU-ARM test harness.

**Spec:** [`docs/superpowers/specs/2026-07-16-usb-midi-host-design.md`](../specs/2026-07-16-usb-midi-host-design.md)

## Global Constraints

- **Alloc-free.** `deluge-bsp` has no heap dependency. Do not add one. No `alloc`, no `Box`, no `Vec`.
- **MIDI 1.0 only.** No UMP / MIDI 2.0. No GTB descriptors. No alt-setting negotiation beyond alt 0.
- **`no_std`.** Every new module is `no_std`-compatible.
- **Tests run under QEMU ARM, not host x86_64.** `deluge-bsp` depends on `rza1l-hal`, which uses ARM inline asm and cannot build for x86_64. The canonical command is:
  `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib`
  Everyday `cargo` commands default to `armv7a-none-eabihf` (`.cargo/config.toml`), so **always pass `--target` explicitly when testing.**
- **Test gating.** Unit tests use `#[cfg(all(test, not(target_os = "none")))]` — the established idiom in `crates/deluge-bsp/src/jacks.rs` and `flash.rs`.
- **Pipe budget is hardware-fixed** (TRM §28.1(4)): host mode has bulk pipes 1–5, interrupt 6–9, iso 1–2 only. Pipes 10–15 are function-mode only and unusable. Never "widen the range".
- **Reference implementation:** `~/GitHub/DelugeFirmware-clean` solves this on identical silicon. Consult it when a design question arises; do not invent where it has an answer.

---

## File Structure

| File | Responsibility |
|---|---|
| `crates/rza1l-hal/src/usb/host.rs` | *Modify.* Device-address bounds guard (Task 1); OUT-pipe multiplexing (Task 8). |
| `crates/deluge-bsp/Cargo.toml` | *Modify.* Make `embassy-usb-host` available on the test target (Task 2). |
| `crates/deluge-bsp/src/usb/mod.rs` | *Modify.* Declare `pub mod host;`. |
| `crates/deluge-bsp/src/usb/host/mod.rs` | *Create.* `DeviceId`, merged channel, supervisor task, per-device task. |
| `crates/deluge-bsp/src/usb/host/midi.rs` | *Create.* MIDI interface matching, `MidiHost`, packet decode. |
| `crates/deluge-bsp/src/usb/host/mock.rs` | *Create.* `#[cfg(test)]` mock `UsbHostAllocator` / `UsbPipe` + descriptor fixtures. |

---

## Task 1: Device-address bounds guard in the HAL

`HCD_MAX_DEV = 6` caps device addresses at 1–5, but `device_open` only checks it with `debug_assert!`. In release, a 6th device would index `ctl_mps[6]` on a 6-element array and panic. Single-device firmware never reaches this; multi-device does.

**Files:**
- Modify: `crates/rza1l-hal/src/usb/host.rs` (~line 84 and ~line 493)
- Test: `crates/rza1l-hal/src/usb/host.rs` (new `#[cfg(test)]` module at end of file)

**Interfaces:**
- Consumes: nothing.
- Produces: `pub(crate) const fn dev_addr_supported(dev_addr: u8) -> bool` — `true` iff `dev_addr` is a usable device address (1..=5). Used by Task 8.

- [ ] **Step 1: Write the failing test**

Append to `crates/rza1l-hal/src/usb/host.rs`:

```rust
#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn dev_addr_zero_is_not_supported() {
        // Address 0 is the enumeration default address, never a target.
        assert!(!dev_addr_supported(0));
    }

    #[test]
    fn dev_addr_in_range_is_supported() {
        for addr in 1..HCD_MAX_DEV as u8 {
            assert!(dev_addr_supported(addr), "addr {addr} should be supported");
        }
    }

    #[test]
    fn dev_addr_at_or_above_max_is_not_supported() {
        // HCD_MAX_DEV is exclusive: ctl_mps and ep_to_pipe are sized from it.
        assert!(!dev_addr_supported(HCD_MAX_DEV as u8));
        assert!(!dev_addr_supported(127));
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib dev_addr`
Expected: FAIL — `cannot find function 'dev_addr_supported' in this scope`.

- [ ] **Step 3: Add the guard function**

Add near `HCD_MAX_DEV` (~line 84) in `crates/rza1l-hal/src/usb/host.rs`:

```rust
/// True iff `dev_addr` is a device address this driver can target.
///
/// Address 0 is the enumeration default address. The upper bound is
/// `HCD_MAX_DEV`, which sizes `HcdAlloc::ctl_mps` and `HcdAlloc::ep_to_pipe` —
/// exceeding it would index out of bounds.
pub(crate) const fn dev_addr_supported(dev_addr: u8) -> bool {
    dev_addr > 0 && (dev_addr as usize) < HCD_MAX_DEV
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib dev_addr`
Expected: PASS — 3 passed.

- [ ] **Step 5: Make `device_open` return a real error**

Change the signature and replace the `debug_assert!` at ~line 493:

```rust
    pub fn device_open(
        &self,
        dev_addr: u8,
        mps: u16,
        speed: Speed,
        hub_addr: u8,
        hub_port: u8,
    ) -> Result<(), HostError> {
        if !dev_addr_supported(dev_addr) && dev_addr != 0 {
            return Err(HostError::OutOfSlots);
        }
        // hub_addr 0 means direct; 1-10 are valid hub USB addresses.
        debug_assert!(hub_addr <= 10);
        // hub_port 0 means direct; 1-7 are valid hub port numbers.
        debug_assert!(hub_port <= 7);
```

Add `Ok(())` as the final expression of the function body (after the existing
`critical_section::with(...)` block).

> Note: `dev_addr == 0` is permitted — enumeration programs the DCP at the
> default address before `SET_ADDRESS`. `ctl_mps[0]` is in bounds.

- [ ] **Step 6: Update the one caller**

In `Rusb1Allocator::alloc_pipe` (~line 1536), propagate the error:

```rust
            EndpointType::Control => {
                // Pipe 0 (DCP).  Programme DCPMAXP + DEVADD for this device.
                drv.device_open(addr, endpoint.max_packet_size, speed, hub_addr, hub_port)?;
                Ok(Rusb1Pipe {
```

- [ ] **Step 7: Verify the whole HAL still builds and tests**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib`
Expected: PASS, no regressions.
Run: `cargo build-fw -p demo-firmware`
Expected: builds clean (proves the bare-metal target still compiles).

- [ ] **Step 8: Commit**

```bash
git add crates/rza1l-hal/src/usb/host.rs
git commit -m "fix(rza1l-hal): guard device addresses against HCD_MAX_DEV

device_open only checked dev_addr with debug_assert!, so a 6th enumerated
device would index ctl_mps[6] on a 6-element array and panic in release.
Return HostError::OutOfSlots instead."
```

---

## Task 2: Cargo wiring + mock allocator

`embassy-usb-host` currently sits under `[target.'cfg(target_os = "none")'.dependencies]` in `deluge-bsp`, but tests run on `armv7-unknown-linux-gnueabihf` (`target_os = "linux"`) — so the trait wouldn't exist where the tests run. This task makes it available and lands the mock the rest of the plan tests against.

This wiring is **verified working**: `embassy-usb-host` compiles for the QEMU ARM target, and a mock allocator links and runs there with `embassy-time/mock-driver` supplying the time driver.

**Files:**
- Modify: `crates/deluge-bsp/Cargo.toml`
- Modify: `crates/deluge-bsp/src/usb/mod.rs`
- Create: `crates/deluge-bsp/src/usb/host/mod.rs`
- Create: `crates/deluge-bsp/src/usb/host/mock.rs`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `mock::MockAlloc` — `Clone`, implements `UsbHostAllocator<'d>`.
  - `mock::MockPipe<T, D>` — implements `UsbPipe<T, D>`; `MockPipe::script(&[[u8; 4]])` queues IN data; `MockPipe::sent() -> Vec<u8, 256>` returns what was written OUT.

- [ ] **Step 1: Move the USB deps so they exist on the test target**

In `crates/deluge-bsp/Cargo.toml`, move `embassy-usb-driver`, `embassy-usb-host`, `embassy-sync`, and `embassy-time` out of the `cfg(target_os = "none")` block into `[dependencies]`, and add the test-target dev-deps. The resulting dependency sections:

```toml
[dependencies]
rza1l-hal = { path = "../rza1l-hal", version = "0.1.0" }
embedded-sdmmc = { version = "0.9", default-features = false, features = ["log"] }
critical-section = { workspace = true }
log = { workspace = true }
embedded-graphics-core = { version = "0.4", optional = true }
heapless = "0.9"
# USB host stack. Unconditional (not cfg(target_os = "none")) because the
# host class drivers are generic over UsbHostAllocator and are unit-tested on
# armv7-unknown-linux-gnueabihf under QEMU — see tools/test.sh.
embassy-sync = { workspace = true }
embassy-time = { workspace = true }
embassy-usb-driver = { workspace = true }
embassy-usb-host = { workspace = true }

[target.'cfg(target_os = "none")'.dependencies]
cortex-ar = { workspace = true }
embassy-futures = { workspace = true }
embassy-time = { workspace = true, features = ["tick-hz-1_000_000"] }
embassy-usb = { workspace = true }

[target.'cfg(not(target_os = "none"))'.dev-dependencies]
critical-section = { workspace = true, features = ["std"] }
# embassy-time needs a registered time driver to link the test binary.
embassy-time = { workspace = true, features = ["mock-driver"] }
```

- [ ] **Step 2: Verify the existing suite still passes after the dep move**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib`
Expected: PASS — 54 passed (the pre-existing baseline), 0 failed.
Run: `cargo build-fw -p demo-firmware`
Expected: builds clean.

> If the firmware build breaks because `usb::classes::midi` needs `embassy-usb`,
> that module is already `#[cfg(target_os = "none")]`-gated and `embassy-usb`
> stays target-gated above — no change needed.

- [ ] **Step 3: Declare the module**

In `crates/deluge-bsp/src/usb/mod.rs`, add to the module list:

```rust
pub mod bot;
pub mod classes;
pub mod host;
pub mod ids;
```

- [ ] **Step 4: Create the host module skeleton**

Create `crates/deluge-bsp/src/usb/host/mod.rs`:

```rust
//! USB **host**-side class drivers.
//!
//! The device-side classes live in [`super::classes`]. This module is the
//! mirror image: drivers for devices the Deluge hosts, built on
//! `embassy-usb-host` and the RUSB1 host driver in [`rza1l_hal::usb`].
//!
//! Class drivers here are generic over [`UsbHostAllocator`], so they are unit
//! tested against [`mock::MockAlloc`] under QEMU — see `tools/test.sh`.
//!
//! [`UsbHostAllocator`]: embassy_usb_driver::host::UsbHostAllocator

pub mod midi;

#[cfg(all(test, not(target_os = "none")))]
pub(crate) mod mock;
```

- [ ] **Step 5: Create the mock allocator**

Create `crates/deluge-bsp/src/usb/host/mock.rs`:

```rust
//! Mock [`UsbHostAllocator`] / [`UsbPipe`] for unit tests.
//!
//! Lets the generic class drivers be exercised with scripted pipe responses,
//! with no hardware and no `rza1l-hal` register access.

// This module only compiles under `cfg(all(test, not(target_os = "none")))`,
// where std is available — see the crate's test bucket in tools/test.sh.
extern crate std;
use std::boxed::Box;

use core::cell::RefCell;

use embassy_usb_driver::host::{
    pipe, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator, UsbPipe,
};
use embassy_usb_driver::EndpointInfo;
use heapless::Vec;

/// What a [`MockPipe`] should do on the next `request_in`.
#[derive(Clone, Default)]
pub struct Script {
    /// Bytes returned by successive `request_in` calls.
    pub reads: Vec<Vec<u8, 64>, 8>,
    /// Error returned instead of the next read, if set.
    pub read_err: Option<PipeError>,
}

/// Records what a mock pipe was asked to do.
#[derive(Default)]
pub struct PipeLog {
    /// Bytes passed to `request_out`, concatenated.
    pub sent: Vec<u8, 256>,
    /// Number of `request_in` calls.
    pub reads_issued: usize,
    /// Number of `reset_data_toggle` calls.
    pub toggle_resets: usize,
}

/// A record of one `alloc_pipe` call.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct AllocRecord {
    pub addr: u8,
    pub ep_addr: u8,
    pub max_packet_size: u16,
}

#[derive(Clone)]
pub struct MockAlloc {
    inner: &'static MockState,
}

pub struct MockState {
    pub script: RefCell<Script>,
    pub log: RefCell<PipeLog>,
    pub allocs: RefCell<Vec<AllocRecord, 8>>,
    /// When set, `alloc_pipe` fails with this error (simulates pipe exhaustion).
    pub alloc_err: RefCell<Option<HostError>>,
}

impl MockState {
    pub fn new() -> Self {
        Self {
            script: RefCell::new(Script {
                reads: Vec::new(),
                read_err: None,
            }),
            log: RefCell::new(PipeLog {
                sent: Vec::new(),
                reads_issued: 0,
                toggle_resets: 0,
            }),
            allocs: RefCell::new(Vec::new()),
            alloc_err: RefCell::new(None),
        }
    }

    /// Leak a fresh state and hand back a `&'static` reference.
    ///
    /// [`UsbHostAllocator`] is parameterised by `'d`, and the class drivers use
    /// `'static` in practice. `MockState` holds `RefCell`s and so is not `Sync`,
    /// which rules out a plain `static` — leaking gives each test its own
    /// isolated `'static` state with no cross-test interference.
    pub fn leak() -> &'static MockState {
        Box::leak(Box::new(MockState::new()))
    }
}

impl MockAlloc {
    pub fn new(state: &'static MockState) -> Self {
        Self { inner: state }
    }

    pub fn state(&self) -> &'static MockState {
        self.inner
    }
}

pub struct MockPipe<T: pipe::Type, D: pipe::Direction> {
    inner: &'static MockState,
    _p: core::marker::PhantomData<(T, D)>,
}

impl<'d> UsbHostAllocator<'d> for MockAlloc {
    type Pipe<T: pipe::Type, D: pipe::Direction> = MockPipe<T, D>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        _split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        if let Some(e) = *self.inner.alloc_err.borrow() {
            return Err(e);
        }
        let _ = self.inner.allocs.borrow_mut().push(AllocRecord {
            addr,
            ep_addr: u8::from(endpoint.addr),
            max_packet_size: endpoint.max_packet_size,
        });
        Ok(MockPipe {
            inner: self.inner,
            _p: core::marker::PhantomData,
        })
    }
}

impl<T: pipe::Type, D: pipe::Direction> UsbPipe<T, D> for MockPipe<T, D> {
    async fn control_in(&mut self, _setup: &[u8; 8], _buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        Ok(0)
    }

    async fn control_out(&mut self, _setup: &[u8; 8], _buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        Ok(())
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        self.inner.log.borrow_mut().reads_issued += 1;
        let mut script = self.inner.script.borrow_mut();
        if let Some(e) = script.read_err.take() {
            return Err(e);
        }
        if script.reads.is_empty() {
            return Ok(0);
        }
        let data = script.reads.remove(0);
        let n = data.len().min(buf.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }

    async fn request_out(&mut self, buf: &[u8], _z: bool) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        let mut log = self.inner.log.borrow_mut();
        log.sent.extend_from_slice(buf).map_err(|_| PipeError::BufferOverflow)
    }

    fn set_timeout(&mut self, _t: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        self.inner.log.borrow_mut().toggle_resets += 1;
    }
}
```

- [ ] **Step 6: Create `midi.rs` as a stub so the module compiles**

Create `crates/deluge-bsp/src/usb/host/midi.rs`:

```rust
//! USB **host**-side MIDI 1.0 class driver.
```

- [ ] **Step 7: Write a smoke test proving the mock links under QEMU**

Append to `crates/deluge-bsp/src/usb/host/mock.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use embassy_usb_driver::EndpointType;

    #[test]
    fn mock_allocator_records_alloc_and_links() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let info = EndpointInfo {
            addr: 0x81.into(),
            ep_type: EndpointType::Bulk,
            max_packet_size: 64,
            interval_ms: 0,
        };
        let _pipe = alloc
            .alloc_pipe::<pipe::Bulk, pipe::In>(1, &info, None)
            .expect("alloc should succeed");
        let allocs = state.allocs.borrow();
        assert_eq!(allocs.len(), 1);
        assert_eq!(allocs[0].ep_addr, 0x81);
        assert_eq!(allocs[0].addr, 1);
    }
}
```

- [ ] **Step 8: Run the test**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib mock`
Expected: PASS — 1 passed.

- [ ] **Step 9: Verify the firmware target still builds**

Run: `cargo build-fw -p demo-firmware`
Expected: builds clean.

- [ ] **Step 10: Commit**

```bash
git add crates/deluge-bsp/Cargo.toml crates/deluge-bsp/src/usb/mod.rs crates/deluge-bsp/src/usb/host/
git commit -m "feat(deluge-bsp): scaffold usb::host with a mock allocator

Moves the USB host deps out of the cfg(target_os = \"none\") block so the
generic class drivers can be unit-tested on armv7-unknown-linux-gnueabihf
under QEMU, and lands the mock UsbHostAllocator the class drivers test against."
```

---

## Task 3: MIDI interface matching

Find the MIDIStreaming interface in an enumerated device's configuration and pick its bulk endpoints. This is where sink-only synths, source-only controllers, and composite audio+MIDI devices are handled correctly.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/midi.rs`

**Interfaces:**
- Consumes: `mock::MockAlloc` (Task 2).
- Produces:
  - `pub struct MidiInterface { pub interface_number: u8, pub ep_in: Option<EndpointDescriptor>, pub ep_out: Option<EndpointDescriptor> }`
  - `pub fn find_midi_interface(cfg: &ConfigurationDescriptor<'_>) -> Option<MidiInterface>`

- [ ] **Step 1: Write the failing tests**

Append to `crates/deluge-bsp/src/usb/host/midi.rs`:

```rust
#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    /// Config descriptor builder: 9-byte config header + supplied body bytes.
    fn cfg_bytes(body: &[u8]) -> heapless::Vec<u8, 128> {
        let total = 9 + body.len();
        let mut v = heapless::Vec::new();
        v.extend_from_slice(&[
            9, 0x02, total as u8, 0, 1, 1, 0, 0x80, 50,
        ])
        .unwrap();
        v.extend_from_slice(body).unwrap();
        v
    }

    /// Audio/MIDIStreaming interface descriptor with `n` endpoints.
    fn ms_iface(n: u8) -> [u8; 9] {
        [9, 0x04, 0, 0, n, 0x01, 0x03, 0, 0]
    }

    /// Bulk endpoint descriptor. `addr` carries the 0x80 IN bit.
    fn bulk_ep(addr: u8) -> [u8; 7] {
        [7, 0x05, addr, 0x02, 64, 0, 0]
    }

    #[test]
    fn matches_bidirectional_device() {
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(2)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        body.extend_from_slice(&bulk_ep(0x01)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("should match");
        assert_eq!(m.interface_number, 0);
        assert_eq!(m.ep_in.unwrap().endpoint_address, 0x81);
        assert_eq!(m.ep_out.unwrap().endpoint_address, 0x01);
    }

    #[test]
    fn matches_source_only_controller() {
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("should match");
        assert_eq!(m.ep_in.unwrap().endpoint_address, 0x81);
        assert!(m.ep_out.is_none(), "controller has no OUT endpoint");
    }

    #[test]
    fn matches_sink_only_synth() {
        // A USB synth/sound module only receives MIDI: bulk OUT, no bulk IN.
        // This must match — the Deluge sequences external synths.
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x02)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("sink-only synth must match");
        assert!(m.ep_in.is_none());
        assert_eq!(m.ep_out.unwrap().endpoint_address, 0x02);
    }

    #[test]
    fn skips_audio_control_interface_and_finds_midi() {
        // Composite device: AudioControl (subclass 0x01) then MIDIStreaming.
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&[9, 0x04, 0, 0, 0, 0x01, 0x01, 0, 0]).unwrap();
        let mut ms = ms_iface(1);
        ms[2] = 1; // interface_number = 1
        body.extend_from_slice(&ms).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let m = find_midi_interface(&cfg).expect("should find the MS interface");
        assert_eq!(m.interface_number, 1);
    }

    #[test]
    fn rejects_non_midi_device() {
        // HID keyboard: class 0x03, nothing to do with MIDI.
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&[9, 0x04, 0, 0, 1, 0x03, 0x01, 1, 0]).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        assert!(find_midi_interface(&cfg).is_none());
    }

    #[test]
    fn rejects_midi_interface_with_no_endpoints() {
        let raw = cfg_bytes(&ms_iface(0));
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        assert!(find_midi_interface(&cfg).is_none(), "no endpoints = unusable");
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::midi`
Expected: FAIL — `cannot find function 'find_midi_interface'`.

- [ ] **Step 3: Implement the matcher**

Replace the contents of `crates/deluge-bsp/src/usb/host/midi.rs` (above the test module) with:

```rust
//! USB **host**-side MIDI 1.0 class driver.
//!
//! Matches class-compliant USB-MIDI 1.0 devices (Audio class, MIDIStreaming
//! subclass, alt setting 0) and exchanges 4-byte USB-MIDI event packets over
//! bulk endpoints.
//!
//! MIDI 2.0 / UMP is **not** supported here — see the device-side
//! [`crate::usb::classes::midi`] for that.

use embassy_usb_driver::host::{pipe, HostError, UsbHostAllocator, UsbPipe};
use embassy_usb_driver::{Direction, EndpointType};
use embassy_usb_host::descriptor::{ConfigurationDescriptor, EndpointDescriptor};

/// USB Audio class code (USB-MIDI lives under the Audio class).
const USB_CLASS_AUDIO: u8 = 0x01;
/// MIDIStreaming subclass code.
const USB_SUBCLASS_MIDI_STREAMING: u8 = 0x03;

/// A matched MIDIStreaming interface and its bulk endpoints.
///
/// At least one of `ep_in` / `ep_out` is always `Some` — a MIDIStreaming
/// interface with no bulk endpoints is unusable and does not match.
#[derive(Clone, Copy, Debug)]
pub struct MidiInterface {
    /// `bInterfaceNumber` of the MIDIStreaming interface.
    pub interface_number: u8,
    /// Bulk IN endpoint (device → host). `None` for a sink-only device.
    pub ep_in: Option<EndpointDescriptor>,
    /// Bulk OUT endpoint (host → device). `None` for a source-only device.
    pub ep_out: Option<EndpointDescriptor>,
}

/// Find the first usable MIDIStreaming interface in `cfg`.
///
/// Matches on the *interface* class/subclass rather than the device class, so
/// composite devices (e.g. an audio interface with a MIDI port) are handled.
/// Returns `None` if there is no MIDIStreaming interface, or if it declares no
/// bulk endpoints.
pub fn find_midi_interface(cfg: &ConfigurationDescriptor<'_>) -> Option<MidiInterface> {
    for iface in cfg.iter_interface() {
        if iface.interface_class != USB_CLASS_AUDIO
            || iface.interface_subclass != USB_SUBCLASS_MIDI_STREAMING
        {
            continue;
        }

        let mut ep_in = None;
        let mut ep_out = None;
        for ep in iface.iter_endpoints() {
            if ep.ep_type() != EndpointType::Bulk {
                continue;
            }
            match ep.ep_dir() {
                Direction::In if ep_in.is_none() => ep_in = Some(ep),
                Direction::Out if ep_out.is_none() => ep_out = Some(ep),
                _ => {}
            }
        }

        if ep_in.is_some() || ep_out.is_some() {
            return Some(MidiInterface {
                interface_number: iface.interface_number,
                ep_in,
                ep_out,
            });
        }
    }
    None
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::midi`
Expected: PASS — 6 passed.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/midi.rs
git commit -m "feat(deluge-bsp): match USB MIDIStreaming interfaces

Matches on interface class/subclass so composite audio+MIDI devices work, and
accepts source-only, sink-only, and bidirectional endpoint layouts."
```

---

## Task 4: Packet decode

A bulk IN read returns up to 64 bytes = up to 16 USB-MIDI event packets, zero-padded. Split it into 4-byte packets and drop the padding.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/midi.rs`

**Interfaces:**
- Consumes: nothing.
- Produces: `pub fn decode_packets(buf: &[u8]) -> impl Iterator<Item = [u8; 4]> + '_`

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `crates/deluge-bsp/src/usb/host/midi.rs`:

```rust
    #[test]
    fn decodes_a_single_note_on() {
        // Cable 0, CIN 0x9 (Note On), channel 0, note 60, velocity 100.
        let buf = [0x09, 0x90, 60, 100];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0], [0x09, 0x90, 60, 100]);
    }

    #[test]
    fn decodes_multiple_packets() {
        let buf = [0x09, 0x90, 60, 100, 0x08, 0x80, 60, 0];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 2);
        assert_eq!(out[1], [0x08, 0x80, 60, 0]);
    }

    #[test]
    fn skips_zero_padding() {
        // Devices zero-pad the rest of the 64-byte buffer. CIN 0 is reserved,
        // so a zero header byte is never a real packet.
        let mut buf = [0u8; 64];
        buf[..4].copy_from_slice(&[0x09, 0x90, 60, 100]);
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 1, "padding must not become packets");
    }

    #[test]
    fn preserves_cable_number() {
        // Cable 2, CIN 0xB (Control Change).
        let buf = [0x2B, 0xB0, 7, 127];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out[0][0] >> 4, 2, "cable number must survive");
    }

    #[test]
    fn passes_sysex_packets_through_untouched() {
        // CIN 0x4 = SysEx start/continue, CIN 0x5 = SysEx end with 1 byte.
        let buf = [0x04, 0xF0, 0x7E, 0x00, 0x05, 0xF7, 0, 0];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0], [0x04, 0xF0, 0x7E, 0x00]);
        assert_eq!(out[1], [0x05, 0xF7, 0, 0]);
    }

    #[test]
    fn ignores_a_trailing_partial_packet() {
        let buf = [0x09, 0x90, 60, 100, 0x08, 0x80];
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&buf).collect();
        assert_eq!(out.len(), 1, "a 2-byte tail is not a packet");
    }

    #[test]
    fn empty_buffer_yields_nothing() {
        let out: heapless::Vec<[u8; 4], 16> = decode_packets(&[]).collect();
        assert!(out.is_empty());
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::midi::tests::decode`
Expected: FAIL — `cannot find function 'decode_packets'`.

- [ ] **Step 3: Implement the decoder**

Add to `crates/deluge-bsp/src/usb/host/midi.rs`:

```rust
/// Split a bulk IN buffer into 4-byte USB-MIDI event packets.
///
/// USB-MIDI 1.0 (§4) frames every event as 4 bytes: byte 0 is
/// `(cable_number << 4) | code_index_number`, bytes 1–3 are the MIDI data.
/// Devices zero-pad the remainder of the buffer; CIN 0 is reserved, so a zero
/// header byte marks padding rather than an event.
///
/// Packets are yielded verbatim — cable number, CIN, and SysEx framing
/// (CIN 0x4–0x7) are all preserved for the caller to interpret.
pub fn decode_packets(buf: &[u8]) -> impl Iterator<Item = [u8; 4]> + '_ {
    buf.chunks_exact(4)
        .filter(|c| c[0] != 0)
        .map(|c| [c[0], c[1], c[2], c[3]])
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::midi`
Expected: PASS — 13 passed (6 from Task 3 + 7 here).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/midi.rs
git commit -m "feat(deluge-bsp): decode USB-MIDI event packets from bulk reads"
```

---

## Task 5: `MidiHost` — claim pipes and pump packets

Bind a matched interface to real pipes and expose RX/TX. This is the class driver proper.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/midi.rs`

**Interfaces:**
- Consumes: `find_midi_interface`, `decode_packets` (Tasks 3–4); `mock::MockAlloc` (Task 2).
- Produces:
  - `pub struct MidiHost<'d, A: UsbHostAllocator<'d>>`
  - `pub fn MidiHost::try_register(alloc: &A, addr: u8, split: Option<SplitInfo>, cfg: &ConfigurationDescriptor<'_>) -> Result<Self, MidiError>`
  - `pub async fn MidiHost::read(&mut self, out: &mut [[u8; 4]]) -> Result<usize, MidiError>`
  - `pub async fn MidiHost::write(&mut self, packets: &[[u8; 4]]) -> Result<(), MidiError>`
  - `pub fn MidiHost::can_send(&self) -> bool`
  - `pub enum MidiError { NoMidiInterface, NoPipe(HostError), Transfer(PipeError), NotSupported }`

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `crates/deluge-bsp/src/usb/host/midi.rs`:

```rust
    use crate::usb::host::mock::{MockAlloc, MockState};

    fn bidir_cfg() -> heapless::Vec<u8, 128> {
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(2)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        body.extend_from_slice(&bulk_ep(0x01)).unwrap();
        cfg_bytes(&body)
    }

    #[test]
    fn registers_bidirectional_device_and_claims_both_pipes() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = MidiHost::try_register(&alloc, 1, None, &cfg).expect("register");
        assert!(host.can_send());
        assert_eq!(state.allocs.borrow().len(), 2, "one IN pipe + one OUT pipe");
    }

    #[test]
    fn sink_only_device_registers_without_an_in_pipe() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x02)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let host = MidiHost::try_register(&alloc, 1, None, &cfg).expect("synth must register");
        assert!(host.can_send());
        assert!(!host.can_receive());
        assert_eq!(state.allocs.borrow().len(), 1, "OUT pipe only");
    }

    #[test]
    fn rejects_when_rx_pipe_is_unavailable() {
        let state = MockState::leak();
        *state.alloc_err.borrow_mut() = Some(HostError::OutOfPipes);
        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        let err = MidiHost::try_register(&alloc, 1, None, &cfg);
        assert!(matches!(err, Err(MidiError::NoPipe(_))));
    }

    #[test]
    fn rejects_non_midi_device() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&[9, 0x04, 0, 0, 1, 0x03, 0x01, 1, 0]).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();

        assert!(matches!(
            MidiHost::try_register(&alloc, 1, None, &cfg),
            Err(MidiError::NoMidiInterface)
        ));
    }

    #[test]
    fn read_yields_scripted_packets() {
        let state = MockState::leak();
        let mut data = heapless::Vec::<u8, 64>::new();
        data.extend_from_slice(&[0x09, 0x90, 60, 100, 0x08, 0x80, 60, 0]).unwrap();
        state.script.borrow_mut().reads.push(data).unwrap();

        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = MidiHost::try_register(&alloc, 1, None, &cfg).unwrap();

        let mut out = [[0u8; 4]; 16];
        let n = embassy_futures::block_on(host.read(&mut out)).expect("read");
        assert_eq!(n, 2);
        assert_eq!(out[0], [0x09, 0x90, 60, 100]);
        assert_eq!(out[1], [0x08, 0x80, 60, 0]);
    }

    #[test]
    fn write_sends_packet_bytes() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let raw = bidir_cfg();
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = MidiHost::try_register(&alloc, 1, None, &cfg).unwrap();

        embassy_futures::block_on(host.write(&[[0x09, 0x90, 64, 127]])).expect("write");
        assert_eq!(&state.log.borrow().sent[..], &[0x09, 0x90, 64, 127]);
    }

    #[test]
    fn write_to_source_only_device_is_not_supported() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut body = heapless::Vec::<u8, 128>::new();
        body.extend_from_slice(&ms_iface(1)).unwrap();
        body.extend_from_slice(&bulk_ep(0x81)).unwrap();
        let raw = cfg_bytes(&body);
        let cfg = ConfigurationDescriptor::try_from_slice(&raw).unwrap();
        let mut host = MidiHost::try_register(&alloc, 1, None, &cfg).unwrap();

        assert!(!host.can_send());
        assert!(matches!(
            embassy_futures::block_on(host.write(&[[0x09, 0x90, 64, 127]])),
            Err(MidiError::NotSupported)
        ));
    }
```

> `embassy-futures` must be added to the `cfg(not(target_os = "none"))`
> dev-dependencies in `crates/deluge-bsp/Cargo.toml` for `block_on`:
> `embassy-futures = { workspace = true }`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::midi`
Expected: FAIL — `cannot find type 'MidiHost'`.

- [ ] **Step 3: Implement `MidiHost`**

Add to `crates/deluge-bsp/src/usb/host/midi.rs`:

```rust
use embassy_usb_driver::host::{PipeError, SplitInfo};

/// Maximum bytes read from a bulk IN endpoint in one transfer.
///
/// 64 bytes = 16 USB-MIDI event packets, and matches the full-speed bulk MPS.
const RX_BUF_LEN: usize = 64;

/// Errors from the host MIDI class driver.
#[derive(Debug)]
pub enum MidiError {
    /// The device has no usable MIDIStreaming interface.
    NoMidiInterface,
    /// A required pipe could not be allocated (the device ceiling).
    NoPipe(HostError),
    /// A transfer failed.
    Transfer(PipeError),
    /// The operation is not supported by this device (e.g. TX on a
    /// source-only controller).
    NotSupported,
}

impl From<PipeError> for MidiError {
    fn from(e: PipeError) -> Self {
        Self::Transfer(e)
    }
}

/// A hosted USB MIDI 1.0 device.
///
/// Generic over the allocator so it can be unit-tested against a mock — see
/// [`super::mock`].
pub struct MidiHost<'d, A: UsbHostAllocator<'d>> {
    ep_in: Option<A::Pipe<pipe::Bulk, pipe::In>>,
    ep_out: Option<A::Pipe<pipe::Bulk, pipe::Out>>,
    _p: core::marker::PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> MidiHost<'d, A> {
    /// Claim pipes for an enumerated device's MIDIStreaming interface.
    ///
    /// Claims whichever directions the device declares. A sink-only synth
    /// (bulk OUT only) and a source-only controller (bulk IN only) are both
    /// supported. Fails only if there is no MIDIStreaming interface, or if a
    /// declared endpoint's pipe cannot be allocated.
    ///
    /// MIDI 1.0 devices use alt setting 0, so no `SET_INTERFACE` is issued.
    pub fn try_register(
        alloc: &A,
        addr: u8,
        split: Option<SplitInfo>,
        cfg: &ConfigurationDescriptor<'_>,
    ) -> Result<Self, MidiError> {
        let iface = find_midi_interface(cfg).ok_or(MidiError::NoMidiInterface)?;

        let ep_in = match iface.ep_in {
            Some(ep) => Some(
                alloc
                    .alloc_pipe::<pipe::Bulk, pipe::In>(addr, &ep.into(), split)
                    .map_err(MidiError::NoPipe)?,
            ),
            None => None,
        };
        let ep_out = match iface.ep_out {
            Some(ep) => Some(
                alloc
                    .alloc_pipe::<pipe::Bulk, pipe::Out>(addr, &ep.into(), split)
                    .map_err(MidiError::NoPipe)?,
            ),
            None => None,
        };

        Ok(Self {
            ep_in,
            ep_out,
            _p: core::marker::PhantomData,
        })
    }

    /// True if this device accepts MIDI from the host.
    pub fn can_send(&self) -> bool {
        self.ep_out.is_some()
    }

    /// True if this device sends MIDI to the host.
    pub fn can_receive(&self) -> bool {
        self.ep_in.is_some()
    }

    /// Read up to `out.len()` event packets. Returns how many were decoded.
    ///
    /// Awaits the next bulk IN transfer, so this pends until the device sends
    /// something. Returns [`MidiError::NotSupported`] for a sink-only device.
    pub async fn read(&mut self, out: &mut [[u8; 4]]) -> Result<usize, MidiError> {
        let ep = self.ep_in.as_mut().ok_or(MidiError::NotSupported)?;
        let mut buf = [0u8; RX_BUF_LEN];
        let n = ep.request_in(&mut buf).await?;
        let mut count = 0;
        for pkt in decode_packets(&buf[..n]) {
            if count == out.len() {
                break;
            }
            out[count] = pkt;
            count += 1;
        }
        Ok(count)
    }

    /// Send event packets to the device.
    ///
    /// Returns [`MidiError::NotSupported`] for a source-only device.
    pub async fn write(&mut self, packets: &[[u8; 4]]) -> Result<(), MidiError> {
        let ep = self.ep_out.as_mut().ok_or(MidiError::NotSupported)?;
        let mut buf = [0u8; RX_BUF_LEN];
        let n = packets.len().min(RX_BUF_LEN / 4);
        for (i, p) in packets.iter().take(n).enumerate() {
            buf[i * 4..i * 4 + 4].copy_from_slice(p);
        }
        ep.request_out(&buf[..n * 4], false).await?;
        Ok(())
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::midi`
Expected: PASS — 20 passed.

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/Cargo.toml crates/deluge-bsp/src/usb/host/midi.rs
git commit -m "feat(deluge-bsp): MidiHost class driver with RX/TX over bulk pipes

Claims whichever directions the device declares, so sink-only synths and
source-only controllers both work."
```

---

## Task 6: `DeviceId`, merged channel, and the supervisor

Wire enumeration to the class driver. This is the first task that runs on hardware.

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/mod.rs`
- Test: `crates/deluge-bsp/src/usb/host/mod.rs`

**Interfaces:**
- Consumes: `MidiHost`, `MidiError` (Task 5).
- Produces:
  - `pub struct DeviceId` — `Copy + Eq`, with `pub const fn slot(&self) -> u8` and `pub const fn generation(&self) -> u16`
  - `pub const MAX_MIDI_DEVICES: usize` (= 4)
  - `pub fn try_recv_midi() -> Option<(DeviceId, [u8; 4])>`
  - `pub async fn recv_midi() -> (DeviceId, [u8; 4])`
  - `pub struct MidiHandle` with `pub async fn send(&self, packet: [u8; 4])`, `pub fn try_send(&self, packet: [u8; 4]) -> Result<(), [u8; 4]>`, `pub const fn id(&self) -> DeviceId`
  - `pub const fn midi_handle(id: DeviceId) -> MidiHandle`
  - `#[embassy_executor::task] pub async fn usb_host_supervisor(driver: Rusb1HostDriver, spawner: Spawner)`

> **`pool_size = MAX_MIDI_DEVICES`**: `#[embassy_executor::task]` accepts a const
> expression for `pool_size` in embassy-executor 0.10. If the macro rejects it,
> substitute the literal `4` and leave a comment tying it to `MAX_MIDI_DEVICES`.

- [ ] **Step 1: Write the failing test for `DeviceId` generation semantics**

Add to `crates/deluge-bsp/src/usb/host/mod.rs`:

```rust
#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::*;

    #[test]
    fn device_ids_in_the_same_slot_differ_across_generations() {
        // A USB address is freed and reused on hot-plug. If DeviceId were just
        // the address, a stale id would silently alias a different controller.
        let a = DeviceId::new(2, 7);
        let b = DeviceId::new(2, 8);
        assert_ne!(a, b, "same slot, later generation must not compare equal");
        assert_eq!(a.slot(), b.slot());
    }

    #[test]
    fn device_ids_are_equal_within_a_generation() {
        assert_eq!(DeviceId::new(3, 1), DeviceId::new(3, 1));
    }

    #[test]
    fn handle_try_send_routes_to_the_devices_own_queue() {
        let h = midi_handle(DeviceId::new(2, 1));
        h.try_send([0x09, 0x90, 60, 100]).expect("queue has space");

        assert_eq!(
            MIDI_TX[2].try_receive().ok(),
            Some([0x09, 0x90, 60, 100]),
            "packet must land in slot 2's queue"
        );
        assert!(
            MIDI_TX[3].try_receive().is_err(),
            "no other device's queue may be touched"
        );
    }

    #[test]
    fn handle_try_send_reports_a_full_queue_instead_of_blocking() {
        let h = midi_handle(DeviceId::new(4, 1));
        for _ in 0..MIDI_TX_DEPTH {
            h.try_send([0x0F, 0xF8, 0, 0]).expect("fills to capacity");
        }
        // The audio path must never block on USB backpressure.
        assert_eq!(h.try_send([0x0F, 0xF8, 0, 0]), Err([0x0F, 0xF8, 0, 0]));
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::tests`
Expected: FAIL — `cannot find type 'DeviceId'`.

- [ ] **Step 3: Implement `DeviceId` and the merged channel**

Add to `crates/deluge-bsp/src/usb/host/mod.rs`:

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Maximum concurrently hosted MIDI devices.
///
/// Bounded by the RUSB1 host pipe budget. RX pipes are dedicated per device
/// (USB is host-polled, so a receive must be armed to catch unsolicited MIDI),
/// while TX shares one pipe from Task 8 onward. With bulk pipe 1 reserved for
/// shared TX, bulk RX has pipes 2-5 — **4 devices**.
///
/// The original firmware reaches 6 by also servicing MIDI devices that use
/// *interrupt* endpoints on RX pipes 7-8 (`USB_CFG_HMIDI_INT_RECV_MIN/MAX`).
/// [`midi::find_midi_interface`] only claims bulk endpoints, so that last 2 is
/// not available here; interrupt-endpoint MIDI is a documented follow-up.
///
/// Before Task 8 lands, TX also consumes a dedicated pipe, so the practical
/// ceiling is lower still (~2 bidirectional devices).
pub const MAX_MIDI_DEVICES: usize = 4;

/// Depth of the merged RX channel, in event packets.
const MIDI_RX_DEPTH: usize = 64;

/// Identifies a hosted device.
///
/// Not the raw USB address: addresses are freed and reused on hot-plug, so a
/// bare address could alias a different device. The generation counter makes
/// stale ids detectable.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DeviceId {
    slot: u8,
    generation: u16,
}

impl DeviceId {
    pub(crate) const fn new(slot: u8, generation: u16) -> Self {
        Self { slot, generation }
    }

    /// Device slot index (`0..MAX_MIDI_DEVICES`).
    pub const fn slot(&self) -> u8 {
        self.slot
    }

    /// Generation counter — bumped each time this slot is reused.
    pub const fn generation(&self) -> u16 {
        self.generation
    }
}

/// Depth of each per-device TX channel, in event packets.
const MIDI_TX_DEPTH: usize = 16;

/// Number of device slots. Addresses run 1..=5 (`HCD_MAX_DEV` is 6), so index
/// by address directly and size for the widest address the HAL allows.
const MIDI_SLOTS: usize = 6;

/// Merged RX stream from every hosted MIDI device, tagged by source.
static MIDI_RX: Channel<CriticalSectionRawMutex, (DeviceId, [u8; 4]), MIDI_RX_DEPTH> =
    Channel::new();

/// Per-device TX queues, indexed by [`DeviceId::slot`].
///
/// `MidiHost` is owned by its device task and cannot be shared, so this channel
/// is the only route to a device's OUT endpoint.
static MIDI_TX: [Channel<CriticalSectionRawMutex, [u8; 4], MIDI_TX_DEPTH>; MIDI_SLOTS] =
    [const { Channel::new() }; MIDI_SLOTS];

/// Non-blocking read of the next hosted-MIDI packet, if any.
///
/// Mirrors the device-side idiom (`usb::classes::midi::try_recv_from_host`).
pub fn try_recv_midi() -> Option<(DeviceId, [u8; 4])> {
    MIDI_RX.try_receive().ok()
}

/// Await the next hosted-MIDI packet.
pub async fn recv_midi() -> (DeviceId, [u8; 4]) {
    MIDI_RX.receive().await
}

/// A send handle for one hosted device.
///
/// Obtained from [`midi_handle`]. Cheap and `Copy` — it is just an id.
#[derive(Clone, Copy, Debug)]
pub struct MidiHandle {
    id: DeviceId,
}

impl MidiHandle {
    /// The device this handle addresses.
    pub const fn id(&self) -> DeviceId {
        self.id
    }

    /// Queue a packet for the device, awaiting space if the queue is full.
    pub async fn send(&self, packet: [u8; 4]) {
        MIDI_TX[self.id.slot() as usize].send(packet).await;
    }

    /// Queue a packet, returning it if the queue is full.
    ///
    /// Prefer this on the audio path — never block audio on USB.
    pub fn try_send(&self, packet: [u8; 4]) -> Result<(), [u8; 4]> {
        MIDI_TX[self.id.slot() as usize]
            .try_send(packet)
            .map_err(|e| match e {
                embassy_sync::channel::TrySendError::Full(p) => p,
            })
    }
}

/// A send handle for `id`.
///
/// The handle does not verify the device is still attached: if it has detached,
/// packets queue and are drained when the slot is reused. Compare
/// [`DeviceId::generation`] against a freshly observed id to detect staleness.
pub const fn midi_handle(id: DeviceId) -> MidiHandle {
    MidiHandle { id }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib host::tests`
Expected: PASS — 4 passed.

- [ ] **Step 5: Add the per-device task and supervisor**

Add to `crates/deluge-bsp/src/usb/host/mod.rs`:

```rust
#[cfg(target_os = "none")]
mod runtime {
    use embassy_executor::Spawner;
    use embassy_futures::select::{select, Either};
    use embassy_usb_driver::host::DeviceEvent;
    use embassy_usb_host::descriptor::ConfigurationDescriptor;
    use embassy_usb_host::handler::BusRoute;
    use embassy_usb_host::{bus, BusHandle, BusState};
    use log::{error, info, warn};
    use rza1l_hal::usb::{Rusb1Allocator, Rusb1HostDriver};

    use super::{DeviceId, MAX_MIDI_DEVICES, MIDI_RX, MIDI_TX};
    use crate::usb::host::midi::MidiHost;

    /// Services one hosted MIDI device until it errors or detaches.
    ///
    /// Owns the pipes, so they drop — and the hardware pipes free — the moment
    /// this exits. That is the whole detach cleanup path.
    ///
    /// Reads and writes are serviced together via `select`: the device's TX
    /// channel is the only route to its OUT endpoint, since `MidiHost` lives
    /// here and cannot be shared.
    #[embassy_executor::task(pool_size = MAX_MIDI_DEVICES)]
    async fn midi_device_task(mut host: MidiHost<'static, Rusb1Allocator>, id: DeviceId) {
        let tx = &MIDI_TX[id.slot() as usize];
        let mut packets = [[0u8; 4]; 16];

        loop {
            if !host.can_receive() {
                // Sink-only device (a synth): only ever send.
                match tx.receive().await {
                    p => {
                        if let Err(e) = host.write(&[p]).await {
                            info!("usb_host: device {:?} write ended: {:?}", id, e);
                            return;
                        }
                    }
                }
                continue;
            }

            match select(host.read(&mut packets), tx.receive()).await {
                Either::First(Ok(n)) => {
                    for p in &packets[..n] {
                        // Drop rather than block: a stalled consumer must not
                        // stall USB.
                        let _ = MIDI_RX.try_send((id, *p));
                    }
                }
                Either::First(Err(e)) => {
                    info!("usb_host: device {:?} read ended: {:?}", id, e);
                    return;
                }
                Either::Second(p) => {
                    if let Err(e) = host.write(&[p]).await {
                        // A send error is normal when a device attaches or
                        // detaches from a hub mid-traffic — the original
                        // firmware logs and continues rather than dropping the
                        // device (midi_engine.cpp:128-135). Do the same.
                        warn!("usb_host: device {:?} write error: {:?}", id, e);
                    }
                }
            }
        }
    }

    /// Owns the bus: waits for attach, enumerates, binds a class driver.
    #[embassy_executor::task]
    pub async fn usb_host_supervisor(driver: Rusb1HostDriver, spawner: Spawner) {
        static BUS_STATE: BusState = BusState::new();
        let (mut controller, handle) = bus(driver, &BUS_STATE);
        let mut config_buf = [0u8; 512];
        let mut generation: u16 = 0;

        loop {
            let speed = controller.wait_for_connection().await;
            info!("usb_host: device connected ({:?})", speed);

            let (dev_info, _) = match handle
                .enumerate(BusRoute::Direct(speed), &mut config_buf)
                .await
            {
                Ok(v) => v,
                Err(e) => {
                    error!("usb_host: enumeration failed: {:?}", e);
                    continue;
                }
            };
            let addr = dev_info.device_address;

            let cfg = match ConfigurationDescriptor::try_from_slice(&config_buf) {
                Ok(c) => c,
                Err(e) => {
                    error!("usb_host: bad config descriptor: {:?}", e);
                    handle.free_address(addr);
                    continue;
                }
            };

            generation = generation.wrapping_add(1);
            let id = DeviceId::new(addr, generation);

            match MidiHost::try_register(&handle, addr, dev_info.split(), &cfg) {
                Ok(host) => {
                    info!(
                        "usb_host: MIDI device VID={:04x} PID={:04x} addr={}",
                        dev_info.device_desc.vendor_id, dev_info.device_desc.product_id, addr
                    );
                    if spawner.spawn(midi_device_task(host, id)).is_err() {
                        error!("usb_host: no free device task slot");
                        handle.free_address(addr);
                        continue;
                    }
                }
                Err(e) => {
                    // Free the address immediately so an unsupported device
                    // does not consume one of the scarce device slots.
                    warn!("usb_host: unsupported device: {:?}", e);
                    handle.free_address(addr);
                    continue;
                }
            }

            // Wait for detach before accepting another device.
            loop {
                if controller.wait_for_device_event().await == DeviceEvent::Disconnected {
                    info!("usb_host: device disconnected");
                    handle.free_address(addr);
                    break;
                }
            }
        }
    }
}

#[cfg(target_os = "none")]
pub use runtime::usb_host_supervisor;
```

> `embassy-executor` must be added to the `cfg(target_os = "none")`
> dependencies in `crates/deluge-bsp/Cargo.toml`:
> `embassy-executor = { workspace = true }`.
>
> `rza1l_hal::usb::Rusb1Allocator` must be `pub` — check and export it if not.

- [ ] **Step 6: Verify both targets build**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib`
Expected: PASS — 54 pre-existing + 25 new (1 mock + 20 midi + 4 host), 0 failed.
Run: `cargo build-fw -p demo-firmware`
Expected: builds clean.

- [ ] **Step 7: Hardware validation**

Replace the body of `firmwares/demo-firmware/src/tasks/usb_host.rs`'s spawn site to use `deluge_bsp::usb::host::usb_host_supervisor`, flash, and plug in a class-compliant controller.

Expected log output:
```
usb_host: device connected (Full)
usb_host: MIDI device VID=xxxx PID=xxxx addr=1
```
Then pressing a key on the controller should surface packets via `try_recv_midi()`.

- [ ] **Step 8: Commit**

```bash
git add crates/deluge-bsp/Cargo.toml crates/deluge-bsp/src/usb/host/mod.rs
git commit -m "feat(deluge-bsp): USB host supervisor + merged MIDI channel

Enumerates, binds MidiHost, and pumps packets into a source-tagged static
channel from pooled per-device tasks. DeviceId carries a generation counter so
stale ids cannot alias a reused USB address."
```

---

## Task 7: Hub support

**Files:**
- Modify: `crates/deluge-bsp/src/usb/host/mod.rs`

**Interfaces:**
- Consumes: `usb_host_supervisor`, `midi_device_task`, `DeviceId` (Task 6).
- Produces: `hub_task` (internal); the supervisor gains hub detection.

**API notes — verified against `embassy-usb-host-0.1.0/src/class/hub.rs`:**
- `HubHandler::try_register(bus: &BusHandle<'d, A>, enum_info: &EnumerationInfo) -> Result<Self, RegisterError>`
- `HubHandler::wait_for_event(&mut self) -> Result<HandlerEvent<HubEvent>, HostError>`
- `HubHandler::enumerate_port(&mut self, config_buffer: &mut [u8], port: u8, speed: Speed) -> Result<(EnumerationInfo, usize), EnumerationError>`
- `enum HubEvent { DeviceDetected { port: u8, speed: Speed }, DeviceRemoved { address: Option<NonZeroU8>, port: u8 } }`
- `enum HandlerEvent<T> { NoChange, HandlerDisconnected, HandlerEvent(T) }`

`enumerate_port` already performs the port reset, the USB 2.0 §7.1.7.5 delay,
and computes the correct `BusRoute`/`SplitInfo` (including parent-hub TT
handling). **Do not hand-roll any of that.** Because it takes `&mut self`, the
hub task must own both the handler and the enumeration — hub-attached devices
cannot be enumerated by the supervisor through a channel.

- [ ] **Step 1: Add the hub task**

In the `runtime` module of `crates/deluge-bsp/src/usb/host/mod.rs`, add:

```rust
    use embassy_usb_host::class::hub::{HubEvent, HubHandler};
    use embassy_usb_host::handler::HandlerEvent;

    /// Maximum hub ports serviced.
    ///
    /// The RUSB1 `DEVADDn.HUBPORT` field is 3 bits (TRM §28.3), so a device can
    /// only be addressed on hub ports 1-7.
    const MAX_HUB_PORTS: usize = 7;

    /// Owns a hub: services port changes and enumerates devices behind it.
    ///
    /// Enumeration lives here rather than in the supervisor because
    /// `enumerate_port` needs `&mut HubHandler`.
    #[embassy_executor::task]
    async fn hub_task(
        mut hub: HubHandler<'static, Rusb1Allocator, MAX_HUB_PORTS>,
        handle: BusHandle<'static, Rusb1Allocator>,
        spawner: Spawner,
    ) {
        let mut config_buf = [0u8; 512];
        let mut generation: u16 = 0;

        loop {
            let event = match hub.wait_for_event().await {
                Ok(HandlerEvent::HandlerEvent(e)) => e,
                Ok(HandlerEvent::NoChange) => continue,
                Ok(HandlerEvent::HandlerDisconnected) => {
                    info!("usb_host: hub disconnected");
                    return;
                }
                Err(e) => {
                    error!("usb_host: hub error: {:?}", e);
                    return;
                }
            };

            match event {
                HubEvent::DeviceDetected { port, speed } => {
                    info!("usb_host: hub port {} attached ({:?})", port, speed);
                    // Performs the port reset and picks the right BusRoute.
                    let (dev_info, _) = match hub.enumerate_port(&mut config_buf, port, speed).await
                    {
                        Ok(v) => v,
                        Err(e) => {
                            error!("usb_host: hub port {} enumeration failed: {:?}", port, e);
                            continue;
                        }
                    };
                    let addr = dev_info.device_address;

                    let cfg = match ConfigurationDescriptor::try_from_slice(&config_buf) {
                        Ok(c) => c,
                        Err(e) => {
                            error!("usb_host: hub port {} bad descriptor: {:?}", port, e);
                            handle.free_address(addr);
                            continue;
                        }
                    };

                    generation = generation.wrapping_add(1);
                    let id = DeviceId::new(addr, generation);

                    match MidiHost::try_register(&handle, addr, dev_info.split(), &cfg) {
                        Ok(host) => {
                            info!("usb_host: hub port {} -> MIDI device addr={}", port, addr);
                            if spawner.spawn(midi_device_task(host, id)).is_err() {
                                error!("usb_host: no free device task slot");
                                handle.free_address(addr);
                            }
                        }
                        Err(e) => {
                            warn!("usb_host: hub port {} unsupported: {:?}", port, e);
                            handle.free_address(addr);
                        }
                    }
                }
                HubEvent::DeviceRemoved { address, port } => {
                    info!("usb_host: hub port {} detached", port);
                    // The device task exits on its next failed read, dropping
                    // its pipes. Only the address needs reclaiming here.
                    if let Some(a) = address {
                        handle.free_address(a.get());
                    }
                }
            }
        }
    }
```

> `midi_device_task` must lose its `pub`-less privacy barrier only as far as the
> `runtime` module — it is already declared there, so no change is needed.
> Add `use embassy_usb_host::BusHandle;` to the module's imports.

- [ ] **Step 2: Detect hubs in the supervisor**

In `usb_host_supervisor`, before attempting `MidiHost::try_register`, try the hub
first — a hub is not a MIDI device and would otherwise be rejected and freed:

```rust
            match HubHandler::<Rusb1Allocator, MAX_HUB_PORTS>::try_register(&handle, &dev_info)
                .await
            {
                Ok(hub) => {
                    info!("usb_host: hub at addr={}", addr);
                    if spawner.spawn(hub_task(hub, handle.clone(), spawner)).is_err() {
                        error!("usb_host: could not spawn hub task");
                        handle.free_address(addr);
                    }
                    // The hub task owns everything behind it from here.
                    continue;
                }
                Err(RegisterError::NoSupportedInterface) => {
                    // Not a hub — fall through to the MIDI matcher below.
                }
                Err(e) => {
                    error!("usb_host: hub register failed: {:?}", e);
                    handle.free_address(addr);
                    continue;
                }
            }
```

Add `use embassy_usb_host::handler::RegisterError;` to the imports.

> `BusHandle` is `Clone` (it wraps the allocator plus a `&'static BusState`), so
> handing a clone to the hub task is correct and cheap.

- [ ] **Step 3: Verify the `continue` does not strand the root port**

The supervisor's existing "wait for detach" loop must still run for a hub, or a
hub unplug will never be noticed. Restructure so the hub path spawns its task and
then falls into the same detach wait as a direct device, rather than `continue`
skipping it.

- [ ] **Step 4: Verify both targets build**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib`
Expected: PASS, no regressions.
Run: `cargo build-fw -p demo-firmware`
Expected: builds clean.

- [ ] **Step 5: Hardware validation**

Plug a hub in with two class-compliant controllers.
Expected: both enumerate and both produce packets tagged with distinct `DeviceId`s.
Then hot-unplug one; the other must keep working.

Also verify a device plugged *directly* still works — the hub matcher runs first
now, and `RegisterError::NoSupportedInterface` must fall through to MIDI rather
than swallowing the device.

- [ ] **Step 6: Commit**

```bash
git add crates/deluge-bsp/src/usb/host/mod.rs
git commit -m "feat(deluge-bsp): hub support for hosted MIDI devices"
```

---

## Task 8: HAL OUT-pipe multiplexing

The payoff task: share one bulk TX pipe across all devices so RX pipes are the only per-device cost, lifting the ceiling from ~2 devices to 6. This mirrors the original firmware's `change_destination_of_send_pipe`.

**This is the riskiest change in the plan** — it lives in the transfer path, where a bug corrupts all USB traffic, not just MIDI. It is last so everything above is already proven.

**Files:**
- Modify: `crates/rza1l-hal/src/usb/host.rs`

**Interfaces:**
- Consumes: `dev_addr_supported` (Task 1).
- Produces: no public API change — `alloc_pipe` transparently returns a shared-pipe handle for bulk OUT.

- [ ] **Step 1: Read the reference implementation first**

Read, in order:
- `~/GitHub/DelugeFirmware-clean/src/deluge/io/midi/midi_engine.cpp:263-291` (`flushUSBMIDIToHostedDevice` — the retarget call site and the `sq` flip)
- `~/GitHub/DelugeFirmware-clean/src/RZA1/usb/userdef/r_usb_hmidi_config.h` (the pipe map)
- the definition of `change_destination_of_send_pipe` (grep the `src/RZA1/usb` tree)

Do not proceed until the toggle save/restore sequence is understood.

- [ ] **Step 2: Write the failing test for toggle bookkeeping**

The register writes cannot be unit-tested, but the *bookkeeping* can. Add to
the `tests` module in `crates/rza1l-hal/src/usb/host.rs`:

```rust
    #[test]
    fn tx_target_tracks_the_current_owner() {
        let mut t = TxPipeTarget::new();
        assert!(t.needs_retarget(1, 0x01), "first use must program the pipe");
        t.set_owner(1, 0x01);
        assert!(!t.needs_retarget(1, 0x01), "same owner needs no retarget");
        assert!(t.needs_retarget(2, 0x01), "different device needs retarget");
        assert!(t.needs_retarget(1, 0x02), "different endpoint needs retarget");
    }

    #[test]
    fn tx_target_saves_and_restores_toggle_per_owner() {
        let mut t = TxPipeTarget::new();
        t.set_owner(1, 0x01);
        t.save_toggle(true); // device 1 advanced to DATA1
        t.set_owner(2, 0x01);
        t.save_toggle(false);
        assert_eq!(
            t.toggle_for(1, 0x01),
            Some(true),
            "device 1's toggle must survive device 2 borrowing the pipe"
        );
        assert_eq!(t.toggle_for(2, 0x01), Some(false));
    }

    #[test]
    fn tx_target_forgets_a_detached_device() {
        let mut t = TxPipeTarget::new();
        t.set_owner(1, 0x01);
        t.save_toggle(true);
        t.forget(1);
        assert_eq!(t.toggle_for(1, 0x01), None, "detach must clear saved toggle");
        assert!(t.needs_retarget(1, 0x01), "a reused address must reprogram");
    }
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib tx_target`
Expected: FAIL — `cannot find type 'TxPipeTarget'`.

- [ ] **Step 4: Implement `TxPipeTarget`**

Add to `crates/rza1l-hal/src/usb/host.rs`:

```rust
/// Tracks which (device, endpoint) currently owns a shared TX pipe, and each
/// borrower's data toggle.
///
/// The RUSB1 data toggle is per-*pipe* hardware state (PIPECTR SQMON/SQSET/
/// SQCLR), so retargeting a shared pipe to another endpoint clobbers the
/// previous owner's sequence. Mirrors `ConnectedUSBMIDIDevice::sq` in the
/// original firmware.
pub(crate) struct TxPipeTarget {
    /// Current owner, `(dev_addr, ep_addr)`. `None` = unprogrammed.
    owner: Option<(u8, u8)>,
    /// Saved toggle per `(dev_addr, ep_addr)`; `true` = DATA1.
    toggles: [Option<((u8, u8), bool)>; HCD_MAX_DEV],
}

impl TxPipeTarget {
    pub(crate) const fn new() -> Self {
        Self {
            owner: None,
            toggles: [None; HCD_MAX_DEV],
        }
    }

    /// True if the pipe must be reprogrammed before talking to this endpoint.
    pub(crate) fn needs_retarget(&self, dev_addr: u8, ep_addr: u8) -> bool {
        self.owner != Some((dev_addr, ep_addr))
    }

    /// Record that the pipe is now programmed for this endpoint.
    pub(crate) fn set_owner(&mut self, dev_addr: u8, ep_addr: u8) {
        self.owner = Some((dev_addr, ep_addr));
    }

    /// Record the current owner's toggle before handing the pipe over.
    pub(crate) fn save_toggle(&mut self, toggle: bool) {
        let Some(key) = self.owner else { return };
        for slot in self.toggles.iter_mut() {
            match slot {
                Some((k, t)) if *k == key => {
                    *t = toggle;
                    return;
                }
                _ => {}
            }
        }
        for slot in self.toggles.iter_mut() {
            if slot.is_none() {
                *slot = Some((key, toggle));
                return;
            }
        }
    }

    /// The saved toggle for an endpoint, if it has used the pipe before.
    pub(crate) fn toggle_for(&self, dev_addr: u8, ep_addr: u8) -> Option<bool> {
        self.toggles
            .iter()
            .flatten()
            .find(|(k, _)| *k == (dev_addr, ep_addr))
            .map(|(_, t)| *t)
    }

    /// Forget an endpoint's toggle (on detach).
    pub(crate) fn forget(&mut self, dev_addr: u8) {
        for slot in self.toggles.iter_mut() {
            if matches!(slot, Some(((d, _), _)) if *d == dev_addr) {
                *slot = None;
            }
        }
        if matches!(self.owner, Some((d, _)) if d == dev_addr) {
            self.owner = None;
        }
    }
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib tx_target`
Expected: PASS — 2 passed.

- [ ] **Step 6: Wire the shared pipe into `alloc_pipe` and `request_out`**

1. Add `const TX_BULK_PIPE: usize = 1;` and reserve it in `HcdAlloc::alloc_pipe`
   so it is never handed out for RX (change the bulk range to `2..=5`).
2. In `Rusb1Allocator::alloc_pipe`, for `EndpointType::Bulk` + `Direction::Out`,
   skip `edpt_open` and return a `Rusb1Pipe` with `pipe: TX_BULK_PIPE as u8`.
3. In `Rusb1Pipe::request_out`, before transferring, take an async mutex over
   the TX pipe and, if `needs_retarget`, run the sequence:
   - `PID = NAK`, spin until `PBUSY == 0`
   - read `PIPECTR.SQMON`, `save_toggle(..)`
   - rewrite `PIPESEL` / `PIPECFG.EPNUM` / `PIPEMAXP.DEVSEL` (and `DEVADDn` if
     the device changed)
   - restore the incoming toggle with `SQSET` / `SQCLR`
   - `set_owner(..)`
4. **Hold the mutex until the transfer completes**, not just across the
   retarget. This is the hub race from the design spec §2.4: a hub transaction
   can otherwise retarget the pipe mid-transfer.
5. In `Rusb1Pipe::drop` for a shared TX pipe, do **not** free the hardware
   pipe; call `TxPipeTarget::forget(dev_addr)` instead.

- [ ] **Step 7: Verify both targets build and nothing regressed**

Run: `cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib`
Expected: PASS.
Run: `cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib`
Expected: PASS.
Run: `cargo build-fw -p demo-firmware`
Expected: builds clean.

- [ ] **Step 8: Hardware validation — the real test**

1. One bidirectional controller: send and receive simultaneously. No dropped or
   duplicated messages (toggle handling is correct).
2. Two bidirectional devices on a hub, both sending and receiving at once.
   Sustained traffic for several minutes.
3. Hot-unplug one mid-send; the other must keep working and the lock must not
   strand.
4. Confirm >2 bidirectional devices now attach — the point of the task.

> Symptom of a toggle bug: dropped or duplicated MIDI under multi-device load,
> not a clean failure. Watch for stuck notes.

- [ ] **Step 9: Commit**

```bash
git add crates/rza1l-hal/src/usb/host.rs
git commit -m "feat(rza1l-hal): multiplex bulk OUT pipes across devices

USB is host-polled, so RX must be armed per device but TX can be serialized.
Share one bulk TX pipe, retargeting it per transfer with software data-toggle
save/restore, mirroring the original Deluge firmware's
change_destination_of_send_pipe. Lifts the hosted-device ceiling from ~2 to 6."
```

---

## Verification

**Automated:**
```bash
cargo test --target armv7-unknown-linux-gnueabihf -p rza1l-hal --lib
cargo test --target armv7-unknown-linux-gnueabihf -p deluge-bsp --lib
cargo build-fw -p demo-firmware
./tools/test.sh          # full suite, both buckets
```

**On hardware** (nothing below is provable in CI):
1. Class-compliant controller → notes reach `try_recv_midi()`.
2. Sink-only USB synth → the Deluge can send to it; it registers without an RX pipe.
3. Bidirectional keyboard-synth → simultaneous send + receive, no stuck notes.
4. Hub with 2+ controllers → distinct `DeviceId`s, both work.
5. Hot-plug churn during sustained TX → no stranded lock, no wedged pipe.
6. >2 bidirectional devices attach after Task 8.
