//! Mock [`UsbHostAllocator`] / [`UsbPipe`] for unit tests.
//!
//! Lets the generic class drivers be exercised with scripted pipe responses,
//! with no hardware and no `rza1l-hal` register access.

// This module only compiles under `cfg(all(test, not(target_os = "none")))`,
// where std is available — see the crate's QEMU bucket in tools/test.sh.
extern crate std;

use core::cell::RefCell;
use std::boxed::Box;

use embassy_usb_driver::EndpointInfo;
use embassy_usb_driver::host::{
    HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator, UsbPipe, pipe,
};
use heapless::Vec;

/// What a [`MockPipe`] should do on successive `request_in` calls.
#[derive(Clone, Default)]
pub struct Script {
    /// Bytes returned by successive `request_in` calls, in order.
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

/// Shared mock state. Obtain a `&'static` one via [`MockState::leak`].
pub struct MockState {
    pub script: RefCell<Script>,
    pub log: RefCell<PipeLog>,
    pub allocs: RefCell<Vec<AllocRecord, 8>>,
    /// When set, `alloc_pipe` fails with this error (simulates exhaustion).
    pub alloc_err: RefCell<Option<HostError>>,
    /// Responses returned by successive `control_in` calls (front popped).
    pub control_reads: RefCell<Vec<Vec<u8, 64>, 8>>,
    /// Every SETUP packet seen by `control_in` / `control_out`, in order.
    pub setups: RefCell<Vec<[u8; 8], 16>>,
    /// Concatenated data-stage bytes from `control_out`.
    pub control_out_data: RefCell<Vec<u8, 256>>,
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
            control_reads: RefCell::new(Vec::new()),
            setups: RefCell::new(Vec::new()),
            control_out_data: RefCell::new(Vec::new()),
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

impl Default for MockState {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone)]
pub struct MockAlloc {
    inner: &'static MockState,
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
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        self.inner.setups.borrow_mut().push(*setup).ok();
        let mut reads = self.inner.control_reads.borrow_mut();
        if reads.is_empty() {
            return Ok(0);
        }
        let data = reads.remove(0);
        let n = data.len().min(buf.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(n)
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        self.inner.setups.borrow_mut().push(*setup).ok();
        self.inner
            .control_out_data
            .borrow_mut()
            .extend_from_slice(buf)
            .map_err(|_| PipeError::BufferOverflow)?;
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
        log.sent
            .extend_from_slice(buf)
            .map_err(|_| PipeError::BufferOverflow)
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

#[cfg(all(test, not(target_os = "none")))]
mod control_mock_tests {
    use super::*;
    use embassy_usb_driver::host::{pipe, UsbHostAllocator, UsbPipe};
    use embassy_usb_driver::EndpointType;

    #[test]
    fn control_in_pops_scripted_response_and_logs_setup() {
        let state = MockState::leak();
        state
            .control_reads
            .borrow_mut()
            .push(Vec::from_slice(&[0x44, 0xAC, 0x00, 0x00]).unwrap())
            .unwrap();
        let alloc = MockAlloc::new(state);
        let mut pipe = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(
                1,
                &EndpointInfo { addr: 0u8.into(), ep_type: EndpointType::Control, max_packet_size: 64, interval_ms: 0 },
                None,
            )
            .unwrap();

        let setup = [0xA1, 0x01, 0x00, 0x01, 0x00, 0x00, 0x04, 0x00];
        let mut buf = [0u8; 4];
        let n = embassy_futures::block_on(pipe.control_in(&setup, &mut buf)).unwrap();
        assert_eq!(n, 4);
        assert_eq!(u32::from_le_bytes(buf), 44_100);
        assert_eq!(state.setups.borrow()[0], setup);
    }

    #[test]
    fn control_out_records_setup_and_data() {
        let state = MockState::leak();
        let alloc = MockAlloc::new(state);
        let mut pipe = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(
                1,
                &EndpointInfo { addr: 0u8.into(), ep_type: EndpointType::Control, max_packet_size: 64, interval_ms: 0 },
                None,
            )
            .unwrap();
        let setup = [0x21, 0x01, 0x00, 0x01, 0x00, 0x01, 0x04, 0x00];
        embassy_futures::block_on(pipe.control_out(&setup, &44_100u32.to_le_bytes())).unwrap();
        assert_eq!(state.setups.borrow()[0], setup);
        assert_eq!(&state.control_out_data.borrow()[..], &44_100u32.to_le_bytes());
    }
}
