use std::os::raw::c_void;
use std::ptr::NonNull;

/// The boxed audio callback. Double-boxed (see `Deluge::audio_cb`) so the
/// pointer handed to C is a heap address independent of where `Deluge` lives —
/// moving `Deluge` after `audio_start` must not dangle the RT thread's `ctx`.
type BoxedCb = Box<dyn FnMut(&[[f32; 2]], &mut [[f32; 2]]) + Send>;

/// The boxed input callback. Double-boxed for the same reason as `BoxedCb`
/// (see `Deluge::input_cb`): the pointer handed to C must be a heap address
/// independent of where `Deluge` lives, since the input thread runs
/// concurrently with any move of `Deluge`.
type BoxedInputCb = Box<dyn FnMut(Event) + Send>;

/// The boxed USB hotplug callback. Double-boxed for the same reason as
/// `BoxedCb`/`BoxedInputCb` (see `Deluge::usb_cb`).
type BoxedUsbCb = Box<dyn FnMut(UsbEvent) + Send + 'static>;

/// A safe mirror of `deluge_event`. Field names/widths match exactly.
#[derive(Clone, Copy, Debug)]
pub struct Event {
    pub kind: u8,
    pub id: u8,
    pub value: i16,
    pub x: u16,
    pub y: u16,
    pub pressure: u16,
}

/// A safe mirror of `deluge_usb_role`. Represents the device USB role.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UsbRole {
    Peripheral,
    Host,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbHotplug {
    Added,
    Removed,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbClass {
    Midi,
    Audio,
    Input,
    Storage,
    Other,
}

/// A USB hotplug event delivered to a `usb_watch` callback.
#[derive(Debug, Clone)]
pub struct UsbEvent {
    pub action: UsbHotplug,
    pub class: UsbClass,
    pub devnode: String,
}

/// A snapshot of the UAC2 gadget card's capabilities (from `usb_audio_probe`).
#[derive(Debug, Clone, Copy)]
pub struct UsbAudioInfo {
    pub playback_channels: u32,
    pub capture_channels: u32,
    pub rate_min: u32,
    pub rate_max: u32,
    pub formats: i32,
}

#[derive(Debug)]
pub struct Error(pub i32);
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let s = unsafe { std::ffi::CStr::from_ptr(deluge_sys::deluge_strerror(self.0)) };
        write!(f, "{}", s.to_string_lossy())
    }
}
impl std::error::Error for Error {}

pub struct Deluge {
    raw: NonNull<deluge_sys::deluge_t>,
    /// Heap pointer to the boxed callback, owned by this handle. `None` until
    /// `audio_start`. Reclaimed in `Drop` AFTER `deluge_close` joins the RT
    /// thread, so the thread never dereferences a freed `ctx`.
    audio_cb: Option<NonNull<BoxedCb>>,
    /// Heap pointer to the boxed input callback, owned by this handle. `None`
    /// until `input_start`. Reclaimed in `Drop` AFTER `deluge_close` (which
    /// stops the input thread via `deluge_input_free`), so the thread never
    /// dereferences a freed `ctx`. `input_stop` deliberately does NOT reclaim
    /// this box — only `Drop` does — so there is exactly one free site and no
    /// double-free risk between an explicit `input_stop()` call and `Drop`.
    input_cb: Option<NonNull<BoxedInputCb>>,
    /// Heap pointer to the boxed USB hotplug callback, owned by this handle.
    /// `None` until `usb_watch`. Reclaimed by `usb_unwatch` or `Drop` (exactly
    /// one of the two, since each clears the field via `take()`), mirroring
    /// `input_cb`. On the `Drop` path, `deluge_close` -> `deluge_usb_free` ->
    /// `deluge_usb_unwatch` JOINS the hotplug thread before the box is
    /// reclaimed here, so the trampoline never dereferences a freed `ctx`.
    usb_cb: Option<NonNull<BoxedUsbCb>>,
}

// SAFETY: every field is an opaque handle address (the `deluge_t*` context, or
// a `Box::into_raw` callback pointer reclaimed only in `Drop`/`*_stop`/`*_unwatch`).
// None of them are ever dereferenced as a `NonNull<T>` from Rust code directly —
// `raw` is passed straight back into the (thread-safe) `deluge_sys` C API, and
// the callback pointers are only read by the trampolines, which the C library
// already invokes from its own RT/input/hotplug threads (distinct from whatever
// thread called `audio_start`/`input_start`/`usb_watch`), hence those callbacks
// are themselves bounded `Send`. Moving `Deluge` to another thread — e.g. to
// park it behind a process-wide `Mutex`, as the SDK's Linux backend does — just
// transfers ownership of these addresses; nothing is accessed concurrently.
unsafe impl Send for Deluge {}

extern "C" fn trampoline(inp: *const f32, out: *mut f32, n: i32, ctx: *mut c_void) {
    // ctx is the *mut BoxedCb produced by Box::into_raw in audio_start.
    let cb = unsafe { &mut *(ctx as *mut BoxedCb) };
    let len = if n < 0 { 0 } else { n as usize }; // each element is one stereo frame
    let i = unsafe { std::slice::from_raw_parts(inp as *const [f32; 2], len) };
    let o = unsafe { std::slice::from_raw_parts_mut(out as *mut [f32; 2], len) };
    cb(i, o);
}

extern "C" fn input_trampoline(ev: *const deluge_sys::deluge_event, ctx: *mut c_void) {
    // ctx is the *mut BoxedInputCb produced by Box::into_raw in input_start.
    let cb = unsafe { &mut *(ctx as *mut BoxedInputCb) };
    // ev is non-null and points at a valid deluge_event for the duration of
    // this call (owned by the SDK's input thread, per deluge_input_cb contract).
    let e = unsafe { *ev };
    cb(Event {
        kind: e.kind,
        id: e.id,
        value: e.value,
        x: e.x,
        y: e.y,
        pressure: e.pressure,
    });
}

extern "C" fn usb_trampoline(
    ev: deluge_sys::deluge_usb_hotplug,
    cls: deluge_sys::deluge_usb_class,
    devnode: *const std::os::raw::c_char,
    ctx: *mut c_void,
) {
    let cb = unsafe { &mut *(ctx as *mut BoxedUsbCb) };
    let action = if ev == deluge_sys::deluge_usb_hotplug_DELUGE_USB_REMOVED {
        UsbHotplug::Removed
    } else {
        UsbHotplug::Added
    };
    let class = match cls {
        deluge_sys::deluge_usb_class_DELUGE_USB_CLASS_MIDI => UsbClass::Midi,
        deluge_sys::deluge_usb_class_DELUGE_USB_CLASS_AUDIO => UsbClass::Audio,
        deluge_sys::deluge_usb_class_DELUGE_USB_CLASS_INPUT => UsbClass::Input,
        deluge_sys::deluge_usb_class_DELUGE_USB_CLASS_STORAGE => UsbClass::Storage,
        _ => UsbClass::Other,
    };
    let devnode = if devnode.is_null() {
        String::new()
    } else {
        unsafe { std::ffi::CStr::from_ptr(devnode) }
            .to_string_lossy()
            .into_owned()
    };
    (cb)(UsbEvent {
        action,
        class,
        devnode,
    });
}

impl Deluge {
    pub fn open() -> Result<Self, Error> {
        let p = unsafe { deluge_sys::deluge_open() };
        NonNull::new(p)
            .map(|raw| Deluge {
                raw,
                audio_cb: None,
                input_cb: None,
                usb_cb: None,
            })
            .ok_or(Error(-1))
    }

    pub fn audio_start(
        &mut self,
        cb: impl FnMut(&[[f32; 2]], &mut [[f32; 2]]) + Send + 'static,
    ) -> Result<(), Error> {
        if self.audio_cb.is_some() {
            return Err(Error(-4)); // DELUGE_ERR_STATE: already running
        }
        let boxed: BoxedCb = Box::new(cb);
        // Double-box: `ctx` is a heap address that survives moves of `Deluge`.
        let ctx: *mut BoxedCb = Box::into_raw(Box::new(boxed));
        let a = unsafe { deluge_sys::deluge_audio(self.raw.as_ptr()) };
        let rc = unsafe { deluge_sys::deluge_audio_start(a, Some(trampoline), ctx as *mut c_void) };
        if rc == 0 {
            self.audio_cb = Some(unsafe { NonNull::new_unchecked(ctx) });
            Ok(())
        } else {
            // C did not take ownership; reclaim the allocation.
            drop(unsafe { Box::from_raw(ctx) });
            Err(Error(rc))
        }
    }

    /// Start delivering decoded input events (pads, buttons, encoders, clock)
    /// on an SDK-owned thread. Returns `Err(Error(-4))` if already started,
    /// `Err(Error(-1))` if input devices are unavailable on this host (e.g.
    /// no hardware), or `Err(Error(rc))` if the SDK fails to start the thread.
    pub fn input_start(&mut self, cb: impl FnMut(Event) + Send + 'static) -> Result<(), Error> {
        if self.input_cb.is_some() {
            return Err(Error(-4)); // DELUGE_ERR_STATE: already running
        }
        let boxed: BoxedInputCb = Box::new(cb);
        // Double-box: `ctx` is a heap address that survives moves of `Deluge`.
        let ctx: *mut BoxedInputCb = Box::into_raw(Box::new(boxed));
        let h = unsafe { deluge_sys::deluge_input(self.raw.as_ptr()) };
        if h.is_null() {
            // Input unavailable on this host; C never saw ctx, reclaim it.
            drop(unsafe { Box::from_raw(ctx) });
            return Err(Error(-1));
        }
        let rc = unsafe {
            deluge_sys::deluge_input_start(h, Some(input_trampoline), ctx as *mut c_void)
        };
        if rc == 0 {
            self.input_cb = Some(unsafe { NonNull::new_unchecked(ctx) });
            Ok(())
        } else {
            // C did not take ownership; reclaim the allocation.
            drop(unsafe { Box::from_raw(ctx) });
            Err(Error(rc))
        }
    }

    /// Stop the input delivery thread and reclaim its boxed callback.
    /// Safe to call multiple times or not at all; `Drop` always stops
    /// (via `deluge_close`) and reclaims regardless. After this returns,
    /// `input_start` can be called again (unlike before the fix).
    pub fn input_stop(&mut self) {
        let h = unsafe { deluge_sys::deluge_input(self.raw.as_ptr()) };
        if !h.is_null() {
            unsafe { deluge_sys::deluge_input_stop(h) }; // joins the input thread
        }
        // Thread is joined, so the trampoline can't fire again: safe to reclaim.
        // Clearing input_cb also re-enables input_start (restart), and Drop's
        // take() then sees None -> no double free.
        if let Some(p) = self.input_cb.take() {
            drop(unsafe { Box::from_raw(p.as_ptr()) });
        }
    }

    pub fn midi_write(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        let m = unsafe { deluge_sys::deluge_midi(self.raw.as_ptr()) };
        let rc = unsafe { deluge_sys::deluge_midi_write(m, bytes.as_ptr(), bytes.len()) };
        if rc < 0 {
            Err(Error(rc))
        } else {
            Ok(rc as usize)
        }
    }

    pub fn midi_read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let m = unsafe { deluge_sys::deluge_midi(self.raw.as_ptr()) };
        let rc = unsafe { deluge_sys::deluge_midi_read(m, buf.as_mut_ptr(), buf.len()) };
        if rc < 0 {
            Err(Error(rc))
        } else {
            Ok(rc as usize)
        }
    }

    /// Start push delivery of received MIDI bytes.
    ///
    /// `cb` runs on libdeluge's MIDI reader thread — hence `Send + 'static` —
    /// and must be short and non-blocking: queue and signal, don't do work.
    /// Mirrors [`input_start`](Self::input_start); the same leak note applies,
    /// the closure lives for the process.
    pub fn midi_start(&mut self, cb: impl FnMut(&[u8]) + Send + 'static) -> Result<(), Error> {
        let m = unsafe { deluge_sys::deluge_midi(self.raw.as_ptr()) };
        if m.is_null() {
            return Err(Error(-1));
        }

        unsafe extern "C" fn trampoline(data: *const u8, n: usize, ctx: *mut core::ffi::c_void) {
            // SAFETY: `ctx` is the Box we leaked below, and libdeluge calls this
            // only from the reader thread it owns, which is joined before the
            // handle is freed — so the closure outlives every call.
            let f = unsafe { &mut *(ctx as *mut Box<dyn FnMut(&[u8]) + Send>) };
            let bytes = unsafe { core::slice::from_raw_parts(data, n) };
            f(bytes);
        }

        let boxed: Box<Box<dyn FnMut(&[u8]) + Send>> = Box::new(Box::new(cb));
        let ctx = Box::into_raw(boxed) as *mut core::ffi::c_void;
        let rc = unsafe { deluge_sys::deluge_midi_start(m, Some(trampoline), ctx) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn leds_indicator(&mut self, id: i32, on: bool) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_leds(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_leds_indicator(h, id, on as i32) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn leds_gold(&mut self, col: i32, i: i32, brightness: i32) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_leds(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_leds_gold(h, col, i, brightness) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    /// Drive the SYNC LED (P6_7 GPIO). Returns `Err` if the LED-class device
    /// is absent (kernel without the `deluge:sync` gpio-led node).
    pub fn leds_sync(&mut self, on: bool) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_leds(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_leds_sync(h, on as i32) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn cv_set_raw(&mut self, ch: i32, raw: u16) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_cv(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_cv_set_raw(h, ch, raw) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn cv_set_volts(&mut self, ch: i32, volts: f32) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_cv(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_cv_set_volts(h, ch, volts) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn gate_set(&mut self, n: i32, on: bool) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_cv(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_gate_set(h, n, on as i32) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    /// Cached SRAM scratch as a mutable slice (empty if unavailable). Bound to &mut self.
    pub fn sram(&mut self) -> Option<&mut [u8]> {
        let h = unsafe { deluge_sys::deluge_sram(self.raw.as_ptr()) };
        if h.is_null() {
            return None;
        }
        let mut len = 0usize;
        let p = unsafe { deluge_sys::deluge_sram_map(h, &mut len) } as *mut u8;
        if p.is_null() || len == 0 {
            return None;
        }
        Some(unsafe { std::slice::from_raw_parts_mut(p, len) })
    }

    pub fn oled_write(&mut self, buf: &[u8]) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_display(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_display_oled_write(h, buf.as_ptr(), buf.len()) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn pads_write(&mut self, buf: &[u8]) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_display(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_display_pads_write(h, buf.as_ptr(), buf.len()) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    /// Pad-LED refresh interval in ms, 0..=25 — **lower is brighter** (it is the
    /// PIC's refresh period, so a shorter period is a higher duty cycle). Drives
    /// PIC command 19, the same command and range as the bare-metal backend's
    /// `pic::set_refresh_time`.
    pub fn pads_set_refresh(&mut self, interval_ms: i32) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_display(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_display_pads_set_refresh(h, interval_ms) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    /// Is `jack` inserted? `jack` is a `deluge_jack` discriminant, ordered to
    /// match `deluge_bsp::jacks::Jack`.
    pub fn jack_inserted(&mut self, jack: u32) -> Result<bool, Error> {
        let h = unsafe { deluge_sys::deluge_jacks(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_jacks_get(h, jack) };
        if rc >= 0 {
            Ok(rc != 0)
        } else {
            Err(Error(rc))
        }
    }

    /// Request the on-board speaker amplifier on/off.
    ///
    /// **Advisory, not authoritative.** The kernel owns the policy — the amp is
    /// energised only when this request is set AND no output jack is inserted —
    /// so this cannot force the speaker on over plugged-in headphones. The
    /// bare-metal backend drives the amp GPIO directly and *is* authoritative.
    pub fn jacks_set_speaker(&mut self, on: bool) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_jacks(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_jacks_set_speaker(h, on as i32) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    pub fn usb_role_get(&mut self) -> Result<UsbRole, Error> {
        let h = unsafe { deluge_sys::deluge_usb(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let mut raw: deluge_sys::deluge_usb_role = 0;
        let rc = unsafe { deluge_sys::deluge_usb_role_get(h, &mut raw) };
        if rc != 0 {
            return Err(Error(rc));
        }
        Ok(if raw == deluge_sys::deluge_usb_role_DELUGE_USB_HOST {
            UsbRole::Host
        } else {
            UsbRole::Peripheral
        })
    }

    pub fn usb_role_set(&mut self, role: UsbRole) -> Result<(), Error> {
        let h = unsafe { deluge_sys::deluge_usb(self.raw.as_ptr()) };
        if h.is_null() {
            return Err(Error(-1));
        }
        let raw = match role {
            UsbRole::Host => deluge_sys::deluge_usb_role_DELUGE_USB_HOST,
            UsbRole::Peripheral => deluge_sys::deluge_usb_role_DELUGE_USB_PERIPHERAL,
        };
        let rc = unsafe { deluge_sys::deluge_usb_role_set(h, raw) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    /// Start an SDK-owned thread that invokes `cb` per USB hotplug event.
    pub fn usb_watch(&mut self, cb: impl FnMut(UsbEvent) + Send + 'static) -> Result<(), Error> {
        if self.usb_cb.is_some() {
            return Err(Error(-4));
        }
        let boxed: BoxedUsbCb = Box::new(cb);
        let ctx: *mut BoxedUsbCb = Box::into_raw(Box::new(boxed));
        let h = unsafe { deluge_sys::deluge_usb(self.raw.as_ptr()) };
        if h.is_null() {
            drop(unsafe { Box::from_raw(ctx) });
            return Err(Error(-1));
        }
        let rc =
            unsafe { deluge_sys::deluge_usb_watch(h, Some(usb_trampoline), ctx as *mut c_void) };
        if rc == 0 {
            self.usb_cb = Some(unsafe { NonNull::new_unchecked(ctx) });
            Ok(())
        } else {
            drop(unsafe { Box::from_raw(ctx) });
            Err(Error(rc))
        }
    }

    pub fn usb_unwatch(&mut self) {
        let h = unsafe { deluge_sys::deluge_usb(self.raw.as_ptr()) };
        if !h.is_null() {
            unsafe { deluge_sys::deluge_usb_unwatch(h) };
        }
        if let Some(p) = self.usb_cb.take() {
            drop(unsafe { Box::from_raw(p.as_ptr()) });
        }
    }

    pub fn usb_hotplug_fd(&mut self) -> i32 {
        let h = unsafe { deluge_sys::deluge_usb(self.raw.as_ptr()) };
        if h.is_null() {
            -1
        } else {
            unsafe { deluge_sys::deluge_usb_fd(h) }
        }
    }

    pub fn usb_midi_write(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        let m = unsafe { deluge_sys::deluge_usb_midi(self.raw.as_ptr()) };
        if m.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_usb_midi_write(m, bytes.as_ptr(), bytes.len()) };
        if rc < 0 {
            Err(Error(rc))
        } else {
            Ok(rc as usize)
        }
    }

    pub fn usb_midi_read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let m = unsafe { deluge_sys::deluge_usb_midi(self.raw.as_ptr()) };
        if m.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_usb_midi_read(m, buf.as_mut_ptr(), buf.len()) };
        if rc < 0 {
            Err(Error(rc))
        } else {
            Ok(rc as usize)
        }
    }

    /// The UAC2 gadget card's ALSA id, or None if no gadget audio card is bound.
    pub fn usb_audio_card_id(&mut self) -> Option<String> {
        let a = unsafe { deluge_sys::deluge_usb_audio(self.raw.as_ptr()) };
        if a.is_null() {
            return None;
        }
        let p = unsafe { deluge_sys::deluge_usb_audio_card_id(a) };
        if p.is_null() {
            return None;
        }
        Some(
            unsafe { std::ffi::CStr::from_ptr(p) }
                .to_string_lossy()
                .into_owned(),
        )
    }

    /// Read-only probe of the UAC2 gadget card (channels/rate/format). Starts no stream.
    /// TODO(USB-3b): the streaming path is not implemented — see the C header.
    pub fn usb_audio_probe(&mut self) -> Result<UsbAudioInfo, Error> {
        let a = unsafe { deluge_sys::deluge_usb_audio(self.raw.as_ptr()) };
        if a.is_null() {
            return Err(Error(-1));
        }
        let mut info = deluge_sys::deluge_usb_audio_info_t {
            playback_channels: 0,
            capture_channels: 0,
            rate_min: 0,
            rate_max: 0,
            formats: 0,
        };
        let rc = unsafe { deluge_sys::deluge_usb_audio_probe(a, &mut info) };
        if rc != 0 {
            return Err(Error(rc));
        }
        Ok(UsbAudioInfo {
            playback_channels: info.playback_channels,
            capture_channels: info.capture_channels,
            rate_min: info.rate_min,
            rate_max: info.rate_max,
            formats: info.formats,
        })
    }

    /// Re-discover + rebind the USB audio card (gadget or plugged host). Ok when
    /// bound, Err(NODEV) when none. Call from a usb_watch AUDIO hotplug event.
    pub fn usb_audio_rebind(&mut self) -> Result<(), Error> {
        let a = unsafe { deluge_sys::deluge_usb_audio(self.raw.as_ptr()) };
        if a.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_usb_audio_rebind(a) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }

    /// Re-discover + rebind the USB MIDI device (gadget or plugged host). Ok when
    /// bound, Err(NODEV) when none. Call from a usb_watch MIDI hotplug event.
    pub fn usb_midi_rebind(&mut self) -> Result<(), Error> {
        let m = unsafe { deluge_sys::deluge_usb_midi(self.raw.as_ptr()) };
        if m.is_null() {
            return Err(Error(-1));
        }
        let rc = unsafe { deluge_sys::deluge_usb_midi_rebind(m) };
        if rc == 0 {
            Ok(())
        } else {
            Err(Error(rc))
        }
    }
}

impl Drop for Deluge {
    fn drop(&mut self) {
        // deluge_close joins the RT thread and stops the input thread (via
        // deluge_input_free) first, so neither callback is in use by the
        // time we free its box below.
        unsafe { deluge_sys::deluge_close(self.raw.as_ptr()) };
        if let Some(p) = self.audio_cb.take() {
            drop(unsafe { Box::from_raw(p.as_ptr()) });
        }
        if let Some(p) = self.input_cb.take() {
            drop(unsafe { Box::from_raw(p.as_ptr()) });
        }
        if let Some(p) = self.usb_cb.take() {
            drop(unsafe { Box::from_raw(p.as_ptr()) });
        }
    }
}
