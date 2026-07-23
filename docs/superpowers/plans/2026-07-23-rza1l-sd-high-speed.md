# RZ/A1L SD High-Speed (33.3 MHz) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Double SD bus throughput on the Deluge (RZ/A1L) by switching cards to SD High-Speed mode via CMD6 and raising SD_CLK from P1/4 (16.7 MHz) to P1/2 (33.3 MHz), with automatic fallback to today's behavior for cards that can't switch.

**Architecture:** The register-level HAL (`rza1l-hal/src/sdhi.rs`) gains a parameterized clock-divider setter and a 64-byte status-block read (CMD6's response data). The SD protocol layer (`deluge-bsp/src/sd.rs`) gains a `try_high_speed()` step in `run_protocol` — CMD6 query → CMD6 switch → verify → raise clock; any failure falls back to the current 16.7 MHz path. A new `sd-bench` firmware validates on hardware (throughput + CRC soak).

**Tech Stack:** Rust `no_std` (armv7a-none-eabihf, Cortex-A9), Embassy async, host-side unit tests on x86-64. Spec: `docs/superpowers/specs/2026-07-23-rza1l-sd-high-speed-design.md`.

## Global Constraints

- We are currently on `main` — Task 1 creates branch `sd-high-speed` before any commit.
- RZ/A1L only. Do NOT touch `crates/rza2m-hal/src/sdhi.rs`.
- Firmware/target builds: `cargo build-fw -p <pkg>` (build-std alias; plain `cargo build` will NOT work for target crates).
- Host tests need an explicit target: `cargo test -p <pkg> --target x86_64-unknown-linux-gnu` (the workspace default target is armv7a-none-eabihf).
- Key TRM facts (already verified against `vendor/docs/rza1/rza1l-hw-manual/ch38-sd-host-interface.md`):
  - SD_CLK divider `0x00` = P1/2 ≈ 33.3 MHz; `0x01` = P1/4 ≈ 16.7 MHz. `0x00` is the only legal non-one-hot divider encoding.
  - CMD6 must use SD_CMD extended mode; the TRM's own example table 38.7 gives **CMD6 = `0x1C06`** (MD[2:0]=100 extended/R1, MD3=with-data, MD4=read, MD5=0 single-block).
  - `SD_SIZE` (block size) supports 1–512 bytes for single-block transfers; do not write `SD_SIZE`, `SD_CLK_CTRL`, or `SD_CMD` while `SD_INFO2.SCLKDIVEN` = 0.
- Deploy target is 32-bit (`usize == u32`) — no offset math that only works on 64-bit.
- All new Wren-free / DSP-free code: no NEON requirements here (control path only).

---

### Task 1: HAL clock divider — `CLK_DIV_2` + `set_clock_div`

**Files:**
- Modify: `crates/rza1l-hal/src/sdhi.rs` (divider consts ~line 212, `set_clock_fast` ~line 392, `Sdhi` wrapper ~line 1514, tests mod ~line 1676)

**Interfaces:**
- Produces: `pub const CLK_DIV_2: u16`, `pub const CLK_DIV_4: u16`, `pub unsafe fn set_clock_div(port: u8, div: u16)`, `Sdhi::set_clock_div(&self, div: u16)`. Task 4 and Task 5 call `set_clock_div` with these consts.

- [ ] **Step 1: Create the working branch**

```bash
git checkout -b sd-high-speed
```

- [ ] **Step 2: Write the failing test**

In the `tests` mod at the bottom of `crates/rza1l-hal/src/sdhi.rs`, replace the existing `clock_dividers_are_one_hot_per_trm` test with:

```rust
    #[test]
    fn clock_dividers_match_trm() {
        // SD_CLK_CTRL[7:0] divider select (TRM ch.38 §38.2.10).
        assert_eq!(CLK_DIV_512, 0x80); // ~130 kHz identification clock
        assert_eq!(CLK_DIV_4, 0x01); //  ~16.7 MHz default-speed clock
        assert_eq!(CLK_DIV_2, 0x00); //  ~33.3 MHz high-speed clock
        // Non-zero encodings are one-hot; 0x00 (= P1/2) is the sole legal
        // all-zero encoding.
        assert_eq!(CLK_DIV_512.count_ones(), 1);
        assert_eq!(CLK_DIV_4.count_ones(), 1);
    }
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cargo test -p rza1l-hal --target x86_64-unknown-linux-gnu clock_dividers`
Expected: FAIL — `cannot find value CLK_DIV_2 in this scope` (compile error counts as the failing state).

- [ ] **Step 4: Implement the divider const and `set_clock_div`**

At the divider constants (~line 212), change the block to (note: `CLK_DIV_4` and the new `CLK_DIV_2` become `pub` — Task 5's bench firmware uses them):

```rust
const CLK_DIV_512: u16 = 0x80; // P1/512 ≈ 130 kHz  (identification clock)
/// P1/4 ≈ 16.7 MHz — default-speed data clock (any card).
pub const CLK_DIV_4: u16 = 0x01;
/// P1/2 ≈ 33.3 MHz — requires the card in High-Speed mode (CMD6) first.
/// The sole legal non-one-hot divider encoding (TRM §38.2.10: 0x00 = P1/2).
pub const CLK_DIV_2: u16 = 0x00;
const CLK_ENABLE: u16 = 1 << 8; // Bit 8 (SCLKEN) = clock output enable
```

Replace the body of `set_clock_fast` (~line 386–421) with a generalized function plus a thin wrapper:

```rust
/// Set the SD clock divider (`CLK_DIV_*` constant).
///
/// Waits for `SCLKDIVEN` before and after the change per TRM §38.2.10
/// ("Do not write to SD_CLK_CTRL while the SCLKDIVEN bit in SD_INFO2 is 0").
///
/// `CLK_DIV_2` (33.3 MHz) exceeds the 25 MHz default-speed limit — only set
/// it after the card has been switched to High-Speed mode via CMD6.
///
/// # Safety
/// Writes to memory-mapped SDHI register.
pub unsafe fn set_clock_div(port: u8, div: u16) {
    unsafe {
        let base = port_base(port);
        // Wait for clock divider change to settle (SCLKDIVEN = 1 means ready).
        let mut ok = false;
        for _ in 0..10_000u32 {
            if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
                ok = true;
                break;
            }
        }
        if !ok {
            log::warn!("sdhi{}: set_clock_div: SCLKDIVEN pre-change timeout", port);
        }
        reg16(base, OFF_CLK_CTRL).write_volatile(div | CLK_ENABLE);
        ok = false;
        for _ in 0..10_000u32 {
            if reg16(base, OFF_INFO2).read_volatile() & INFO2_SCLKDIVEN != 0 {
                ok = true;
                break;
            }
        }
        if !ok {
            log::warn!("sdhi{}: set_clock_div: SCLKDIVEN post-change timeout", port);
        }
    }
}

/// Switch the SD clock to the default-speed data clock (~16.7 MHz, P1/4).
///
/// Call after successful card initialization.
///
/// # Safety
/// Writes to memory-mapped SDHI register.
pub unsafe fn set_clock_fast(port: u8) {
    unsafe { set_clock_div(port, CLK_DIV_4) }
}
```

In the `Sdhi<PORT>` impl (next to the existing `set_clock_fast` method ~line 1514), add:

```rust
    /// Set the SD clock divider (`CLK_DIV_*` constant).
    ///
    /// # Safety
    /// Writes memory-mapped SDHI registers.
    pub unsafe fn set_clock_div(&self, div: u16) {
        unsafe { set_clock_div(PORT, div) }
    }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p rza1l-hal --target x86_64-unknown-linux-gnu`
Expected: PASS (all tests, including the renamed `clock_dividers_match_trm`).

- [ ] **Step 6: Verify the target build still compiles**

Run: `cargo build-fw -p rza1l-hal`
Expected: clean build (warnings-free for the touched code).

- [ ] **Step 7: Commit**

```bash
git add crates/rza1l-hal/src/sdhi.rs
git commit -m "feat(rza1l-hal): parameterized SD clock divider with CLK_DIV_2 (33.3 MHz)"
```

---

### Task 2A: HAL shared async wait primitives (behavior-preserving refactor)

**Files:**
- Modify: `crates/rza1l-hal/src/sdhi.rs` (new helpers after `clear_info` ~line 530; call-site conversions in `send_cmd`, `read_blocks_sw`, `write_blocks_sw`, `read_blocks_dma`, `write_blocks_dma`)

**Interfaces:**
- Consumes: existing `STATE`, `check_info2_errors`, `INFO1_RESP`/`INFO1_DATA_TRNS`/`INFO2_BRE`/`INFO2_BWE` masks.
- Produces: private `async fn wait_resp(port: u8) -> Result<(), SdhiError>`, `async fn wait_buf_ready(port: u8, ready_bit: u16) -> Result<(), SdhiError>`, `async fn wait_access_end(port: u8) -> Result<(), SdhiError>`. Task 2B uses all three.

The file currently contains five near-identical inline `poll_fn` wait blocks. This task factors them into three named helpers and converts every async call site, so Task 2B doesn't add a sixth copy. **This is a strict behavior-preserving refactor**: the helper bodies are the existing poll_fn blocks verbatim (including the register-then-re-check pattern — it closes a wake race; do not "simplify" it); no register access, mask write, or error path may change. The synchronous `*_poll` functions (spin loops, no waker) are a different mechanism and stay untouched.

- [ ] **Step 1: Add the three helpers**

Insert after `clear_info` (~line 530), before the "Command/argument helpers" section:

```rust
// ---------------------------------------------------------------------------
// Shared async wait primitives
// ---------------------------------------------------------------------------
//
// Each helper is the interrupt-driven wait used by the command/data paths:
// check accumulated state, register the waker, then re-check (closing the
// race where the interrupt fires between the load and the register).

/// Await response end (`INFO1_RESP`), surfacing INFO2 errors.
async fn wait_resp(port: u8) -> Result<(), SdhiError> {
    poll_fn(|cx| {
        let st = &STATE[port as usize];
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_RESP != 0 {
            return Poll::Ready(Ok(()));
        }
        st.waker.register(cx.waker());
        // Re-check in case the interrupt fired between the load and register.
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_RESP != 0 {
            return Poll::Ready(Ok(()));
        }
        Poll::Pending
    })
    .await
}

/// Await a FIFO-ready bit (`INFO2_BRE` or `INFO2_BWE`) and consume it.
async fn wait_buf_ready(port: u8, ready_bit: u16) -> Result<(), SdhiError> {
    poll_fn(|cx| {
        let st = &STATE[port as usize];
        let i2 = st.info2.load(Ordering::Acquire);
        if i2 & ready_bit != 0 {
            st.info2.fetch_and(!ready_bit, Ordering::AcqRel);
            return Poll::Ready(Ok::<(), SdhiError>(()));
        }
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        st.waker.register(cx.waker());
        // Re-check in case the interrupt fired between the load and register.
        let i2 = st.info2.load(Ordering::Acquire);
        if i2 & ready_bit != 0 {
            st.info2.fetch_and(!ready_bit, Ordering::AcqRel);
            return Poll::Ready(Ok(()));
        }
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        Poll::Pending
    })
    .await
}

/// Await access end (`INFO1_DATA_TRNS`), surfacing INFO2 errors.
async fn wait_access_end(port: u8) -> Result<(), SdhiError> {
    poll_fn(|cx| {
        let st = &STATE[port as usize];
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_DATA_TRNS != 0 {
            return Poll::Ready(Ok(()));
        }
        st.waker.register(cx.waker());
        // Re-check in case the interrupt fired between the load and register.
        let i1 = st.info1.load(Ordering::Acquire);
        let i2 = st.info2.load(Ordering::Acquire);
        if let Err(e) = check_info2_errors(i2) {
            return Poll::Ready(Err(e));
        }
        if i1 & INFO1_DATA_TRNS != 0 {
            return Poll::Ready(Ok(()));
        }
        Poll::Pending
    })
    .await
}
```

- [ ] **Step 2: Convert the five async call sites**

Each conversion replaces an inline `poll_fn(…).await` block with the matching helper call. The surrounding mask writes, FIFO loops, and cleanup code stay byte-identical.

1. `send_cmd` (~line 664): `let result = poll_fn(…).await;` → `let result = wait_resp(port).await;`
2. `read_blocks_sw`: the per-block BRE wait → `wait_buf_ready(port, INFO2_BRE).await?;` and the final access-end wait → `wait_access_end(port).await?;`
3. `write_blocks_sw`: the per-block BWE wait → `wait_buf_ready(port, INFO2_BWE).await?;` and the final access-end wait → `wait_access_end(port).await?;`
4. `read_blocks_dma`: `let cmd_result = poll_fn(…).await;` → `let cmd_result = wait_resp(port).await;` and `let data_result = poll_fn(…).await;` → `let data_result = wait_access_end(port).await;`
5. `write_blocks_dma`: same two conversions as `read_blocks_dma`.

After this step the only remaining `poll_fn` calls in the file should be inside the three helpers — verify with `grep -n "poll_fn" crates/rza1l-hal/src/sdhi.rs`.

- [ ] **Step 3: Run host tests and target build**

Run: `cargo test -p rza1l-hal --target x86_64-unknown-linux-gnu`
Expected: PASS.

Run: `cargo build-fw -p rza1l-hal` and `cargo build-fw -p msc-firmware`
Expected: clean builds (msc-firmware exercises the SD paths end-to-end at compile time).

- [ ] **Step 4: Commit**

```bash
git add crates/rza1l-hal/src/sdhi.rs
git commit -m "refactor(rza1l-hal): factor shared SDHI async wait primitives"
```

---

### Task 2B: HAL 64-byte status-block read (`read_status_block_sw`)

**Files:**
- Modify: `crates/rza1l-hal/src/sdhi.rs` (insert after `write_blocks_sw`, before the "DMA data transfer" section header)

**Interfaces:**
- Consumes: existing `send_cmd`, `set_arg`, `wait_clk_stable`, `clear_info`; Task 2A's `wait_buf_ready`, `wait_access_end`; `INFO2_ERR_ALL`/`INFO2_BRE`/`INFO1_DATA_TRNS` masks.
- Produces: `pub async unsafe fn read_status_block_sw(port: u8, cmd_val: u16, arg: u32, buf: &mut [u8; 64]) -> Result<(), SdhiError>`. Task 4 calls it with `cmd_val = 0x1C06`.

There is no host-testable behavior here (pure MMIO); the verification is compile + the existing register-offset tests + on-device Task 6.

- [ ] **Step 1: Implement the function pair**

Insert after `write_blocks_sw` (before the "DMA data transfer" section header):

```rust
// ---------------------------------------------------------------------------
// Short (64-byte) status-block transfer — CMD6 switch-function status
// ---------------------------------------------------------------------------

/// Issue an extended-mode single-block read command whose data phase is the
/// 64-byte switch-function status block (CMD6), and read that block.
///
/// Unlike [`read_blocks_sw`], this issues the command itself: `SD_SIZE` must
/// be programmed to 64 *before* the command starts the data transfer.  It is
/// restored to 512 afterwards — on the error path too.
///
/// `cmd_val` must be an extended-mode single-block-read encoding: for CMD6
/// this is `0x1C06` (TRM table 38.7 — MD[2:0]=100 extended/R1, MD3=with-data,
/// MD4=read, MD5=0 single-block).  `arg` is the CMD6 mode/function argument.
///
/// # Safety
/// Reads/writes SDHI peripheral registers.  Must not be called concurrently
/// for the same port.
pub async unsafe fn read_status_block_sw(
    port: u8,
    cmd_val: u16,
    arg: u32,
    buf: &mut [u8; 64],
) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        // SD_SIZE ← 64 before the command (TRM §38.2.11; single-block
        // transfers accept 1–512 bytes).  Never while SCLKDIVEN = 0.
        wait_clk_stable(port);
        reg16(base, OFF_SIZE).write_volatile(64);
        // Single block: SEC (block-count enable) off.
        reg16(base, OFF_STOP).write_volatile(0x0000);
        reg16(base, OFF_SECCNT).write_volatile(1);

        set_arg(port, arg);
        let result = read_status_block_inner(port, cmd_val, buf).await;

        // Quiesce interrupts and restore the 512-byte sector block size —
        // error path included, so a failed CMD6 can't poison later sector IO.
        reg16(base, OFF_INFO1_MASK).write_volatile(0xFFFF);
        reg16(base, OFF_INFO2_MASK).write_volatile(0xFFFF);
        clear_info(port);
        wait_clk_stable(port);
        reg16(base, OFF_SIZE).write_volatile(512);

        result
    }
}

/// Body of [`read_status_block_sw`]: command issue + 64-byte FIFO drain.
/// Split out so the caller can restore `SD_SIZE` on every exit path.
async unsafe fn read_status_block_inner(
    port: u8,
    cmd_val: u16,
    buf: &mut [u8; 64],
) -> Result<(), SdhiError> {
    unsafe {
        let base = port_base(port);

        // Issue the command and await the R1 response.
        send_cmd(port, cmd_val).await?;

        // Enable DATA_TRNS (access end), BRE, and all errors.
        // Hardware polarity: 0 = enabled, 1 = masked → write bitwise complement.
        reg16(base, OFF_INFO1_MASK).write_volatile(!INFO1_DATA_TRNS);
        reg16(base, OFF_INFO2_MASK).write_volatile(!(INFO2_ERR_ALL | INFO2_BRE));

        // Wait for FIFO data (BRE), drain 64 bytes (16 × 32-bit reads).
        wait_buf_ready(port, INFO2_BRE).await?;
        let fifo = reg32(base, OFF_BUF0);
        for w in 0..16usize {
            let word = fifo.read_volatile();
            let p = buf.as_mut_ptr().add(w * 4) as *mut u32;
            p.write_unaligned(word);
        }

        // Wait for access end.
        wait_access_end(port).await
    }
}
```

- [ ] **Step 2: Run host tests (regression) and target build**

Run: `cargo test -p rza1l-hal --target x86_64-unknown-linux-gnu`
Expected: PASS (no test changes; confirms host build unbroken).

Run: `cargo build-fw -p rza1l-hal`
Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add crates/rza1l-hal/src/sdhi.rs
git commit -m "feat(rza1l-hal): 64-byte switch-status block read for CMD6"
```

---

### Task 3: BSP switch-status parser (`parse_switch_status`)

**Files:**
- Modify: `crates/deluge-bsp/src/sd.rs` (insert in the *shared* section, after the `From<SdhiError>` impl ~line 68, **outside** the `#[cfg(target_os = "none")] mod device` — it must compile host-side for tests)

**Interfaces:**
- Produces: `pub(crate) struct SwitchStatus { pub hs_supported: bool, pub group1_selected: u8 }` and `pub(crate) fn parse_switch_status(buf: &[u8; 64]) -> SwitchStatus`. Task 4 calls `super::parse_switch_status` from the `device` mod.

- [ ] **Step 1: Write the failing tests**

Add at the bottom of `crates/deluge-bsp/src/sd.rs`:

```rust
// ---------------------------------------------------------------------------
// Unit tests (host-side)
// ---------------------------------------------------------------------------

#[cfg(all(test, not(target_os = "none")))]
mod switch_status_tests {
    use super::*;

    #[test]
    fn hs_supported_and_selected() {
        let mut buf = [0u8; 64];
        buf[13] = 0x03; // group 1 supports functions 0 and 1
        buf[16] = 0x01; // group 1 switched to function 1 (High-Speed)
        let st = parse_switch_status(&buf);
        assert!(st.hs_supported);
        assert_eq!(st.group1_selected, 0x1);
    }

    #[test]
    fn hs_unsupported() {
        let mut buf = [0u8; 64];
        buf[13] = 0x01; // only function 0 (default speed)
        let st = parse_switch_status(&buf);
        assert!(!st.hs_supported);
        assert_eq!(st.group1_selected, 0x0);
    }

    #[test]
    fn switch_refused_is_0xf_and_nibbles_do_not_leak() {
        let mut buf = [0u8; 64];
        buf[13] = 0x03;
        buf[16] = 0x0F; // 0xF = function error / switch refused
        assert_eq!(parse_switch_status(&buf).group1_selected, 0xF);
        buf[16] = 0xF1; // upper nibble belongs to group 2 — must not leak
        assert_eq!(parse_switch_status(&buf).group1_selected, 0x1);
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p deluge-bsp --target x86_64-unknown-linux-gnu switch_status`
Expected: FAIL — `cannot find function parse_switch_status`.

- [ ] **Step 3: Implement the parser**

Insert in the shared section (after the `From<SdhiError>` impl, before the `pub use device::…` re-exports):

```rust
// ---------------------------------------------------------------------------
// CMD6 (SWITCH_FUNC) status block (shared: parsed on device, tested on host)
// ---------------------------------------------------------------------------

/// Decoded fields of the CMD6 (SWITCH_FUNC) 512-bit status block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SwitchStatus {
    /// Function group 1 (access mode) supports function 1 (High-Speed).
    pub hs_supported: bool,
    /// Function group 1 selected-function nibble
    /// (0x1 = High-Speed selected, 0xF = switch refused / function error).
    pub group1_selected: u8,
}

/// Parse the CMD6 switch-function status block.
///
/// The block arrives MSB-first: `buf[0]` holds status bits 511:504.
///   - Group-1 support mask = status bits 415:400 → bytes 12–13;
///     function 1 (High-Speed) = bit 401 = byte 13, bit 1.
///   - Group-1 selected function = status bits 379:376 → byte 16 low nibble.
pub(crate) fn parse_switch_status(buf: &[u8; 64]) -> SwitchStatus {
    SwitchStatus {
        hs_supported: buf[13] & 0x02 != 0,
        group1_selected: buf[16] & 0x0F,
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p deluge-bsp --target x86_64-unknown-linux-gnu`
Expected: PASS (all three new tests + existing suite).

- [ ] **Step 5: Commit**

```bash
git add crates/deluge-bsp/src/sd.rs
git commit -m "feat(deluge-bsp): CMD6 switch-function status block parser"
```

---

### Task 4: BSP `try_high_speed` + protocol wiring + `is_hs`

**Files:**
- Modify: `crates/deluge-bsp/src/sd.rs`:
  - module doc protocol overview (~line 26)
  - `pub use device::{…}` re-export list (~line 71)
  - `device` mod: command consts (~line 118), card state statics (~line 164), `is_hc` neighborhood (search `pub fn is_hc`), `run_protocol` (~line 405)

**Interfaces:**
- Consumes: `rza1l_hal::sdhi::{read_status_block_sw, set_clock_div, set_clock_fast, CLK_DIV_2}` (Tasks 1–2), `super::parse_switch_status` (Task 3).
- Produces: `pub fn is_hs() -> bool` (re-exported from `deluge_bsp::sd`); `run_protocol` now lands cards in HS/33.3 MHz when possible. Task 5's bench calls `sd::is_hs()`.

- [ ] **Step 1: Add CMD6 constants**

In the `device` mod next to the existing command constants (~line 118, after `CMD55`):

```rust
    /// CMD6 (SWITCH_FUNC) — extended mode, single-block read, R1 + 64-byte data.
    /// SD_CMD encoding 0x1C06 per TRM table 38.7 (MD[2:0]=100 extended/R1,
    /// MD3 = with-data, MD4 = read, MD5 = 0 single-block).  Normal mode cannot
    /// be used: the SDHI does not auto-decode CMD6 as a data command.
    const CMD6_DATA: u16 = 0x1C06;

    /// CMD6 mode-0 (query) argument: group 1 → function 1, groups 2–6 = 0xF
    /// (no change).  Answers "is High-Speed switchable?" without switching.
    const CMD6_ARG_QUERY_HS: u32 = 0x00FF_FF01;
    /// CMD6 mode-1 (switch) argument: switch group 1 to function 1 (High-Speed).
    const CMD6_ARG_SWITCH_HS: u32 = 0x80FF_FF01;
```

- [ ] **Step 2: Add the `CARD_HS` state + accessor**

Next to `CARD_HC` (~line 166):

```rust
    /// `true` if the card accepted the CMD6 switch to High-Speed mode
    /// (SD_CLK = 33.3 MHz).  `false` = default speed (16.7 MHz).
    static CARD_HS: AtomicBool = AtomicBool::new(false);
```

Next to `pub fn is_hc` (search for it; it's in the public API section):

```rust
    /// Return `true` if the card is running in High-Speed mode (33.3 MHz SD_CLK).
    pub fn is_hs() -> bool {
        CARD_HS.load(Ordering::Acquire)
    }
```

Update the re-export list (~line 71) to include `is_hs`:

```rust
pub use device::{
    DelugeBlockDevice, DelugeTimeSource, PartitionShim, init, is_hc, is_hs, is_inserted, is_ready,
    is_write_protected, read_sectors, total_sectors, write_sectors,
};
```

Note: the host mod (~line 76) does NOT get `is_hs` — host callers that need it are firmware-only (sd-bench). If a dual-target consumer later wants it, add a `pub fn is_hs() -> bool { false }` to `host` then.

- [ ] **Step 3: Implement `try_high_speed`**

In the `device` mod, after `send_acmd` (~line 200):

```rust
    /// Attempt the CMD6 switch to High-Speed mode (SD spec ≥ 1.10 cards).
    ///
    /// Runs after the card is in Transfer state (post CMD7/ACMD6).  On success
    /// the *card* is in High-Speed mode and the caller may raise SD_CLK to
    /// P1/2 (33.3 MHz).  Every error is non-fatal to init: the caller falls
    /// back to the default-speed clock.  Pre-1.10 cards reject CMD6 as an
    /// illegal command → surfaces here as a response error → fallback.
    async unsafe fn try_high_speed() -> Result<(), SdError> {
        unsafe {
            let mut status = [0u8; 64];

            // Mode 0 (query): does the card support group-1 function 1?
            sdhi::read_status_block_sw(SD_PORT, CMD6_DATA, CMD6_ARG_QUERY_HS, &mut status)
                .await?;
            let st = super::parse_switch_status(&status);
            if !st.hs_supported {
                return Err(SdError::UnsupportedCard);
            }

            // Mode 1 (switch): actually switch to High-Speed.
            sdhi::read_status_block_sw(SD_PORT, CMD6_DATA, CMD6_ARG_SWITCH_HS, &mut status)
                .await?;
            let st = super::parse_switch_status(&status);
            if st.group1_selected != 0x1 {
                return Err(SdError::Protocol);
            }
            Ok(())
        }
    }
```

- [ ] **Step 4: Wire into `run_protocol`**

Replace the final clock-switch block of `run_protocol` (~line 405):

```rust
            // ---- Switch to high-speed clock ----
            sdhi::set_clock_fast(SD_PORT);
```

with:

```rust
            // ---- Clock: try CMD6 High-Speed (33.3 MHz), else default (16.7) ----
            // The card is in Transfer state; CMD6 needs the data lines, so this
            // must come after ACMD6 (4-bit bus).  The SD spec allows the new
            // timing 8 clocks after the switch-status end bit — DATA_TRNS
            // (awaited inside read_status_block_sw) is past that point.
            match try_high_speed().await {
                Ok(()) => {
                    sdhi::set_clock_div(SD_PORT, sdhi::CLK_DIV_2);
                    CARD_HS.store(true, Ordering::Release);
                    log::info!("sd: High-Speed mode, SD_CLK = 33.3 MHz (P1/2)");
                }
                Err(e) => {
                    sdhi::set_clock_fast(SD_PORT);
                    CARD_HS.store(false, Ordering::Release);
                    log::info!("sd: default speed, SD_CLK = 16.7 MHz (P1/4): {:?}", e);
                }
            }
```

- [ ] **Step 5: Update the module doc protocol overview**

Replace line 26 (`//!   Switch clock from ~130 kHz (P1/512) to ~16.7 MHz (P1/4)`) with:

```rust
//!   CMD6  → query + switch to High-Speed mode (skipped on old cards)
//!   Switch clock from ~130 kHz (P1/512) to 33.3 MHz (P1/2, High-Speed) or
//!   16.7 MHz (P1/4) if the CMD6 switch is unsupported/refused
```

- [ ] **Step 6: Run host tests and target builds**

Run: `cargo test -p deluge-bsp --target x86_64-unknown-linux-gnu`
Expected: PASS.

Run: `cargo build-fw -p msc-firmware`
Expected: clean build — proves the `device` mod (which is `cfg`'d out on host) compiles with the new code in a real firmware consumer.

- [ ] **Step 7: Commit**

```bash
git add crates/deluge-bsp/src/sd.rs
git commit -m "feat(deluge-bsp): switch SD cards to High-Speed mode (33.3 MHz) via CMD6"
```

---

### Task 5: `sd-bench` firmware

**Files:**
- Create: `firmwares/sd-bench/` (scaffold copied from `firmwares/wp-probe/`)
- Create: `firmwares/sd-bench/src/tasks/bench.rs`
- Modify: `firmwares/sd-bench/Cargo.toml`, `firmwares/sd-bench/src/main.rs`, `firmwares/sd-bench/src/tasks/mod.rs`
- Modify: `Cargo.toml` (workspace members)

**Interfaces:**
- Consumes: `deluge_bsp::sd::{init, is_hc, is_hs, total_sectors, read_sectors, write_sectors, SdError}`, `rza1l_hal::sdhi::{set_clock_div, CLK_DIV_2, CLK_DIV_4}`.
- Produces: a flashable diagnostic firmware; no downstream code depends on it.

- [ ] **Step 1: Scaffold from wp-probe**

```bash
cp -r firmwares/wp-probe firmwares/sd-bench
rm firmwares/sd-bench/src/tasks/probe.rs
```

In `firmwares/sd-bench/Cargo.toml`, change the package and bin name (keep everything else — deps, features — identical):

```toml
[package]
publish = false  # binary crate — not for crates.io
name = "sd-bench"
version = "0.1.0"
edition = "2024"
license.workspace = true

[[bin]]
name = "sd-bench"
test = false
```

Add `"firmwares/sd-bench",` to the workspace `members` list in the root `Cargo.toml` (after `"firmwares/wp-probe",`).

- [ ] **Step 2: Trim `main.rs`**

Edit `firmwares/sd-bench/src/main.rs`, keeping the wp-probe structure (RTT init, panic handler, heaps, `init_clocks`, heartbeat pin, executor) but: update the module doc, drop the OLED/CV RSPI init (`cv_gate`), and spawn `bench_task` instead of `probe_task`. Resulting diff from wp-probe's `main.rs`:

Replace the doc comment (lines 1–10) with:

```rust
//! SD High-Speed bench firmware.
//!
//! A throwaway diagnostic image: brings up the platform and SD card, logs the
//! CMD6 High-Speed query/switch outcome, measures sequential read throughput
//! at 16.7 MHz (P1/4) vs 33.3 MHz (P1/2), then runs a CRC soak.  Optional
//! write soak via [`tasks::bench::WRITE_SOAK`] (dev cards only).
//!
//! Build: `cargo build-fw -p sd-bench --features rtt` — output over RTT.
```

Remove the lines:

```rust
use deluge_bsp::cv_gate;
```
```rust
    // RSPI0 init — shared between OLED (8-bit) and CV DAC (32-bit).
    // cv_gate::init() enables the RSPI0 module clock; oled::init() then switches
    // it to 8-bit mode for the panel.
    unsafe { cv_gate::init() };
    info!("RSPI0: initialised via cv_gate::init");
```

Change the two `info!` boot banners mentioning "WP-probe" to "sd-bench", and change the spawn block to:

```rust
    executor.run(|spawner: Spawner| {
        spawner.spawn(tasks::pic::pic_task().unwrap());
        spawner.spawn(tasks::bench::bench_task().unwrap());
    });
```

(The PIC/UART task is kept — same known-good scaffold as wp-probe.)

In `firmwares/sd-bench/src/tasks/mod.rs`, replace the `probe` module declaration with `bench`:

```rust
pub mod bench;
pub mod pic;
```

(If `mod.rs` has doc comments referencing `probe`, update them accordingly.)

- [ ] **Step 3: Write the bench task**

Create `firmwares/sd-bench/src/tasks/bench.rs`:

```rust
//! SD High-Speed bench: throughput at P1/4 vs P1/2, then a CRC soak.
//!
//! All results go to the RTT log.  Reads are non-destructive (LBA 0 upward);
//! the optional write soak targets a scratch region near the end of the card
//! and is off by default — enable [`WRITE_SOAK`] on dev cards only.

use deluge_bsp::sd;
use embassy_time::Instant;
use log::{error, info, warn};
use rza1l_hal::sdhi;

/// Sectors per read chunk (matches the BSP DMA bounce buffer size).
const CHUNK_SECTORS: u32 = 128; // 64 KiB
/// Data volume per throughput measurement, in MiB.
const BENCH_MIB: u32 = 64;
/// CRC soak passes over the bench region.
const SOAK_PASSES: u32 = 8;
/// Enable the destructive write soak (dev cards only!).
pub const WRITE_SOAK: bool = false;
/// Distance of the write scratch region from the end of the card, in sectors.
const SCRATCH_FROM_END: u32 = 64 * 1024; // 32 MiB clearance

/// Transfer bounce buffer (BSS).  Access is serialised: one bench task.
static mut BUF: [u8; (CHUNK_SECTORS as usize) * 512] = [0; (CHUNK_SECTORS as usize) * 512];

/// FNV-1a 64-bit checksum.
fn checksum(buf: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in buf {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Read `sectors` sectors starting at `start`, returning a combined checksum.
async fn read_region(start: u32, sectors: u32) -> Result<u64, sd::SdError> {
    let mut h: u64 = 0;
    let mut lba = start;
    let end = start + sectors;
    while lba < end {
        let n = (end - lba).min(CHUNK_SECTORS);
        #[allow(static_mut_refs)]
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
        sd::read_sectors(lba, n, &mut buf[..(n as usize) * 512]).await?;
        h ^= checksum(&buf[..(n as usize) * 512]);
        lba += n;
    }
    Ok(h)
}

/// Timed sequential read of [`BENCH_MIB`] MiB from LBA 0.
async fn measure_read(label: &str) -> Result<u64, sd::SdError> {
    let sectors = BENCH_MIB * 2048; // MiB → 512-byte sectors
    let t0 = Instant::now();
    let h = read_region(0, sectors).await?;
    let elapsed_ms = t0.elapsed().as_millis() as u32;
    let kib = sectors / 2;
    info!(
        "bench[{}]: {} KiB in {} ms = {} KiB/s (csum {:016x})",
        label,
        kib,
        elapsed_ms,
        // kib ≤ 65536 → kib*1000 ≤ 6.6e7: no u32 overflow.
        (kib * 1000) / elapsed_ms.max(1),
        h
    );
    Ok(h)
}

#[embassy_executor::task]
pub async fn bench_task() {
    info!("sd-bench: initialising card");
    if let Err(e) = sd::init().await {
        error!("sd-bench: init failed: {:?}", e);
        return;
    }
    info!(
        "sd-bench: card ready — {} sectors, HC={}, HS={}",
        sd::total_sectors(),
        sd::is_hc(),
        sd::is_hs()
    );

    // ---- Throughput: forced default-speed clock (P1/4 = 16.7 MHz) ----
    // Safe regardless of card mode: a High-Speed card also runs at 16.7 MHz.
    unsafe { sdhi::set_clock_div(1, sdhi::CLK_DIV_4) };
    let _ = measure_read("P1/4 16.7MHz").await;

    // ---- Throughput: high-speed clock (P1/2 = 33.3 MHz) ----
    if sd::is_hs() {
        unsafe { sdhi::set_clock_div(1, sdhi::CLK_DIV_2) };
        let _ = measure_read("P1/2 33.3MHz").await;
    } else {
        warn!("sd-bench: card not in High-Speed mode; skipping 33.3 MHz run");
    }

    // ---- CRC soak at the final (fastest working) clock ----
    let sectors = BENCH_MIB * 2048;
    let mut reference: Option<u64> = None;
    let mut failures = 0u32;
    for pass in 1..=SOAK_PASSES {
        match read_region(0, sectors).await {
            Ok(h) => match reference {
                None => {
                    reference = Some(h);
                    info!("soak: pass {}/{}: csum {:016x} (reference)", pass, SOAK_PASSES, h);
                }
                Some(r) if r == h => info!("soak: pass {}/{}: OK", pass, SOAK_PASSES),
                Some(r) => {
                    failures += 1;
                    error!(
                        "soak: pass {}/{}: MISMATCH {:016x} != {:016x}",
                        pass, SOAK_PASSES, h, r
                    );
                }
            },
            Err(e) => {
                failures += 1;
                error!("soak: pass {}/{}: read error {:?}", pass, SOAK_PASSES, e);
            }
        }
    }
    if failures == 0 {
        info!("soak: PASS — {} passes, 0 errors", SOAK_PASSES);
    } else {
        error!("soak: FAIL — {} error(s) in {} passes", failures, SOAK_PASSES);
    }

    if WRITE_SOAK {
        write_soak().await;
    }

    info!("sd-bench: done");
}

/// Destructive write/read-back verify against a scratch region near the end
/// of the card.  Guarded by [`WRITE_SOAK`].
async fn write_soak() {
    let total = sd::total_sectors();
    if total < SCRATCH_FROM_END + CHUNK_SECTORS {
        error!("write soak: card too small ({} sectors)", total);
        return;
    }
    let base = total - SCRATCH_FROM_END;
    info!("write soak: scratch @ LBA {} ({} sectors) — DESTRUCTIVE", base, CHUNK_SECTORS);

    #[allow(static_mut_refs)]
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
    // Fill with an LCG pattern so stale data can't false-pass.
    let mut x: u32 = 0x1234_5678;
    for b in buf.iter_mut() {
        x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *b = (x >> 24) as u8;
    }
    let expect = checksum(&buf[..]);

    if let Err(e) = sd::write_sectors(base, CHUNK_SECTORS, buf).await {
        error!("write soak: write error {:?}", e);
        return;
    }
    buf.fill(0);
    match sd::read_sectors(base, CHUNK_SECTORS, buf).await {
        Ok(()) => {
            let got = checksum(&buf[..]);
            if got == expect {
                info!("write soak: verify OK ({:016x})", got);
            } else {
                error!("write soak: MISMATCH {:016x} != {:016x}", got, expect);
            }
        }
        Err(e) => error!("write soak: read-back error {:?}", e),
    }
}
```

Implementation notes for this step:
- Verify the exact signatures of `sd::write_sectors` / `sd::read_sectors` / `sd::total_sectors` in `crates/deluge-bsp/src/sd.rs` before finalizing (`read_sectors(lba: u32, count: u32, buf: &mut [u8])` is confirmed at sd.rs:468; mirror whatever `write_sectors` actually takes — if it takes `&[u8]`, pass `&buf[..]`).
- `sdhi::set_clock_div(1, …)` — port 1 is the Deluge SD slot (`SD_PORT` in the BSP is private, hence the literal; add a `// SDHI port 1 = Deluge SD slot` comment).

- [ ] **Step 4: Build the firmware**

Run: `cargo build-fw -p sd-bench --features rtt`
Expected: clean build producing `target/armv7a-none-eabihf/debug/sd-bench`.

Also confirm the plain build works: `cargo build-fw -p sd-bench`
Expected: clean build.

- [ ] **Step 5: Run the full host test suite (regression)**

Run: `cargo test -p rza1l-hal -p deluge-bsp --target x86_64-unknown-linux-gnu`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add Cargo.toml Cargo.lock firmwares/sd-bench
git commit -m "feat(sd-bench): SD High-Speed throughput + CRC soak bench firmware"
```

---

### Task 6: Hardware validation (manual — requires the Deluge + J-Link)

**Files:** none (execution + observation only). **This task is a human checkpoint: Kate runs it; an agent cannot.**

- [ ] **Step 1: Flash and run**

Build `cargo build-fw -p sd-bench --features rtt`, load `target/armv7a-none-eabihf/debug/sd-bench` onto the Deluge via the usual J-Link workflow (`rza1_debug.JLinkScript`), attach an RTT viewer.

- [ ] **Step 2: Record results per card**

For each test card (at minimum: one modern SDHC/SDXC, plus the oldest card available), record from the RTT log:
- `sd: High-Speed mode …` or `sd: default speed …` line
- both `bench[…]` KiB/s figures
- `soak: PASS/FAIL` line

Expected outcomes:
- Modern cards: `HS=true`, ~2× KiB/s at P1/2 vs P1/4, `soak: PASS`.
- Old/CMD6-less cards: `HS=false`, init still succeeds, P1/4 numbers unchanged from before this branch.

- [ ] **Step 3 (optional, dev card): write soak**

Set `WRITE_SOAK = true` in `firmwares/sd-bench/src/tasks/bench.rs`, rebuild, rerun, confirm `write soak: verify OK`, then set it back to `false`.

- [ ] **Step 4: Go/no-go**

If the soak fails or throughput doesn't improve, stop and debug (systematic-debugging skill) before Task 7 — do not merge a failing HS path; the fallback keeps `main` shippable regardless.

---

### Task 7: Changelog + final verification

**Files:**
- Modify: `CHANGELOG.md`

- [ ] **Step 1: Add the changelog entry**

Add under the current unreleased/topmost section, following the file's existing entry format:

```markdown
- SD (RZ/A1L): High-Speed mode support — cards that accept the CMD6 switch now
  run at 33.3 MHz SD_CLK (P1/2) instead of 16.7 MHz, ~2× sequential throughput;
  automatic fallback to 16.7 MHz for cards without CMD6.
```

- [ ] **Step 2: Full verification sweep**

Run and confirm all pass/build:

```bash
cargo test -p rza1l-hal -p deluge-bsp --target x86_64-unknown-linux-gnu
cargo build-fw -p rza1l-hal
cargo build-fw -p msc-firmware
cargo build-fw -p sd-bench --features rtt
```

- [ ] **Step 3: Commit**

```bash
git add CHANGELOG.md
git commit -m "docs: changelog entry for RZ/A1L SD High-Speed support"
```

- [ ] **Step 4: Finish the branch**

Use the superpowers:finishing-a-development-branch skill to decide merge/PR handling for `sd-high-speed`.
