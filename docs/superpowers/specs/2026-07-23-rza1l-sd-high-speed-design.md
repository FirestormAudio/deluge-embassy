# RZ/A1L SD High-Speed (33.3 MHz) Support — Design

**Date:** 2026-07-23
**Scope:** `rza1l-hal` + `deluge-bsp` + a new bench firmware. RZ/A2M explicitly out of scope.

## Background

The RZ/A1L SDHI supports SD High-Speed mode timing (TRM ch38, "Transfer mode:
Default mode, High-Speed mode") with SD_CLK = P1φ/2ⁿ, n = 1–9. The electrical
ceiling is P1/2 ≈ 33.3 MHz (ch47 table 47.26: tSDPP min = 2 × tp1cyc) — no UHS
(no 1.8 V signaling), so the full 50 MHz HS rate is unreachable, but 33.3 MHz
is double what we run today.

Current state: `rza1l-hal/src/sdhi.rs` runs the card at `CLK_DIV_4` = P1/4
≈ 16.7 MHz after init, and `deluge-bsp/src/sd.rs::run_protocol` never issues
CMD6 (SWITCH_FUNC). 16.7 MHz was the correct stopping point without CMD6:
default-speed cards are only guaranteed to 25 MHz, and the next divider step
(P1/2 = 33.3 MHz) exceeds that. Switching the card to High-Speed mode via CMD6
makes 33.3 MHz legal (HS limit is 50 MHz).

**Goal:** ~2× raw SD bus throughput (directly relevant to Sa-3b sample
streaming), with strictly-no-worse behavior for cards that can't switch.

## 1. HAL primitives (`crates/rza1l-hal/src/sdhi.rs`)

Two additions; no behavior change for existing callers.

### `set_clock_div(port: u8, div: u16)`

Generalization of `set_clock_fast`: same SCLKDIVEN wait-before/wait-after
dance, parameterized divider. Add `CLK_DIV_2: u16 = 0x00` (P1/2 ≈ 33.3 MHz).
`set_clock_fast` becomes a thin wrapper (`set_clock_div(port, CLK_DIV_4)`).
The `Sdhi` struct wrapper gains a matching method.

Const-assertion test updated for `CLK_DIV_2`: the divider field is one-hot
*except* that `0x00` is the sole legal all-zero encoding (P1/2); the
`count_ones() == 1` assert pattern gets that carve-out.

### `read_status_block_sw(port: u8, buf: &mut [u8; 64])`

64-byte single-block software (FIFO) read for CMD6's switch-status block.
`read_blocks_sw` hardcodes 512-byte drains and `init` sets `SD_SIZE = 512`,
so this variant:

1. Sets `SD_SIZE = 64` (respecting the "don't write while SCLKDIVEN = 0" rule).
2. Waits for BRE, drains 16 FIFO words.
3. Waits for `INFO1_DATA_TRNS`.
4. Restores `SD_SIZE = 512` — **on the error path too**.

Reuses the existing poll_fn/interrupt machinery (same masks/waker pattern as
`read_blocks_sw`).

### CMD6 issue mode (implementation-time TRM lookup)

CMD6 is a data-transfer command the SDHI does not auto-decode, so it must be
issued with SD_CMD extended-mode bits (data present, read direction, single
block, R1 response) per the ch38 SD_CMD field table — the same mechanism as
the existing `CMD18 | 0x7C00` auto-CMD12-suppression comment. The exact bit
values are looked up from ch38 during implementation, not guessed.

## 2. BSP protocol change (`crates/deluge-bsp/src/sd.rs`)

New private `async fn try_high_speed() -> Result<(), SdError>`, called from
`run_protocol` after ACMD6/CMD16, replacing the bare `set_clock_fast` call:

1. **CMD6 query** — arg `0x00FF_FF01` (mode 0, group 1 → function 1, all other
   groups no-change). Read 64-byte status; check group-1 support bit for
   function 1 (byte 13, bit 1; status bit 401).
2. **CMD6 switch** — arg `0x80FF_FF01` (mode 1). Read status; verify group-1
   selected-function nibble (byte 16 low nibble; status bits 379:376) == `0x1`
   (`0xF` = switch refused).
3. On success: `set_clock_div(SD_PORT, CLK_DIV_2)` → 33.3 MHz.
4. **Any failure is non-fatal**: CMD6 rejected as illegal (pre-spec-1.10
   card), function unsupported, refused switch, or CRC/timeout on the status
   read → log and fall back to `set_clock_fast()` (P1/4 — today's behavior).
   No SCR/ACMD51 read: old cards simply error on CMD6, which *is* the
   fallback path.

Status-block parsing is a pure function
(`parse_switch_status(&[u8; 64]) -> SwitchStatus`) so it is unit-testable
host-side (byte order note: status block arrives MSB-first — `buf[0]` holds
bits 511:504; `EXT_SWAP` stays 0).

Log the final clock mode at info level so bench and field logs always show
which mode a card landed in.

## 3. Bench/validation firmware (`firmwares/sd-bench`)

New tiny firmware following the `wp-probe` pattern:

- Runs card init; logs CMD6 query + switch status bytes and the resulting
  clock mode.
- Sequential multi-block DMA read throughput (MB/s) over a few hundred MB —
  once forced to P1/4, once at P1/2; prints both.
- CRC soak: repeated reads of the same region, comparing checksums across
  passes. Optional write-soak confined to a scratch region near the end of
  the card (clearly logged; dev cards only).

**Acceptance:** zero CRC/compare errors over the soak on the test cards;
read throughput ≈ 2× at 33.3 MHz.

## 4. Error handling & rollout

- BSP falls back to 16.7 MHz on any HS-path error → worst case equals
  today's behavior. Bench first (per validation choice), then land the BSP
  wiring.
- SD_OPTION timeout unchanged: counted in SD_CLK cycles, so wall-clock
  timeouts halve at 33 MHz — 2²⁷ cycles ≈ 4 s, still ample.
- No runtime CRC-error downshift (approach B): the `set_clock_div` seam makes
  it a small follow-up if the soak shows marginal cards.
- Host-side tests: `parse_switch_status` unit tests + divider const
  assertions. On-device: sd-bench.

## Out of scope

- RZ/A2M SDHI (different IP; gets its own treatment later).
- UHS modes (hardware cannot: no 1.8 V signaling, divider floor is P1/2).
- Runtime downshift on accumulated CRC errors (follow-up if soak warrants).
- DMA-path changes: DMA read/write is block-size-agnostic at 512 B and
  untouched; the 64-byte status read is software-FIFO only.
