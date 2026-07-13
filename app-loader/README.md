# app-loader

Second-stage bootloader / **app loader** for the [Synthstrom Deluge] (RZ/A1L).
The Deluge first-stage bootloader loads this image from SPI flash into SRAM at
`0x20020000`; the app loader then picks and launches an application firmware
image from the SD card or on-board flash, can store an SD app into the on-board
flash slot, accepts a direct USB upload in **dev mode**, and provides a USB
data-transfer mode.

| | |
|---|---|
| Package | `app-loader` |
| Binary | `app-loader` |
| Target | `armv7a-none-eabihf` (RZ/A1L, Cortex-A9) |
| Loaded at | SRAM `0x20020000` (by the first-stage bootloader) |

## Boot sequence

1. Initialise the platform (MMU, caches, SDRAM, GIC, OSTM).
2. Mount the SD card's first FAT volume.
3. Enumerate ELF application images from `/APPS/` on the card.
4. Present a GRUB-style boot menu on the OLED with encoder-wheel selection and a
   configurable auto-boot of the default entry (`AUTO-BOOT`, 5 s out of the box;
   see `SETTINGS` below). A valid on-flash
   image is listed first (`BOOT FLASH`) and is the auto-boot default.
5. **Short-press** SELECT to launch the highlighted entry: stream the selected
   ELF, load its `PT_LOAD` segments to their physical addresses, flush all
   caches, and branch to `e_entry`.

The menu also exposes two synthetic entries:

- **`DATA TRANSFER`** — expose the raw SD card over USB Mass Storage.
- **`SETTINGS`** — opens the settings screen: the `AUTO-BOOT` delay
  (`INSTANT` / `1S`–`20S` / `NEVER`) and the persistent dev-mode flag. Written to
  flash on exit.
- **Recovery** — hold **SELECT** while powering on to force the boot menu with no
  countdown, whatever `AUTO-BOOT` is set to.

### Dev mode (USB upload-and-run)

Dev mode is a **persistent, default-off** setting stored in the SPI-flash
**settings sector** (`0x40_0000`, one 256 KB sector above the app slot, outside
the FSB / settings / SSB / app-slot regions — see the `spibsc` flash map and the
second `writable()` window that guards it). Because it survives reboots and is
independent of the SD card, a stock unit never accepts firmware over USB until
the user explicitly turns it on.

While dev mode is **on**, the loader:

- runs a USB **CDC-ACM upload listener** in the background alongside the boot
  menu (`src/devupload.rs`) — there is no separate "upload mode" to enter; and
- **disables the auto-boot countdown**, so the unit waits indefinitely on the
  menu for either a selection or an upload.

When the host (`cargo deluge run`) pushes a framed ELF
(`DLUP | version | flags | len | crc32 | <ELF>`), the listener streams it into a
high-SDRAM scratch window, validates the CRC, loads its `PT_LOAD` segments to RAM
(`elf::load_from_slice`), and launches it — exactly like the SD `/APPS/` path, but
sourced from USB and with no SD shuffling. A menu selection while listening tears
the CDC device down first so a later `DATA TRANSFER` MSC init has the port free.

### Store an app to flash

**Long-press** SELECT on an SD `/APPS` ELF entry to pop up a `WRITE TO FLASH?`
prompt. Choosing `YES` flattens the ELF into a flat `.bin`, validates its FSB
metadata, and programs it into the flash app slot (`0x100000`, above the FSB /
settings / SSB, which the `spibsc` `writable()` guard physically protects). The
stored image then becomes the first/default `BOOT FLASH` entry and the unit can
boot it with no SD card present. Only fully SRAM-linked images can be stored.

## Modules

`src/` —

| Module | Role |
|--------|------|
| [`elf`](src/elf.rs) | Streaming ELF32-LE loader for ARM firmware images, slice loader (`load_from_slice`) + flatten-to-flash-staging |
| [`devupload`](src/devupload.rs) | Dev-mode USB CDC upload listener: receive a framed ELF, load it to RAM, and launch |
| [`settings`](src/settings.rs) | Persistent dev-mode flag in the SPI-flash settings sector (format in `deluge-image`) |
| [`file_browser`](src/file_browser.rs) | Enumerates loadable ELF images from `/APPS` on the SD card |
| [`flashboot`](src/flashboot.rs) | Probes/launches a firmware image stored in SPI flash, and programs the flash slot |
| [`launcher`](src/launcher.rs) | Cache flush + branch-to-application handoff |
| [`ui`](src/ui.rs) | OLED + encoder boot menu (128×48), long-press detection, `WRITE TO FLASH?` prompt |
| [`usbmsc`](src/usbmsc.rs) | USB MSC DATA TRANSFER mode (raw SD card) |

## Building

Run from the workspace root (aliases in `.cargo/config.toml` supply the required
`-Zbuild-std=core` flags):

| Command | Output |
|---------|--------|
| `cargo build-app-loader` | `target/armv7a-none-eabihf/debug/app-loader` (debug ELF) |
| `cargo build-app-loader-bin` | `target/armv7a-none-eabihf/release/app-loader.bin` (raw image — flash as the device firmware) |

Install `app-loader.bin` onto a unit with the [Device setup guide](../docs/device-setup.md);
see the [workspace README](../README.md) for probe-based flashing and debugging.

SD `/APPS` images are ordinary RAM-linked firmware ELFs (the same ones the
`firmwares/` builds and `cargo deluge` produce).

[Synthstrom Deluge]: https://synthstrom.com/product/deluge/
