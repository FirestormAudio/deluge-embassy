//! Friendly names for Deluge front-panel controls.
//!
//! This module collects stable, human-readable IDs for physical buttons,
//! encoder shaft clicks, encoder rotation IDs, and gold-knob indicator bars.
//! It intentionally lives outside [`crate::pic`] because encoder rotation is
//! decoded by the main CPU GPIO/IRQ path rather than by the PIC UART protocol.

/// Named physical button IDs in the raw PIC button-ID space (`0..=35`).
///
/// These are derived from `hid/button.h` in DelugeFirmware and verified against
/// the Spark front-end `protocol.rs` `led_to_index` table. For physical buttons
/// with indicator LEDs, the LED index equals the raw button ID.
pub mod button {
    pub const ENCODER_FUNCTION_1: u8 = 1; // Zmod0
    pub const ENCODER_FUNCTION_5: u8 = 2; // Zmod4
    pub const SCOPE: u8 = 3;
    pub const TIME: u8 = 5;
    pub const SCALE: u8 = 6;
    pub const COPY: u8 = 7;
    pub const SHIFT: u8 = 8;
    pub const ENCODER_FUNCTION_2: u8 = 10; // Zmod1
    pub const ENCODER_FUNCTION_6: u8 = 11; // Zmod5
    pub const SESSION: u8 = 12;
    pub const QUANTIZE: u8 = 14;
    pub const LOAD: u8 = 15;
    pub const BACK: u8 = 16;
    pub const SELECT: u8 = 17;
    pub const ENCODER_FUNCTION_3: u8 = 19; // Zmod2
    pub const ENCODER_FUNCTION_7: u8 = 20; // Zmod6
    pub const CLIP: u8 = 21;
    pub const AUTOMATION: u8 = 23;
    pub const LOOP: u8 = 24;
    pub const FILL: u8 = 25;
    pub const RECORD: u8 = 26;
    pub const ENCODER_FUNCTION_4: u8 = 28; // Zmod3
    pub const ENCODER_FUNCTION_8: u8 = 29; // Zmod7
    pub const KEYBOARD: u8 = 30;
    pub const TRANSFORM: u8 = 32;
    pub const SAVE: u8 = 33;
    pub const TAP_TEMPO: u8 = 34;
    pub const PLAY: u8 = 35;
}

/// Friendly names for the six encoder push-buttons in the raw PIC button-ID space.
///
/// These shaft-click events are reported by the PIC as button presses and are
/// later forwarded on the CDC wire as `144 + raw_id`.
pub mod encoder_button {
    pub const SCROLL_Y: u8 = 0;
    pub const SCROLL_X: u8 = 9;
    pub const TEMPO: u8 = 13;
    pub const MOD_0: u8 = 18;
    pub const MOD_1: u8 = 27;
    pub const SELECT: u8 = 31;
}

/// Friendly names for encoder rotation IDs emitted by the CPU-owned encoder task.
pub mod encoder {
    pub const SCROLL_X: u8 = 0;
    pub const TEMPO: u8 = 1;
    pub const MOD_0: u8 = 2;
    pub const MOD_1: u8 = 3;
    pub const SCROLL_Y: u8 = 4;
    pub const SELECT: u8 = 5;
}

/// Friendly names for the two gold-knob indicator bars.
pub mod knob {
    pub const MOD_0: u8 = 0;
    pub const MOD_1: u8 = 1;
}
