//! Rich colour maths for the pad grid, as an extension trait on the BSP's
//! [`Color`].
//!
//! The plain [`Color`] struct (channels, named constants, `rgb`/`scale`/`hsv`/
//! `monochrome`/`dim`) lives in `deluge-bsp` so the permissive SDK can share it.
//! Everything heavier — HSV-from-float, hue ramps, blending, tinting, derived
//! tail/blur colours, the rotation matrix — is grid-specific and lives here in
//! the GPL toolkit. Bring [`ColorExt`] into scope to get the method/constructor
//! syntax (`Color::from_hsv(..)`, `c.blend(..)`, `c.for_tail()`, …).
//!
//! Ported from the Deluge C++ `RGB` class.

#[allow(unused_imports)] // needed on targets whose `core` lacks inherent f32 math
use crate::float_ext::F32Ext as _;
use deluge_bsp::rgb::Color;

/// Grid colour maths layered onto [`Color`].
pub trait ColorExt: Sized {
    // ── constructors ────────────────────────────────────────────────────────
    fn new(r: u8, g: u8, b: u8) -> Color;
    fn white() -> Color;
    fn black() -> Color;
    fn red() -> Color;
    fn green() -> Color;
    fn blue() -> Color;
    fn yellow() -> Color;
    fn cyan() -> Color;
    fn magenta() -> Color;
    fn orange() -> Color;
    fn purple() -> Color;
    fn pink() -> Color;
    fn lime() -> Color;
    fn teal() -> Color;
    fn indigo() -> Color;
    fn violet() -> Color;
    fn brown() -> Color;
    fn gold() -> Color;
    fn silver() -> Color;
    fn navy() -> Color;
    fn maroon() -> Color;
    fn olive() -> Color;

    /// Construct a colour from a Deluge hue value (wrapped to 0–191).
    fn from_hue(hue: i32) -> Color;
    /// Construct a pastel colour from a Deluge hue value (wrapped to 0–191).
    fn from_hue_pastel(hue: i32) -> Color;
    /// Construct from HSV with float inputs (`hue` in degrees, `s`/`v` 0–1).
    fn from_hsv(hue: f32, saturation: f32, value: f32) -> Color;
    /// Construct from a `[r, g, b]` array.
    fn from_array(arr: [u8; 3]) -> Color;

    // ── transforms ──────────────────────────────────────────────────────────
    fn to_hsv(&self) -> (f32, f32, f32);
    fn dim_float(&self, factor: f32) -> Color;
    fn brighten(&self, factor: f32) -> Color;
    fn blend(&self, other: Color, factor: f32) -> Color;
    fn blend_static(source_a: Color, source_b: Color, index: u16) -> Color;
    fn blend2(source_a: Color, source_b: Color, index_a: u16, index_b: u16) -> Color;
    fn dull(&self) -> Color;
    fn grey_out(&self, proportion: i32) -> Color;
    fn adjust(&self, intensity: u8, brightness_divider: u8) -> Color;
    fn adjust_fractional(&self, numerator: u16, divisor: u16) -> Color;
    fn rotate(&self) -> Color;
    fn for_tail(&self) -> Color;
    fn for_blur(&self) -> Color;
    fn average(a: Color, b: Color) -> Color;
    fn transform<F: Fn(u8) -> u8>(&self, f: F) -> Color;
    fn transform2<F: Fn(u8, u8) -> u8>(a: Color, b: Color, f: F) -> Color;
    fn to_array(&self) -> [u8; 3];
}

impl ColorExt for Color {
    #[inline]
    fn new(r: u8, g: u8, b: u8) -> Color {
        Color { r, g, b }
    }
    #[inline]
    fn white() -> Color {
        Color::rgb(255, 255, 255)
    }
    #[inline]
    fn black() -> Color {
        Color::rgb(0, 0, 0)
    }
    #[inline]
    fn red() -> Color {
        Color::rgb(255, 0, 0)
    }
    #[inline]
    fn green() -> Color {
        Color::rgb(0, 255, 0)
    }
    #[inline]
    fn blue() -> Color {
        Color::rgb(0, 0, 255)
    }
    #[inline]
    fn yellow() -> Color {
        Color::rgb(255, 255, 0)
    }
    #[inline]
    fn cyan() -> Color {
        Color::rgb(0, 255, 255)
    }
    #[inline]
    fn magenta() -> Color {
        Color::rgb(255, 0, 255)
    }
    #[inline]
    fn orange() -> Color {
        Color::rgb(255, 165, 0)
    }
    #[inline]
    fn purple() -> Color {
        Color::rgb(128, 0, 128)
    }
    #[inline]
    fn pink() -> Color {
        Color::rgb(255, 192, 203)
    }
    #[inline]
    fn lime() -> Color {
        Color::rgb(191, 255, 0)
    }
    #[inline]
    fn teal() -> Color {
        Color::rgb(0, 128, 128)
    }
    #[inline]
    fn indigo() -> Color {
        Color::rgb(75, 0, 130)
    }
    #[inline]
    fn violet() -> Color {
        Color::rgb(238, 130, 238)
    }
    #[inline]
    fn brown() -> Color {
        Color::rgb(165, 42, 42)
    }
    #[inline]
    fn gold() -> Color {
        Color::rgb(255, 215, 0)
    }
    #[inline]
    fn silver() -> Color {
        Color::rgb(192, 192, 192)
    }
    #[inline]
    fn navy() -> Color {
        Color::rgb(0, 0, 128)
    }
    #[inline]
    fn maroon() -> Color {
        Color::rgb(128, 0, 0)
    }
    #[inline]
    fn olive() -> Color {
        Color::rgb(128, 128, 0)
    }

    fn from_hue(hue: i32) -> Color {
        let hue = ((hue + 1920) % 192) as u16;
        let mut ch = [0u8; 3];
        for c in 0..3 {
            let channel_darkness = if c == 0 {
                if hue < 64 {
                    hue as i32
                } else {
                    (64_i32).min((192 - hue as i32).abs())
                }
            } else {
                (64_i32).min((c * 64 - hue as i32).abs())
            };
            if channel_darkness < 64 {
                let angle = ((channel_darkness << 3) + 256) & 1023;
                let sine_value = get_sine(angle, 10);
                let adjusted = (sine_value as i64 + (u32::MAX as i64 / 2)) as u32;
                ch[c as usize] = (adjusted >> 24) as u8;
            } else {
                ch[c as usize] = 0;
            }
        }
        Color::rgb(ch[0], ch[1], ch[2])
    }

    fn from_hue_pastel(hue: i32) -> Color {
        const MAX_PASTEL: u32 = 230;
        let hue = ((hue + 1920) % 192) as u16;
        let mut ch = [0u8; 3];
        for c in 0..3 {
            let channel_darkness = if c == 0 {
                if hue < 64 {
                    hue as i32
                } else {
                    (64_i32).min((192 - hue as i32).abs())
                }
            } else {
                (64_i32).min((c * 64 - hue as i32).abs())
            };
            if channel_darkness < 64 {
                let angle = ((channel_darkness << 3) + 256) & 1023;
                let sine_value = get_sine(angle, 10);
                let basic_value = (sine_value as i64 + (u32::MAX as i64 / 2)) as u32;
                let flipped = u32::MAX - basic_value;
                let flipped_scaled = (flipped >> 8) * MAX_PASTEL;
                ch[c as usize] = ((u32::MAX - flipped_scaled) >> 24) as u8;
            } else {
                ch[c as usize] = (256 - MAX_PASTEL) as u8;
            }
        }
        Color::rgb(ch[0], ch[1], ch[2])
    }

    fn from_hsv(hue: f32, saturation: f32, value: f32) -> Color {
        let saturation = saturation.clamp(0.0, 1.0);
        let value = value.clamp(0.0, 1.0);
        let hue = hue % 360.0;
        let hue = if hue < 0.0 { hue + 360.0 } else { hue };

        let c = value * saturation;
        let h = hue / 60.0;
        let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
        let m = value - c;

        let (r, g, b) = if h < 1.0 {
            (c, x, 0.0)
        } else if h < 2.0 {
            (x, c, 0.0)
        } else if h < 3.0 {
            (0.0, c, x)
        } else if h < 4.0 {
            (0.0, x, c)
        } else if h < 5.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };

        Color::rgb(
            ((r + m) * 255.0) as u8,
            ((g + m) * 255.0) as u8,
            ((b + m) * 255.0) as u8,
        )
    }

    #[inline]
    fn from_array(arr: [u8; 3]) -> Color {
        Color::rgb(arr[0], arr[1], arr[2])
    }

    fn to_hsv(&self) -> (f32, f32, f32) {
        let r = self.r as f32 / 255.0;
        let g = self.g as f32 / 255.0;
        let b = self.b as f32 / 255.0;

        let max = r.max(g).max(b);
        let min = r.min(g).min(b);
        let delta = max - min;

        let value = max;
        let saturation = if max == 0.0 { 0.0 } else { delta / max };
        let hue = if delta == 0.0 {
            0.0
        } else if max == r {
            60.0 * (((g - b) / delta) % 6.0)
        } else if max == g {
            60.0 * (((b - r) / delta) + 2.0)
        } else {
            60.0 * (((r - g) / delta) + 4.0)
        };
        let hue = if hue < 0.0 { hue + 360.0 } else { hue };

        (hue, saturation, value)
    }

    fn dim_float(&self, factor: f32) -> Color {
        Color::rgb(
            (self.r as f32 * factor).clamp(0.0, 255.0) as u8,
            (self.g as f32 * factor).clamp(0.0, 255.0) as u8,
            (self.b as f32 * factor).clamp(0.0, 255.0) as u8,
        )
    }

    fn brighten(&self, factor: f32) -> Color {
        Color::rgb(
            (self.r as f32 + (255.0 - self.r as f32) * factor).clamp(0.0, 255.0) as u8,
            (self.g as f32 + (255.0 - self.g as f32) * factor).clamp(0.0, 255.0) as u8,
            (self.b as f32 + (255.0 - self.b as f32) * factor).clamp(0.0, 255.0) as u8,
        )
    }

    fn blend(&self, other: Color, factor: f32) -> Color {
        let index = (factor.clamp(0.0, 1.0) * u16::MAX as f32) as u16;
        Self::blend_static(*self, other, index)
    }

    fn blend_static(source_a: Color, source_b: Color, index: u16) -> Color {
        Self::transform2(source_a, source_b, |a, b| blend_channel(a, b, index))
    }

    fn blend2(source_a: Color, source_b: Color, index_a: u16, index_b: u16) -> Color {
        Self::transform2(source_a, source_b, |a, b| {
            blend_channel2(a, b, index_a, index_b)
        })
    }

    fn dull(&self) -> Color {
        self.transform(|channel| channel.clamp(5, 50))
    }

    fn grey_out(&self, proportion: i32) -> Color {
        let total_rgb = self.r as u32 + self.g as u32 + self.b as u32;
        self.transform(|channel| {
            let val = rshift_round(
                channel as u32 * (0x808080 - proportion as u32)
                    + (total_rgb * (proportion as u32 >> 5)),
                23,
            );
            val.clamp(0, u8::MAX as u32) as u8
        })
    }

    fn adjust(&self, intensity: u8, brightness_divider: u8) -> Color {
        self.transform(|channel| {
            ((channel as u32 * intensity as u32 / 255) / brightness_divider as u32) as u8
        })
    }

    fn adjust_fractional(&self, numerator: u16, divisor: u16) -> Color {
        self.transform(|channel| ((channel as u32 * numerator as u32) / divisor as u32) as u8)
    }

    fn rotate(&self) -> Color {
        xform(self, &R_MAT)
    }

    fn for_tail(&self) -> Color {
        let average_brightness = self.r as u32 + self.g as u32 + self.b as u32;
        self.transform(|channel| {
            (((channel as i32 * 21 + average_brightness as i32) * 120) >> 14) as u8
        })
    }

    fn for_blur(&self) -> Color {
        let average_brightness = self.r as u32 * 5 + self.g as u32 * 9 + self.b as u32 * 9;
        self.transform(|channel| ((channel as u32 * 5 + average_brightness) >> 5) as u8)
    }

    fn average(a: Color, b: Color) -> Color {
        Self::transform2(a, b, |a, b| {
            ((a as u32 + b as u32) / 2).clamp(0, u8::MAX as u32) as u8
        })
    }

    fn transform<F: Fn(u8) -> u8>(&self, f: F) -> Color {
        Color::rgb(f(self.r), f(self.g), f(self.b))
    }

    fn transform2<F: Fn(u8, u8) -> u8>(a: Color, b: Color, f: F) -> Color {
        Color::rgb(f(a.r, b.r), f(a.g, b.g), f(a.b, b.b))
    }

    #[inline]
    fn to_array(&self) -> [u8; 3] {
        [self.r, self.g, self.b]
    }
}

// ── private maths ────────────────────────────────────────────────────────────

fn xform(c: &Color, mat: &[[u32; 4]; 4]) -> Color {
    Color::rgb(
        ((c.r as u32 * mat[0][0] + c.g as u32 * mat[1][0] + c.b as u32 * mat[2][0] + mat[3][0])
            >> 16) as u8,
        ((c.r as u32 * mat[0][1] + c.g as u32 * mat[1][1] + c.b as u32 * mat[2][1] + mat[3][1])
            >> 16) as u8,
        ((c.r as u32 * mat[0][2] + c.g as u32 * mat[1][2] + c.b as u32 * mat[2][2] + mat[3][2])
            >> 16) as u8,
    )
}

fn blend_channel(channel_a: u8, channel_b: u8, index: u16) -> u8 {
    let complement = u16::MAX.saturating_sub(index);
    blend_channel2(channel_a, channel_b, index, complement)
}

fn blend_channel2(channel_a: u8, channel_b: u8, index_a: u16, index_b: u16) -> u8 {
    let new_rgb = rshift_round(channel_a as u32 * index_a as u32, 16)
        + rshift_round(channel_b as u32 * index_b as u32, 16);
    new_rgb.clamp(0, u8::MAX as u32) as u8
}

const ONE_Q16: u32 = 65536;
const C: f32 = 0.5403;
const S: f32 = 0.8414;

const R_MAT: [[u32; 4]; 4] = [
    [
        (C * ONE_Q16 as f32) as u32,
        0,
        (S * ONE_Q16 as f32) as u32,
        0,
    ],
    [
        (S * ONE_Q16 as f32) as u32,
        (C * ONE_Q16 as f32) as u32,
        0,
        0,
    ],
    [
        0,
        (S * ONE_Q16 as f32) as u32,
        (C * ONE_Q16 as f32) as u32,
        0,
    ],
    [0, 0, 0, ONE_Q16],
];

const fn rshift_round(val: u32, shift: u32) -> u32 {
    (val + (1 << (shift - 1))) >> shift
}

const SINE_WAVE_SMALL: [i16; 257] = [
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787,
    21403, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683,
    28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113, 31356, 31580, 31785,
    31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678,
    32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571, 30273,
    29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811,
    24279, 23731, 23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846,
    16151, 15446, 14732, 14010, 13279, 12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179, 6393,
    5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804, -1608, -2410, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732,
    -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510,
    -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757, -32767, -32757, -32728, -32678,
    -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832,
    -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279, -12539, -11793, -11039, -10278,
    -9512, -8739, -7962, -7179, -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804, 0,
];

fn interpolate_table_signed(
    input: u32,
    num_bits_in_input: u32,
    table: &[i16],
    num_bits_in_table_size: u32,
) -> i32 {
    let which_value = (input >> (num_bits_in_input - num_bits_in_table_size)) as usize;
    let rshift_amount = num_bits_in_input as i32 - 16 - num_bits_in_table_size as i32;
    let rshifted = if rshift_amount >= 0 {
        input >> rshift_amount
    } else {
        input << (-rshift_amount)
    };
    let strength2 = (rshifted & 0xFFFF) as i32;
    let strength1 = 0x10000 - strength2;
    table[which_value] as i32 * strength1 + table[which_value + 1] as i32 * strength2
}

fn get_sine(phase: i32, num_bits_in_input: u32) -> i32 {
    interpolate_table_signed(phase as u32, num_bits_in_input, &SINE_WAVE_SMALL, 8)
}

// ── pixel-slice lerp (scalar; SIMD-accelerated under `feature = "simd"`) ──────

/// Linearly interpolate a single colour from `a` to `b` by `progress` (0–1).
#[inline]
pub fn lerp(a: Color, b: Color, progress: f32) -> Color {
    let p = progress.clamp(0.0, 1.0);
    Color::rgb(
        (a.r as f32 * (1.0 - p) + b.r as f32 * p) as u8,
        (a.g as f32 * (1.0 - p) + b.g as f32 * p) as u8,
        (a.b as f32 * (1.0 - p) + b.b as f32 * p) as u8,
    )
}

/// Interpolate two equal-length colour slices into `output`.
///
/// With `feature = "simd"` the bulk is processed 16 pixels at a time via NEON;
/// otherwise (and for the trailing remainder) a scalar path is used.
///
/// # Panics
/// If the three slices do not all have the same length.
pub fn lerp_slice(from: &[Color], to: &[Color], progress: f32, output: &mut [Color]) {
    assert_eq!(from.len(), to.len(), "input slices must have same length");
    assert_eq!(from.len(), output.len(), "output must match input length");

    #[cfg(feature = "simd")]
    let start = {
        let chunks = from.len() / 16;
        for i in 0..chunks {
            let base = i * 16;
            simd::lerp_chunk_16(
                from[base..base + 16].try_into().unwrap(),
                to[base..base + 16].try_into().unwrap(),
                progress,
                (&mut output[base..base + 16]).try_into().unwrap(),
            );
        }
        chunks * 16
    };
    #[cfg(not(feature = "simd"))]
    let start = 0;

    for i in start..from.len() {
        output[i] = lerp(from[i], to[i], progress);
    }
}

/// Allocating convenience wrapper around [`lerp_slice`].
pub fn lerp_slice_vec(from: &[Color], to: &[Color], progress: f32) -> alloc::vec::Vec<Color> {
    let mut output = alloc::vec![Color::BLACK; from.len()];
    lerp_slice(from, to, progress, &mut output);
    output
}

#[cfg(feature = "simd")]
mod simd {
    use core::simd::prelude::*;
    use deluge_bsp::rgb::Color;

    #[cfg(target_arch = "aarch64")]
    use core::arch::aarch64::*;
    #[cfg(target_arch = "arm")]
    use core::arch::arm::*;

    #[cfg(any(target_arch = "aarch64", target_arch = "arm"))]
    fn deinterleave_16(pixels: &[Color; 16]) -> (Simd<u8, 16>, Simd<u8, 16>, Simd<u8, 16>) {
        unsafe {
            let rgb_data = vld3q_u8(pixels.as_ptr() as *const u8);
            (rgb_data.0.into(), rgb_data.1.into(), rgb_data.2.into())
        }
    }
    #[cfg(any(target_arch = "aarch64", target_arch = "arm"))]
    fn reinterleave_16(r: Simd<u8, 16>, g: Simd<u8, 16>, b: Simd<u8, 16>) -> [Color; 16] {
        let mut output = [Color::BLACK; 16];
        let rgb_tuple = uint8x16x3_t(r.into(), g.into(), b.into());
        unsafe {
            vst3q_u8(output.as_mut_ptr() as *mut u8, rgb_tuple);
        }
        output
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "arm")))]
    fn deinterleave_16(pixels: &[Color; 16]) -> (Simd<u8, 16>, Simd<u8, 16>, Simd<u8, 16>) {
        let mut r = [0u8; 16];
        let mut g = [0u8; 16];
        let mut b = [0u8; 16];
        for i in 0..16 {
            r[i] = pixels[i].r;
            g[i] = pixels[i].g;
            b[i] = pixels[i].b;
        }
        (Simd::from(r), Simd::from(g), Simd::from(b))
    }
    #[cfg(not(any(target_arch = "aarch64", target_arch = "arm")))]
    fn reinterleave_16(r: Simd<u8, 16>, g: Simd<u8, 16>, b: Simd<u8, 16>) -> [Color; 16] {
        let mut output = [Color::BLACK; 16];
        for i in 0..16 {
            output[i] = Color::rgb(r[i], g[i], b[i]);
        }
        output
    }

    fn lane_lerp(a: Simd<u8, 16>, b: Simd<u8, 16>, progress: f32) -> Simd<u8, 16> {
        let progress_i16 = (progress.clamp(0.0, 1.0) * 256.0) as i16;
        let a_i16 = a.cast::<i16>();
        let b_i16 = b.cast::<i16>();
        let progress_vec = Simd::<i16, 16>::splat(progress_i16);
        let diff = b_i16 - a_i16;
        let offset = (diff * progress_vec) >> 8;
        (a_i16 + offset).cast::<u8>()
    }

    pub(super) fn lerp_chunk_16(
        from: &[Color; 16],
        to: &[Color; 16],
        progress: f32,
        out: &mut [Color; 16],
    ) {
        let (fr, fg, fb) = deinterleave_16(from);
        let (tr, tg, tb) = deinterleave_16(to);
        let r = lane_lerp(fr, tr, progress);
        let g = lane_lerp(fg, tg, progress);
        let b = lane_lerp(fb, tb, progress);
        *out = reinterleave_16(r, g, b);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_hsv_primaries() {
        assert_eq!(Color::from_hsv(0.0, 1.0, 1.0), Color::rgb(255, 0, 0));
        assert_eq!(Color::from_hsv(120.0, 1.0, 1.0), Color::rgb(0, 255, 0));
        assert_eq!(Color::from_hsv(240.0, 1.0, 1.0), Color::rgb(0, 0, 255));
        assert_eq!(Color::from_hsv(0.0, 0.0, 1.0), Color::WHITE);
    }

    #[test]
    fn hsv_roundtrip() {
        for original in [
            Color::rgb(255, 0, 0),
            Color::rgb(0, 255, 0),
            Color::rgb(0, 0, 255),
            Color::rgb(255, 128, 0),
            Color::rgb(0, 128, 128),
            Color::rgb(192, 192, 192),
        ] {
            let (h, s, v) = original.to_hsv();
            let c = Color::from_hsv(h, s, v);
            assert!(original.r.abs_diff(c.r) <= 1);
            assert!(original.g.abs_diff(c.g) <= 1);
            assert!(original.b.abs_diff(c.b) <= 1);
        }
    }

    #[test]
    fn blend_midpoint() {
        let mid = Color::BLACK.blend(Color::WHITE, 0.5);
        assert!(mid.r > 100 && mid.r < 150);
    }

    #[test]
    fn lerp_slice_endpoints_and_mid() {
        let from = [Color::BLACK; 20];
        let to = [Color::WHITE; 20];
        let mut out = [Color::BLACK; 20];

        lerp_slice(&from, &to, 0.0, &mut out);
        assert!(out.iter().all(|p| *p == Color::BLACK));
        lerp_slice(&from, &to, 1.0, &mut out);
        assert!(out.iter().all(|p| *p == Color::WHITE));
        lerp_slice(&from, &to, 0.5, &mut out);
        assert!(out.iter().all(|p| p.r >= 127 && p.r <= 128));
    }
}
