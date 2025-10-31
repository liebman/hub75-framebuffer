//! Framebuffer implementation for HUB75 LED matrix displays.
//!
//! ## How HUB75 LED Displays Work
//!
//! HUB75 RGB LED matrix panels are scanned, time-multiplexed displays that behave like a long
//! daisy-chained shift register rather than a random-access framebuffer.
//!
//! ### Signal names
//! - **R1 G1 B1 / R2 G2 B2** – Serial colour data for the upper and lower halves of the active scan line
//! - **CLK** – Shift-register clock; every rising edge pushes the six colour bits one pixel to the right
//! - **LAT / STB** – Latch; copies the shift-register contents to the LED drivers for the row currently selected by the address lines
//! - **OE** – Output-Enable (active LOW): LEDs are lit while OE is LOW and blanked when it is HIGH
//! - **A B C D (E)** – Row-address select lines (choose which pair of rows is lit)
//! - **VCC & GND** – 5 V power for panel logic and LED drivers
//!
//! ### Row-pair scanning workflow (e.g., 1/16-scan panel)
//! 1. While the panel is still displaying row pair N − 1, the controller shifts the six-bit colour data for row pair N into the chain (OE remains LOW so row N − 1 stays visible).
//! 2. After the last pixel is clocked in, the controller raises OE HIGH to blank the LEDs.
//! 3. With the panel blanked, it first changes the address lines to select row pair N, lets them settle for a few nanoseconds, and **then** pulses LAT to latch the freshly shifted data into the output drivers for that newly selected row.
//! 4. OE is immediately driven LOW again, lighting row pair N.
//! 5. Steps 1–4 repeat for every row pair fast enough (hundreds of Hz) that the human eye sees a steady image.
//!    - If the first row pair is being shifted, the panel continues showing the last row pair of the previous frame until the first blank-address-latch sequence occurs.
//!
//! ### Brightness and colour depth (Binary Code Modulation)
//! - Full colour is typically achieved using **Binary Code Modulation (BCM)**, also known as *Bit-Angle Modulation (BAM)*. Each bit-plane is displayed for a period proportional to its binary weight (1, 2, 4, 8 …), yielding 2ⁿ intensity levels per channel. See [Batsocks – LED dimming using Binary Code Modulation](https://www.batsocks.co.uk/readme/art_bcm_1.htm) for a deeper explanation.
//! - Because each LED is on for only a fraction of the total frame time, the driver can use relatively high peak currents without overheating while average brightness is preserved.
//!
//! ### Implications for software / hardware drivers
//! - You don't simply "write a pixel" once; you must continuously stream the complete refresh data at MHz-range clock rates.
//! - Precise timing of CLK, OE, address lines, and LAT is critical—especially the order: blank (OE HIGH) → set address → latch → un-blank (OE LOW).
//! - Microcontrollers typically employ DMA, PIO, or parallel GPIO tricks, and FPGAs use dedicated logic, to sustain the data throughput while leaving processing resources free.
//!
//! In short: a HUB75 panel is a high-speed shift-register chain that relies on rapid row-pair scanning and **Binary Code Modulation (BCM)** to create a bright, full-colour image. Keeping OE LOW almost all the time—blanking only long enough to change the address and pulse LAT—maximises brightness without visible artefacts.
//!
//! ## Framebuffer Implementations
//!
//! This module provides two different framebuffer implementations optimized for
//! HUB75 LED matrix displays:
//!
//! 1. **Plain Implementation** (`plain` module)
//!    - No additional hardware requirements
//!    - Simpler implementation suitable for basic displays
//!
//! 2. **Latched Implementation** (`latched` module)
//!    - Requires external latch hardware for address lines
//!
//! Both implementations:
//! - Have configurable row and column dimensions
//! - Support different color depths through Binary Code Modulation (BCM)
//! - Implement the `ReadBuffer` trait for DMA compatibility
//!
//! ## Multiple Panels
//! Use [`tiling::TiledFrameBuffer`] to drive several HUB75 panels as one large
//! virtual display. Combine it with a pixel-remapping policy such as
//! [`tiling::ChainTopRightDown`] and any of the framebuffer flavours above
//! (`plain` or `latched`). The wrapper exposes a single `embedded-graphics`
//! canvas, so for example a 3 × 3 stack of 64 × 32 panels simply looks like a
//! 192 × 96 screen while all coordinate translation happens transparently.
//!
//! ## Available Feature Flags
//!
//! ### `skip-black-pixels` Feature (disabled by default)
//! When enabled, calls to `set_pixel()` with `Color::BLACK` return early without
//! writing to the framebuffer. This provides a significant performance boost for
//! UI applications that frequently draw black pixels (backgrounds, clearing, etc.)
//! by assuming the framebuffer was already cleared.
//!
//! **Important**: This optimization assumes that black pixels represent "no change"
//! rather than "explicitly set to black". By default, black pixels are written
//! normally to ensure correct overwrite behavior. To enable the optimization:
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.6.0", features = ["skip-black-pixels"] }
//! ```
//!
//! ### `esp-hal-dma` Feature (required when using `esp-hal`)
//! **Required** when using the `esp-hal` crate for ESP32 development. This feature
//! switches the `ReadBuffer` trait implementation from `embedded-dma` to `esp-hal::dma`.
//! If you're targeting ESP32 devices with `esp-hal`, you **must** enable this feature
//! for DMA compatibility.
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.6.0", features = ["esp-hal-dma"] }
//! ```
//!
//! ### `esp32-ordering` Feature (required for original ESP32 only)
//! **Required** when targeting the original ESP32 chip (not ESP32-S3 or other variants).
//! This feature adjusts byte ordering to accommodate the quirky requirements of the
//! ESP32's I²S peripheral in 8-bit and 16-bit modes. The original ESP32 has different
//! byte ordering requirements compared to other ESP32 variants (S2, S3, C3, etc.),
//! which do **not** need this feature.
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.6.0", features = ["esp32-ordering"] }
//! ```
//!
//! ### `defmt` Feature
//! Implements `defmt::Format` for framebuffer types so they can be emitted with
//! the `defmt` logging framework. No functional changes; purely adds a trait impl.
//!
//! ### `doc-images` Feature
//! Embeds documentation images when building docs on docs.rs. Not needed for
//! normal usage.
#![no_std]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]

#[cfg(not(feature = "esp-hal-dma"))]
use embedded_dma::ReadBuffer;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::Point;
#[cfg(feature = "esp-hal-dma")]
use esp_hal::dma::ReadBuffer;

pub mod latched;
pub mod plain;
pub mod tiling;

/// Color type used in the framebuffer
pub type Color = Rgb888;

/// Word size configuration for the framebuffer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordSize {
    /// 8-bit word size
    Eight,
    /// 16-bit word size
    Sixteen,
}

/// Computes the NROWS value from ROWS for `DmaFrameBuffer`
///
/// # Arguments
///
/// * `rows` - Total number of rows in the display
///
/// # Returns
///
/// Number of rows needed internally for `DmaFrameBuffer`
#[must_use]
pub const fn compute_rows(rows: usize) -> usize {
    rows / 2
}

/// Computes the number of frames needed for a given bit depth
///
/// This is used to determine how many frames are needed to achieve
/// the desired color depth through Binary Code Modulation (BCM).
///
/// # Arguments
///
/// * `bits` - Number of bits per color channel
///
/// # Returns
///
/// Number of frames required for the given bit depth
#[must_use]
pub const fn compute_frame_count(bits: u8) -> usize {
    (1usize << bits) - 1
}

/// Trait for read-only framebuffers
///
/// This trait defines the basic functionality required for a framebuffer
/// that can be read from and transferred via DMA.
///
/// # Type Parameters
///
/// * `ROWS` - Total number of rows in the display
/// * `COLS` - Number of columns in the display
/// * `NROWS` - Number of rows processed in parallel
/// * `BITS` - Number of bits per color channel
/// * `FRAME_COUNT` - Number of frames needed for BCM
pub trait FrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
>: ReadBuffer
{
    /// Returns the word size configuration for this framebuffer
    fn get_word_size(&self) -> WordSize;
}

/// Trait for mutable framebuffers
///
/// This trait extends `FrameBuffer` with the ability to draw to the framebuffer
/// using the `embedded_graphics` drawing primitives.
///
/// # Type Parameters
///
/// * `ROWS` - Total number of rows in the display
/// * `COLS` - Number of columns in the display
/// * `NROWS` - Number of rows processed in parallel
/// * `BITS` - Number of bits per color channel
/// * `FRAME_COUNT` - Number of frames needed for BCM
pub trait MutableFrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
>:
    FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    + DrawTarget<Color = Color, Error = core::convert::Infallible>
{
}

/// Trait for all operations a user may want to call on a framebuffer.
///
/// # Type Parameters
///
/// * `ROWS` - Total number of rows in the display
/// * `COLS` - Number of columns in the display
/// * `NROWS` - Number of rows processed in parallel
/// * `BITS` - Number of bits per color channel
/// * `FRAME_COUNT` - Number of frames needed for BCM
pub trait FrameBufferOperations<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
>: FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    /// Erase pixel colors while preserving control bits.
    /// This is much faster than `format()` and is the typical way to clear the display.
    fn erase(&mut self);

    /// Set a pixel in the framebuffer.
    fn set_pixel(&mut self, p: Point, color: Color);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::format;

    use super::*;
    use embedded_graphics::pixelcolor::RgbColor;

    #[test]
    fn test_compute_rows() {
        // Test typical panel sizes
        assert_eq!(compute_rows(32), 16);
        assert_eq!(compute_rows(64), 32);
        assert_eq!(compute_rows(16), 8);
        assert_eq!(compute_rows(128), 64);

        // Test edge cases
        assert_eq!(compute_rows(2), 1);
        assert_eq!(compute_rows(0), 0);

        // Test that it always divides by 2
        for rows in [8, 16, 24, 32, 48, 64, 96, 128, 256] {
            assert_eq!(compute_rows(rows), rows / 2);
        }
    }

    #[test]
    fn test_compute_frame_count() {
        // Test common bit depths
        assert_eq!(compute_frame_count(1), 1); // 2^1 - 1 = 1
        assert_eq!(compute_frame_count(2), 3); // 2^2 - 1 = 3
        assert_eq!(compute_frame_count(3), 7); // 2^3 - 1 = 7
        assert_eq!(compute_frame_count(4), 15); // 2^4 - 1 = 15
        assert_eq!(compute_frame_count(5), 31); // 2^5 - 1 = 31
        assert_eq!(compute_frame_count(6), 63); // 2^6 - 1 = 63
        assert_eq!(compute_frame_count(7), 127); // 2^7 - 1 = 127
        assert_eq!(compute_frame_count(8), 255); // 2^8 - 1 = 255

        // Test the formula: (2^bits) - 1
        for bits in 1..=8 {
            let expected = (1usize << bits) - 1;
            assert_eq!(compute_frame_count(bits), expected);
        }
    }

    #[test]
    fn test_compute_frame_count_properties() {
        // Test that frame count grows exponentially
        assert!(compute_frame_count(2) > compute_frame_count(1));
        assert!(compute_frame_count(3) > compute_frame_count(2));
        assert!(compute_frame_count(4) > compute_frame_count(3));

        // Test doubling property: each additional bit approximately doubles frame count
        for bits in 1..=7 {
            let current_frames = compute_frame_count(bits);
            let next_frames = compute_frame_count(bits + 1);
            // next_frames should be approximately 2 * current_frames + 1
            assert_eq!(next_frames, 2 * current_frames + 1);
        }
    }

    #[test]
    fn test_word_size_enum() {
        // Test enum values
        let eight = WordSize::Eight;
        let sixteen = WordSize::Sixteen;

        assert_ne!(eight, sixteen);
        assert_eq!(eight, WordSize::Eight);
        assert_eq!(sixteen, WordSize::Sixteen);
    }

    #[test]
    fn test_word_size_debug() {
        let eight = WordSize::Eight;
        let sixteen = WordSize::Sixteen;

        let eight_debug = format!("{:?}", eight);
        let sixteen_debug = format!("{:?}", sixteen);

        assert_eq!(eight_debug, "Eight");
        assert_eq!(sixteen_debug, "Sixteen");
    }

    #[test]
    fn test_word_size_clone_copy() {
        let original = WordSize::Eight;
        let cloned = original.clone();
        let copied = original;

        assert_eq!(original, cloned);
        assert_eq!(original, copied);
        assert_eq!(cloned, copied);
    }

    #[test]
    fn test_color_type_alias() {
        // Test that Color is an alias for Rgb888
        let red_color: Color = Color::RED;
        let red_rgb888: Rgb888 = Rgb888::RED;

        assert_eq!(red_color, red_rgb888);
        assert_eq!(red_color.r(), 255);
        assert_eq!(red_color.g(), 0);
        assert_eq!(red_color.b(), 0);

        // Test various colors
        let colors = [
            (Color::RED, (255, 0, 0)),
            (Color::GREEN, (0, 255, 0)),
            (Color::BLUE, (0, 0, 255)),
            (Color::WHITE, (255, 255, 255)),
            (Color::BLACK, (0, 0, 0)),
            (Color::CYAN, (0, 255, 255)),
            (Color::MAGENTA, (255, 0, 255)),
            (Color::YELLOW, (255, 255, 0)),
        ];

        for (color, (r, g, b)) in colors {
            assert_eq!(color.r(), r);
            assert_eq!(color.g(), g);
            assert_eq!(color.b(), b);
        }
    }

    #[test]
    fn test_color_construction() {
        // Test Color construction from RGB values
        let custom_color = Color::new(128, 64, 192);
        assert_eq!(custom_color.r(), 128);
        assert_eq!(custom_color.g(), 64);
        assert_eq!(custom_color.b(), 192);

        // Test that it behaves like Rgb888
        let rgb888_color = Rgb888::new(128, 64, 192);
        assert_eq!(custom_color, rgb888_color);
    }

    #[test]
    fn test_helper_functions_const() {
        // Test that helper functions can be used in const contexts
        const ROWS: usize = 32;
        const COMPUTED_NROWS: usize = compute_rows(ROWS);
        const BITS: u8 = 4;
        const COMPUTED_FRAME_COUNT: usize = compute_frame_count(BITS);

        assert_eq!(COMPUTED_NROWS, 16);
        assert_eq!(COMPUTED_FRAME_COUNT, 15);
    }

    #[test]
    fn test_realistic_panel_configurations() {
        // Test common HUB75 panel configurations
        struct PanelConfig {
            rows: usize,
            cols: usize,
            bits: u8,
        }

        let configs = [
            PanelConfig {
                rows: 32,
                cols: 64,
                bits: 3,
            }, // 32x64 panel, 3-bit color
            PanelConfig {
                rows: 64,
                cols: 64,
                bits: 4,
            }, // 64x64 panel, 4-bit color
            PanelConfig {
                rows: 32,
                cols: 32,
                bits: 5,
            }, // 32x32 panel, 5-bit color
            PanelConfig {
                rows: 16,
                cols: 32,
                bits: 6,
            }, // 16x32 panel, 6-bit color
        ];

        for config in configs {
            let nrows = compute_rows(config.rows);
            let frame_count = compute_frame_count(config.bits);

            // Basic sanity checks for rows
            assert!(nrows > 0);
            assert!(nrows <= config.rows);
            assert_eq!(nrows * 2, config.rows);

            // Basic sanity checks for columns
            assert!(config.cols > 0);
            assert!(config.cols <= 256); // Reasonable upper limit for HUB75 panels

            // Frame count checks
            assert!(frame_count > 0);
            assert!(frame_count < 256); // Should be reasonable for typical bit depths

            // Frame count should grow with bit depth
            let prev_frame_count = compute_frame_count(config.bits - 1);
            assert!(frame_count > prev_frame_count);
        }
    }

    #[test]
    fn test_memory_calculations() {
        // Test that we can calculate memory requirements using helper functions
        const ROWS: usize = 64;
        const COLS: usize = 64;
        const BITS: u8 = 4;

        const NROWS: usize = compute_rows(ROWS);
        const FRAME_COUNT: usize = compute_frame_count(BITS);

        // These should be compile-time constants
        assert_eq!(NROWS, 32);
        assert_eq!(FRAME_COUNT, 15);

        // Verify the relationship between parameters
        assert_eq!(NROWS * 2, ROWS);
        assert_eq!(FRAME_COUNT, (1 << BITS) - 1);

        // Verify COLS is reasonable for memory calculations
        assert!(COLS > 0);
        assert!(COLS <= 256); // Reasonable limit for HUB75 panels
    }

    #[test]
    fn test_edge_cases() {
        // Test minimum values
        assert_eq!(compute_rows(2), 1);
        assert_eq!(compute_frame_count(1), 1);

        // Test maximum reasonable values
        assert_eq!(compute_rows(512), 256);
        assert_eq!(compute_frame_count(8), 255);

        // Test zero (though not practical)
        assert_eq!(compute_rows(0), 0);
    }

    // Note: We can't easily test the traits directly since they're abstract,
    // but they are thoroughly tested through their implementations in
    // the plain and latched modules.

    #[test]
    fn test_word_size_equality() {
        // Test all combinations of equality
        assert_eq!(WordSize::Eight, WordSize::Eight);
        assert_eq!(WordSize::Sixteen, WordSize::Sixteen);
        assert_ne!(WordSize::Eight, WordSize::Sixteen);
        assert_ne!(WordSize::Sixteen, WordSize::Eight);
    }

    #[test]
    fn test_bit_depth_limits() {
        // Test that our bit depth calculations work for the full range
        for bits in 1..=8 {
            let frame_count = compute_frame_count(bits);

            // Frame count should be positive
            assert!(frame_count > 0);

            // Frame count should be less than 2^bits
            assert!(frame_count < (1 << bits));

            // Frame count should be exactly (2^bits) - 1
            assert_eq!(frame_count, (1 << bits) - 1);
        }
    }

    #[test]
    fn test_documentation_examples() {
        // Test the example values from the documentation
        const ROWS: usize = 32;
        const COLS: usize = 64;
        const NROWS: usize = ROWS / 2;
        const BITS: u8 = 8;
        const FRAME_COUNT: usize = (1 << BITS) - 1;

        // Verify using our helper functions
        assert_eq!(compute_rows(ROWS), NROWS);
        assert_eq!(compute_frame_count(BITS), FRAME_COUNT);

        // Verify the values match documentation
        assert_eq!(ROWS, 32);
        assert_eq!(COLS, 64);
        assert_eq!(NROWS, 16);
        assert_eq!(FRAME_COUNT, 255);

        // Verify this matches typical panel dimensions
        assert!(COLS > 0);
        assert_eq!(NROWS * 2, ROWS);
    }
}
