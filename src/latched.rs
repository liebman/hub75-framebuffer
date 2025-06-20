//! DMA-friendly framebuffer implementation for HUB75 LED panels with external
//! latch circuit support.
//!
//! This module provides a framebuffer implementation with memory
//! layout optimized for efficient transfer to HUB75 LED panels. The data is
//! structured for direct signal mapping, making it ideal for DMA transfers but
//! also suitable for programmatic transfer. It supports RGB color and brightness
//! control through multiple frames using Binary Code Modulation (BCM).
//!
//! # Hardware Requirements
//! This implementation can be used by any microcontroller that has a peripheral
//! capable of outputting a clock signal and 8 bits in parallel. A latch circuit
//! similar to the one shown below can be used to hold the row address. The clock
//! is gated so it does not reach the HUB75 interface when the latch is open.
//! Since there is typically 4 2 input nand gates on a chip the 4th is used to allow
//! PWM to gate the output enable providing much finer grained overall brightness control.
//!
// Important: note the blank line of documentation on each side of the image lookup table.
// The "image lookup table" can be placed anywhere, but we place it here together with the
// warning if the `doc-images` feature is not enabled.
#![cfg_attr(feature = "doc-images",
cfg_attr(all(),
doc = ::embed_doc_image::embed_image!("latch-circuit", "images/latch-circuit.png")))]
#![cfg_attr(
    not(feature = "doc-images"),
    doc = "**Doc images not enabled**. Compile with feature `doc-images` and Rust version >= 1.54 \
           to enable."
)]
//!
//! ![Latch Circuit][latch-circuit]
//!
//! # Key Differences from Plain Implementation
//! - Uses an external latch circuit to hold the row address and gate the pixel
//!   clock, reducing memory usage
//! - 8-bit entries instead of 16-bit, halving memory requirements
//! - Separate address and data words for better control
//! - Requires an external latch circuit; not compatible with plain HUB75 wiring
//!
//! # Features
//! - Support for RGB color with brightness control
//! - Multiple frame buffers for Binary Code Modulation (BCM)
//! - Integration with embedded-graphics for easy drawing
//! - Memory-efficient 8-bit format
//!
//! # Brightness Control
//! Brightness is controlled through Binary Code Modulation (BCM):
//! - The number of brightness levels is determined by the `BITS` parameter
//! - Each additional bit doubles the number of brightness levels
//! - More bits provide better brightness resolution but require more memory
//! - Memory usage grows exponentially with the number of bits: `(2^BITS)-1`
//!   frames
//! - Example: 8 bits = 256 levels, 4 bits = 16 levels
//!
//! # Memory Usage
//! The framebuffer's memory usage is determined by:
//! - Panel size (ROWS × COLS)
//! - Number of brightness bits (BITS)
//! - Memory grows exponentially with bits: `(2^BITS)-1` frames
//! - 8-bit entries reduce memory usage compared to 16-bit implementations
//!
//! # Example
//! ```rust
//! use embedded_graphics::pixelcolor::RgbColor;
//! use embedded_graphics::prelude::*;
//! use embedded_graphics::primitives::Circle;
//! use embedded_graphics::primitives::Rectangle;
//! use embedded_graphics::primitives::PrimitiveStyle;
//! use hub75_framebuffer::compute_frame_count;
//! use hub75_framebuffer::compute_rows;
//! use hub75_framebuffer::Color;
//! use hub75_framebuffer::latched::DmaFrameBuffer;
//!
//! // Create a framebuffer for a 64x32 panel with 3-bit color depth
//! const ROWS: usize = 32;
//! const COLS: usize = 64;
//! const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
//! const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
//! const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
//!
//! let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
//!
//! // Draw a red rectangle
//! Rectangle::new(Point::new(10, 10), Size::new(20, 20))
//!     .into_styled(PrimitiveStyle::with_fill(Color::RED))
//!     .draw(&mut framebuffer)
//!     .unwrap();
//!
//! // Draw a blue circle
//! Circle::new(Point::new(40, 20), 10)
//!     .into_styled(PrimitiveStyle::with_fill(Color::BLUE))
//!     .draw(&mut framebuffer)
//!     .unwrap();
//! ```
//!
//! # Implementation Details
//! The framebuffer is organized to efficiently use memory while maintaining
//! HUB75 compatibility:
//! - Each row contains both data and address words
//! - 8-bit entries store RGB data for two sub-pixels
//! - Separate address words control row selection and timing
//! - Multiple frames are used to achieve Binary Code Modulation (BCM)
//! - DMA transfers the data directly to the panel without
//!   transformation
//!
//! # HUB75 Signal Bit Mapping (8-bit words)
//! Two distinct 8-bit words are streamed to the panel:
//!
//! 1. **Address / Timing (`Address`)** – row-select and latch control.
//! 2. **Pixel Data (`Entry`)**       – RGB bits for two sub-pixels plus OE/LAT shadow bits.
//!
//! The bit layouts intentionally overlap so that *the very same GPIO lines*
//! can transmit either word without any run-time bit twiddling:
//!
//! ```text
//! Address word (row select & timing)
//! ┌──7─┬──6──┬─5─┬─4─┬─3─┬─2─┬─1─┬─0─┐
//! │ OE │ LAT │   │ E │ D │ C │ B │ A │
//! └────┴─────┴───┴───┴───┴───┴───┴───┘
//!        ^                ^
//!        |                └── Row-address lines (LSB = A)
//!        └── Latch pulse – when HIGH the current address is latched and
//!            external glue logic gates the pixel clock (`CLK`).
//! ````
//! ```text
//! Entry word (pixel data for two sub-pixels)
//! ┌──7─┬──6──┬─5──┬─4──┬─3──┬─2──┬─1──┬─0──┐
//! │ OE │ LAT │ B2 │ G2 │ R2 │ B1 │ G1 │ R1 │
//! └────┴─────┴────┴────┴────┴────┴────┴────┘
//! ```
//!
//! *Bits 7–6* (OE/LAT) mirror those in the `Address` word so the control lines
//! remain valid throughout the entire DMA stream.
//!
//! # External Latch Timing Sequence
//! 1. Pixel data for row *N* is clocked out while `OE` is LOW.
//! 2. `OE` is raised **HIGH** – LEDs blank.
//! 3. An **`Address` word** with the new row index is transmitted while
//!    `LAT` is HIGH; the CPLD/logic also blocks `CLK` during this period.
//! 4. `LAT` returns LOW and `OE` is driven LOW again.
//!
//! This keeps visual artefacts to a minimum while allowing the framebuffer to
//! use just 8 data bits.
//!
//! # Binary Code Modulation (BCM) Frames
//! Brightness is realised with Binary-Code-Modulation just like the *plain*
//! implementation—see <https://www.batsocks.co.uk/readme/art_bcm_1.htm>.
//! With a colour depth of `BITS` the driver allocates
//! `FRAME_COUNT = 2^BITS − 1` frames. Frame *n* (0-based) is displayed for a
//! time slice proportional to `2^n`.
//!
//! For each channel the driver compares the 8-bit colour value against a per-frame
//! threshold:
//!
//! ```text
//! brightness_step = 256 / 2^BITS
//! threshold_n     = (n + 1) * brightness_step
//! ```
//!
//! The channel bit is set in frame *n* iff `value >= threshold_n`. Streaming the
//! frames from LSB to MSB therefore reproduces the intended 8-bit intensity
//! without extra processing.
//!
//! # Memory Layout
//! Each row consists of:
//! - 4 address words (8 bits each) for row selection and timing
//! - COLS data words (8 bits each) for pixel data
//!
//! # Safety
//! This implementation uses unsafe code for DMA operations. The framebuffer
//! must be properly aligned in memory and the DMA configuration must match the
//! buffer layout.
use core::convert::Infallible;

use super::Color;
use bitfield::bitfield;
#[cfg(not(feature = "esp-dma"))]
use embedded_dma::ReadBuffer;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::Point;
#[cfg(feature = "esp-dma")]
use esp_hal::dma::ReadBuffer;

bitfield! {
    /// 8-bit word carrying the row-address and timing control signals that are
    /// driven on a HUB75 connector.
    ///
    /// Relationship to [`Entry`]
    /// -------------------------
    /// The control bits—output-enable (`OE`) and latch (`LAT`)—occupy **exactly**
    /// the same bit positions as in [`Entry`].
    /// This deliberate overlap allows both structures to be streamed through the
    /// same GPIO/DMA path without any run-time bit remapping.
    ///
    /// Field summary
    /// -------------
    /// - Row-address lines `A`–`E` (5 bits)
    /// - Latch signal `LAT`        (1 bit)
    /// - Output-enable `OE`        (1 bit)
    ///
    /// Bit layout
    /// ----------
    /// - Bit 7 `OE`  : Output enable
    /// - Bit 6 `LAT` : Row-latch strobe
    ///   When asserted:
    ///   1. The address bits (`A`–`E`) are latched by the panel driver.
    ///   2. External glue logic gates the pixel clock (`CLK`), preventing any
    ///      new pixel data from being shifted into the display while the latch
    ///      is open.
    /// - Bits 4–0 `A`–`E` : Row address (LSB =`A`)
    ///
    /// Behaviour notes
    /// ---------------
    /// * The address bits take effect only while `LAT` is high; they may be
    ///   changed safely at any other time.
    /// * Because `CLK` is inhibited during the latch interval, the pixel data
    ///   stream produced from [`Entry`] words is paused until the latch is
    ///   released.
    #[derive(Clone, Copy, Default, PartialEq, Eq)]
    #[repr(transparent)]
    struct Address(u8);
    impl Debug;
    pub output_enable, set_output_enable: 7;
    pub latch, set_latch: 6;
    pub addr, set_addr: 4, 0;
}

impl Address {
    pub const fn new() -> Self {
        Self(0)
    }
}

bitfield! {
    /// 8-bit word representing the pixel data and control signals.
    ///
    /// This structure contains the RGB data for two sub-pixels and control signals:
    /// - RGB data for two sub-pixels (color0 and color1)
    /// - Output enable signal
    /// - Latch signal
    ///
    /// The bit layout is as follows:
    /// - Bit 7: Output enable
    /// - Bit 6: Latch signal
    /// - Bit 5: Blue channel for color1
    /// - Bit 4: Green channel for color1
    /// - Bit 3: Red channel for color1
    /// - Bit 2: Blue channel for color0
    /// - Bit 1: Green channel for color0
    /// - Bit 0: Red channel for color0
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    struct Entry(u8);
    impl Debug;
    pub output_enable, set_output_enable: 7;
    pub latch, set_latch: 6;
    pub blu2, set_blu2: 5;
    pub grn2, set_grn2: 4;
    pub red2, set_red2: 3;
    pub blu1, set_blu1: 2;
    pub grn1, set_grn1: 1;
    pub red1, set_red1: 0;
}

impl Entry {
    pub const fn new() -> Self {
        Self(0)
    }

    // Optimized color bit manipulation constants and methods
    const COLOR0_MASK: u8 = 0b0000_0111; // bits 0-2: R1, G1, B1
    const COLOR1_MASK: u8 = 0b0011_1000; // bits 3-5: R2, G2, B2

    #[inline]
    fn set_color0_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::COLOR0_MASK) | (bits & Self::COLOR0_MASK);
    }

    #[inline]
    fn set_color1_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::COLOR1_MASK) | ((bits << 3) & Self::COLOR1_MASK);
    }
}

/// Represents a single row of pixels with external latch circuit support.
///
/// Each row contains both pixel data and address information:
/// - 4 address words for row selection and timing
/// - COLS data words for pixel data
///
/// The address words are arranged to match the external latch circuit's
/// timing requirements. When the `esp32` feature is enabled, a specific
/// mapping (2, 3, 0, 1) is applied to correct for the strange byte ordering
/// required for the ESP32's I2S peripheral.
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
struct Row<const COLS: usize> {
    data: [Entry; COLS],
    address: [Address; 4],
}

// bytes are output in the order 2, 3, 0, 1
#[inline]
const fn map_index(index: usize) -> usize {
    #[cfg(feature = "esp32-ordering")]
    {
        index ^ 2
    }
    #[cfg(not(feature = "esp32-ordering"))]
    {
        index
    }
}

/// Pre-computed address table for all possible row addresses (0-31).
/// Each entry contains the 4 address words needed for that row.
const fn make_addr_table() -> [[Address; 4]; 32] {
    let mut tbl = [[Address::new(); 4]; 32];
    let mut addr = 0;
    while addr < 32 {
        let mut i = 0;
        while i < 4 {
            let latch = i != 3;
            let mapped_i = map_index(i);
            let latch_bit = if latch { 1u8 << 6 } else { 0u8 };
            tbl[addr][mapped_i].0 = latch_bit | addr as u8;
            i += 1;
        }
        addr += 1;
    }
    tbl
}

static ADDR_TABLE: [[Address; 4]; 32] = make_addr_table();

/// Pre-computed data template for a row with the given number of columns.
/// This template has the correct OE/LAT bits set for each column position.
const fn make_data_template<const COLS: usize>() -> [Entry; COLS] {
    let mut data = [Entry::new(); COLS];
    let mut i = 0;
    while i < COLS {
        let mapped_i = map_index(i);
        // Set latch to false and output_enable to true for all except last column
        // Note: Check the logical index (i), not the mapped index (mapped_i)
        data[mapped_i].0 = if i == COLS - 1 { 0 } else { 0b1000_0000 }; // OE bit
        i += 1;
    }
    data
}

impl<const COLS: usize> Row<COLS> {
    pub const fn new() -> Self {
        Self {
            address: [Address::new(); 4],
            data: [Entry::new(); COLS],
        }
    }

    #[inline]
    pub fn format(&mut self, addr: u8) {
        // Use pre-computed address table
        self.address.copy_from_slice(&ADDR_TABLE[addr as usize]);

        // Use pre-computed data template - create it each time since we can't use generics in static
        let data_template = make_data_template::<COLS>();
        self.data.copy_from_slice(&data_template);
    }

    /// Fast clear that only zeros the color bits, preserving OE/LAT control bits
    #[inline]
    pub fn clear_colors(&mut self) {
        // Clear color bits while preserving timing and control bits
        const COLOR_CLEAR_MASK: u8 = !0b0011_1111; // Clear bits 0-5 (R1,G1,B1,R2,G2,B2)

        for entry in &mut self.data {
            entry.0 &= COLOR_CLEAR_MASK;
        }
    }

    #[inline]
    pub fn set_color0(&mut self, col: usize, r: bool, g: bool, b: bool) {
        let bits = (u8::from(b) << 2) | (u8::from(g) << 1) | u8::from(r);
        let col = map_index(col);
        self.data[col].set_color0_bits(bits);
    }

    #[inline]
    pub fn set_color1(&mut self, col: usize, r: bool, g: bool, b: bool) {
        let bits = (u8::from(b) << 2) | (u8::from(g) << 1) | u8::from(r);
        let col = map_index(col);
        self.data[col].set_color1_bits(bits);
    }
}

impl<const COLS: usize> Default for Row<COLS> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
struct Frame<const ROWS: usize, const COLS: usize, const NROWS: usize> {
    rows: [Row<COLS>; NROWS],
}

impl<const ROWS: usize, const COLS: usize, const NROWS: usize> Frame<ROWS, COLS, NROWS> {
    pub const fn new() -> Self {
        Self {
            rows: [Row::new(); NROWS],
        }
    }

    #[inline]
    pub fn format(&mut self) {
        for (addr, row) in self.rows.iter_mut().enumerate() {
            row.format(addr as u8);
        }
    }

    /// Fast clear that only zeros the color bits, preserving control bits
    #[inline]
    pub fn clear_colors(&mut self) {
        for row in &mut self.rows {
            row.clear_colors();
        }
    }

    #[inline]
    pub fn set_pixel(&mut self, y: usize, x: usize, red: bool, green: bool, blue: bool) {
        let row = &mut self.rows[if y < NROWS { y } else { y - NROWS }];
        if y < NROWS {
            row.set_color0(x, red, green, blue);
        } else {
            row.set_color1(x, red, green, blue);
        }
    }
}

impl<const ROWS: usize, const COLS: usize, const NROWS: usize> Default
    for Frame<ROWS, COLS, NROWS>
{
    fn default() -> Self {
        Self::new()
    }
}

/// DMA-compatible framebuffer for HUB75 LED panels with external latch circuit
/// support.
///
/// This implementation is optimized for memory usage and external latch circuit
/// support:
/// - Uses 8-bit entries instead of 16-bit
/// - Separates address and data words
/// - Supports the external latch circuit for row selection
/// - Implements the embedded-graphics `DrawTarget` trait
///
/// # Type Parameters
/// - `ROWS`: Total number of rows in the panel
/// - `COLS`: Number of columns in the panel
/// - `NROWS`: Number of rows per scan (typically half of ROWS)
/// - `BITS`: Color depth (1-8 bits)
/// - `FRAME_COUNT`: Number of frames used for Binary Code Modulation
///
/// # Helper Functions
/// Use these functions to compute the correct values:
/// - `esp_hub75::compute_frame_count(BITS)`: Computes the required number of
///   frames
/// - `esp_hub75::compute_rows(ROWS)`: Computes the number of rows per scan
///
/// # Memory Layout
/// The buffer is aligned to ensure efficient DMA transfers and contains:
/// - An array of frames, each containing the full panel data
/// - Each frame contains NROWS rows
/// - Each row contains both data and address words
#[derive(Copy, Clone)]
#[repr(C)]
#[repr(align(4))]
pub struct DmaFrameBuffer<
    const ROWS: usize,
    const COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
> {
    frames: [Frame<ROWS, COLS, NROWS>; FRAME_COUNT],
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > Default for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    /// Create a new framebuffer with the given number of frames.
    /// The framebuffer is automatically formatted and ready to use.
    /// # Example
    /// ```rust,no_run
    /// use hub75_framebuffer::{latched::DmaFrameBuffer,compute_rows,compute_frame_count};
    ///
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
    ///
    /// let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// // Ready to use immediately
    /// ```
    #[must_use]
    pub fn new() -> Self {
        let mut fb = Self {
            frames: [Frame::new(); FRAME_COUNT],
        };
        fb.format();
        fb
    }

    /// This returns the size of the DMA buffer in bytes.  Its used to calculate
    /// the number of DMA descriptors needed for `esp-hal`.
    /// # Example
    /// ```rust,no_run
    /// use hub75_framebuffer::{latched::DmaFrameBuffer,compute_rows,compute_frame_count};
    ///
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
    ///
    /// type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
    /// let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());
    /// ```
    #[cfg(feature = "esp-dma")]
    pub const fn dma_buffer_size_bytes() -> usize {
        core::mem::size_of::<[Frame<ROWS, COLS, NROWS>; FRAME_COUNT]>()
    }

    /// Format the framebuffer, setting up all control bits and clearing pixel data.
    /// This method does a full format of all control bits and clears all pixel data.
    /// Normally you don't need to call this as `new()` automatically formats the framebuffer.
    /// # Example
    /// ```rust,no_run
    /// use hub75_framebuffer::{Color,latched::DmaFrameBuffer,compute_rows,compute_frame_count};
    ///
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
    ///
    /// let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// // framebuffer.format(); // Not needed - new() already calls this
    /// ```
    pub fn format(&mut self) {
        for frame in &mut self.frames {
            frame.format();
        }
    }

    /// Erase pixel colors while preserving control bits.
    /// This is much faster than `format()` and is the typical way to clear the display.
    /// # Example
    /// ```rust,no_run
    /// use hub75_framebuffer::{Color,latched::DmaFrameBuffer,compute_rows,compute_frame_count};
    ///
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
    ///
    /// let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// // ... draw some pixels ...
    /// framebuffer.erase();
    /// ```
    #[inline]
    pub fn erase(&mut self) {
        for frame in &mut self.frames {
            frame.clear_colors();
        }
    }

    /// Set a pixel in the framebuffer.
    /// # Example
    /// ```rust,no_run
    /// use hub75_framebuffer::{Color,latched::DmaFrameBuffer,compute_rows,compute_frame_count};
    /// use embedded_graphics::prelude::*;
    ///
    /// const ROWS: usize = 32;
    /// const COLS: usize = 64;
    /// const BITS: u8 = 3; // Color depth (8 brightness levels, 7 frames)
    /// const NROWS: usize = compute_rows(ROWS); // Number of rows per scan
    /// const FRAME_COUNT: usize = compute_frame_count(BITS); // Number of frames for BCM
    ///
    /// let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
    /// framebuffer.set_pixel(Point::new(10, 10), Color::RED);
    /// ```
    pub fn set_pixel(&mut self, p: Point, color: Color) {
        if p.x < 0 || p.y < 0 {
            return;
        }
        self.set_pixel_internal(p.x as usize, p.y as usize, color);
    }

    #[inline]
    fn frames_on(v: u8) -> usize {
        // v / brightness_step but the compiler resolves the shift at build-time
        (v as usize) >> (8 - BITS)
    }

    #[inline]
    fn set_pixel_internal(&mut self, x: usize, y: usize, color: Rgb888) {
        if x >= COLS || y >= ROWS {
            return;
        }

        // Early exit for black pixels - common in UI backgrounds
        // Only enabled when skip-black-pixels feature is active
        #[cfg(feature = "skip-black-pixels")]
        if color == Rgb888::BLACK {
            return;
        }

        // Pre-compute how many frames each channel should be on
        let red_frames = Self::frames_on(color.r());
        let green_frames = Self::frames_on(color.g());
        let blue_frames = Self::frames_on(color.b());

        // Set the pixel in all frames based on pre-computed frame counts
        for (frame_idx, frame) in self.frames.iter_mut().enumerate() {
            frame.set_pixel(
                y,
                x,
                frame_idx < red_frames,
                frame_idx < green_frames,
                frame_idx < blue_frames,
            );
        }
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::prelude::OriginDimensions
    for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(COLS as u32, ROWS as u32)
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::draw_target::DrawTarget
    for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    type Color = Color;

    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            self.set_pixel_internal(pixel.0.x as usize, pixel.0.y as usize, pixel.1);
        }
        Ok(())
    }
}

unsafe impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > ReadBuffer for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    #[cfg(not(feature = "esp-dma"))]
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        let ptr = (&raw const self.frames).cast::<u8>();
        let len = core::mem::size_of_val(&self.frames);
        (ptr, len)
    }
}

unsafe impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > ReadBuffer for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    #[cfg(not(feature = "esp-dma"))]
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        let ptr = (&raw const self.frames).cast::<u8>();
        let len = core::mem::size_of_val(&self.frames);
        (ptr, len)
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > core::fmt::Debug for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let brightness_step = 1 << (8 - BITS);
        f.debug_struct("DmaFrameBuffer")
            .field("size", &core::mem::size_of_val(&self.frames))
            .field("frame_count", &self.frames.len())
            .field("frame_size", &core::mem::size_of_val(&self.frames[0]))
            .field("brightness_step", &&brightness_step)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > defmt::Format for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn format(&self, f: defmt::Formatter) {
        let brightness_step = 1 << (8 - BITS);
        defmt::write!(
            f,
            "DmaFrameBuffer<{}, {}, {}, {}, {}>",
            ROWS,
            COLS,
            NROWS,
            BITS,
            FRAME_COUNT
        );
        defmt::write!(f, " size: {}", core::mem::size_of_val(&self.frames));
        defmt::write!(
            f,
            " frame_size: {}",
            core::mem::size_of_val(&self.frames[0])
        );
        defmt::write!(f, " brightness_step: {}", brightness_step);
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > super::FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    for DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn get_word_size(&self) -> super::WordSize {
        super::WordSize::Eight
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > embedded_graphics::prelude::OriginDimensions
    for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(COLS as u32, ROWS as u32)
    }
}

impl<
        const ROWS: usize,
        const COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
    > super::FrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    for &mut DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
{
    fn get_word_size(&self) -> super::WordSize {
        super::WordSize::Eight
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::format;
    use std::vec;

    use super::*;
    use crate::{FrameBuffer, WordSize};
    use embedded_graphics::pixelcolor::RgbColor;
    use embedded_graphics::prelude::*;
    use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle};

    const TEST_ROWS: usize = 32;
    const TEST_COLS: usize = 64;
    const TEST_NROWS: usize = TEST_ROWS / 2;
    const TEST_BITS: u8 = 3;
    const TEST_FRAME_COUNT: usize = (1 << TEST_BITS) - 1; // 7 frames for 3-bit depth

    type TestFrameBuffer =
        DmaFrameBuffer<TEST_ROWS, TEST_COLS, TEST_NROWS, TEST_BITS, TEST_FRAME_COUNT>;

    #[test]
    fn test_address_construction() {
        let addr = Address::new();
        assert_eq!(addr.0, 0);
        assert_eq!(addr.latch(), false);
        assert_eq!(addr.addr(), 0);
    }

    #[test]
    fn test_address_setters() {
        let mut addr = Address::new();

        addr.set_latch(true);
        assert_eq!(addr.latch(), true);
        assert_eq!(addr.0 & 0b01000000, 0b01000000);

        addr.set_addr(0b11111);
        assert_eq!(addr.addr(), 0b11111);
        assert_eq!(addr.0 & 0b00011111, 0b00011111);
    }

    #[test]
    fn test_address_bit_isolation() {
        let mut addr = Address::new();

        // Test that setting one field doesn't affect others
        addr.set_addr(0b11111);
        addr.set_latch(true);
        assert_eq!(addr.addr(), 0b11111);
        assert_eq!(addr.latch(), true);
    }

    #[test]
    fn test_entry_construction() {
        let entry = Entry::new();
        assert_eq!(entry.0, 0);
        assert_eq!(entry.output_enable(), false);
        assert_eq!(entry.latch(), false);
        assert_eq!(entry.red1(), false);
        assert_eq!(entry.grn1(), false);
        assert_eq!(entry.blu1(), false);
        assert_eq!(entry.red2(), false);
        assert_eq!(entry.grn2(), false);
        assert_eq!(entry.blu2(), false);
    }

    #[test]
    fn test_entry_setters() {
        let mut entry = Entry::new();

        entry.set_output_enable(true);
        assert_eq!(entry.output_enable(), true);
        assert_eq!(entry.0 & 0b10000000, 0b10000000);

        entry.set_latch(true);
        assert_eq!(entry.latch(), true);
        assert_eq!(entry.0 & 0b01000000, 0b01000000);

        // Test RGB channels for color0 (bits 0-2)
        entry.set_red1(true);
        entry.set_grn1(true);
        entry.set_blu1(true);
        assert_eq!(entry.red1(), true);
        assert_eq!(entry.grn1(), true);
        assert_eq!(entry.blu1(), true);
        assert_eq!(entry.0 & 0b00000111, 0b00000111);

        // Test RGB channels for color1 (bits 3-5)
        entry.set_red2(true);
        entry.set_grn2(true);
        entry.set_blu2(true);
        assert_eq!(entry.red2(), true);
        assert_eq!(entry.grn2(), true);
        assert_eq!(entry.blu2(), true);
        assert_eq!(entry.0 & 0b00111000, 0b00111000);
    }

    #[test]
    fn test_entry_set_color0() {
        let mut entry = Entry::new();

        let bits = (u8::from(true) << 2) | (u8::from(false) << 1) | u8::from(true); // b=1, g=0, r=1 = 0b101
        entry.set_color0_bits(bits);
        assert_eq!(entry.red1(), true);
        assert_eq!(entry.grn1(), false);
        assert_eq!(entry.blu1(), true);
        assert_eq!(entry.0 & 0b00000111, 0b00000101); // Red and blue bits set
    }

    #[test]
    fn test_entry_set_color1() {
        let mut entry = Entry::new();

        let bits = (u8::from(true) << 2) | (u8::from(true) << 1) | u8::from(false); // b=1, g=1, r=0 = 0b110
        entry.set_color1_bits(bits);
        assert_eq!(entry.red2(), false);
        assert_eq!(entry.grn2(), true);
        assert_eq!(entry.blu2(), true);
        assert_eq!(entry.0 & 0b00111000, 0b00110000); // Green and blue bits set
    }

    #[test]
    fn test_row_construction() {
        let row: Row<TEST_COLS> = Row::new();
        assert_eq!(row.data.len(), TEST_COLS);
        assert_eq!(row.address.len(), 4);

        // Check that all entries are initialized to zero
        for entry in &row.data {
            assert_eq!(entry.0, 0);
        }
        for addr in &row.address {
            assert_eq!(addr.0, 0);
        }
    }

    #[test]
    fn test_row_format() {
        let mut row: Row<TEST_COLS> = Row::new();
        let test_addr = 5;

        row.format(test_addr);

        // Check address words configuration
        for addr in &row.address {
            assert_eq!(addr.addr(), test_addr);
            // The latch values are pre-computed in the address table based on the logical
            // arrangement, so we don't need to reverse-map. Just verify the table matches
            // what we expect from the make_addr_table function.
        }
        // Since the address table is complex with ESP32 mapping, let's just verify
        // that exactly one address has latch=false (from logical index 3) and the
        // rest have latch=true
        let latch_false_count = row.address.iter().filter(|addr| !addr.latch()).count();
        assert_eq!(latch_false_count, 1);

        // Check data entries configuration
        for entry in &row.data {
            assert_eq!(entry.latch(), false);
        }
        // The output enable bits are pre-computed in the data template with ESP32 mapping
        // taken into account. Since make_data_template checks the logical index (i) not
        // the mapped index, exactly one entry should have output_enable=false (the one
        // corresponding to the last logical column)
        let oe_false_count = row
            .data
            .iter()
            .filter(|entry| !entry.output_enable())
            .count();
        assert_eq!(oe_false_count, 1);
    }

    #[test]
    fn test_row_set_color0() {
        let mut row: Row<TEST_COLS> = Row::new();

        row.set_color0(0, true, false, true);

        let mapped_col_0 = map_index(0);
        assert_eq!(row.data[mapped_col_0].red1(), true);
        assert_eq!(row.data[mapped_col_0].grn1(), false);
        assert_eq!(row.data[mapped_col_0].blu1(), true);

        // Test another column
        row.set_color0(1, false, true, false);

        let mapped_col_1 = map_index(1);
        assert_eq!(row.data[mapped_col_1].red1(), false);
        assert_eq!(row.data[mapped_col_1].grn1(), true);
        assert_eq!(row.data[mapped_col_1].blu1(), false);
    }

    #[test]
    fn test_row_set_color1() {
        let mut row: Row<TEST_COLS> = Row::new();

        row.set_color1(0, true, true, false);

        let mapped_col_0 = map_index(0);
        assert_eq!(row.data[mapped_col_0].red2(), true);
        assert_eq!(row.data[mapped_col_0].grn2(), true);
        assert_eq!(row.data[mapped_col_0].blu2(), false);
    }

    #[test]
    fn test_frame_construction() {
        let frame: Frame<TEST_ROWS, TEST_COLS, TEST_NROWS> = Frame::new();
        assert_eq!(frame.rows.len(), TEST_NROWS);
    }

    #[test]
    fn test_frame_format() {
        let mut frame: Frame<TEST_ROWS, TEST_COLS, TEST_NROWS> = Frame::new();

        frame.format();

        for (addr, row) in frame.rows.iter().enumerate() {
            // Check that each row was formatted with its address
            for address in &row.address {
                assert_eq!(address.addr() as usize, addr);
            }
        }
    }

    #[test]
    fn test_frame_set_pixel() {
        let mut frame: Frame<TEST_ROWS, TEST_COLS, TEST_NROWS> = Frame::new();

        // Test setting pixel in upper half (y < NROWS)
        frame.set_pixel(5, 10, true, false, true);

        let mapped_col_10 = map_index(10);
        assert_eq!(frame.rows[5].data[mapped_col_10].red1(), true);
        assert_eq!(frame.rows[5].data[mapped_col_10].grn1(), false);
        assert_eq!(frame.rows[5].data[mapped_col_10].blu1(), true);

        // Test setting pixel in lower half (y >= NROWS)
        frame.set_pixel(TEST_NROWS + 5, 15, false, true, false);

        let mapped_col_15 = map_index(15);
        assert_eq!(frame.rows[5].data[mapped_col_15].red2(), false);
        assert_eq!(frame.rows[5].data[mapped_col_15].grn2(), true);
        assert_eq!(frame.rows[5].data[mapped_col_15].blu2(), false);
    }

    #[test]
    fn test_row_default() {
        let row1: Row<TEST_COLS> = Row::new();
        let row2: Row<TEST_COLS> = Row::default();

        // Both should be equivalent
        assert_eq!(row1, row2);
        assert_eq!(row1.data.len(), row2.data.len());
        assert_eq!(row1.address.len(), row2.address.len());

        // Check that all entries are initialized to zero
        for (entry1, entry2) in row1.data.iter().zip(row2.data.iter()) {
            assert_eq!(entry1.0, entry2.0);
            assert_eq!(entry1.0, 0);
        }
        for (addr1, addr2) in row1.address.iter().zip(row2.address.iter()) {
            assert_eq!(addr1.0, addr2.0);
            assert_eq!(addr1.0, 0);
        }
    }

    #[test]
    fn test_frame_default() {
        let frame1: Frame<TEST_ROWS, TEST_COLS, TEST_NROWS> = Frame::new();
        let frame2: Frame<TEST_ROWS, TEST_COLS, TEST_NROWS> = Frame::default();

        // Both should be equivalent
        assert_eq!(frame1.rows.len(), frame2.rows.len());

        // Check that all rows are equivalent
        for (row1, row2) in frame1.rows.iter().zip(frame2.rows.iter()) {
            assert_eq!(row1, row2);

            // Verify all entries are zero-initialized
            for (entry1, entry2) in row1.data.iter().zip(row2.data.iter()) {
                assert_eq!(entry1.0, entry2.0);
                assert_eq!(entry1.0, 0);
            }
            for (addr1, addr2) in row1.address.iter().zip(row2.address.iter()) {
                assert_eq!(addr1.0, addr2.0);
                assert_eq!(addr1.0, 0);
            }
        }
    }

    #[test]
    fn test_dma_framebuffer_construction() {
        let fb = TestFrameBuffer::new();
        assert_eq!(fb.frames.len(), TEST_FRAME_COUNT);
    }

    #[test]
    #[cfg(feature = "esp-dma")]
    fn test_dma_framebuffer_dma_buffer_size() {
        let expected_size =
            core::mem::size_of::<[Frame<TEST_ROWS, TEST_COLS, TEST_NROWS>; TEST_FRAME_COUNT]>();
        assert_eq!(TestFrameBuffer::dma_buffer_size_bytes(), expected_size);
    }

    #[test]
    fn test_dma_framebuffer_format() {
        let mut fb = TestFrameBuffer {
            frames: [Frame::new(); TEST_FRAME_COUNT],
        };
        fb.format();

        // After formatting, all frames should be formatted
        for frame in &fb.frames {
            for (addr, row) in frame.rows.iter().enumerate() {
                for address in &row.address {
                    assert_eq!(address.addr() as usize, addr);
                }
            }
        }
    }

    #[test]
    fn test_dma_framebuffer_set_pixel_bounds() {
        let mut fb = TestFrameBuffer::new();

        // Test negative coordinates
        fb.set_pixel(Point::new(-1, 5), Color::RED);
        fb.set_pixel(Point::new(5, -1), Color::RED);

        // Test coordinates out of bounds (should not panic)
        fb.set_pixel(Point::new(TEST_COLS as i32, 5), Color::RED);
        fb.set_pixel(Point::new(5, TEST_ROWS as i32), Color::RED);
    }

    #[test]
    fn test_dma_framebuffer_set_pixel_internal() {
        let mut fb = TestFrameBuffer::new();

        let red_color = Rgb888::new(255, 0, 0);
        fb.set_pixel_internal(10, 5, red_color);

        // With 3-bit depth, brightness steps are 32 (256/8)
        // Frames represent thresholds: 32, 64, 96, 128, 160, 192, 224
        // Red value 255 should activate all frames
        for frame in &fb.frames {
            // Check upper half pixel
            let mapped_col_10 = map_index(10);
            assert_eq!(frame.rows[5].data[mapped_col_10].red1(), true);
            assert_eq!(frame.rows[5].data[mapped_col_10].grn1(), false);
            assert_eq!(frame.rows[5].data[mapped_col_10].blu1(), false);
        }
    }

    #[test]
    fn test_dma_framebuffer_brightness_modulation() {
        let mut fb = TestFrameBuffer::new();

        // Test with a medium brightness value
        let brightness_step = 1 << (8 - TEST_BITS); // 32 for 3-bit
        let test_brightness = brightness_step * 3; // 96
        let color = Rgb888::new(test_brightness, 0, 0);

        fb.set_pixel_internal(0, 0, color);

        // Should activate frames 0, 1, 2 (thresholds 32, 64, 96)
        // but not frames 3, 4, 5, 6 (thresholds 128, 160, 192, 224)
        for (frame_idx, frame) in fb.frames.iter().enumerate() {
            let frame_threshold = (frame_idx as u8 + 1) * brightness_step;
            let should_be_active = test_brightness >= frame_threshold;

            let mapped_col_0 = map_index(0);
            assert_eq!(frame.rows[0].data[mapped_col_0].red1(), should_be_active);
        }
    }

    #[test]
    fn test_origin_dimensions() {
        let fb = TestFrameBuffer::new();
        let size = fb.size();
        assert_eq!(size.width, TEST_COLS as u32);
        assert_eq!(size.height, TEST_ROWS as u32);

        // Test mutable reference
        let mut fb = TestFrameBuffer::new();
        let fb_ref = &mut fb;
        let size = fb_ref.size();
        assert_eq!(size.width, TEST_COLS as u32);
        assert_eq!(size.height, TEST_ROWS as u32);
    }

    #[test]
    fn test_draw_target() {
        let mut fb = TestFrameBuffer::new();

        let pixels = vec![
            embedded_graphics::Pixel(Point::new(0, 0), Color::RED),
            embedded_graphics::Pixel(Point::new(1, 1), Color::GREEN),
            embedded_graphics::Pixel(Point::new(2, 2), Color::BLUE),
        ];

        let result = fb.draw_iter(pixels);
        assert!(result.is_ok());
    }

    #[test]
    fn test_draw_iter_pixel_verification() {
        let mut fb = TestFrameBuffer::new();

        // Create test pixels with specific colors and positions
        let pixels = vec![
            // Upper half pixels (y < NROWS) - should set color0
            embedded_graphics::Pixel(Point::new(5, 2), Color::RED), // (5, 2) -> red
            embedded_graphics::Pixel(Point::new(10, 5), Color::GREEN), // (10, 5) -> green
            embedded_graphics::Pixel(Point::new(15, 8), Color::BLUE), // (15, 8) -> blue
            embedded_graphics::Pixel(Point::new(20, 10), Color::WHITE), // (20, 10) -> white
            // Lower half pixels (y >= NROWS) - should set color1
            embedded_graphics::Pixel(Point::new(25, (TEST_NROWS + 3) as i32), Color::RED), // (25, 19) -> red
            embedded_graphics::Pixel(Point::new(30, (TEST_NROWS + 7) as i32), Color::GREEN), // (30, 23) -> green
            embedded_graphics::Pixel(Point::new(35, (TEST_NROWS + 12) as i32), Color::BLUE), // (35, 28) -> blue
            // Edge case: black pixel (should not be visible in first frame)
            embedded_graphics::Pixel(Point::new(40, 1), Color::BLACK), // (40, 1) -> black
            // Low brightness pixel that should not appear in first frame
            embedded_graphics::Pixel(Point::new(45, 3), Rgb888::new(16, 16, 16)), // Below threshold
        ];

        let result = fb.draw_iter(pixels);
        assert!(result.is_ok());

        // Check the first frame only
        let first_frame = &fb.frames[0];
        let brightness_step = 1 << (8 - TEST_BITS); // 32 for 3-bit
        let first_frame_threshold = brightness_step; // 32

        // Test upper half pixels (color0)
        // Red pixel at (5, 2) - should be red in first frame
        let col_idx = map_index(5);
        assert_eq!(
            first_frame.rows[2].data[col_idx].red1(),
            Color::RED.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[2].data[col_idx].grn1(),
            Color::RED.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[2].data[col_idx].blu1(),
            Color::RED.b() >= first_frame_threshold
        );

        // Green pixel at (10, 5) - should be green in first frame
        let col_idx = map_index(10);
        assert_eq!(
            first_frame.rows[5].data[col_idx].red1(),
            Color::GREEN.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[5].data[col_idx].grn1(),
            Color::GREEN.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[5].data[col_idx].blu1(),
            Color::GREEN.b() >= first_frame_threshold
        );

        // Blue pixel at (15, 8) - should be blue in first frame
        let col_idx = map_index(15);
        assert_eq!(
            first_frame.rows[8].data[col_idx].red1(),
            Color::BLUE.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[8].data[col_idx].grn1(),
            Color::BLUE.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[8].data[col_idx].blu1(),
            Color::BLUE.b() >= first_frame_threshold
        );

        // White pixel at (20, 10) - should be white in first frame
        let col_idx = map_index(20);
        assert_eq!(
            first_frame.rows[10].data[col_idx].red1(),
            Color::WHITE.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[10].data[col_idx].grn1(),
            Color::WHITE.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[10].data[col_idx].blu1(),
            Color::WHITE.b() >= first_frame_threshold
        );

        // Test lower half pixels (color1)
        // Red pixel at (25, TEST_NROWS + 3) -> row 3, color1
        let col_idx = map_index(25);
        assert_eq!(
            first_frame.rows[3].data[col_idx].red2(),
            Color::RED.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[3].data[col_idx].grn2(),
            Color::RED.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[3].data[col_idx].blu2(),
            Color::RED.b() >= first_frame_threshold
        );

        // Green pixel at (30, TEST_NROWS + 7) -> row 7, color1
        let col_idx = map_index(30);
        assert_eq!(
            first_frame.rows[7].data[col_idx].red2(),
            Color::GREEN.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[7].data[col_idx].grn2(),
            Color::GREEN.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[7].data[col_idx].blu2(),
            Color::GREEN.b() >= first_frame_threshold
        );

        // Blue pixel at (35, TEST_NROWS + 12) -> row 12, color1
        let col_idx = map_index(35);
        assert_eq!(
            first_frame.rows[12].data[col_idx].red2(),
            Color::BLUE.r() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[12].data[col_idx].grn2(),
            Color::BLUE.g() >= first_frame_threshold
        );
        assert_eq!(
            first_frame.rows[12].data[col_idx].blu2(),
            Color::BLUE.b() >= first_frame_threshold
        );

        // Test black pixel - should not be visible in any frame
        let col_idx = map_index(40);
        assert_eq!(first_frame.rows[1].data[col_idx].red1(), false);
        assert_eq!(first_frame.rows[1].data[col_idx].grn1(), false);
        assert_eq!(first_frame.rows[1].data[col_idx].blu1(), false);

        // Test low brightness pixel (16, 16, 16) - should not be visible in first frame (threshold 32)
        let col_idx = map_index(45);
        assert_eq!(
            first_frame.rows[3].data[col_idx].red1(),
            16 >= first_frame_threshold
        ); // false
        assert_eq!(
            first_frame.rows[3].data[col_idx].grn1(),
            16 >= first_frame_threshold
        ); // false
        assert_eq!(
            first_frame.rows[3].data[col_idx].blu1(),
            16 >= first_frame_threshold
        ); // false
    }

    #[test]
    fn test_embedded_graphics_integration() {
        let mut fb = TestFrameBuffer::new();

        // Draw a rectangle
        let result = Rectangle::new(Point::new(5, 5), Size::new(10, 8))
            .into_styled(PrimitiveStyle::with_fill(Color::RED))
            .draw(&mut fb);
        assert!(result.is_ok());

        // Draw a circle
        let result = Circle::new(Point::new(30, 15), 8)
            .into_styled(PrimitiveStyle::with_fill(Color::BLUE))
            .draw(&mut fb);
        assert!(result.is_ok());
    }

    #[test]
    fn test_read_buffer_implementation() {
        let fb = TestFrameBuffer::new();

        // Test direct implementation
        unsafe {
            let (ptr, len) = fb.read_buffer();
            assert!(!ptr.is_null());
            assert_eq!(len, core::mem::size_of_val(&fb.frames));
        }

        // Test mutable reference implementation
        let mut fb = TestFrameBuffer::new();
        let fb_ref = &mut fb;
        unsafe {
            let (ptr, len) = fb_ref.read_buffer();
            assert!(!ptr.is_null());
            assert_eq!(len, core::mem::size_of_val(&fb.frames));
        }
    }

    #[test]
    fn test_framebuffer_trait() {
        let fb = TestFrameBuffer::new();
        assert_eq!(fb.get_word_size(), WordSize::Eight);

        let mut fb = TestFrameBuffer::new();
        let fb_ref = &mut fb;
        assert_eq!(fb_ref.get_word_size(), WordSize::Eight);
    }

    #[test]
    fn test_debug_formatting() {
        let fb = TestFrameBuffer::new();
        let debug_string = format!("{:?}", fb);
        assert!(debug_string.contains("DmaFrameBuffer"));
        assert!(debug_string.contains("frame_count"));
        assert!(debug_string.contains("frame_size"));
        assert!(debug_string.contains("brightness_step"));
    }

    #[test]
    fn test_default_implementation() {
        let fb1 = TestFrameBuffer::new();
        let fb2 = TestFrameBuffer::default();

        // Both should be equivalent
        assert_eq!(fb1.frames.len(), fb2.frames.len());
    }

    #[cfg(feature = "esp32-ordering")]
    #[test]
    fn test_esp32_mapping() {
        // Test the ESP32-specific index mapping
        assert_eq!(map_index(0), 2);
        assert_eq!(map_index(1), 3);
        assert_eq!(map_index(2), 0);
        assert_eq!(map_index(3), 1);
        assert_eq!(map_index(4), 6); // 4 & !0b11 | 2 = 4 | 2 = 6
        assert_eq!(map_index(5), 7); // 5 & !0b11 | 3 = 4 | 3 = 7
    }

    #[test]
    fn test_memory_alignment() {
        let fb = TestFrameBuffer::new();
        let ptr = &fb as *const _ as usize;

        // Should be 4-byte aligned as specified in repr(align(4))
        assert_eq!(ptr % 4, 0);
    }

    #[test]
    fn test_color_values() {
        let mut fb = TestFrameBuffer::new();

        // Test different color values
        let colors = [
            (Color::RED, (255, 0, 0)),
            (Color::GREEN, (0, 255, 0)),
            (Color::BLUE, (0, 0, 255)),
            (Color::WHITE, (255, 255, 255)),
            (Color::BLACK, (0, 0, 0)),
        ];

        for (i, (color, (r, g, b))) in colors.iter().enumerate() {
            fb.set_pixel(Point::new(i as i32, 0), *color);
            assert_eq!(color.r(), *r);
            assert_eq!(color.g(), *g);
            assert_eq!(color.b(), *b);
        }
    }

    #[test]
    fn test_bits_assertion() {
        // Test that BITS <= 8 assertion is enforced at compile time
        // This test mainly documents the constraint
        assert!(TEST_BITS <= 8);
    }

    #[test]
    #[cfg(feature = "skip-black-pixels")]
    fn test_skip_black_pixels_enabled() {
        let mut fb = TestFrameBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first frame
        let mapped_col_10 = map_index(10);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].red1(), true);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].grn1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].blu1(), false);

        // Now set it to black - with skip-black-pixels enabled, this should be ignored
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should still be red (black write was skipped)
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].red1(), true);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].grn1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].blu1(), false);
    }

    #[test]
    #[cfg(not(feature = "skip-black-pixels"))]
    fn test_skip_black_pixels_disabled() {
        let mut fb = TestFrameBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first frame
        let mapped_col_10 = map_index(10);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].red1(), true);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].grn1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].blu1(), false);

        // Now set it to black - with skip-black-pixels disabled, this should overwrite
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should now be black (all bits false)
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].red1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].grn1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].blu1(), false);
    }

    #[test]
    fn test_bcm_frame_overwrite() {
        let mut fb = TestFrameBuffer::new();

        // First write a white pixel (255, 255, 255)
        fb.set_pixel_internal(10, 5, Color::WHITE);

        let mapped_col_10 = map_index(10);

        // Verify white pixel is lit in all frames (255 >= all thresholds)
        for frame in fb.frames.iter() {
            // White (255) should be active in all frames since it's >= all thresholds
            assert_eq!(frame.rows[5].data[mapped_col_10].red1(), true);
            assert_eq!(frame.rows[5].data[mapped_col_10].grn1(), true);
            assert_eq!(frame.rows[5].data[mapped_col_10].blu1(), true);
        }

        // Now overwrite with 50% white (128, 128, 128)
        let half_white = embedded_graphics::pixelcolor::Rgb888::new(128, 128, 128);
        fb.set_pixel_internal(10, 5, half_white);

        // Verify only the correct frames are lit for 50% white
        // With 3-bit depth: thresholds are 32, 64, 96, 128, 160, 192, 224
        // 128 should activate frames 0, 1, 2, 3 (thresholds 32, 64, 96, 128)
        // but not frames 4, 5, 6 (thresholds 160, 192, 224)
        let brightness_step = 1 << (8 - TEST_BITS); // 32 for 3-bit
        for (frame_idx, frame) in fb.frames.iter().enumerate() {
            let frame_threshold = (frame_idx as u8 + 1) * brightness_step;
            let should_be_active = 128 >= frame_threshold;

            assert_eq!(frame.rows[5].data[mapped_col_10].red1(), should_be_active);
            assert_eq!(frame.rows[5].data[mapped_col_10].grn1(), should_be_active);
            assert_eq!(frame.rows[5].data[mapped_col_10].blu1(), should_be_active);
        }

        // Specifically verify the expected pattern for 3-bit depth
        // Frames 0-3 should be active (thresholds 32, 64, 96, 128)
        for frame_idx in 0..4 {
            assert_eq!(
                fb.frames[frame_idx].rows[5].data[mapped_col_10].red1(),
                true
            );
        }
        // Frames 4-6 should be inactive (thresholds 160, 192, 224)
        for frame_idx in 4..TEST_FRAME_COUNT {
            assert_eq!(
                fb.frames[frame_idx].rows[5].data[mapped_col_10].red1(),
                false
            );
        }
    }

    #[test]
    fn test_new_auto_formats() {
        let fb = TestFrameBuffer::new();

        // After new(), all frames should be formatted
        for frame in &fb.frames {
            for (addr, row) in frame.rows.iter().enumerate() {
                for address in &row.address {
                    assert_eq!(address.addr() as usize, addr);
                }
            }
        }
    }

    #[test]
    fn test_erase() {
        let mut fb = TestFrameBuffer::new();

        // Set some pixels
        fb.set_pixel_internal(10, 5, Color::RED);
        fb.set_pixel_internal(20, 10, Color::GREEN);

        let mapped_col_10 = map_index(10);
        let mapped_col_20 = map_index(20);

        // Verify pixels are set
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].red1(), true);
        assert_eq!(fb.frames[0].rows[10].data[mapped_col_20].grn1(), true);

        // erase
        fb.erase();

        // Verify pixels are cleared but control bits are preserved
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].red1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].grn1(), false);
        assert_eq!(fb.frames[0].rows[5].data[mapped_col_10].blu1(), false);
        assert_eq!(fb.frames[0].rows[10].data[mapped_col_20].red1(), false);
        assert_eq!(fb.frames[0].rows[10].data[mapped_col_20].grn1(), false);
        assert_eq!(fb.frames[0].rows[10].data[mapped_col_20].blu1(), false);

        // Verify control bits are still correct
        for frame in &fb.frames {
            for (addr, row) in frame.rows.iter().enumerate() {
                // Check address words
                for address in &row.address {
                    assert_eq!(address.addr() as usize, addr);
                }
                // Check OE bits in data - should be exactly one false (for last logical column)
                let oe_false_count = row
                    .data
                    .iter()
                    .filter(|entry| !entry.output_enable())
                    .count();
                assert_eq!(oe_false_count, 1);
            }
        }
    }

    #[test]
    fn test_row_clear_colors() {
        let mut row: Row<TEST_COLS> = Row::new();
        row.format(5);

        // Set some colors
        row.set_color0(0, true, false, true);
        row.set_color1(1, false, true, false);

        let mapped_col_0 = map_index(0);
        let mapped_col_1 = map_index(1);

        // Verify colors are set
        assert_eq!(row.data[mapped_col_0].red1(), true);
        assert_eq!(row.data[mapped_col_0].blu1(), true);
        assert_eq!(row.data[mapped_col_1].grn2(), true);

        // Store original control bits
        let original_oe_0 = row.data[mapped_col_0].output_enable();
        let original_latch_0 = row.data[mapped_col_0].latch();
        let original_oe_1 = row.data[mapped_col_1].output_enable();
        let original_latch_1 = row.data[mapped_col_1].latch();

        // Clear colors
        row.clear_colors();

        // Verify colors are cleared
        assert_eq!(row.data[mapped_col_0].red1(), false);
        assert_eq!(row.data[mapped_col_0].grn1(), false);
        assert_eq!(row.data[mapped_col_0].blu1(), false);
        assert_eq!(row.data[mapped_col_1].red2(), false);
        assert_eq!(row.data[mapped_col_1].grn2(), false);
        assert_eq!(row.data[mapped_col_1].blu2(), false);

        // Verify control bits are preserved
        assert_eq!(row.data[mapped_col_0].output_enable(), original_oe_0);
        assert_eq!(row.data[mapped_col_0].latch(), original_latch_0);
        assert_eq!(row.data[mapped_col_1].output_enable(), original_oe_1);
        assert_eq!(row.data[mapped_col_1].latch(), original_latch_1);
    }

    #[test]
    fn test_make_addr_table_function() {
        // Test the make_addr_table function directly to ensure code coverage
        let table = make_addr_table();

        // Verify basic properties of the generated table
        assert_eq!(table.len(), 32); // Should have 32 address entries (0-31)

        // Check first address (0)
        let addr_0 = &table[0];
        assert_eq!(addr_0.len(), 4); // Should have 4 address words

        // Verify that exactly one address word has latch=false (index 3 in logical order)
        let latch_false_count = addr_0.iter().filter(|addr| !addr.latch()).count();
        assert_eq!(latch_false_count, 1);

        // All addresses should have addr field set to 0 for the first entry
        for addr in addr_0 {
            assert_eq!(addr.addr(), 0);
        }

        // Check last address (31)
        let addr_31 = &table[31];
        let latch_false_count = addr_31.iter().filter(|addr| !addr.latch()).count();
        assert_eq!(latch_false_count, 1);

        // All addresses should have addr field set to 31 for the last entry
        for addr in addr_31 {
            assert_eq!(addr.addr(), 31);
        }
    }

    #[test]
    fn test_make_data_template_function() {
        // Test the make_data_template function directly to ensure code coverage
        let template = make_data_template::<TEST_COLS>();

        // Verify basic properties
        assert_eq!(template.len(), TEST_COLS);

        // All entries should have latch=false
        for entry in &template {
            assert_eq!(entry.latch(), false);
        }

        // Exactly one entry should have output_enable=false (the last logical column)
        let oe_false_count = template
            .iter()
            .filter(|entry| !entry.output_enable())
            .count();
        assert_eq!(oe_false_count, 1);

        // Test with a small template size to verify edge cases
        let small_template = make_data_template::<4>();
        assert_eq!(small_template.len(), 4);

        let oe_false_count = small_template
            .iter()
            .filter(|entry| !entry.output_enable())
            .count();
        assert_eq!(oe_false_count, 1);

        // Test with single column - but skip this test if ESP32 ordering is enabled
        // because the mapping function assumes at least 4 columns for proper mapping
        #[cfg(not(feature = "esp32-ordering"))]
        {
            let single_template = make_data_template::<1>();
            assert_eq!(single_template.len(), 1);
            assert_eq!(single_template[0].output_enable(), false); // Single column should have OE=false
            assert_eq!(single_template[0].latch(), false);
        }
    }

    #[test]
    fn test_addr_table_correctness() {
        // Test that the pre-computed address table matches the original logic
        for addr in 0..32 {
            let mut expected_addresses = [Address::new(); 4];

            // Original logic
            for i in 0..4 {
                let latch = !matches!(i, 3);
                #[cfg(feature = "esp32-ordering")]
                let mapped_i = map_index(i);
                #[cfg(not(feature = "esp32-ordering"))]
                let mapped_i = i;

                expected_addresses[mapped_i].set_latch(latch);
                expected_addresses[mapped_i].set_addr(addr);
            }

            // Compare with table
            let table_addresses = &ADDR_TABLE[addr as usize];
            for i in 0..4 {
                assert_eq!(table_addresses[i].0, expected_addresses[i].0);
            }
        }
    }

    // Helper constants for the glyph dimensions used by FONT_6X10
    const CHAR_W: i32 = 6;
    const CHAR_H: i32 = 10;

    /// Draws the glyph 'A' at `origin` and verifies every pixel against a software reference.  
    /// Re-usable for different panel locations.
    fn verify_glyph_at(fb: &mut TestFrameBuffer, origin: Point) {
        use embedded_graphics::mock_display::MockDisplay;
        use embedded_graphics::mono_font::ascii::FONT_6X10;
        use embedded_graphics::mono_font::MonoTextStyle;
        use embedded_graphics::text::{Baseline, Text};

        // Draw the character on the framebuffer.
        let style = MonoTextStyle::new(&FONT_6X10, Color::WHITE);
        Text::with_baseline("A", origin, style, Baseline::Top)
            .draw(fb)
            .unwrap();

        // Reference bitmap for the glyph at (0,0)
        let mut reference: MockDisplay<Color> = MockDisplay::new();
        Text::with_baseline("A", Point::zero(), style, Baseline::Top)
            .draw(&mut reference)
            .unwrap();

        // Iterate over the glyph's bounding box and compare pixel states.
        for dy in 0..CHAR_H {
            for dx in 0..CHAR_W {
                let expected_on = reference
                    .get_pixel(Point::new(dx, dy))
                    .unwrap_or(Color::BLACK)
                    != Color::BLACK;

                let gx = (origin.x + dx) as usize;
                let gy = (origin.y + dy) as usize;

                // we have computed the origin to be within the panel, so we don't need to check for bounds
                // if gx >= TEST_COLS || gy >= TEST_ROWS {
                //     continue;
                // }

                // Fetch the entry from frame 0 directly.
                let frame0 = &fb.frames[0];
                let e = if gy < TEST_NROWS {
                    &frame0.rows[gy].data[map_index(gx)]
                } else {
                    &frame0.rows[gy - TEST_NROWS].data[map_index(gx)]
                };

                let (r, g, b) = if gy >= TEST_NROWS {
                    (e.red2(), e.grn2(), e.blu2())
                } else {
                    (e.red1(), e.grn1(), e.blu1())
                };

                if expected_on {
                    assert!(r && g && b);
                } else {
                    assert!(!r && !g && !b);
                }
            }
        }
    }

    #[test]
    fn test_draw_char_corners() {
        // Upper-left and lower-right character placement.
        let upper_left = Point::new(0, 0);
        let lower_right = Point::new(TEST_COLS as i32 - CHAR_W, TEST_ROWS as i32 - CHAR_H);

        let mut fb = TestFrameBuffer::new();

        // Verify glyph in the upper-left corner.
        verify_glyph_at(&mut fb, upper_left);
        // Verify glyph in the lower-right corner.
        verify_glyph_at(&mut fb, lower_right);
    }
}
