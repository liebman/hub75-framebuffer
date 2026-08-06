//! Bitplane framebuffer for an 8-bit latched HUB75 interface.
//!
//! This module provides a framebuffer that stores colour data as separate
//! bit-planes rather than the threshold-based frames used by
//! [`crate::latched::DmaFrameBuffer`]. Each plane holds one bit of every
//! colour channel, giving `PLANES` planes total (typically 8 for full 8-bit
//! colour). Row addressing is carried by four trailing `Address` bytes per
//! row, identical to the non-bitplane latched layout.
//!
//! # Hardware Requirements
//! Requires a parallel output peripheral capable of clocking 8 bits at a time,
//! plus an external latch circuit to hold the row address and gate the pixel
//! clock (same circuit as the non-bitplane latched variant).
//!
//! # HUB75 Signal Bit Mapping (8-bit words)
//! Two distinct 8-bit words are streamed to the panel:
//!
//! 1. **Address / Timing (`Address`)** -- row-select and latch control.
//! 2. **Pixel Data (`Entry`)** -- RGB bits for two sub-pixels plus OE/LAT
//!    shadow bits.
//!
//! ```text
//! Address word (row select & timing)
//! ┌──7─┬──6──┬─5─-┬─4─-┬─3-─┬─2-─┬─1-─┬─0-─┐
//! │ OE │ LAT │    │ E  │ D  │ C  │ B  │ A  │
//! └────┴─────┴───-┴───-┴───-┴───-┴───-┴───-┘
//! ```
//! ```text
//! Entry word (pixel data)
//! ┌──7─┬──6──┬─5──┬─4──┬─3──┬─2──┬─1──┬─0──┐
//! │ OE │ LAT │ B2 │ G2 │ R2 │ B1 │ G1 │ R1 │
//! └────┴─────┴────┴────┴────┴────┴────┴────┘
//! ```
//!
//! Bits 7-6 (OE/LAT) occupy the same positions in both words so the control
//! lines stay valid throughout the DMA stream.
//!
//! # Bitplane BCM Rendering
//! The framebuffer is organised into `PLANES` bit-planes. Plane 0 carries the
//! MSB (bit 7) of each colour channel, plane 1 carries bit 6, and so on down
//! to plane 7 which carries the LSB (bit 0).
//!
//! To produce correct brightness via Binary Code Modulation, configure the DMA
//! descriptor chain so that each plane's data is output (scanned) a number of
//! times equal to its bit-weight:
//!
//! ```text
//! plane 0 (bit 7) → output 2^7 = 128 times
//! plane 1 (bit 6) → output 2^6 =  64 times
//! plane 2 (bit 5) → output 2^5 =  32 times
//!   …
//! plane 7 (bit 0) → output 2^0 =   1 time
//! ```
//!
//! That is, each plane is scanned `2^(7 - plane_index)` times. The weighted
//! repetition counts sum to 255, reproducing the full 8-bit intensity range.
//! See <https://www.batsocks.co.uk/readme/art_bcm_1.htm> for background on
//! BCM.
//!
//! # Memory Usage
//! Memory scales linearly with `PLANES`: the buffer contains `PLANES` copies
//! of the row data (one per bit-plane). Unlike the threshold-based
//! [`crate::latched::DmaFrameBuffer`] whose frame count grows as
//! `2^BITS - 1`, this layout uses exactly `PLANES` planes regardless of
//! colour depth.
//!
//! Each row is `COLS` data bytes plus 4 address bytes, so total size is
//! `PLANES * NROWS * (COLS + 4)` bytes.

use core::convert::Infallible;

use bitfield::bitfield;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, Point, Size};

use crate::Color;
use crate::{BcmSegment, FrameBuffer};
use crate::{FrameBufferOperations, MutableFrameBuffer};

#[cfg(feature = "lead-blank-1")]
const LEAD_BLANK_DELAY: usize = 1;
#[cfg(feature = "lead-blank-2")]
const LEAD_BLANK_DELAY: usize = 2;
#[cfg(feature = "lead-blank-4")]
const LEAD_BLANK_DELAY: usize = 4;
#[cfg(feature = "lead-blank-8")]
const LEAD_BLANK_DELAY: usize = 8;
#[cfg(feature = "lead-blank-16")]
const LEAD_BLANK_DELAY: usize = 16;
#[cfg(feature = "lead-blank-32")]
const LEAD_BLANK_DELAY: usize = 32;

#[cfg(not(any(
    feature = "lead-blank-1",
    feature = "lead-blank-2",
    feature = "lead-blank-4",
    feature = "lead-blank-8",
    feature = "lead-blank-16",
    feature = "lead-blank-32"
)))]
const LEAD_BLANK_DELAY: usize = 0;

#[cfg(feature = "trail-blank-1")]
const TRAIL_BLANK_DELAY: usize = 1;
#[cfg(feature = "trail-blank-2")]
const TRAIL_BLANK_DELAY: usize = 2;
#[cfg(feature = "trail-blank-4")]
const TRAIL_BLANK_DELAY: usize = 4;
#[cfg(feature = "trail-blank-8")]
const TRAIL_BLANK_DELAY: usize = 8;
#[cfg(feature = "trail-blank-16")]
const TRAIL_BLANK_DELAY: usize = 16;
#[cfg(feature = "trail-blank-32")]
const TRAIL_BLANK_DELAY: usize = 32;

#[cfg(not(any(
    feature = "trail-blank-1",
    feature = "trail-blank-2",
    feature = "trail-blank-4",
    feature = "trail-blank-8",
    feature = "trail-blank-16",
    feature = "trail-blank-32"
)))]
const TRAIL_BLANK_DELAY: usize = 0;

#[cfg(feature = "inter-row-blank-4")]
const INTER_ROW_BLANK: usize = 4;
#[cfg(feature = "inter-row-blank-8")]
const INTER_ROW_BLANK: usize = 8;
#[cfg(feature = "inter-row-blank-16")]
const INTER_ROW_BLANK: usize = 16;
#[cfg(feature = "inter-row-blank-32")]
const INTER_ROW_BLANK: usize = 32;
#[cfg(not(any(
    feature = "inter-row-blank-4",
    feature = "inter-row-blank-8",
    feature = "inter-row-blank-16",
    feature = "inter-row-blank-32"
)))]
const INTER_ROW_BLANK: usize = 0;

#[cfg(not(feature = "invert-oe"))]
const OE_ACTIVE: u8 = 0b1000_0000;
#[cfg(not(feature = "invert-oe"))]
const OE_BLANK: u8 = 0;

#[cfg(feature = "invert-oe")]
const OE_ACTIVE: u8 = 0;
#[cfg(feature = "invert-oe")]
const OE_BLANK: u8 = 0b1000_0000;

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq, Eq)]
    #[repr(transparent)]
    pub(crate) struct Address(u8);
    impl Debug;
    pub(crate) output_enable, set_output_enable: 7;
    pub(crate) latch, set_latch: 6;
    pub(crate) addr, set_addr: 4, 0;
}

impl Address {
    pub const fn new() -> Self {
        Self(0)
    }
}

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    pub(crate) struct Entry(u8);
    impl Debug;
    pub(crate) output_enable, set_output_enable: 7;
    pub(crate) latch, set_latch: 6;
    pub(crate) blu2, set_blu2: 5;
    pub(crate) grn2, set_grn2: 4;
    pub(crate) red2, set_red2: 3;
    pub(crate) blu1, set_blu1: 2;
    pub(crate) grn1, set_grn1: 1;
    pub(crate) red1, set_red1: 0;
}

impl Entry {
    pub const fn new() -> Self {
        Self(0)
    }

    const COLOR0_MASK: u8 = 0b0000_0111;
    const COLOR1_MASK: u8 = 0b0011_1000;

    #[inline]
    fn set_color0_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::COLOR0_MASK) | (bits & Self::COLOR0_MASK);
    }

    #[inline]
    fn set_color1_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::COLOR1_MASK) | ((bits << 3) & Self::COLOR1_MASK);
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
/// A single BCM row payload for 8-bit latched output.
///
/// Each row contains color-stream data for `COLS` pixels followed by four
/// address/control bytes that clock the row address into the external latch.
pub struct Row<const COLS: usize> {
    pub(crate) data: [Entry; COLS],
    pub(crate) address: [Address; 4],
    pub(crate) gap: [Entry; INTER_ROW_BLANK],
}

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

const fn make_addr_table() -> [[Address; 4]; 32] {
    let mut tbl = [[Address::new(); 4]; 32];
    let mut addr = 0;
    while addr < 32 {
        tbl[addr][map_index(0)].0 = OE_BLANK | 1u8 << 6 | addr as u8;
        tbl[addr][map_index(1)].0 = OE_BLANK | 1u8 << 6 | addr as u8;
        tbl[addr][map_index(2)].0 = OE_BLANK | addr as u8;
        tbl[addr][map_index(3)].0 = OE_BLANK;
        addr += 1;
    }
    tbl
}

const ADDR_TABLE: [[Address; 4]; 32] = make_addr_table();

#[allow(clippy::absurd_extreme_comparisons)]
const fn make_data_template<const COLS: usize>() -> [Entry; COLS] {
    let mut data = [Entry::new(); COLS];
    let mut i = 0;
    while i < COLS {
        let mapped_i = map_index(i);
        data[mapped_i].0 =
            if i >= TRAIL_BLANK_DELAY && i < COLS.saturating_sub(LEAD_BLANK_DELAY + 1) {
                OE_ACTIVE
            } else {
                OE_BLANK
            };
        i += 1;
    }
    data
}

impl<const COLS: usize> Row<COLS> {
    /// Creates a zero-initialized row.
    ///
    /// Call [`Self::format`] before first use to populate row address/control
    /// metadata.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            data: [Entry::new(); COLS],
            address: [Address::new(); 4],
            gap: [Entry::new(); INTER_ROW_BLANK],
        }
    }

    /// Formats this row for the provided multiplexed row address.
    ///
    /// This sets the trailing address bytes and initializes output-enable/latch
    /// bits in the pixel stream template.
    ///
    /// # Panics
    ///
    /// Panics if `addr` is out of range for the address lookup table.
    #[inline]
    pub const fn format(&mut self, addr: u8) {
        assert!((addr as usize) < ADDR_TABLE.len());
        let src_addr = ADDR_TABLE[addr as usize];
        self.address[0] = src_addr[0];
        self.address[1] = src_addr[1];
        self.address[2] = src_addr[2];
        self.address[3] = src_addr[3];

        let data_template = make_data_template::<COLS>();
        let mut i = 0;
        while i < COLS {
            self.data[i] = data_template[i];
            i += 1;
        }

        let mut i = 0;
        // INTER_ROW_BLANK is 0 unless an inter-row-blank-* feature is enabled.
        #[allow(clippy::absurd_extreme_comparisons)]
        while i < INTER_ROW_BLANK {
            self.gap[i].0 = OE_BLANK;
            i += 1;
        }
    }
}

impl<const COLS: usize> Default for Row<COLS> {
    fn default() -> Self {
        Self::new()
    }
}

/// The entire BCM Frame Buffer (Contiguous Memory)
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DmaFrameBuffer<const NROWS: usize, const COLS: usize, const PLANES: usize> {
    pub(crate) planes: [[Row<COLS>; NROWS]; PLANES],
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize>
    DmaFrameBuffer<NROWS, COLS, PLANES>
{
    /// Creates a new frame buffer.
    ///
    /// # Panics
    /// Panics if `NROWS` is not within `1..=32` (5-bit row address) or
    /// `PLANES` is not within `1..=8` (8-bit color depth). In const contexts
    /// (e.g. `static` framebuffers) this is a compile-time error.
    #[must_use]
    pub const fn new() -> Self {
        assert!(NROWS >= 1 && NROWS <= 32, "NROWS must be within 1..=32");
        assert!(PLANES >= 1 && PLANES <= 8, "PLANES must be within 1..=8");
        let mut instance = Self {
            planes: [[Row::new(); NROWS]; PLANES],
        };
        instance.format();
        instance
    }

    /// Returns the number of BCM chunks (one per bit-plane).
    #[must_use]
    pub const fn bcm_chunk_count() -> usize {
        PLANES
    }

    /// Returns the byte size of one BCM chunk (a single bit-plane).
    #[must_use]
    pub const fn bcm_chunk_bytes() -> usize {
        NROWS * core::mem::size_of::<Row<COLS>>()
    }

    /// Computes the number of DMA descriptors required for this framebuffer.
    ///
    /// `max_chunk` is the platform-specific maximum DMA transfer size in bytes.
    #[must_use]
    pub const fn dma_descriptor_count(max_chunk: usize) -> usize {
        let chunk_bytes = NROWS * core::mem::size_of::<Row<COLS>>();
        let descs_per_plane = chunk_bytes.div_ceil(max_chunk);
        let total_reps = (1usize << PLANES) - 1;
        descs_per_plane * total_reps
    }

    /// Returns the number of rows per plane for row-based BCM.
    #[must_use]
    pub const fn bcm_row_count() -> usize {
        NROWS
    }

    /// Formats the frame buffer with row addresses and control bits.
    #[inline]
    pub const fn format(&mut self) {
        let mut p = 0;
        while p < PLANES {
            let mut row_idx = 0;
            while row_idx < NROWS {
                self.planes[p][row_idx].format(row_idx as u8);
                row_idx += 1;
            }
            p += 1;
        }
    }

    /// Erase pixel colors while preserving row control data.
    #[inline]
    pub fn erase(&mut self) {
        const MASK: u8 = !0b0011_1111;
        for plane in &mut self.planes {
            for row in plane {
                for entry in &mut row.data {
                    entry.0 &= MASK;
                }
            }
        }
    }

    /// Set a pixel in the framebuffer.
    #[inline]
    pub fn set_pixel(&mut self, p: Point, color: Color) {
        if p.x < 0 || p.y < 0 {
            return;
        }
        self.set_pixel_internal(p.x as usize, p.y as usize, color);
    }

    #[inline]
    fn set_pixel_internal(&mut self, x: usize, y: usize, color: Color) {
        if x >= COLS || y >= NROWS * 2 {
            return;
        }

        // Early exit for black pixels - common in UI backgrounds
        // Only enabled when skip-black-pixels feature is active
        #[cfg(feature = "skip-black-pixels")]
        if color == Color::BLACK {
            return;
        }

        let row_idx = if y < NROWS { y } else { y - NROWS };
        let is_top = y < NROWS;
        let red = color.r();
        let green = color.g();
        let blue = color.b();

        for plane_idx in 0..PLANES {
            let bit = 7_u32.saturating_sub(plane_idx as u32);
            let bits = ((u8::from(((blue >> bit) & 1) != 0)) << 2)
                | ((u8::from(((green >> bit) & 1) != 0)) << 1)
                | u8::from(((red >> bit) & 1) != 0);
            let col_idx = map_index(x);
            let entry = &mut self.planes[plane_idx][row_idx].data[col_idx];
            if is_top {
                entry.set_color0_bits(bits);
            } else {
                entry.set_color1_bits(bits);
            }
        }
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> Default
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> core::fmt::Debug
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DmaFrameBuffer")
            .field("size", &core::mem::size_of_val(&self.planes))
            .field("plane_count", &self.planes.len())
            .field("plane_size", &core::mem::size_of_val(&self.planes[0]))
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<const NROWS: usize, const COLS: usize, const PLANES: usize> defmt::Format
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "DmaFrameBuffer<{}, {}, {}>", NROWS, COLS, PLANES);
        defmt::write!(f, " size: {}", core::mem::size_of_val(&self.planes));
        defmt::write!(
            f,
            " plane_size: {}",
            core::mem::size_of_val(&self.planes[0])
        );
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> FrameBuffer
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    type Word = u8;

    fn bcm_segment_count(&self) -> usize {
        PLANES
    }

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        assert!(
            index < PLANES,
            "segment index {index} out of range for {PLANES} planes"
        );
        let ptr = self.planes[index].as_ptr().cast::<u8>();
        let len = NROWS * core::mem::size_of::<Row<COLS>>();
        let reps = 1usize << (PLANES - 1 - index);
        BcmSegment { ptr, len, reps }
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> FrameBufferOperations
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    #[inline]
    fn erase(&mut self) {
        DmaFrameBuffer::<NROWS, COLS, PLANES>::erase(self);
    }

    #[inline]
    fn set_pixel(&mut self, p: Point, color: Color) {
        DmaFrameBuffer::<NROWS, COLS, PLANES>::set_pixel(self, p, color);
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> MutableFrameBuffer
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> OriginDimensions
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    fn size(&self) -> Size {
        Size::new(COLS as u32, (NROWS * 2) as u32)
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> DrawTarget
    for DmaFrameBuffer<NROWS, COLS, PLANES>
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

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_graphics::prelude::*;
    use std::format;

    const TEST_COLS: usize = if LEAD_BLANK_DELAY + TRAIL_BLANK_DELAY + 2 > 64 {
        128
    } else {
        64
    };
    type TestBuffer = DmaFrameBuffer<16, TEST_COLS, 8>;

    #[test]
    fn row_format_sets_address_and_control_bits() {
        const TEST_N: usize = TEST_COLS;
        let mut row = Row::<TEST_N>::new();
        row.format(5);
        let latch_count = row.address.iter().filter(|a| a.latch()).count();
        assert_eq!(latch_count, 2);
        assert_eq!(row.address[map_index(0)].addr(), 5);
        assert_eq!(row.address[map_index(1)].addr(), 5);
        assert_eq!(row.address[map_index(2)].addr(), 5);
        assert_eq!(row.address[map_index(3)].0, OE_BLANK);
        let oe_active = !cfg!(feature = "invert-oe");
        let active_count = TEST_N.saturating_sub(LEAD_BLANK_DELAY + TRAIL_BLANK_DELAY + 1);
        let blank_count = TEST_N - active_count;
        let oe_blank_count = row
            .data
            .iter()
            .filter(|entry| entry.output_enable() != oe_active)
            .count();
        assert_eq!(oe_blank_count, blank_count);
    }

    #[test]
    fn format_sets_expected_row_addresses_for_all_rows() {
        let mut fb = TestBuffer::new();
        fb.format();

        for plane_idx in 0..8 {
            for row_idx in 0..16 {
                let row = &fb.planes[plane_idx][row_idx];
                assert_eq!(row.address[map_index(0)].addr(), row_idx as u8);
                assert_eq!(row.address[map_index(1)].addr(), row_idx as u8);
                assert_eq!(row.address[map_index(2)].addr(), row_idx as u8);
                assert_eq!(row.address[map_index(3)].0, OE_BLANK);
            }
        }
    }

    #[test]
    fn set_pixel_maps_top_half_bits_per_plane() {
        let mut fb = TestBuffer::new();
        let color = Color::new(0b1010_0101, 0b0101_1010, 0b1111_0000);
        fb.set_pixel(Point::new(2, 3), color);

        for plane_idx in 0..8 {
            let bit = 7 - plane_idx;
            let entry = fb.planes[plane_idx][3].data[map_index(2)];
            assert_eq!(entry.red1(), ((color.r() >> bit) & 1) != 0);
            assert_eq!(entry.grn1(), ((color.g() >> bit) & 1) != 0);
            assert_eq!(entry.blu1(), ((color.b() >> bit) & 1) != 0);
        }
    }

    #[test]
    fn set_pixel_maps_bottom_half_bits_per_plane() {
        let mut fb = TestBuffer::new();
        let color = Color::new(0b1100_0011, 0b0011_1100, 0b1001_0110);
        fb.set_pixel(Point::new(4, 20), color);

        for plane_idx in 0..8 {
            let bit = 7 - plane_idx;
            let entry = fb.planes[plane_idx][4].data[map_index(4)];
            assert_eq!(entry.red2(), ((color.r() >> bit) & 1) != 0);
            assert_eq!(entry.grn2(), ((color.g() >> bit) & 1) != 0);
            assert_eq!(entry.blu2(), ((color.b() >> bit) & 1) != 0);
        }
    }

    #[test]
    fn erase_clears_only_color_bits() {
        let mut fb = TestBuffer::new();
        let oe_before = fb.planes[0][0].data[0].output_enable();
        fb.set_pixel(Point::new(0, 0), Color::WHITE);
        fb.erase();

        for plane in &fb.planes {
            for row in plane {
                for entry in &row.data {
                    assert!(!entry.red1());
                    assert!(!entry.grn1());
                    assert!(!entry.blu1());
                    assert!(!entry.red2());
                    assert!(!entry.grn2());
                    assert!(!entry.blu2());
                }
            }
        }

        assert_eq!(fb.planes[0][0].data[0].output_enable(), oe_before);
    }

    #[test]
    fn draw_target_iter_sets_pixels() {
        let mut fb = TestBuffer::new();
        let pixels = [Pixel(Point::new(1, 1), Color::RED)];
        let result = fb.draw_iter(pixels);
        assert!(result.is_ok());

        for plane_idx in 0..8 {
            let bit = 7 - plane_idx;
            let entry = fb.planes[plane_idx][1].data[map_index(1)];
            assert_eq!(entry.red1(), ((Color::RED.r() >> bit) & 1) != 0);
            assert!(!entry.grn1());
            assert!(!entry.blu1());
        }
    }

    #[test]
    fn set_pixel_ignores_out_of_bounds_and_negative() {
        let mut fb = TestBuffer::new();
        let before = fb.planes;
        fb.set_pixel(Point::new(-1, 0), Color::WHITE);
        fb.set_pixel(Point::new(0, -1), Color::WHITE);
        fb.set_pixel(Point::new(TEST_COLS as i32, 0), Color::WHITE);
        fb.set_pixel(Point::new(0, 32), Color::WHITE);
        assert_eq!(fb.planes, before);
    }

    #[test]
    #[cfg(feature = "skip-black-pixels")]
    fn test_skip_black_pixels_enabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.planes[0][5].data[mapped_col_10].red1());
        assert!(!fb.planes[0][5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][5].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels enabled, this should be ignored
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should still be red (black write was skipped)
        assert!(fb.planes[0][5].data[mapped_col_10].red1());
        assert!(!fb.planes[0][5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][5].data[mapped_col_10].blu1());
    }

    #[test]
    #[cfg(not(feature = "skip-black-pixels"))]
    fn test_skip_black_pixels_disabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.planes[0][5].data[mapped_col_10].red1());
        assert!(!fb.planes[0][5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][5].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels disabled, this should overwrite
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should now be black (all bits false)
        assert!(!fb.planes[0][5].data[mapped_col_10].red1());
        assert!(!fb.planes[0][5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][5].data[mapped_col_10].blu1());
    }

    #[test]
    #[should_panic(expected = "NROWS must be within 1..=32")]
    fn new_panics_for_too_many_rows() {
        let _ = DmaFrameBuffer::<33, TEST_COLS, 8>::new();
    }

    #[test]
    #[should_panic(expected = "PLANES must be within 1..=8")]
    fn new_panics_for_too_many_planes() {
        let _ = DmaFrameBuffer::<16, TEST_COLS, 9>::new();
    }

    #[test]
    fn bcm_chunk_info_for_common_panel() {
        assert_eq!(TestBuffer::bcm_chunk_count(), 8);
        assert_eq!(
            TestBuffer::bcm_chunk_bytes(),
            16 * core::mem::size_of::<Row<TEST_COLS>>()
        );
    }

    #[test]
    fn origin_dimensions_match_panel_geometry() {
        let fb = TestBuffer::new();
        assert_eq!(fb.size(), Size::new(TEST_COLS as u32, 32));
    }

    #[test]
    fn debug_impl_includes_shape_information() {
        let fb = TestBuffer::new();
        let s = format!("{fb:?}");
        assert!(s.contains("DmaFrameBuffer"));
        assert!(s.contains("plane_count"));
        assert!(s.contains("plane_size"));
    }

    #[test]
    fn row_format_sets_exactly_one_data_word_with_oe_low() {
        const TEST_N: usize = TEST_COLS;
        let mut row = Row::<TEST_N>::new();
        row.format(9);

        let oe_active = !cfg!(feature = "invert-oe");
        let active_count = TEST_N.saturating_sub(LEAD_BLANK_DELAY + TRAIL_BLANK_DELAY + 1);
        let blank_count = TEST_N - active_count;
        let oe_blank_indices: std::vec::Vec<_> = row
            .data
            .iter()
            .enumerate()
            .filter_map(|(i, entry)| (entry.output_enable() != oe_active).then_some(i))
            .collect();
        assert_eq!(oe_blank_indices.len(), blank_count);
        assert!(oe_blank_indices.contains(&map_index(TEST_N - 1)));
    }

    #[test]
    fn default_constructors_match_new() {
        let row_default = Row::<TEST_COLS>::default();
        let row_new = Row::<TEST_COLS>::new();
        assert_eq!(row_default, row_new);

        let fb_default = TestBuffer::default();
        let fb_new = TestBuffer::new();
        assert_eq!(fb_default.planes, fb_new.planes);
    }

    #[test]
    fn framebuffer_operations_trait_delegates_correctly() {
        let mut fb = TestBuffer::new();
        FrameBufferOperations::set_pixel(&mut fb, Point::new(3, 5), Color::GREEN);

        assert!(fb.planes[0][5].data[map_index(3)].grn1());

        FrameBufferOperations::erase(&mut fb);
        for plane in &fb.planes {
            for row in plane {
                for entry in &row.data {
                    assert!(!entry.red1());
                    assert!(!entry.grn1());
                    assert!(!entry.blu1());
                    assert!(!entry.red2());
                    assert!(!entry.grn2());
                    assert!(!entry.blu2());
                }
            }
        }
    }

    #[test]
    fn addr_table_entries_are_consistent() {
        let table = make_addr_table();
        for addr in 0..32u8 {
            let row = &table[addr as usize];
            // First two clocks: latch asserted with row address
            assert!(row[map_index(0)].latch());
            assert_eq!(row[map_index(0)].addr(), addr);
            assert!(row[map_index(1)].latch());
            assert_eq!(row[map_index(1)].addr(), addr);
            // Third clock: latch released, address still driven
            assert!(!row[map_index(2)].latch());
            assert_eq!(row[map_index(2)].addr(), addr);
            // Fourth clock: clear cycle, only OE_BLANK
            assert!(!row[map_index(3)].latch());
            assert_eq!(row[map_index(3)].0, OE_BLANK);
        }
        assert_eq!(table, ADDR_TABLE);
    }

    #[test]
    fn test_blanking_delay() {
        let mut row = Row::<TEST_COLS>::new();
        row.format(5);

        let oe_active = !cfg!(feature = "invert-oe");

        // Trail blank: indices 0..TRAIL_BLANK_DELAY-1 (DMA start = physical right edge)
        if TRAIL_BLANK_DELAY > 0 {
            let trail_blank_idx = map_index(TRAIL_BLANK_DELAY - 1);
            assert_eq!(row.data[trail_blank_idx].output_enable(), !oe_active);
        }

        let first_active_idx = map_index(TRAIL_BLANK_DELAY);
        assert_eq!(row.data[first_active_idx].output_enable(), oe_active);

        // Lead blank: indices before latch (DMA end = physical left edge)
        let last_active_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 2);
        assert_eq!(row.data[last_active_idx].output_enable(), oe_active);

        let lead_blank_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
        assert_eq!(row.data[lead_blank_idx].output_enable(), !oe_active);

        let last_pixel_idx = map_index(TEST_COLS - 1);
        assert_eq!(row.data[last_pixel_idx].output_enable(), !oe_active);
    }

    static STATIC_FB: TestBuffer = TestBuffer::new();

    #[test]
    fn test_static_construction_is_formatted() {
        let runtime_fb = TestBuffer::new();

        for (pi, plane) in STATIC_FB.planes.iter().enumerate() {
            for (ri, row) in plane.iter().enumerate() {
                assert_eq!(
                    row.data, runtime_fb.planes[pi][ri].data,
                    "static vs runtime data mismatch at plane {pi}, row {ri}"
                );
                assert_eq!(
                    row.address, runtime_fb.planes[pi][ri].address,
                    "static vs runtime address mismatch at plane {pi}, row {ri}"
                );
                assert_eq!(
                    row.gap, runtime_fb.planes[pi][ri].gap,
                    "static vs runtime gap mismatch at plane {pi}, row {ri}"
                );
            }
        }
    }

    #[test]
    fn test_format_reinitializes_at_runtime() {
        let mut fb = TestBuffer::new();
        fb.erase();
        fb.format();

        for (pi, plane) in fb.planes.iter().enumerate() {
            for (ri, row) in plane.iter().enumerate() {
                assert_eq!(
                    row.data, STATIC_FB.planes[pi][ri].data,
                    "re-formatted vs static mismatch at plane {pi}, row {ri}"
                );
            }
        }
    }

    #[test]
    fn bcm_segment_count_equals_planes() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        assert_eq!(fb.bcm_segment_count(), 8);
    }

    #[test]
    fn bcm_segments_have_correct_reps() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        for i in 0..8 {
            let seg = fb.bcm_segment(i);
            assert_eq!(seg.reps, 1 << (7 - i), "wrong reps for segment {i}");
        }
    }

    #[test]
    fn bcm_segments_point_to_plane_data() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let plane_len = 16 * core::mem::size_of::<Row<TEST_COLS>>();
        for i in 0..8 {
            let seg = fb.bcm_segment(i);
            let expected_ptr = fb.planes[i].as_ptr().cast::<u8>();
            assert_eq!(seg.ptr, expected_ptr, "wrong ptr for segment {i}");
            assert_eq!(seg.len, plane_len, "wrong len for segment {i}");
        }
    }

    #[test]
    fn bcm_segment_total_reps_equals_2_pow_planes_minus_1() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let total: usize = (0..fb.bcm_segment_count())
            .map(|i| fb.bcm_segment(i).reps)
            .sum();
        assert_eq!(total, (1 << 8) - 1);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn bcm_segment_panics_for_invalid_index() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let _ = fb.bcm_segment(8);
    }

    #[test]
    fn dma_descriptor_count_matches_expected() {
        let chunk_bytes = 16 * core::mem::size_of::<Row<TEST_COLS>>();
        let max_chunk = 4092;
        let descs_per_plane = chunk_bytes.div_ceil(max_chunk);
        let total_reps = (1usize << 8) - 1;
        let expected = descs_per_plane * total_reps;
        assert_eq!(TestBuffer::dma_descriptor_count(max_chunk), expected);
    }
}
