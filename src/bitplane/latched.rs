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
use crate::FrameBuffer;
use crate::WordSize;
use crate::{FrameBufferOperations, MutableFrameBuffer};

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

const fn make_data_template<const COLS: usize>() -> [Entry; COLS] {
    let mut data = [Entry::new(); COLS];
    let mut i = 0;
    while i < COLS {
        let mapped_i = map_index(i);
        data[mapped_i].0 = if i == COLS - 1 { 0 } else { 0b1000_0000 };
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
        }
    }

    /// Formats this row for the provided multiplexed row address.
    ///
    /// This sets the trailing address bytes and initializes output-enable/latch
    /// bits in the pixel stream template.
    #[inline]
    pub fn format(&mut self, addr: u8) {
        debug_assert!((addr as usize) < ADDR_TABLE.len());
        let src_addr = &ADDR_TABLE[addr as usize];
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
    #[must_use]
    pub fn new() -> Self {
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

    /// Formats the frame buffer with row addresses and control bits.
    #[inline]
    pub fn format(&mut self) {
        for plane in &mut self.planes {
            for (row_idx, row) in plane.iter_mut().enumerate() {
                row.format(row_idx as u8);
            }
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
    fn get_word_size(&self) -> WordSize {
        WordSize::Eight
    }

    fn plane_count(&self) -> usize {
        PLANES
    }

    fn plane_ptr_len(&self, plane_idx: usize) -> (*const u8, usize) {
        assert!(
            plane_idx < PLANES,
            "plane_idx {plane_idx} out of range for {PLANES} planes"
        );
        let ptr = self.planes[plane_idx].as_ptr().cast::<u8>();
        let len = NROWS * core::mem::size_of::<Row<COLS>>();
        (ptr, len)
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

    type TestBuffer = DmaFrameBuffer<16, 64, 8>;

    #[test]
    fn row_format_sets_address_and_control_bits() {
        let mut row = Row::<8>::new();
        row.format(5);
        let latch_false_count = row.address.iter().filter(|addr| !addr.latch()).count();
        assert_eq!(latch_false_count, 1);
        for addr in &row.address {
            assert_eq!(addr.addr(), 5);
        }
        let oe_false_count = row
            .data
            .iter()
            .filter(|entry| !entry.output_enable())
            .count();
        assert_eq!(oe_false_count, 1);
    }

    #[test]
    fn format_sets_expected_row_addresses_for_all_rows() {
        let mut fb = TestBuffer::new();
        fb.format();

        for plane_idx in 0..8 {
            for row_idx in 0..16 {
                for addr in &fb.planes[plane_idx][row_idx].address {
                    assert_eq!(addr.addr(), row_idx as u8);
                }
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
        fb.set_pixel(Point::new(64, 0), Color::WHITE);
        fb.set_pixel(Point::new(0, 32), Color::WHITE);
        assert_eq!(fb.planes, before);
    }

    #[test]
    fn bcm_chunk_info_for_common_panel() {
        assert_eq!(TestBuffer::bcm_chunk_count(), 8);
        assert_eq!(
            TestBuffer::bcm_chunk_bytes(),
            16 * core::mem::size_of::<Row<64>>()
        );
    }
}
