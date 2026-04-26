//! Framebuffer format for a 16-bit plain HUB75 interface with true bitplane BCM.
//!
//! This variant uses 16-bit entries where row addressing, latch, OE, and colour
//! data are all packed into each word (no external latch circuit required).
//! BCM timing is achieved by reusing DMA descriptors in a weighted chain, with
//! one bit-plane per colour bit.

use core::convert::Infallible;

use bitfield::bitfield;
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, Point, Size};

use crate::Color;
use crate::FrameBuffer;
use crate::WordSize;
use crate::{FrameBufferOperations, MutableFrameBuffer};

const BLANKING_DELAY: usize = 1;

#[inline]
const fn map_index(i: usize) -> usize {
    #[cfg(feature = "esp32-ordering")]
    {
        i ^ 1
    }
    #[cfg(not(feature = "esp32-ordering"))]
    {
        i
    }
}

#[inline]
const fn make_data_template<const COLS: usize>(addr: u8, prev_addr: u8) -> [Entry; COLS] {
    let mut data = [Entry::new(); COLS];
    let mut i = 0;

    while i < COLS {
        let mut entry = Entry::new();
        entry.0 = prev_addr as u16;

        if i == 1 {
            entry.0 |= 0b1_0000_0000; // OE
        } else if i == COLS - BLANKING_DELAY - 1 {
            // OE stays false
        } else if i == COLS - 1 {
            entry.0 |= 0b0010_0000; // latch
            entry.0 = (entry.0 & !0b0001_1111) | (addr as u16); // new address
        } else if i > 1 && i < COLS - BLANKING_DELAY - 1 {
            entry.0 |= 0b1_0000_0000; // OE
        }

        data[map_index(i)] = entry;
        i += 1;
    }

    data
}

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    pub(crate) struct Entry(u16);
    pub(crate) dummy2, set_dummy2: 15;
    pub(crate) blu2, set_blu2: 14;
    pub(crate) grn2, set_grn2: 13;
    pub(crate) red2, set_red2: 12;
    pub(crate) blu1, set_blu1: 11;
    pub(crate) grn1, set_grn1: 10;
    pub(crate) red1, set_red1: 9;
    pub(crate) output_enable, set_output_enable: 8;
    pub(crate) dummy1, set_dummy1: 7;
    pub(crate) dummy0, set_dummy0: 6;
    pub(crate) latch, set_latch: 5;
    pub(crate) addr, set_addr: 4, 0;
}

impl core::fmt::Debug for Entry {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("Entry")
            .field(&format_args!("{:#x}", self.0))
            .finish()
    }
}

impl Entry {
    const fn new() -> Self {
        Self(0)
    }

    const COLOR0_MASK: u16 = 0b0000_1110_0000_0000; // bits 9-11: R1, G1, B1
    const COLOR1_MASK: u16 = 0b0111_0000_0000_0000; // bits 12-14: R2, G2, B2

    #[inline]
    fn set_color0_bits(&mut self, bits: u8) {
        let bits16 = u16::from(bits) << 9;
        self.0 = (self.0 & !Self::COLOR0_MASK) | (bits16 & Self::COLOR0_MASK);
    }

    #[inline]
    fn set_color1_bits(&mut self, bits: u8) {
        let bits16 = u16::from(bits) << 12;
        self.0 = (self.0 & !Self::COLOR1_MASK) | (bits16 & Self::COLOR1_MASK);
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
/// A single BCM row payload for 16-bit plain output.
///
/// Row addressing, latch, OE, and pixel colour data are all encoded into the
/// 16-bit `Entry` words -- no separate address bytes are needed.
pub struct Row<const COLS: usize> {
    pub(crate) data: [Entry; COLS],
}

impl<const COLS: usize> Row<COLS> {
    /// Creates a zero-initialized row.
    ///
    /// Call [`Self::format`] before first use to populate row control metadata.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            data: [Entry::new(); COLS],
        }
    }

    /// Formats this row for the provided multiplexed row address.
    ///
    /// Sets up blanking delay, output-enable, latch, and address bits in the
    /// pixel stream template.
    #[inline]
    pub fn format(&mut self, addr: u8, prev_addr: u8) {
        let template = make_data_template::<COLS>(addr, prev_addr);
        self.data.copy_from_slice(&template);
    }
}

impl<const COLS: usize> Default for Row<COLS> {
    fn default() -> Self {
        Self::new()
    }
}

/// The entire BCM Frame Buffer (per-plane storage).
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DmaFrameBuffer<const NROWS: usize, const COLS: usize, const PLANES: usize> {
    pub(crate) planes: [[Row<COLS>; NROWS]; PLANES],
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize>
    DmaFrameBuffer<NROWS, COLS, PLANES>
{
    /// Creates a new frame buffer, pre-formatted and ready for use.
    #[must_use]
    pub fn new() -> Self {
        let mut instance = Self {
            planes: [[Row::new(); NROWS]; PLANES],
        };
        instance.format();
        instance
    }

    /// Returns the number of BCM bit-planes.
    #[must_use]
    pub const fn plane_count() -> usize {
        PLANES
    }

    /// Returns the byte size of a single bit-plane.
    #[must_use]
    pub const fn plane_size_bytes() -> usize {
        NROWS * core::mem::size_of::<Row<COLS>>()
    }

    /// Formats the frame buffer with row addresses and control bits.
    #[inline]
    pub fn format(&mut self) {
        for plane in &mut self.planes {
            for (row_idx, row) in plane.iter_mut().enumerate() {
                let prev_addr = if row_idx == 0 {
                    NROWS as u8 - 1
                } else {
                    row_idx as u8 - 1
                };
                row.format(row_idx as u8, prev_addr);
            }
        }
    }

    /// Erase pixel colors while preserving row control data.
    #[inline]
    pub fn erase(&mut self) {
        const MASK: u16 = !0b0111_1110_0000_0000; // clear bits 9-14 (R1,G1,B1,R2,G2,B2)
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
        WordSize::Sixteen
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
        row.format(5, 4);

        let last_idx = map_index(7);
        assert_eq!(row.data[last_idx].latch(), true);
        assert_eq!(row.data[last_idx].addr(), 5);

        let first_idx = map_index(0);
        assert_eq!(row.data[first_idx].addr(), 4);
        assert_eq!(row.data[first_idx].latch(), false);
    }

    #[test]
    fn format_sets_expected_row_addresses_for_all_rows() {
        let mut fb = TestBuffer::new();
        fb.format();

        for plane_idx in 0..8 {
            for row_idx in 0..16 {
                let last_col = map_index(63);
                assert_eq!(
                    fb.planes[plane_idx][row_idx].data[last_col].addr(),
                    row_idx as u16
                );
                assert_eq!(fb.planes[plane_idx][row_idx].data[last_col].latch(), true);
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
        let oe_before = fb.planes[0][0].data[map_index(1)].output_enable();
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

        assert_eq!(
            fb.planes[0][0].data[map_index(1)].output_enable(),
            oe_before
        );
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
    fn plane_info_for_common_panel() {
        assert_eq!(TestBuffer::plane_count(), 8);
        assert_eq!(
            TestBuffer::plane_size_bytes(),
            16 * core::mem::size_of::<Row<64>>()
        );
    }
}
