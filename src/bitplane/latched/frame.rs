//! Plane-major bitplane framebuffer for an 8-bit latched HUB75 interface.
//!
//! Stores all rows for one bit-plane contiguously, then the next plane, etc.
//! Planes are stored **LSB-first**: plane 0 carries the LSB (bit 0) of each
//! colour channel and is displayed once per frame, plane `PLANES-1` carries
//! the MSB (bit 7) and is displayed `2^(PLANES-1)` times per frame.
//!
//! Each row is `COLS` data bytes plus 4 address bytes (and an optional
//! inter-row gap), so total size is
//! `PLANES × NROWS × (COLS + 4 + INTER_ROW_BLANK)` bytes.
//!
//! # DMA Descriptor Pattern
//!
//! The planes are contiguous (`repr(C)`), so each BCM segment streams a
//! whole *suffix* of planes: the segment for plane `k` starts at plane `k`
//! and covers planes `k..PLANES`, repeated just enough times to bring plane
//! `k`'s total coverage to `2^k` displays per frame. This halves the number
//! of DMA transfers per frame — `2^(PLANES-1)` instead of `2^PLANES - 1` —
//! while the streamed bytes, and therefore the brightness, are unchanged.
//! The four address bytes and the inter-row gap (both embedded in each row)
//! travel inside the coalesced segments automatically.
//!
//! ```text
//! planes 0..PLANES      × 1 rep        → planes[0..]
//! planes 1..PLANES      × 1 rep        → planes[1..]
//! planes 2..PLANES      × 2 reps       → planes[2..]
//! …
//! plane  PLANES-1 (MSB) × 2^(PLANES-2) → planes[PLANES-1]
//! ```

use core::convert::Infallible;

use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, Point, Size};

use super::{make_data_template, map_index, Address, Entry, ADDR_TABLE, INTER_ROW_BLANK, OE_BLANK};
use crate::Color;
use crate::{map_row_index, slot_addresses};
use crate::{BcmSegment, FrameBuffer, BCM_SEGMENT_SHAPES_CAPACITY};
use crate::{FrameBufferOperations, MutableFrameBuffer};

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

/// Shape of the coalesced BCM segment for plane `plane_idx`: returns
/// `(covered_planes, reps)` — how many planes the segment spans (starting at
/// `plane_idx`) and how many times that suffix is streamed. See the
/// module-level "DMA Descriptor Pattern" section for the full scheme.
const fn plane_seg_shape(plane_idx: usize, planes: usize) -> (usize, usize) {
    let covered = planes - plane_idx;
    let reps = if plane_idx == 0 {
        1
    } else {
        1 << (plane_idx - 1)
    };
    (covered, reps)
}

/// `(len, reps)` shapes of the single BCM sequence (the whole frame), in
/// scan order: one segment per plane, each streaming the contiguous plane
/// suffix `plane..PLANES`. Entries past `PLANES` are `(0, 0)` padding.
const fn segment_shapes<const NROWS: usize, const COLS: usize, const PLANES: usize>(
) -> [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] {
    assert!(PLANES <= BCM_SEGMENT_SHAPES_CAPACITY);
    let plane_bytes = NROWS * core::mem::size_of::<Row<COLS>>();
    let mut shapes = [(0usize, 0usize); BCM_SEGMENT_SHAPES_CAPACITY];
    let mut plane = 0usize;
    while plane < PLANES {
        let (covered, reps) = plane_seg_shape(plane, PLANES);
        shapes[plane] = (plane_bytes * covered, reps);
        plane += 1;
    }
    shapes
}

/// The entire BCM Frame Buffer (Contiguous Memory)
#[derive(Copy, Clone)]
#[repr(C, align(4))]
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
            let mut slot = 0;
            while slot < NROWS {
                let (_, addr) = slot_addresses::<NROWS>(slot);
                self.planes[p][slot].format(addr);
                slot += 1;
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

        let row_idx = map_row_index::<NROWS>(if y < NROWS { y } else { y - NROWS });
        let is_top = y < NROWS;
        let red = color.r();
        let green = color.g();
        let blue = color.b();

        for plane_idx in 0..PLANES {
            let bit = (8 - PLANES + plane_idx) as u32;
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

    const BCM_SEGMENT_SHAPES: [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] =
        segment_shapes::<NROWS, COLS, PLANES>();

    const BCM_SEQUENCE_LEN: usize = PLANES;

    const BCM_SEQUENCE_COUNT: usize = 1;

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        assert!(
            index < PLANES,
            "segment index {index} out of range for {PLANES} planes"
        );
        // Plane k: stream the contiguous plane suffix k..PLANES (embedded
        // per-row address bytes and gaps included) just enough times to
        // bring plane k's total coverage to 2^k.
        let ptr = self.planes[index].as_ptr().cast::<u8>();
        let (covered, reps) = plane_seg_shape(index, PLANES);
        let len = NROWS * core::mem::size_of::<Row<COLS>>() * covered;
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

    use super::super::{make_addr_table, LEAD_BLANK_DELAY, TRAIL_BLANK_DELAY};
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
            for slot in 0..16 {
                let (_, addr) = slot_addresses::<16>(slot);
                let row = &fb.planes[plane_idx][slot];
                assert_eq!(row.address[map_index(0)].addr(), addr);
                assert_eq!(row.address[map_index(1)].addr(), addr);
                assert_eq!(row.address[map_index(2)].addr(), addr);
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
            let bit = plane_idx;
            let entry = fb.planes[plane_idx][map_row_index::<16>(3)].data[map_index(2)];
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
            let bit = plane_idx;
            let entry = fb.planes[plane_idx][map_row_index::<16>(4)].data[map_index(4)];
            assert_eq!(entry.red2(), ((color.r() >> bit) & 1) != 0);
            assert_eq!(entry.grn2(), ((color.g() >> bit) & 1) != 0);
            assert_eq!(entry.blu2(), ((color.b() >> bit) & 1) != 0);
        }
    }

    #[test]
    #[cfg(feature = "reverse-row-order")]
    fn reverse_row_order_stores_rows_back_to_front() {
        let mut fb = TestBuffer::new();

        // Slot 0 is streamed first and renders the last panel row; the final
        // slot renders panel row 0. The address bytes carry the slot's own
        // row address.
        assert_eq!(fb.planes[0][0].address[map_index(0)].addr(), 15);
        assert_eq!(fb.planes[0][15].address[map_index(0)].addr(), 0);

        // Logical row 0 maps to the last memory slot.
        fb.set_pixel(Point::new(2, 0), Color::RED);
        let col2 = map_index(2);
        assert!(fb.planes[7][15].data[col2].red1());
        assert!(!fb.planes[7][0].data[col2].red1());
    }

    #[test]
    fn erase_clears_only_color_bits() {
        let mut fb = TestBuffer::new();
        let oe_before = fb.planes[0][map_row_index::<16>(0)].data[0].output_enable();
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
            fb.planes[0][map_row_index::<16>(0)].data[0].output_enable(),
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
            let bit = plane_idx;
            let entry = fb.planes[plane_idx][map_row_index::<16>(1)].data[map_index(1)];
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
        assert!(fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].red1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels enabled, this should be ignored
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should still be red (black write was skipped)
        assert!(fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].red1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].blu1());
    }

    #[test]
    #[cfg(not(feature = "skip-black-pixels"))]
    fn test_skip_black_pixels_disabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].red1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels disabled, this should overwrite
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should now be black (all bits false)
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].red1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].grn1());
        assert!(!fb.planes[0][map_row_index::<16>(5)].data[mapped_col_10].blu1());
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

        assert!(fb.planes[0][map_row_index::<16>(5)].data[map_index(3)].grn1());

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
            let expected = if i == 0 { 1 } else { 1 << (i - 1) };
            assert_eq!(seg.reps, expected, "wrong reps for segment {i}");
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
            assert_eq!(seg.len, plane_len * (8 - i), "wrong len for segment {i}");
        }
    }

    #[test]
    fn bcm_segment_total_reps_is_halved_by_coalescing() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let total: usize = (0..fb.bcm_segment_count())
            .map(|i| fb.bcm_segment(i).reps)
            .sum();
        assert_eq!(total, 1 << (8 - 1));
    }

    #[test]
    fn bcm_segment_plane_coverage_matches_bcm_weights() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let plane_bytes = 16 * core::mem::size_of::<Row<TEST_COLS>>();

        for plane in 0..8usize {
            // Sum the reps of every segment that spans this plane: the
            // segment for plane `first` covers `len / plane_bytes`
            // consecutive planes starting at `first`.
            let mut coverage = 0;
            for first in 0..=plane {
                let seg = fb.bcm_segment(first);
                if first + seg.len / plane_bytes > plane {
                    coverage += seg.reps;
                }
            }
            assert_eq!(coverage, 1 << plane, "plane {plane} coverage wrong");
        }
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn bcm_segment_panics_for_invalid_index() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let _ = fb.bcm_segment(8);
    }

    #[test]
    fn bcm_segment_shapes_match_runtime_segments() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        assert_eq!(
            TestBuffer::BCM_SEGMENT_COUNT,
            TestBuffer::BCM_SEQUENCE_LEN * TestBuffer::BCM_SEQUENCE_COUNT
        );
        assert_eq!(fb.bcm_segment_count(), TestBuffer::BCM_SEGMENT_COUNT);
        assert_eq!(
            fb.bcm_segments_per_group(),
            TestBuffer::BCM_SEGMENTS_PER_GROUP
        );
        assert_eq!(
            TestBuffer::BCM_SEQUENCE_LEN % TestBuffer::BCM_SEGMENTS_PER_GROUP,
            0
        );
        for i in 0..TestBuffer::BCM_SEGMENT_COUNT {
            let (len, reps) = TestBuffer::BCM_SEGMENT_SHAPES[i % TestBuffer::BCM_SEQUENCE_LEN];
            let seg = fb.bcm_segment(i);
            assert_eq!((seg.len, seg.reps), (len, reps), "segment {i} shape");
            assert!(!seg.ptr.is_null(), "segment {i} has null pointer");
        }
        for &(len, reps) in &TestBuffer::BCM_SEGMENT_SHAPES[TestBuffer::BCM_SEQUENCE_LEN..] {
            assert_eq!((len, reps), (0, 0), "padding must be zero");
        }
    }
}
