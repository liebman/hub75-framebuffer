//! Plane-major bitplane framebuffer.
//!
//! Stores all rows for one bit-plane contiguously, then the next plane, etc.
//! Memory scales linearly with `PLANES`: the buffer contains `PLANES` copies
//! of the row data (one per bit-plane).
//!
//! Each row is `COLS` 16-bit entries plus an optional inter-row gap, so total
//! size is approximately `PLANES × NROWS × (COLS + INTER_ROW_BLANK) × 2`
//! bytes.
//!
//! Planes are stored **LSB-first**: plane 0 carries the least-significant
//! bit and is displayed once per frame; plane `PLANES-1` carries the MSB and
//! is displayed `2^(PLANES-1)` times per frame.
//!
//! # DMA Descriptor Pattern
//!
//! The planes are contiguous (`repr(C)`), so each BCM segment streams a
//! whole *suffix* of planes: the segment for plane `k` starts at plane `k`
//! and covers planes `k..PLANES`, repeated just enough times to bring plane
//! `k`'s total coverage to `2^k` displays per frame. This halves the number
//! of DMA transfers per frame — `2^(PLANES-1)` instead of `2^PLANES - 1` —
//! while the streamed bytes, and therefore the brightness, are unchanged.
//! The inter-row gap (embedded in each row) and the optional tail word
//! (embedded in each plane) travel inside the coalesced segments
//! automatically.
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

use crate::Color;
use crate::{BcmSegment, FrameBuffer};
use crate::{FrameBufferOperations, MutableFrameBuffer};

use super::{make_data_template, map_index, Entry, INTER_ROW_BLANK, OE_BLANK};

#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
/// A single BCM row payload for 16-bit plain output.
///
/// Row addressing, latch, OE, and pixel colour data are all encoded into the
/// 16-bit `Entry` words -- no separate address bytes are needed.
pub struct Row<const COLS: usize> {
    pub(crate) data: [Entry; COLS],
    pub(crate) gap: [Entry; INTER_ROW_BLANK],
}

impl<const COLS: usize> Row<COLS> {
    /// Creates a zero-initialized row.
    ///
    /// Call [`Self::format`] before first use to populate row control metadata.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            data: [Entry::new(); COLS],
            gap: [Entry::new(); INTER_ROW_BLANK],
        }
    }

    /// Formats this row for the provided multiplexed row address.
    ///
    /// `prev_addr` is the *previous* scan row's address: it is driven onto
    /// the address lines while this row's pixels are shifted in, so the
    /// panel keeps displaying the previously latched row during the shift.
    ///
    /// Sets up blanking delay, output-enable, latch, and address bits in the
    /// pixel stream template.
    #[inline]
    pub const fn format(&mut self, prev_addr: u8) {
        let template = make_data_template::<COLS>(prev_addr);
        let mut i = 0;
        while i < COLS {
            self.data[i] = template[i];
            i += 1;
        }

        let gap_val = (prev_addr as u16) | OE_BLANK;
        let mut i = 0;
        // INTER_ROW_BLANK is 0 unless an inter-row-blank-* feature is enabled.
        #[allow(clippy::absurd_extreme_comparisons)]
        while i < INTER_ROW_BLANK {
            self.gap[i].0 = gap_val;
            i += 1;
        }
    }
}

impl<const COLS: usize> Default for Row<COLS> {
    fn default() -> Self {
        Self::new()
    }
}

/// A single bit-plane's DMA payload: all rows for the plane, optionally
/// followed by a tail word (see the `tail-closes-latch` feature).
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct PlaneData<const NROWS: usize, const COLS: usize> {
    pub(crate) rows: [Row<COLS>; NROWS],
    #[cfg(all(feature = "tail-closes-latch", feature = "esp32-ordering"))]
    pub(crate) padding: Entry,
    #[cfg(feature = "tail-closes-latch")]
    pub(crate) tail: Entry,
}

impl<const NROWS: usize, const COLS: usize> PlaneData<NROWS, COLS> {
    const fn new() -> Self {
        Self {
            rows: [Row::new(); NROWS],
            #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
            padding: Entry::new(),
            #[cfg(feature = "tail-closes-latch")]
            tail: Entry::new(),
        }
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

/// Plane-major BCM framebuffer (per-plane storage).
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DmaFrameBuffer<const NROWS: usize, const COLS: usize, const PLANES: usize> {
    pub(crate) planes: [PlaneData<NROWS, COLS>; PLANES],
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize>
    DmaFrameBuffer<NROWS, COLS, PLANES>
{
    /// Creates a new frame buffer, pre-formatted and ready for use.
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
            planes: [PlaneData::new(); PLANES],
        };
        instance.format();
        instance
    }

    /// Returns the number of BCM chunks (one per bit-plane).
    #[must_use]
    pub const fn bcm_chunk_count() -> usize {
        PLANES
    }

    /// Returns the byte size of one BCM chunk (a single bit-plane including tail word).
    #[must_use]
    pub const fn bcm_chunk_bytes() -> usize {
        core::mem::size_of::<PlaneData<NROWS, COLS>>()
    }

    /// Computes the number of DMA descriptors required for this framebuffer.
    ///
    /// `max_chunk` is the platform-specific maximum DMA transfer size in bytes.
    ///
    /// One descriptor chain per BCM segment repetition; a segment spanning
    /// several planes may itself need multiple descriptors when it exceeds
    /// `max_chunk`.
    #[must_use]
    pub const fn dma_descriptor_count(max_chunk: usize) -> usize {
        let plane_bytes = core::mem::size_of::<PlaneData<NROWS, COLS>>();
        let mut descs = 0usize;
        let mut plane = 0usize;
        while plane < PLANES {
            let (covered, reps) = plane_seg_shape(plane, PLANES);
            descs += (plane_bytes * covered).div_ceil(max_chunk) * reps;
            plane += 1;
        }
        descs
    }

    /// Formats the frame buffer with row addresses and control bits.
    #[inline]
    pub const fn format(&mut self) {
        let mut p = 0;
        while p < PLANES {
            let mut row_idx = 0;
            while row_idx < NROWS {
                let prev_addr = if row_idx == 0 {
                    NROWS as u8 - 1
                } else {
                    row_idx as u8 - 1
                };
                self.planes[p].rows[row_idx].format(prev_addr);
                row_idx += 1;
            }
            #[cfg(feature = "tail-closes-latch")]
            {
                self.planes[p].tail.0 = 0x1f | OE_BLANK;
            }
            #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
            {
                self.planes[p].padding.0 = 0x1f | OE_BLANK;
            }
            p += 1;
        }
    }

    /// Erase pixel colors while preserving row control data.
    #[inline]
    pub fn erase(&mut self) {
        const MASK: u16 = !0b0111_1110_0000_0000; // clear bits 9-14 (R1,G1,B1,R2,G2,B2)
        for plane in &mut self.planes {
            for row in &mut plane.rows {
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
            let bit = (8 - PLANES + plane_idx) as u32;
            let bits = ((u8::from(((blue >> bit) & 1) != 0)) << 2)
                | ((u8::from(((green >> bit) & 1) != 0)) << 1)
                | u8::from(((red >> bit) & 1) != 0);
            let col_idx = map_index(x);
            let entry = &mut self.planes[plane_idx].rows[row_idx].data[col_idx];
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
    type Word = u16;

    fn bcm_segment_count(&self) -> usize {
        PLANES
    }

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        assert!(
            index < PLANES,
            "segment index {index} out of range for {PLANES} planes"
        );
        // Plane k: stream the contiguous plane suffix k..PLANES (embedded
        // per-row gaps and per-plane tail words included) just enough times
        // to bring plane k's total coverage to 2^k.
        let ptr = (&raw const self.planes[index]).cast::<u8>();
        let (covered, reps) = plane_seg_shape(index, PLANES);
        let len = core::mem::size_of::<PlaneData<NROWS, COLS>>() * covered;
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

    use super::super::{LEAD_BLANK_DELAY, TRAIL_BLANK_DELAY};
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
        let mut row = Row::<TEST_COLS>::new();
        row.format(4);

        let last_idx = map_index(TEST_COLS - 1);
        assert_eq!(row.data[last_idx].latch(), true);
        assert_eq!(row.data[last_idx].addr(), 4);

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
                let prev_addr = if row_idx == 0 { 15 } else { row_idx - 1 };
                let last_col = map_index(TEST_COLS - 1);
                assert_eq!(
                    fb.planes[plane_idx].rows[row_idx].data[last_col].addr(),
                    prev_addr as u16
                );
                assert_eq!(
                    fb.planes[plane_idx].rows[row_idx].data[last_col].latch(),
                    true
                );
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
            let entry = fb.planes[plane_idx].rows[3].data[map_index(2)];
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
            let entry = fb.planes[plane_idx].rows[4].data[map_index(4)];
            assert_eq!(entry.red2(), ((color.r() >> bit) & 1) != 0);
            assert_eq!(entry.grn2(), ((color.g() >> bit) & 1) != 0);
            assert_eq!(entry.blu2(), ((color.b() >> bit) & 1) != 0);
        }
    }

    #[test]
    fn erase_clears_only_color_bits() {
        let mut fb = TestBuffer::new();
        let oe_before = fb.planes[0].rows[0].data[map_index(1)].output_enable();
        fb.set_pixel(Point::new(0, 0), Color::WHITE);
        fb.erase();

        for plane in &fb.planes {
            for row in &plane.rows {
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
            fb.planes[0].rows[0].data[map_index(1)].output_enable(),
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
            let entry = fb.planes[plane_idx].rows[1].data[map_index(1)];
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
        assert!(fb.planes[0].rows[5].data[mapped_col_10].red1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels enabled, this should be ignored
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should still be red (black write was skipped)
        assert!(fb.planes[0].rows[5].data[mapped_col_10].red1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].blu1());
    }

    #[test]
    #[cfg(not(feature = "skip-black-pixels"))]
    fn test_skip_black_pixels_disabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.planes[0].rows[5].data[mapped_col_10].red1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels disabled, this should overwrite
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should now be black (all bits false)
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].red1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].grn1());
        assert!(!fb.planes[0].rows[5].data[mapped_col_10].blu1());
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
            core::mem::size_of::<PlaneData<16, TEST_COLS>>()
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
    fn row_format_sets_expected_blank_and_latch_positions() {
        let mut row = Row::<TEST_COLS>::new();
        row.format(4);

        let oe_active = !cfg!(feature = "invert-oe");

        let idx_active = map_index(TRAIL_BLANK_DELAY);
        assert_eq!(row.data[idx_active].output_enable(), oe_active);
        assert_eq!(row.data[idx_active].addr(), 4);

        let idx_blank = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
        assert_eq!(row.data[idx_blank].output_enable(), !oe_active);

        let idx_last = map_index(TEST_COLS - 1);
        assert!(row.data[idx_last].latch());
        assert_eq!(row.data[idx_last].addr(), 4);
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

        assert!(fb.planes[0].rows[5].data[map_index(3)].grn1());

        FrameBufferOperations::erase(&mut fb);
        for plane in &fb.planes {
            for row in &plane.rows {
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
    fn entry_debug_shows_hex_value() {
        let mut entry = Entry::new();
        entry.set_red1(true);
        let s = format!("{entry:?}");
        assert!(s.contains("Entry"));
        assert!(s.contains("0x"));
    }

    #[test]
    fn entry_new_oe_matches_feature() {
        let entry = Entry::new();
        if cfg!(feature = "invert-oe") {
            assert!(entry.output_enable());
        } else {
            assert!(!entry.output_enable());
        }
    }

    #[test]
    fn make_data_template_oe_polarity() {
        let mut row = Row::<TEST_COLS>::new();
        row.format(4);

        let active_idx = map_index(TRAIL_BLANK_DELAY);
        let blank_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
        let latch_idx = map_index(TEST_COLS - 1);

        if cfg!(feature = "invert-oe") {
            assert!(!row.data[active_idx].output_enable());
            assert!(row.data[blank_idx].output_enable());
            assert!(row.data[latch_idx].output_enable());
        } else {
            assert!(row.data[active_idx].output_enable());
            assert!(!row.data[blank_idx].output_enable());
            assert!(!row.data[latch_idx].output_enable());
        }
    }

    static STATIC_FB: TestBuffer = TestBuffer::new();

    #[test]
    fn test_static_construction_is_formatted() {
        let runtime_fb = TestBuffer::new();

        for (pi, plane) in STATIC_FB.planes.iter().enumerate() {
            for (ri, row) in plane.rows.iter().enumerate() {
                assert_eq!(
                    row.data, runtime_fb.planes[pi].rows[ri].data,
                    "static vs runtime mismatch at plane {pi}, row {ri}"
                );
                assert_eq!(
                    row.gap, runtime_fb.planes[pi].rows[ri].gap,
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
            for (ri, row) in plane.rows.iter().enumerate() {
                assert_eq!(
                    row.data, STATIC_FB.planes[pi].rows[ri].data,
                    "re-formatted vs static mismatch at plane {pi}, row {ri}"
                );
            }
        }
    }

    #[test]
    fn bcm_segment_count_equals_planes() {
        let fb = TestBuffer::new();
        assert_eq!(fb.bcm_segment_count(), 8);
    }

    #[test]
    fn bcm_segments_per_group_is_one() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        assert_eq!(fb.bcm_segments_per_group(), 1);
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
        let plane_len = core::mem::size_of::<PlaneData<16, TEST_COLS>>();
        for i in 0..8 {
            let seg = fb.bcm_segment(i);
            let expected_ptr = (&raw const fb.planes[i]).cast::<u8>();
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
        let plane_bytes = core::mem::size_of::<PlaneData<16, TEST_COLS>>();

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
    fn dma_descriptor_count_matches_expected() {
        let plane_bytes = core::mem::size_of::<PlaneData<16, TEST_COLS>>();
        let max_chunk = 4092;
        let mut expected = 0;
        for plane in 0..8usize {
            let covered = 8 - plane;
            let reps = if plane == 0 { 1 } else { 1 << (plane - 1) };
            expected += (plane_bytes * covered).div_ceil(max_chunk) * reps;
        }
        assert_eq!(TestBuffer::dma_descriptor_count(max_chunk), expected);
    }

    #[test]
    fn dma_descriptor_count_chunks_oversized_segments() {
        // With a tiny max_chunk every coalesced segment is split into
        // per-plane-sized descriptors.
        let plane_bytes = core::mem::size_of::<PlaneData<16, TEST_COLS>>();
        let mut expected = 0;
        for plane in 0..8usize {
            let covered = 8 - plane;
            let reps = if plane == 0 { 1 } else { 1 << (plane - 1) };
            expected += covered * reps;
        }
        assert_eq!(TestBuffer::dma_descriptor_count(plane_bytes), expected);
    }
}
