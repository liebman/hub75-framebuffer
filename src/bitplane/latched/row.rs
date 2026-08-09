//! Row-major bitplane framebuffer for an 8-bit latched HUB75 interface.
//!
//! Unlike the plane-major [`super::frame::DmaFrameBuffer`] which stores all
//! rows for one bit-plane together, this framebuffer groups all bit-planes
//! for a single row contiguously. This layout is optimized for row-by-row
//! BCM rendering where the DMA driver replays each plane's pixel data
//! multiple times (for brightness weighting) before moving to the next row.
//!
//! # Memory Layout
//!
//! ```text
//! DmaFrameBuffer<NROWS, COLS, PLANES>
//! ├── rows: [RowData<COLS, PLANES>; NROWS]
//! │   └── RowData
//! │       ├── planes: [PlaneRow<COLS>; PLANES]   -- LSB-first
//! │       │   └── PlaneRow
//! │       │       ├── data: [Entry; COLS]
//! │       │       └── address: [Address; 4]
//! │       └── gap: [Entry; INTER_ROW_BLANK]      -- inter-row blanking gap
//! ```
//!
//! The gap is stored after the planes but is *streamed* between plane 0
//! and plane 1: each BCM segment carries its own pointer, so streaming
//! order is independent of storage order.
//!
//! Planes are stored **LSB-first**: plane 0 carries the least-significant
//! bit and is displayed once per row; plane `PLANES-1` carries the MSB and
//! is displayed `2^(PLANES-1)` times per row. This ordering means the stale
//! display during the first plane after a row change has minimal visual
//! weight (a single rep at LSB weight), eliminating the need for a separate
//! primer segment.
//!
//! # Addressing and Blanking
//!
//! Every plane's pixel data is followed by its four address bytes, which are
//! always blanked and pulse LAT — so OE is guaranteed to be blank during
//! every latch without any per-plane blanking. The row address changes
//! exactly once per scan row, inside plane 0's address bytes (i.e. after
//! plane 0's pixel data has been shifted out): the external latch holds the
//! previous row's address throughout plane 0's shift, so every plane simply
//! carries the *current* row address. Unlike the plain row-major layout
//! there is no `prev_addr` handling.
//!
//! Blanking is applied only around that single address change, mirroring the
//! plain row-major layout:
//!
//! - **Plane 0** (LSB, first): `LEAD_BLANK_DELAY` trailing blanked entries —
//!   its address bytes change the row address.
//! - **Inter-row gap** (with an `inter-row-blank-*` feature): blanked dead
//!   entries streamed immediately after plane 0's address bytes, giving slow
//!   row drivers extra blanked time right at the latch and address change.
//! - **Plane 1** (second): `TRAIL_BLANK_DELAY` leading blanked entries — it
//!   immediately follows the address change (and the gap).
//! - **Plane PLANES-1** (MSB, last): runs full-width like the middle planes;
//!   its address bytes are followed by the next row's plane 0, and no
//!   address change occurs there.
//! - **PLANES == 1**: the single plane both precedes and follows the address
//!   change, so it gets both the lead and the trail blank. The gap is
//!   streamed after the single plane's address bytes.
//! - All other planes run a full-width OE window.
//!
//! `LEAD_BLANK_DELAY` and `TRAIL_BLANK_DELAY` default to 0 for latched
//! framebuffers (the four blanked address bytes already cover the latch);
//! enable the `lead-blank-*` / `trail-blank-*` features for panels that need
//! more settling time around the latch and address change.
//!
//! The inter-row gap appears only once per scan row — streamed between
//! plane 0 (whose address bytes latch the data and change the row address)
//! and plane 1 — rather than once per plane as in the plane-major layout.
//!
//! # DMA Descriptor Pattern
//!
//! The plane rows of a row are contiguous (`repr(C)`), so each BCM segment
//! streams a whole *suffix* of planes (address bytes included): the segment
//! for plane `k` starts at plane `k` and covers planes `k..PLANES`, repeated
//! just enough times to bring plane `k`'s total coverage to `2^k` displays
//! per row. This halves the number of DMA transfers per row —
//! `2^(PLANES-1)` instead of `2^PLANES - 1` — while the streamed bytes, and
//! therefore the brightness, are unchanged.
//!
//! In [`FrameBuffer`] terms, one row's segments form
//! one *sequence* — which is also one *group* (a single DMA transfer per
//! row); one refresh is that sequence repeated `NROWS` times.
//!
//! For each row the driver builds descriptors like (no inter-row gap):
//!
//! ```text
//! planes 0..PLANES      × 1 rep        → rows[r].planes[0..]
//! planes 1..PLANES      × 1 rep        → rows[r].planes[1..]
//! planes 2..PLANES      × 2 reps       → rows[r].planes[2..]
//! …
//! plane  PLANES-1 (MSB) × 2^(PLANES-2) → rows[r].planes[PLANES-1]
//! ```
//!
//! With an `inter-row-blank-*` feature enabled, plane 0 stands alone so the
//! gap can be streamed right after its address bytes (where the row address
//! changes), and plane 1's segment gets a second rep to preserve its weight:
//!
//! ```text
//! plane 0 (LSB)         × 1 rep        → rows[r].planes[0]
//! inter-row gap         × 1            → rows[r].gap
//! planes 1..PLANES      × 2 reps       → rows[r].planes[1..]
//! planes 2..PLANES      × 2 reps       → rows[r].planes[2..]
//! …
//! plane  PLANES-1 (MSB) × 2^(PLANES-2) → rows[r].planes[PLANES-1]
//! ```

use core::convert::Infallible;

use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, Point, Size};

use super::{
    map_index, Address, Entry, ADDR_TABLE, INTER_ROW_BLANK, LEAD_BLANK_DELAY, OE_ACTIVE, OE_BLANK,
    TRAIL_BLANK_DELAY,
};
use crate::Color;
use crate::{map_row_index, slot_addresses};
use crate::{BcmSegment, FrameBuffer, BCM_SEGMENT_SHAPES_CAPACITY};
use crate::{FrameBufferOperations, MutableFrameBuffer};

/// One bit-plane's payload for a single scan row: `COLS` pixel bytes
/// followed by the four address bytes that latch the shifted data and select
/// the row.
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct PlaneRow<const COLS: usize> {
    pub(crate) data: [Entry; COLS],
    pub(crate) address: [Address; 4],
}

impl<const COLS: usize> PlaneRow<COLS> {
    const fn new() -> Self {
        Self {
            data: [Entry::new(); COLS],
            address: [Address::new(); 4],
        }
    }
}

/// Byte size of the inter-row blanking gap segment (streamed between
/// plane 0 and plane 1).
const GAP_BYTES: usize = INTER_ROW_BLANK * core::mem::size_of::<Entry>();

/// Whether this configuration has a non-empty inter-row gap segment.
#[allow(clippy::absurd_extreme_comparisons)]
const HAS_GAP: bool = GAP_BYTES > 0;

/// Shape of the coalesced BCM segment for plane `plane_idx`: returns
/// `(covered_planes, reps)` — how many planes the segment spans (starting at
/// `plane_idx`) and how many times that suffix is streamed. See the
/// module-level "DMA Descriptor Pattern" section for the full scheme.
///
/// With an inter-row gap, plane 0 stands alone (the gap segment is streamed
/// between plane 0 and plane 1), so plane 1's segment needs 2 reps to
/// preserve its weight.
const fn plane_seg_shape(plane_idx: usize, planes: usize) -> (usize, usize) {
    let covered = if HAS_GAP && plane_idx == 0 {
        1
    } else {
        planes - plane_idx
    };
    let reps = if plane_idx == 0 {
        1
    } else if HAS_GAP && plane_idx == 1 {
        2
    } else {
        1 << (plane_idx - 1)
    };
    (covered, reps)
}

/// `(len, reps)` shapes of one BCM sequence (a single row), in scan
/// order: one segment per plane (each streaming the contiguous plane suffix
/// `plane..PLANES`), plus the inter-row gap segment when enabled. Entries
/// past the sequence are `(0, 0)` padding.
#[allow(clippy::absurd_extreme_comparisons)]
const fn segment_shapes<const COLS: usize, const PLANES: usize>(
) -> [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] {
    let gap = HAS_GAP as usize;
    let seq_len = PLANES + gap;
    assert!(seq_len <= BCM_SEGMENT_SHAPES_CAPACITY);
    let plane_bytes = core::mem::size_of::<PlaneRow<COLS>>();
    let mut shapes = [(0usize, 0usize); BCM_SEGMENT_SHAPES_CAPACITY];
    let mut within = 0usize;
    while within < seq_len {
        let shape = if within == 0 {
            let (covered, reps) = plane_seg_shape(0, PLANES);
            (plane_bytes * covered, reps)
        } else if HAS_GAP && within == 1 {
            (GAP_BYTES, 1)
        } else {
            let (covered, reps) = plane_seg_shape(within - gap, PLANES);
            (plane_bytes * covered, reps)
        };
        shapes[within] = shape;
        within += 1;
    }
    shapes
}

/// Builds a per-plane pixel template for the row-major layout.
///
/// Blanking is applied only around the single address change, which happens
/// inside plane 0's address bytes (mirroring the plain row-major layout).
/// The address bytes themselves are always blanked, so blank-during-latch
/// needs no help from the pixel stream.
///
/// - `trail_blank`: blank the `TRAIL_BLANK_DELAY` leading entries — used by
///   the plane that immediately follows the address change (plane 1, or
///   plane 0 when `PLANES == 1`).
/// - `lead_blank`: blank the `LEAD_BLANK_DELAY` trailing entries — used by
///   plane 0, whose address bytes change the row address.
#[inline]
#[allow(clippy::absurd_extreme_comparisons)]
const fn make_row_plane_template<const COLS: usize>(
    trail_blank: bool,
    lead_blank: bool,
) -> [Entry; COLS] {
    let trailing = if lead_blank { LEAD_BLANK_DELAY } else { 0 };
    let lead_start = COLS.saturating_sub(trailing);

    let mut data = [Entry::new(); COLS];
    let mut i = 0;
    while i < COLS {
        let blanked = (trail_blank && i < TRAIL_BLANK_DELAY) || i >= lead_start;
        data[map_index(i)].0 = if blanked { OE_BLANK } else { OE_ACTIVE };
        i += 1;
    }
    data
}

/// One scan row's complete data: all bit-planes (LSB-first), each with its
/// own address bytes, followed by the inter-row blanking gap.
///
/// The gap is stored after the planes but is *streamed* between plane 0
/// (whose address bytes change the row address) and plane 1.
///
/// Every plane carries the *current* row address; the external latch holds
/// the previous row's address throughout plane 0's shift, so the previously
/// latched plane displays on the correct row during every shift window.
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct RowData<const COLS: usize, const PLANES: usize> {
    pub(crate) planes: [PlaneRow<COLS>; PLANES],
    pub(crate) gap: [Entry; INTER_ROW_BLANK],
}

impl<const COLS: usize, const PLANES: usize> RowData<COLS, PLANES> {
    const fn new() -> Self {
        Self {
            planes: [PlaneRow::new(); PLANES],
            gap: [Entry::new(); INTER_ROW_BLANK],
        }
    }

    /// Format control signals for this row.
    ///
    /// `addr` is the current row address, carried by every plane's address
    /// bytes. The address change happens inside plane 0's address bytes, so
    /// plane 0 gets the lead blank and plane 1 gets the trail blank (plane 0
    /// gets both when `PLANES == 1`). All other planes run full-width.
    const fn format(&mut self, addr: u8) {
        let src_addr = ADDR_TABLE[addr as usize];

        let mut p = 0;
        while p < PLANES {
            let trail_blank = p == 1 || (p == 0 && PLANES == 1);
            let lead_blank = p == 0;
            let template = make_row_plane_template::<COLS>(trail_blank, lead_blank);
            let mut c = 0;
            while c < COLS {
                self.planes[p].data[c] = template[c];
                c += 1;
            }
            self.planes[p].address[0] = src_addr[0];
            self.planes[p].address[1] = src_addr[1];
            self.planes[p].address[2] = src_addr[2];
            self.planes[p].address[3] = src_addr[3];
            p += 1;
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

/// Row-major BCM framebuffer for row-by-row Binary Code Modulation.
///
/// See the [module-level documentation](self) for layout details and DMA
/// descriptor patterns.
///
/// # Type Parameters
/// - `NROWS`: number of multiplexed row pairs (panel height / 2)
/// - `COLS`: number of columns (panel width)
/// - `PLANES`: number of bit-planes (typically 8 for full 8-bit colour)
#[derive(Copy, Clone)]
#[repr(C, align(4))]
pub struct DmaFrameBuffer<const NROWS: usize, const COLS: usize, const PLANES: usize> {
    pub(crate) rows: [RowData<COLS, PLANES>; NROWS],
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize>
    DmaFrameBuffer<NROWS, COLS, PLANES>
{
    /// Creates a new framebuffer, pre-formatted and ready for use.
    ///
    /// # Panics
    /// Panics if `NROWS` is not within `1..=32` (5-bit row address) or
    /// `PLANES` is not within `1..=8` (8-bit color depth). With the
    /// `esp32-ordering` feature, also panics if `COLS` is not divisible
    /// by 4 (the ESP32's byte-order rearrangement requires column counts
    /// that are multiples of 4). In const contexts (e.g. `static`
    /// framebuffers) this is a compile-time error.
    #[must_use]
    pub const fn new() -> Self {
        assert!(NROWS >= 1 && NROWS <= 32, "NROWS must be within 1..=32");
        assert!(PLANES >= 1 && PLANES <= 8, "PLANES must be within 1..=8");
        #[cfg(feature = "esp32-ordering")]
        assert!(
            COLS % 4 == 0,
            "esp32-ordering feature requires COLS to be divisible by 4"
        );
        let mut instance = Self {
            rows: [RowData::new(); NROWS],
        };
        instance.format();
        instance
    }

    /// Number of bit-planes.
    #[must_use]
    pub const fn bcm_chunk_count() -> usize {
        PLANES
    }

    /// Byte size of one plane's payload for a single row (pixel data plus
    /// its four address bytes).
    #[must_use]
    pub const fn bcm_chunk_bytes() -> usize {
        core::mem::size_of::<PlaneRow<COLS>>()
    }

    /// Number of multiplexed scan rows.
    #[must_use]
    pub const fn bcm_row_count() -> usize {
        NROWS
    }

    /// Byte size of the per-row inter-row blanking gap.
    #[must_use]
    pub const fn bcm_row_bytes() -> usize {
        GAP_BYTES
    }

    /// Formats the framebuffer with row addresses and control bits.
    ///
    /// Every plane carries the current row address; blanking is applied
    /// around the plane0→plane1 address change. See the module-level
    /// documentation for the exact distribution.
    #[inline]
    pub const fn format(&mut self) {
        let mut slot = 0;
        while slot < NROWS {
            let (_, addr) = slot_addresses::<NROWS>(slot);
            self.rows[slot].format(addr);
            slot += 1;
        }
    }

    /// Erase pixel colours while preserving row control data.
    #[inline]
    pub fn erase(&mut self) {
        const MASK: u8 = !0b0011_1111;
        for row in &mut self.rows {
            for plane in &mut row.planes {
                for entry in &mut plane.data {
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
            let entry = &mut self.rows[row_idx].planes[plane_idx].data[col_idx];
            if is_top {
                entry.set_color0_bits(bits);
            } else {
                entry.set_color1_bits(bits);
            }
        }
    }

    /// Returns a pointer and byte length for a plane's payload (pixel data
    /// plus its four address bytes) within a single row.
    ///
    /// # Panics
    /// Panics if `row_idx >= NROWS` or `plane_idx >= PLANES`.
    #[must_use]
    pub fn pixel_row_ptr_len(&self, row_idx: usize, plane_idx: usize) -> (*const u8, usize) {
        assert!(
            row_idx < NROWS,
            "row_idx {row_idx} out of range for {NROWS} rows"
        );
        assert!(
            plane_idx < PLANES,
            "plane_idx {plane_idx} out of range for {PLANES} planes"
        );
        let ptr = (&raw const self.rows[row_idx].planes[plane_idx]).cast::<u8>();
        let len = core::mem::size_of::<PlaneRow<COLS>>();
        (ptr, len)
    }

    /// Returns a pointer and byte length for a row's inter-row blanking gap.
    ///
    /// The gap is streamed between plane 0 (whose address bytes latch the
    /// data and change the row address) and plane 1. Returns length 0 when
    /// no `inter-row-blank-*` feature is enabled.
    ///
    /// # Panics
    /// Panics if `row_idx >= NROWS`.
    #[must_use]
    pub fn gap_ptr_len(&self, row_idx: usize) -> (*const u8, usize) {
        assert!(
            row_idx < NROWS,
            "row_idx {row_idx} out of range for {NROWS} rows"
        );
        let ptr = (&raw const self.rows[row_idx].gap).cast::<u8>();
        (ptr, GAP_BYTES)
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
            .field("size", &core::mem::size_of_val(self))
            .field("row_count", &NROWS)
            .field("planes", &PLANES)
            .field("cols", &COLS)
            .finish()
    }
}

#[cfg(feature = "defmt")]
impl<const NROWS: usize, const COLS: usize, const PLANES: usize> defmt::Format
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "DmaFrameBuffer<{}, {}, {}>", NROWS, COLS, PLANES);
        defmt::write!(f, " size: {}", core::mem::size_of_val(self));
    }
}

impl<const NROWS: usize, const COLS: usize, const PLANES: usize> FrameBuffer
    for DmaFrameBuffer<NROWS, COLS, PLANES>
{
    type Word = u8;

    const BCM_SEGMENT_SHAPES: [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] =
        segment_shapes::<COLS, PLANES>();

    const BCM_SEQUENCE_LEN: usize = PLANES + HAS_GAP as usize;

    const BCM_SEQUENCE_COUNT: usize = NROWS;

    const BCM_SEGMENTS_PER_GROUP: usize = PLANES + HAS_GAP as usize;

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        let gap = usize::from(HAS_GAP);
        let segments_per_row = PLANES + gap;
        assert!(
            index < NROWS * segments_per_row,
            "segment index {index} out of range"
        );
        let row_idx = index / segments_per_row;
        let within_row = index % segments_per_row;
        if within_row == 0 {
            // Plane 0 (LSB): 1 rep. Its address bytes change the row
            // address. Without an inter-row gap this segment streams the
            // whole contiguous plane block (all planes, one pass); with a
            // gap it covers plane 0 only and the gap segment follows.
            let (ptr, plane_len) = self.pixel_row_ptr_len(row_idx, 0);
            let (covered, reps) = plane_seg_shape(0, PLANES);
            BcmSegment {
                ptr,
                len: plane_len * covered,
                reps,
            }
        } else if HAS_GAP && within_row == 1 {
            // Inter-row gap: blanked dead clocks right after the latch and
            // address change in plane 0's address bytes.
            let (ptr, len) = self.gap_ptr_len(row_idx);
            BcmSegment { ptr, len, reps: 1 }
        } else {
            // Plane k: stream the contiguous plane suffix k..PLANES just
            // enough times to bring plane k's total coverage to 2^k.
            let plane_idx = within_row - gap;
            let (ptr, plane_len) = self.pixel_row_ptr_len(row_idx, plane_idx);
            let (covered, reps) = plane_seg_shape(plane_idx, PLANES);
            BcmSegment {
                ptr,
                len: plane_len * covered,
                reps,
            }
        }
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
            if pixel.0.x < 0 || pixel.0.y < 0 {
                continue;
            }
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
    const TEST_PLANES: usize = 8;
    type TestBuffer = DmaFrameBuffer<16, TEST_COLS, TEST_PLANES>;

    #[test]
    fn format_sets_current_row_address_for_all_planes() {
        let fb = TestBuffer::new();

        for slot in 0..16 {
            let (_, addr) = slot_addresses::<16>(slot);
            for plane_idx in 0..TEST_PLANES {
                let plane = &fb.rows[slot].planes[plane_idx];
                assert_eq!(
                    plane.address[map_index(0)].addr(),
                    addr,
                    "slot {slot} plane {plane_idx} addr byte 0 should be the current row"
                );
                assert_eq!(
                    plane.address[map_index(1)].addr(),
                    addr,
                    "slot {slot} plane {plane_idx} addr byte 1 should be the current row"
                );
                assert_eq!(
                    plane.address[map_index(2)].addr(),
                    addr,
                    "slot {slot} plane {plane_idx} addr byte 2 should be the current row"
                );
                assert_eq!(
                    plane.address[map_index(3)].0,
                    OE_BLANK,
                    "slot {slot} plane {plane_idx} addr byte 3 should be blanked with addr 0"
                );
            }
        }
    }

    #[test]
    fn address_bytes_have_two_latch_pulses() {
        let fb = TestBuffer::new();

        for row in &fb.rows {
            for plane in &row.planes {
                let latch_count = plane.address.iter().filter(|a| a.latch()).count();
                assert_eq!(latch_count, 2, "expected exactly two latch pulses");
                assert!(plane.address[map_index(0)].latch());
                assert!(plane.address[map_index(1)].latch());
            }
        }
    }

    #[test]
    fn planes_match_row_plane_template() {
        let fb = TestBuffer::new();

        for row_idx in 0..16 {
            for p in 0..TEST_PLANES {
                let expected = make_row_plane_template::<TEST_COLS>(p == 1, p == 0);
                assert_eq!(
                    fb.rows[row_idx].planes[p].data, expected,
                    "row {row_idx} plane {p} doesn't match template"
                );
            }
        }
    }

    #[test]
    fn first_plane_has_lead_blank_only() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        for row in &fb.rows {
            let plane = &row.planes[0];

            // No trail blank: plane 0 follows the previous row's last plane
            // and its (blanked) address bytes; no address change happens at
            // its start.
            let first_idx = map_index(0);
            assert_eq!(
                plane.data[first_idx].output_enable(),
                oe_active,
                "plane 0: first pixel should have OE active (no trail blank)"
            );

            // Lead blank: plane 0's address bytes change the row address.
            for i in (TEST_COLS - LEAD_BLANK_DELAY)..TEST_COLS {
                assert_eq!(
                    plane.data[map_index(i)].output_enable(),
                    !oe_active,
                    "plane 0: lead blank entry {i} should have OE blank"
                );
            }
            let active_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
            assert_eq!(
                plane.data[active_idx].output_enable(),
                oe_active,
                "plane 0: entry before the lead blank should have OE active"
            );
        }
    }

    #[test]
    fn second_plane_has_trail_blank_only() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        for row in &fb.rows {
            let plane = &row.planes[1];

            // Trail blank: plane 1 immediately follows the address change.
            for i in 0..TRAIL_BLANK_DELAY {
                assert_eq!(
                    plane.data[map_index(i)].output_enable(),
                    !oe_active,
                    "plane 1: trail blank entry {i} should have OE blank"
                );
            }
            let active_idx = map_index(TRAIL_BLANK_DELAY);
            assert_eq!(
                plane.data[active_idx].output_enable(),
                oe_active,
                "plane 1: entry after the trail blank should have OE active"
            );

            // No lead blank on plane 1: its address bytes do not change the
            // row address.
            let last_idx = map_index(TEST_COLS - 1);
            assert_eq!(
                plane.data[last_idx].output_enable(),
                oe_active,
                "plane 1: last pixel should have OE active (no lead blank)"
            );
        }
    }

    #[test]
    fn middle_planes_have_full_width_window() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        // Middle planes are 2..PLANES-1 (plane 0 has the lead blank, plane 1
        // the trail blank).
        for row in &fb.rows {
            for plane in row.planes.iter().take(TEST_PLANES - 1).skip(2) {
                let first_idx = map_index(0);
                assert_eq!(
                    plane.data[first_idx].output_enable(),
                    oe_active,
                    "middle plane: first pixel should have OE active"
                );
                let last_idx = map_index(TEST_COLS - 1);
                assert_eq!(
                    plane.data[last_idx].output_enable(),
                    oe_active,
                    "middle plane: last pixel should have OE active"
                );
            }
        }
    }

    #[test]
    fn last_plane_has_full_width_window() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");
        let last = TEST_PLANES - 1;

        for row in &fb.rows {
            let plane = &row.planes[last];

            // No lead blank on the last plane: its address bytes do not
            // change the row address, and panels that need more setup time
            // use the lead-blank-* features instead.
            let last_idx = map_index(TEST_COLS - 1);
            assert_eq!(
                plane.data[last_idx].output_enable(),
                oe_active,
                "last plane: last pixel should have OE active (no lead blank)"
            );

            // No trail blank on the last plane.
            let first_idx = map_index(0);
            assert_eq!(
                plane.data[first_idx].output_enable(),
                oe_active,
                "last plane: first pixel should have OE active (no trail blank)"
            );
        }
    }

    #[test]
    fn oe_window_sizes_match_template() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        for row in &fb.rows {
            for (p, plane) in row.planes.iter().enumerate() {
                let trail = if p == 1 { TRAIL_BLANK_DELAY } else { 0 };
                let lead = if p == 0 { LEAD_BLANK_DELAY } else { 0 };
                let expected_active = TEST_COLS.saturating_sub(trail + lead);
                let active = plane
                    .data
                    .iter()
                    .filter(|e| e.output_enable() == oe_active)
                    .count();
                assert_eq!(active, expected_active, "plane {p} active window size");
            }
        }
    }

    #[test]
    fn single_plane_has_trail_and_lead_blank() {
        type OnePlane = DmaFrameBuffer<16, TEST_COLS, 1>;
        let fb = OnePlane::new();

        for slot in 0..16 {
            let (_, addr) = slot_addresses::<16>(slot);
            let plane = &fb.rows[slot].planes[0];
            assert_eq!(
                plane.address[map_index(0)].addr(),
                addr,
                "single plane slot {slot}: should use the current row address"
            );

            // The single plane both precedes and follows the address change,
            // so it gets both the trail blank and the lead blank.
            let expected = make_row_plane_template::<TEST_COLS>(true, true);
            assert_eq!(
                plane.data, expected,
                "single plane slot {slot}: template mismatch"
            );
        }
    }

    #[test]
    fn set_pixel_maps_top_half_bits_per_plane() {
        let mut fb = TestBuffer::new();
        let color = Color::new(0b1010_0101, 0b0101_1010, 0b1111_0000);
        fb.set_pixel(Point::new(2, 3), color);

        for plane_idx in 0..TEST_PLANES {
            let bit = 8 - TEST_PLANES + plane_idx;
            let entry = fb.rows[map_row_index::<16>(3)].planes[plane_idx].data[map_index(2)];
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

        for plane_idx in 0..TEST_PLANES {
            let bit = 8 - TEST_PLANES + plane_idx;
            let entry = fb.rows[map_row_index::<16>(4)].planes[plane_idx].data[map_index(4)];
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
        assert_eq!(fb.rows[0].planes[0].address[map_index(0)].addr(), 15);
        assert_eq!(fb.rows[15].planes[0].address[map_index(0)].addr(), 0);

        // Logical row 0 maps to the last memory slot.
        fb.set_pixel(Point::new(2, 0), Color::RED);
        let col2 = map_index(2);
        assert!(fb.rows[15].planes[TEST_PLANES - 1].data[col2].red1());
        assert!(!fb.rows[0].planes[TEST_PLANES - 1].data[col2].red1());
    }

    #[test]
    fn erase_clears_only_color_bits() {
        let mut fb = TestBuffer::new();
        let oe_before =
            fb.rows[map_row_index::<16>(0)].planes[0].data[map_index(1)].output_enable();
        let addr_before = fb.rows[map_row_index::<16>(3)].planes[2].address;
        fb.set_pixel(Point::new(0, 0), Color::WHITE);
        fb.erase();

        for row in &fb.rows {
            for plane in &row.planes {
                for entry in &plane.data {
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
            fb.rows[map_row_index::<16>(0)].planes[0].data[map_index(1)].output_enable(),
            oe_before
        );
        assert_eq!(
            fb.rows[map_row_index::<16>(3)].planes[2].address,
            addr_before
        );
    }

    #[test]
    fn draw_target_iter_sets_pixels() {
        let mut fb = TestBuffer::new();
        let pixels = [Pixel(Point::new(1, 1), Color::RED)];
        let result = fb.draw_iter(pixels);
        assert!(result.is_ok());

        for plane_idx in 0..TEST_PLANES {
            let bit = 8 - TEST_PLANES + plane_idx;
            let entry = fb.rows[map_row_index::<16>(1)].planes[plane_idx].data[map_index(1)];
            assert_eq!(entry.red1(), ((Color::RED.r() >> bit) & 1) != 0);
            assert!(!entry.grn1());
            assert!(!entry.blu1());
        }
    }

    #[test]
    fn set_pixel_ignores_out_of_bounds_and_negative() {
        let mut fb = TestBuffer::new();
        let before = fb.rows;
        fb.set_pixel(Point::new(-1, 0), Color::WHITE);
        fb.set_pixel(Point::new(0, -1), Color::WHITE);
        fb.set_pixel(Point::new(TEST_COLS as i32, 0), Color::WHITE);
        fb.set_pixel(Point::new(0, 32), Color::WHITE);
        assert_eq!(fb.rows, before);
    }

    #[test]
    #[cfg(feature = "skip-black-pixels")]
    fn test_skip_black_pixels_enabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels enabled, this should be ignored
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should still be red (black write was skipped)
        assert!(fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].blu1());
    }

    #[test]
    #[cfg(not(feature = "skip-black-pixels"))]
    fn test_skip_black_pixels_disabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels disabled, this should overwrite
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should now be black (all bits false)
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].planes[0].data[mapped_col_10].blu1());
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
    fn pixel_row_ptr_len_returns_correct_pointers() {
        let fb = TestBuffer::new();
        let (ptr, len) = fb.pixel_row_ptr_len(0, 0);
        assert_eq!(len, core::mem::size_of::<PlaneRow<TEST_COLS>>());
        assert_eq!(ptr, (&raw const fb.rows[0].planes[0]).cast::<u8>());
    }

    #[test]
    fn gap_ptr_len_returns_correct_pointers() {
        let fb = TestBuffer::new();
        let (ptr, len) = fb.gap_ptr_len(0);
        assert_eq!(len, GAP_BYTES);
        assert_eq!(ptr, (&raw const fb.rows[0].gap).cast::<u8>());
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn pixel_row_ptr_len_panics_for_invalid_row() {
        let fb = TestBuffer::new();
        let _ = fb.pixel_row_ptr_len(16, 0);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn pixel_row_ptr_len_panics_for_invalid_plane() {
        let fb = TestBuffer::new();
        let _ = fb.pixel_row_ptr_len(0, TEST_PLANES);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn gap_ptr_len_panics_for_invalid_row() {
        let fb = TestBuffer::new();
        let _ = fb.gap_ptr_len(16);
    }

    #[test]
    fn origin_dimensions_match_panel_geometry() {
        let fb = TestBuffer::new();
        assert_eq!(fb.size(), Size::new(TEST_COLS as u32, 32));
    }

    #[test]
    fn default_matches_new() {
        let fb_default = TestBuffer::default();
        let fb_new = TestBuffer::new();
        assert_eq!(fb_default.rows, fb_new.rows);
    }

    #[test]
    fn debug_impl_includes_shape_information() {
        let fb = TestBuffer::new();
        let s = format!("{fb:?}");
        assert!(s.contains("DmaFrameBuffer"));
        assert!(s.contains("row_count"));
        assert!(s.contains("planes"));
    }

    static STATIC_FB: TestBuffer = TestBuffer::new();

    #[test]
    fn static_construction_is_formatted() {
        let runtime_fb = TestBuffer::new();

        for (ri, row) in STATIC_FB.rows.iter().enumerate() {
            assert_eq!(
                row.planes, runtime_fb.rows[ri].planes,
                "static vs runtime plane mismatch at row {ri}"
            );
            assert_eq!(
                row.gap, runtime_fb.rows[ri].gap,
                "static vs runtime gap mismatch at row {ri}"
            );
        }
    }

    #[test]
    fn format_reinitializes_at_runtime() {
        let mut fb = TestBuffer::new();
        fb.erase();
        fb.format();

        for (ri, row) in fb.rows.iter().enumerate() {
            assert_eq!(
                row.planes, STATIC_FB.rows[ri].planes,
                "re-formatted vs static plane mismatch at row {ri}"
            );
            assert_eq!(
                row.gap, STATIC_FB.rows[ri].gap,
                "re-formatted vs static gap mismatch at row {ri}"
            );
        }
    }

    #[test]
    fn framebuffer_operations_trait_delegates_correctly() {
        let mut fb = TestBuffer::new();
        FrameBufferOperations::set_pixel(&mut fb, Point::new(3, 5), Color::GREEN);

        // Plane 7 (MSB) carries bit 7; GREEN = 0xFF so bit 7 is set
        assert!(fb.rows[map_row_index::<16>(5)].planes[TEST_PLANES - 1].data[map_index(3)].grn1());

        FrameBufferOperations::erase(&mut fb);
        for row in &fb.rows {
            for plane in &row.planes {
                for entry in &plane.data {
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
    fn row_data_size_matches_expected_layout() {
        let expected_planes = core::mem::size_of::<PlaneRow<TEST_COLS>>() * TEST_PLANES;
        assert_eq!(
            core::mem::size_of::<RowData<TEST_COLS, TEST_PLANES>>(),
            expected_planes + GAP_BYTES
        );
    }

    #[test]
    fn bcm_segment_count_equals_rows_times_planes_plus_gap() {
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        assert_eq!(fb.bcm_segment_count(), 16 * (TEST_PLANES + gap));
    }

    #[test]
    fn bcm_segments_per_group_equals_planes_plus_gap() {
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        assert_eq!(fb.bcm_segments_per_group(), TEST_PLANES + gap);
        assert_eq!(
            fb.bcm_segment_count() % fb.bcm_segments_per_group(),
            0,
            "segment count must be divisible by segments_per_group"
        );
    }

    #[test]
    fn bcm_segments_interleave_pixel_and_gap() {
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap;

        for row in 0..16usize {
            // Plane 0 (LSB) comes first; without a gap it covers all planes.
            let seg = fb.bcm_segment(row * segments_per_row);
            let (ptr, plane_len) = fb.pixel_row_ptr_len(row, 0);
            let covered0 = if HAS_GAP { 1 } else { TEST_PLANES };
            assert_eq!(seg.ptr, ptr, "wrong ptr at row {row} plane 0");
            assert_eq!(
                seg.len,
                plane_len * covered0,
                "wrong len at row {row} plane 0"
            );
            assert_eq!(seg.reps, 1, "wrong reps at row {row} plane 0");

            // The inter-row gap (if any) is streamed between plane 0 and 1.
            if HAS_GAP {
                let seg = fb.bcm_segment(row * segments_per_row + 1);
                let (ptr, len) = fb.gap_ptr_len(row);
                assert_eq!(seg.ptr, ptr, "wrong gap ptr at row {row}");
                assert_eq!(seg.len, len, "wrong gap len at row {row}");
                assert_eq!(seg.reps, 1, "gap reps must be 1 at row {row}");
            }

            // Planes 1.. follow the gap; each streams the remaining suffix.
            for plane in 1..TEST_PLANES {
                let idx = row * segments_per_row + plane + gap;
                let seg = fb.bcm_segment(idx);
                let (ptr, plane_len) = fb.pixel_row_ptr_len(row, plane);
                let expected_reps = if HAS_GAP && plane == 1 {
                    2
                } else {
                    1 << (plane - 1)
                };
                assert_eq!(seg.ptr, ptr, "wrong ptr at row {row} plane {plane}");
                assert_eq!(
                    seg.len,
                    plane_len * (TEST_PLANES - plane),
                    "wrong len at row {row} plane {plane}"
                );
                assert_eq!(
                    seg.reps, expected_reps,
                    "wrong reps at row {row} plane {plane}"
                );
            }
        }
    }

    #[test]
    fn bcm_segment_total_reps_per_row() {
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap;

        for row in 0..16usize {
            // Suffix coalescing halves the per-row transfer count to
            // 2^(PLANES-1); with an inter-row gap, plane 0 stands alone and
            // plane 1's extra rep adds one more.
            let mut pixel_reps = 0;
            for plane in 0..TEST_PLANES {
                // Plane 0 is at offset 0, planes 1.. at offset 1 + gap.
                let offset = if plane == 0 { 0 } else { plane + gap };
                pixel_reps += fb.bcm_segment(row * segments_per_row + offset).reps;
            }
            let expected = if HAS_GAP {
                (1 << (TEST_PLANES - 1)) + 1
            } else {
                1 << (TEST_PLANES - 1)
            };
            assert_eq!(pixel_reps, expected, "total pixel reps wrong for row {row}");
        }
    }

    #[test]
    fn bcm_segment_plane_coverage_matches_bcm_weights() {
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap;
        let plane_bytes = core::mem::size_of::<PlaneRow<TEST_COLS>>();

        for row in 0..16usize {
            for plane in 0..TEST_PLANES {
                // Sum the reps of every pixel segment that spans this plane:
                // the segment for plane `first` covers `len / plane_bytes`
                // consecutive planes starting at `first`.
                let mut coverage = 0;
                for first in 0..=plane {
                    // Plane 0 is at offset 0, planes 1.. at offset 1 + gap.
                    let offset = if first == 0 { 0 } else { first + gap };
                    let seg = fb.bcm_segment(row * segments_per_row + offset);
                    if first + seg.len / plane_bytes > plane {
                        coverage += seg.reps;
                    }
                }
                assert_eq!(
                    coverage,
                    1 << plane,
                    "plane {plane} coverage wrong for row {row}"
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn bcm_segment_panics_for_invalid_index() {
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap;
        let _ = fb.bcm_segment(16 * segments_per_row);
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

    #[test]
    fn bcm_segment_data_contains_written_pixels() {
        const TRINITY_COLS: usize = if LEAD_BLANK_DELAY + TRAIL_BLANK_DELAY + 2 > 64 {
            128
        } else {
            64
        };
        type TrinityBuf = DmaFrameBuffer<32, TRINITY_COLS, 5>;
        let mut fb = TrinityBuf::new();

        fb.set_pixel(Point::new(10, 0), Color::WHITE);

        let col_idx = map_index(10);
        let gap = usize::from(HAS_GAP);
        // Physical slot holding logical row 0 (reversed by reverse-row-order).
        let row0_base = map_row_index::<32>(0) * (5 + gap);

        for plane_idx in 0..5usize {
            // Row 0: plane 0 is the slot's first segment, planes 1+ follow the
            // gap segment.
            let seg = fb.bcm_segment(if plane_idx == 0 {
                row0_base
            } else {
                row0_base + plane_idx + gap
            });

            let data = unsafe { core::slice::from_raw_parts(seg.ptr, TRINITY_COLS) };
            let raw = data[col_idx];
            let entry = Entry(raw);

            let bit = 8 - 5 + plane_idx; // LSB-first
            let expect_r = (0xFF >> bit) & 1 != 0;
            let expect_g = (0xFF >> bit) & 1 != 0;
            let expect_b = (0xFF >> bit) & 1 != 0;
            assert_eq!(
                entry.red1(),
                expect_r,
                "plane {plane_idx} (bit {bit}) R1 mismatch: raw={raw:#04x}"
            );
            assert_eq!(
                entry.grn1(),
                expect_g,
                "plane {plane_idx} (bit {bit}) G1 mismatch: raw={raw:#04x}"
            );
            assert_eq!(
                entry.blu1(),
                expect_b,
                "plane {plane_idx} (bit {bit}) B1 mismatch: raw={raw:#04x}"
            );
        }
    }

    #[test]
    fn bcm_segment_data_contains_gradient_pixels() {
        const TRINITY_COLS: usize = if LEAD_BLANK_DELAY + TRAIL_BLANK_DELAY + 2 > 64 {
            128
        } else {
            64
        };
        type TrinityBuf = DmaFrameBuffer<32, TRINITY_COLS, 5>;
        let mut fb = TrinityBuf::new();

        let step: u8 = (256 / TRINITY_COLS) as u8;
        for x in 0..TRINITY_COLS {
            let brightness = (x as u8).wrapping_mul(step);
            fb.set_pixel(Point::new(x as i32, 0), Color::new(brightness, 0, 0));
        }

        let mut any_nonzero = false;
        let gap = usize::from(HAS_GAP);
        // Physical slot holding logical row 0 (reversed by reverse-row-order).
        let row0_base = map_row_index::<32>(0) * (5 + gap);
        for plane_idx in 0..5usize {
            // Row 0: plane 0 is the slot's first segment, planes 1+ follow the
            // gap segment.
            let seg = fb.bcm_segment(if plane_idx == 0 {
                row0_base
            } else {
                row0_base + plane_idx + gap
            });
            let data = unsafe { core::slice::from_raw_parts(seg.ptr, TRINITY_COLS) };

            for x in 0..TRINITY_COLS {
                let col_idx = map_index(x);
                let entry = Entry(data[col_idx]);
                let brightness = (x as u8).wrapping_mul(step);
                let bit = 8 - 5 + plane_idx; // LSB-first
                let expect_r = (brightness >> bit) & 1 != 0;
                assert_eq!(
                    entry.red1(),
                    expect_r,
                    "x={x} plane={plane_idx} bit={bit} brightness={brightness} \
                     raw={:#04x}",
                    data[col_idx]
                );
                if expect_r {
                    any_nonzero = true;
                }
            }
        }
        assert!(
            any_nonzero,
            "gradient should produce at least some non-zero color bits"
        );
    }
}
