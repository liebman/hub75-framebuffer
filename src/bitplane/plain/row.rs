//! Row-major bitplane framebuffer.
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
//! │       ├── pixels: [PixelPlane<COLS>; PLANES]   -- LSB-first
//! │       │   └── PixelPlane
//! │       │       └── data: [Entry; COLS]
//! │       ├── gap: [Entry; INTER_ROW_BLANK]        -- inter-row blanking gap
//! │       ├── [padding: Entry]                     -- if esp32-ordering + tail-closes-latch
//! │       └── [tail: Entry]                        -- if tail-closes-latch
//! ```
//!
//! The gap is stored after the planes but is *streamed* between plane 0
//! and plane 1: each BCM segment carries its own pointer, so streaming
//! order is independent of storage order.
//!
//! Planes are stored **LSB-first**: plane 0 carries the least-significant
//! bit and is displayed once per row; plane `PLANES-1` carries the MSB and
//! is displayed `2^(PLANES-1)` times per row. This ordering means the first
//! plane after an address change displays stale data for only 1 rep (minimal
//! visual weight), eliminating the need for a separate primer segment.
//!
//! Each plane's pixel row contains COLS entries with a latch at the very
//! last pixel; the latch entry is always blanked. In steady state the row
//! address changes at exactly one place — the plane0→plane1 boundary within
//! a scan row — so blanking (OE HIGH) is applied only around that boundary;
//! anywhere else would be dead time that only dims the panel:
//!
//! - **Plane 0** (LSB, 1 rep): uses `prev_addr`, lead blank at end
//!   (the address changes to the current row right after this plane and
//!   the inter-row gap).
//!   No trail blank is needed at its start: the previous row's planes 1+
//!   (and the tail trailer) all carry the same `prev_addr`, so no address
//!   change occurs there.
//! - **Inter-row gap** (with an `inter-row-blank-*` feature): dead entries
//!   holding `prev_addr` with OE blank, streamed between plane 0's latch
//!   and the address change, giving slow row drivers extra blanked time
//!   before the address lines move.
//! - **Plane 1** (first current-addr plane): trail blank at start
//!   (the address changes from plane 0's `prev_addr` at its first pixel).
//! - **Planes 2..PLANES-2** (middle): OE active on all entries except
//!   latch — no address change, so no blanking needed.
//! - **Plane PLANES-1** (MSB, most reps): no lead blank — the tail
//!   and next row's plane 0 all keep this row's address, so no address
//!   change follows.
//! - **PLANES == 1**: the single plane uses `prev_addr` with both trail
//!   and lead blank, because in that configuration the address does change
//!   at every row boundary. The gap is streamed after the single plane
//!   and the address changes at the next row's plane 0 first pixel.
//!
//! When `tail-closes-latch` is enabled, a single tail word (LATCH=0,
//! OE=BLANK) is appended at the end of each [`RowData`], after the last
//! plane. This prevents peripherals that continue clocking after DMA
//! completion from re-latching stale data.
//!
//! The inter-row gap appears only once per scan row — streamed between
//! plane 0 (shifted out before the address change) and plane 1 (shifted
//! out at the address change) — rather than once per plane as in the
//! plane-major layout.
//!
//! # DMA Descriptor Pattern
//!
//! The plane rows of a row are contiguous (`repr(C)`), so each BCM segment
//! streams a whole *suffix* of planes: the segment for plane `k` starts at
//! plane `k` and covers planes `k..PLANES`, repeated just enough times to
//! bring plane `k`'s total coverage to `2^k` displays per row. This halves
//! the number of DMA transfers per row — `2^(PLANES-1)` instead of
//! `2^PLANES - 1` — while the streamed bytes, and therefore the brightness,
//! are unchanged.
//!
//! In [`FrameBuffer`] terms, one row's segments form
//! one *sequence* — which is also one *group* (a single DMA transfer per
//! row); one refresh is that sequence repeated `NROWS` times.
//!
//! For each row the driver builds descriptors like (no inter-row gap):
//!
//! ```text
//! planes 0..PLANES      × 1 rep        → rows[r].pixels[0..]  (plane 0: prev_addr)
//! planes 1..PLANES      × 1 rep        → rows[r].pixels[1..]  (addr)
//! planes 2..PLANES      × 2 reps       → rows[r].pixels[2..]
//! …
//! plane  PLANES-1 (MSB) × 2^(PLANES-2) → rows[r].pixels[PLANES-1] (addr)
//! trailer               × 1            → rows[r].padding + tail (if enabled)
//! ```
//!
//! With an `inter-row-blank-*` feature enabled, plane 0 stands alone so the
//! gap can be streamed between plane 0's latch and plane 1's address change,
//! and plane 1's segment gets a second rep to preserve its weight:
//!
//! ```text
//! plane 0 (LSB)         × 1 rep        → rows[r].pixels[0]    (prev_addr)
//! inter-row gap         × 1            → rows[r].gap          (prev_addr, OE blank)
//! planes 1..PLANES      × 2 reps       → rows[r].pixels[1..]  (addr)
//! planes 2..PLANES      × 2 reps       → rows[r].pixels[2..]
//! …
//! plane  PLANES-1 (MSB) × 2^(PLANES-2) → rows[r].pixels[PLANES-1]
//! trailer               × 1            → rows[r].padding + tail (if enabled)
//! ```

use core::convert::Infallible;

use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, Point, Size};

use crate::Color;
use crate::{map_row_index, slot_addresses};
use crate::{BcmSegment, FrameBuffer, BCM_SEGMENT_SHAPES_CAPACITY};
use crate::{FrameBufferOperations, MutableFrameBuffer};

use super::{
    map_index, Entry, INTER_ROW_BLANK, LEAD_BLANK_DELAY, OE_ACTIVE, OE_BLANK, TRAIL_BLANK_DELAY,
};

/// One bit-plane's pixel data for a single row.
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct PixelPlane<const COLS: usize> {
    pub(crate) data: [Entry; COLS],
}

impl<const COLS: usize> PixelPlane<COLS> {
    const fn new() -> Self {
        Self {
            data: [Entry::new(); COLS],
        }
    }
}

/// Byte size of the inter-row blanking gap segment (streamed between
/// plane 0 and plane 1).
const GAP_BYTES: usize = INTER_ROW_BLANK * core::mem::size_of::<Entry>();

/// Byte size of the end-of-row trailer (optional padding + optional tail).
const TRAILER_BYTES: usize = {
    #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
    let padding = core::mem::size_of::<Entry>();
    #[cfg(not(all(feature = "esp32-ordering", feature = "tail-closes-latch")))]
    let padding = 0;

    #[cfg(feature = "tail-closes-latch")]
    let tail = core::mem::size_of::<Entry>();
    #[cfg(not(feature = "tail-closes-latch"))]
    let tail = 0;

    padding + tail
};

/// Whether this configuration has a non-empty inter-row gap segment.
#[allow(clippy::absurd_extreme_comparisons)]
const HAS_GAP: bool = GAP_BYTES > 0;

/// Whether this configuration has a non-empty end-of-row trailer segment.
#[allow(clippy::absurd_extreme_comparisons)]
const HAS_TRAILER: bool = TRAILER_BYTES > 0;

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
/// `plane..PLANES`), plus the inter-row gap and end-of-row trailer segments
/// when enabled. Entries past the sequence are `(0, 0)` padding.
#[allow(clippy::absurd_extreme_comparisons)]
const fn segment_shapes<const COLS: usize, const PLANES: usize>(
) -> [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] {
    let gap = HAS_GAP as usize;
    let trailer = HAS_TRAILER as usize;
    let seq_len = PLANES + gap + trailer;
    assert!(seq_len <= BCM_SEGMENT_SHAPES_CAPACITY);
    let plane_bytes = COLS * core::mem::size_of::<Entry>();
    let mut shapes = [(0usize, 0usize); BCM_SEGMENT_SHAPES_CAPACITY];
    let mut within = 0usize;
    while within < seq_len {
        let shape = if within == 0 {
            let (covered, reps) = plane_seg_shape(0, PLANES);
            (plane_bytes * covered, reps)
        } else if HAS_GAP && within == 1 {
            (GAP_BYTES, 1)
        } else if within < PLANES + gap {
            let (covered, reps) = plane_seg_shape(within - gap, PLANES);
            (plane_bytes * covered, reps)
        } else {
            (TRAILER_BYTES, 1)
        };
        shapes[within] = shape;
        within += 1;
    }
    shapes
}

/// Builds a per-plane pixel template for the row-major layout.
///
/// - `trail_blank`: if true, first `TRAIL_BLANK_DELAY` entries have OE blanked
///   (address just changed from the previous row).
/// - `lead_blank`: if true, last `LEAD_BLANK_DELAY + 1` entries have OE blanked
///   (address change to next row follows).
/// - The latch entry (last pixel) always has OE blanked — many HUB75 driver ICs
///   require OE to be inactive during latch for reliable data transfer.
#[allow(clippy::absurd_extreme_comparisons)]
#[inline]
const fn make_row_plane_template<const COLS: usize>(
    addr: u8,
    trail_blank: bool,
    lead_blank: bool,
) -> [Entry; COLS] {
    let mut data = [Entry::new(); COLS];
    let mut i = 0;

    while i < COLS {
        let is_last = i == COLS - 1;

        let blanked = is_last
            || (trail_blank && i < TRAIL_BLANK_DELAY)
            || (lead_blank && i >= COLS.saturating_sub(LEAD_BLANK_DELAY + 1));

        let oe = if blanked { OE_BLANK } else { OE_ACTIVE };
        let mut val = addr as u16 | oe;
        if is_last {
            val |= 0b0010_0000; // latch
        }

        data[map_index(i)] = Entry::from_raw(val);
        i += 1;
    }

    data
}

/// One scan row's complete data: pixel entries for all bit-planes (LSB-first),
/// the inter-row blanking gap, and an optional tail word.
///
/// The gap is stored after the planes but is *streamed* between plane 0 and
/// plane 1 (see the module-level DMA descriptor pattern).
///
/// Blanking is applied only around the single address change (the
/// plane0→plane1 boundary); see the module-level documentation:
/// - Plane 0 (LSB): `prev_addr`, lead blank (+ trail blank iff `PLANES == 1`)
/// - Gap: `prev_addr`, OE blank — defers the address change to plane 1's
///   first pixel
/// - Plane 1: current addr, trail blank (address just changed from plane 0)
/// - Middle and last planes: current addr, OE active everywhere except latch
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(C)]
pub struct RowData<const COLS: usize, const PLANES: usize> {
    pub(crate) pixels: [PixelPlane<COLS>; PLANES],
    pub(crate) gap: [Entry; INTER_ROW_BLANK],
    #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
    pub(crate) padding: Entry,
    #[cfg(feature = "tail-closes-latch")]
    pub(crate) tail: Entry,
}

impl<const COLS: usize, const PLANES: usize> RowData<COLS, PLANES> {
    const fn new() -> Self {
        Self {
            pixels: [PixelPlane::new(); PLANES],
            gap: [Entry::new(); INTER_ROW_BLANK],
            #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
            padding: Entry::new(),
            #[cfg(feature = "tail-closes-latch")]
            tail: Entry::new(),
        }
    }

    /// Format control signals for this row.
    ///
    /// - `prev_addr`: previous row address (used by plane 0 / LSB and the gap).
    /// - `addr`: current row address (used by planes 1+ and trailer).
    const fn format(&mut self, prev_addr: u8, addr: u8) {
        let mut p = 0;
        while p < PLANES {
            let is_first = p == 0;

            if is_first {
                // Plane 0 (LSB, 1 rep): uses prev_addr, lead blank.
                // With a single plane the address changes at every row
                // boundary (entries alternate prev_addr), so a trail blank
                // is needed as well to let the address settle while blanked.
                let template = make_row_plane_template::<COLS>(prev_addr, PLANES == 1, true);
                let mut c = 0;
                while c < COLS {
                    self.pixels[p].data[c] = template[c];
                    c += 1;
                }
            } else {
                // Planes 1+ use current addr.
                // Plane 1: trail blank (address just changed from prev_addr).
                let trail = p == 1;
                let template = make_row_plane_template::<COLS>(addr, trail, false);
                let mut c = 0;
                while c < COLS {
                    self.pixels[p].data[c] = template[c];
                    c += 1;
                }
            }

            p += 1;
        }

        // The gap is streamed between plane 0 (shifted out with `prev_addr`)
        // and plane 1 (whose first pixel changes the address): it keeps the
        // previous row address with OE blank, deferring the address change
        // until after the gap. With `PLANES == 1` the gap is streamed after
        // the single plane and the address changes at the next row's plane 0
        // first pixel instead.
        let gap_entry = Entry::from_raw(prev_addr as u16 | OE_BLANK);
        let mut i = 0;
        #[allow(clippy::absurd_extreme_comparisons)]
        while i < INTER_ROW_BLANK {
            self.gap[i] = gap_entry;
            i += 1;
        }

        #[cfg(feature = "tail-closes-latch")]
        {
            self.tail = Entry::from_raw(addr as u16 | OE_BLANK);
        }
        #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
        {
            self.padding = Entry::from_raw(addr as u16 | OE_BLANK);
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
    /// `PLANES` is not within `1..=8` (8-bit color depth). In const contexts
    /// (e.g. `static` framebuffers) this is a compile-time error.
    #[must_use]
    pub const fn new() -> Self {
        assert!(NROWS >= 1 && NROWS <= 32, "NROWS must be within 1..=32");
        assert!(PLANES >= 1 && PLANES <= 8, "PLANES must be within 1..=8");
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

    /// Byte size of one plane's pixel row.
    #[must_use]
    pub const fn bcm_chunk_bytes() -> usize {
        COLS * core::mem::size_of::<Entry>()
    }

    /// Number of multiplexed scan rows.
    #[must_use]
    pub const fn bcm_row_count() -> usize {
        NROWS
    }

    /// Byte size of the per-row overhead (inter-row gap + optional trailer).
    #[must_use]
    pub const fn bcm_row_bytes() -> usize {
        GAP_BYTES + TRAILER_BYTES
    }

    /// Formats the framebuffer with row addresses and control bits.
    ///
    /// Plane 0 (LSB, 1 rep) uses `prev_addr` so the brief display of
    /// stale data from the previous row has minimal visual weight.
    /// Planes 1+ use the current row address. Blanking is applied only
    /// around the plane0→plane1 address change (plus plane 0's trail
    /// blank when `PLANES == 1`); see the module-level documentation for
    /// the exact distribution.
    #[inline]
    pub const fn format(&mut self) {
        let mut slot = 0;
        while slot < NROWS {
            let (prev_addr, addr) = slot_addresses::<NROWS>(slot);
            self.rows[slot].format(prev_addr, addr);
            slot += 1;
        }
    }

    /// Erase pixel colours while preserving row control data.
    #[inline]
    pub fn erase(&mut self) {
        const MASK: u16 = !0b0111_1110_0000_0000; // clear bits 9-14
        for row in &mut self.rows {
            for plane in &mut row.pixels {
                for entry in &mut plane.data {
                    entry.mask_raw(MASK);
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
            let entry = &mut self.rows[row_idx].pixels[plane_idx].data[col_idx];
            if is_top {
                entry.set_color0_bits(bits);
            } else {
                entry.set_color1_bits(bits);
            }
        }
    }

    /// Returns a pointer and byte length for a plane's pixel row.
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
        let ptr = (&raw const self.rows[row_idx].pixels[plane_idx]).cast::<u8>();
        let len = COLS * core::mem::size_of::<Entry>();
        (ptr, len)
    }

    /// Returns a pointer and byte length for a row's inter-row blanking gap.
    ///
    /// The gap is streamed between plane 0 (LSB) and plane 1 and holds the
    /// previous row address with OE blank, deferring the address change
    /// until after the gap. Returns length 0 when no `inter-row-blank-*`
    /// feature is enabled.
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

    /// Returns a pointer and byte length for a row's end-of-row trailer (the
    /// optional `esp32-ordering` padding and `tail-closes-latch` word).
    ///
    /// Returns length 0 when the `tail-closes-latch` feature is disabled.
    ///
    /// # Panics
    /// Panics if `row_idx >= NROWS`.
    #[must_use]
    pub fn trailer_ptr_len(&self, row_idx: usize) -> (*const u8, usize) {
        assert!(
            row_idx < NROWS,
            "row_idx {row_idx} out of range for {NROWS} rows"
        );
        #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
        let ptr = (&raw const self.rows[row_idx].padding).cast::<u8>();
        #[cfg(all(feature = "tail-closes-latch", not(feature = "esp32-ordering")))]
        let ptr = (&raw const self.rows[row_idx].tail).cast::<u8>();
        #[cfg(not(feature = "tail-closes-latch"))]
        let ptr = (&raw const self.rows[row_idx].gap).cast::<u8>();
        (ptr, TRAILER_BYTES)
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
    type Word = u16;

    const BCM_SEGMENT_SHAPES: [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] =
        segment_shapes::<COLS, PLANES>();

    const BCM_SEQUENCE_LEN: usize = PLANES + HAS_GAP as usize + HAS_TRAILER as usize;

    const BCM_SEQUENCE_COUNT: usize = NROWS;

    const BCM_SEGMENTS_PER_GROUP: usize = PLANES + HAS_GAP as usize + HAS_TRAILER as usize;

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        let gap = usize::from(HAS_GAP);
        let trailer = usize::from(HAS_TRAILER);
        let segments_per_row = PLANES + gap + trailer;
        assert!(
            index < NROWS * segments_per_row,
            "segment index {index} out of range"
        );
        let row_idx = index / segments_per_row;
        let within_row = index % segments_per_row;
        if within_row == 0 {
            // Plane 0 (LSB, prev_addr): 1 rep. Without an inter-row gap this
            // segment streams the whole contiguous plane block (all planes,
            // one pass); with a gap it covers plane 0 only and the gap
            // segment follows.
            let (ptr, plane_len) = self.pixel_row_ptr_len(row_idx, 0);
            let (covered, reps) = plane_seg_shape(0, PLANES);
            BcmSegment {
                ptr,
                len: plane_len * covered,
                reps,
            }
        } else if HAS_GAP && within_row == 1 {
            // Inter-row gap: blanked dead clocks between plane 0's latch
            // and the address change at plane 1's first pixel.
            let (ptr, len) = self.gap_ptr_len(row_idx);
            BcmSegment { ptr, len, reps: 1 }
        } else if within_row < PLANES + gap {
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
        } else {
            let (ptr, len) = self.trailer_ptr_len(row_idx);
            BcmSegment { ptr, len, reps: 1 }
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
            self.set_pixel_internal(pixel.0.x as usize, pixel.0.y as usize, pixel.1);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::super::{LEAD_BLANK_DELAY, OE_ACTIVE, TRAIL_BLANK_DELAY};
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
    fn format_sets_addresses_per_plane() {
        let fb = TestBuffer::new();

        for slot in 0..16 {
            let (prev_addr, addr) = slot_addresses::<16>(slot);
            let prev_addr = u16::from(prev_addr);

            // Plane 0 (LSB): prev_addr
            for col in 0..TEST_COLS {
                let entry = fb.rows[slot].pixels[0].data[col];
                assert_eq!(
                    entry.addr(),
                    prev_addr,
                    "slot {slot} plane 0 col {col} should use prev_addr"
                );
            }

            // Planes 1+: current addr
            for plane_idx in 1..TEST_PLANES {
                for col in 0..TEST_COLS {
                    let entry = fb.rows[slot].pixels[plane_idx].data[col];
                    assert_eq!(
                        entry.addr(),
                        u16::from(addr),
                        "slot {slot} plane {plane_idx} col {col} should use current addr"
                    );
                }
            }
        }
    }

    #[test]
    fn planes_match_row_plane_template() {
        let fb = TestBuffer::new();

        for slot in 0..16 {
            let (prev_addr, addr) = slot_addresses::<16>(slot);

            // Plane 0 (LSB): prev_addr, lead blank only (no address change
            // at its start, so no trail blank)
            let expected = make_row_plane_template::<TEST_COLS>(prev_addr, false, true);
            assert_eq!(
                fb.rows[slot].pixels[0].data, expected,
                "slot {slot} plane 0 doesn't match template"
            );

            // Planes 1+: current addr, trail blank on plane 1 only, no lead blank
            for p in 1..TEST_PLANES {
                let expected = make_row_plane_template::<TEST_COLS>(addr, p == 1, false);
                assert_eq!(
                    fb.rows[slot].pixels[p].data, expected,
                    "slot {slot} plane {p} doesn't match template"
                );
            }
        }
    }

    #[test]
    fn pixel_rows_have_latch_at_last_entry() {
        let fb = TestBuffer::new();

        for row in &fb.rows {
            for plane in &row.pixels {
                let last_idx = map_index(TEST_COLS - 1);
                assert!(
                    plane.data[last_idx].latch(),
                    "last pixel must have latch set"
                );
            }
        }
    }

    #[test]
    fn first_plane_has_lead_blank_only() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        for row in &fb.rows {
            let plane = &row.pixels[0];

            // No trail blank: the previous row's planes and trailer all carry
            // the same prev_addr, so no address change occurs at plane 0's start.
            let first_idx = map_index(0);
            assert_eq!(
                plane.data[first_idx].output_enable(),
                oe_active,
                "plane 0: first pixel should have OE active (no trail blank)"
            );

            let active_idx = map_index(TRAIL_BLANK_DELAY);
            assert_eq!(
                plane.data[active_idx].output_enable(),
                oe_active,
                "plane 0: pixel after trail blank position should have OE active"
            );

            let lead_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
            assert_eq!(
                plane.data[lead_idx].output_enable(),
                !oe_active,
                "plane 0: lead blank should have OE blank"
            );

            let latch_idx = map_index(TEST_COLS - 1);
            assert_eq!(
                plane.data[latch_idx].output_enable(),
                !oe_active,
                "plane 0: latch should have OE blank"
            );
        }
    }

    #[test]
    fn last_plane_blanks_only_at_latch() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");
        let last = TEST_PLANES - 1;

        for row in &fb.rows {
            let plane = &row.pixels[last];

            let first_idx = map_index(0);
            assert_eq!(
                plane.data[first_idx].output_enable(),
                oe_active,
                "last plane: first pixel should have OE active (no trail blank)"
            );

            // No lead blank: the trailer and the next row's plane 0 keep this
            // row's address, so no address change follows the MSB plane.
            if LEAD_BLANK_DELAY > 0 {
                let lead_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
                assert_eq!(
                    plane.data[lead_idx].output_enable(),
                    oe_active,
                    "last plane: near-end pixel should have OE active (no lead blank)"
                );
            }

            let latch_idx = map_index(TEST_COLS - 1);
            assert_eq!(
                plane.data[latch_idx].output_enable(),
                !oe_active,
                "last plane: latch should have OE blank"
            );
        }
    }

    #[test]
    fn middle_planes_have_oe_active_except_latch() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        // Middle planes are 2..PLANES-2 (plane 1 has trail blank, last has lead blank)
        for row in &fb.rows {
            for plane_idx in 2..TEST_PLANES - 1 {
                for col in 0..TEST_COLS {
                    let entry = row.pixels[plane_idx].data[col];
                    if col == map_index(TEST_COLS - 1) {
                        assert_eq!(
                            entry.output_enable(),
                            !oe_active,
                            "middle plane {plane_idx}: latch should have OE blank"
                        );
                    } else {
                        assert_eq!(
                            entry.output_enable(),
                            oe_active,
                            "middle plane {plane_idx} col {col} should have OE active"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn plane_1_has_trail_blank_only() {
        let fb = TestBuffer::new();
        let oe_active = !cfg!(feature = "invert-oe");

        for row in &fb.rows {
            let plane = &row.pixels[1];

            if TRAIL_BLANK_DELAY > 0 {
                let trail_idx = map_index(0);
                assert_eq!(
                    plane.data[trail_idx].output_enable(),
                    !oe_active,
                    "plane 1: trail blank should have OE blank"
                );
            }

            let active_idx = map_index(TRAIL_BLANK_DELAY);
            assert_eq!(
                plane.data[active_idx].output_enable(),
                oe_active,
                "plane 1: pixel after trail blank should have OE active"
            );

            let near_end = map_index(TEST_COLS - 2);
            assert_eq!(
                plane.data[near_end].output_enable(),
                oe_active,
                "plane 1: near-end pixel should have OE active (no lead blank)"
            );

            let latch_idx = map_index(TEST_COLS - 1);
            assert_eq!(
                plane.data[latch_idx].output_enable(),
                !oe_active,
                "plane 1: latch should have OE blank"
            );
        }
    }

    #[test]
    fn single_plane_buffer_has_both_blanks_with_prev_addr() {
        type OnePlane = DmaFrameBuffer<16, TEST_COLS, 1>;
        let fb = OnePlane::new();
        let oe_active = !cfg!(feature = "invert-oe");

        for slot in 0..16 {
            let (prev_addr, _) = slot_addresses::<16>(slot);
            let prev_addr = u16::from(prev_addr);
            let plane = &fb.rows[slot].pixels[0];

            // Single plane uses prev_addr
            assert_eq!(
                plane.data[map_index(0)].addr(),
                prev_addr,
                "single plane slot {slot}: should use prev_addr"
            );

            if TRAIL_BLANK_DELAY > 0 {
                let trail_idx = map_index(0);
                assert_eq!(
                    plane.data[trail_idx].output_enable(),
                    !oe_active,
                    "single plane: trail blank should have OE blank"
                );
            }

            if LEAD_BLANK_DELAY > 0 {
                let lead_idx = map_index(TEST_COLS - LEAD_BLANK_DELAY - 1);
                assert_eq!(
                    plane.data[lead_idx].output_enable(),
                    !oe_active,
                    "single plane: lead blank should have OE blank"
                );
            }

            let active_idx = map_index(TRAIL_BLANK_DELAY);
            assert_eq!(
                plane.data[active_idx].output_enable(),
                oe_active,
                "single plane: active pixel should have OE active"
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
            let entry = fb.rows[map_row_index::<16>(3)].pixels[plane_idx].data[map_index(2)];
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
            let entry = fb.rows[map_row_index::<16>(4)].pixels[plane_idx].data[map_index(4)];
            assert_eq!(entry.red2(), ((color.r() >> bit) & 1) != 0);
            assert_eq!(entry.grn2(), ((color.g() >> bit) & 1) != 0);
            assert_eq!(entry.blu2(), ((color.b() >> bit) & 1) != 0);
        }
    }

    #[test]
    #[cfg(feature = "reverse-row-order")]
    fn reverse_row_order_stores_rows_back_to_front() {
        let mut fb = TestBuffer::new();

        // Slot 0 is streamed first and renders the last panel row: plane 0
        // carries prev_addr 0 (row 0 is displayed while it is shifted) and
        // planes 1+ carry the slot's own address 15.
        assert_eq!(fb.rows[0].pixels[0].data[map_index(0)].addr(), 0);
        assert_eq!(fb.rows[0].pixels[1].data[map_index(0)].addr(), 15);
        // The final slot renders panel row 0: prev_addr 1, own address 0.
        assert_eq!(fb.rows[15].pixels[0].data[map_index(0)].addr(), 1);
        assert_eq!(fb.rows[15].pixels[1].data[map_index(0)].addr(), 0);

        // Logical row 0 maps to the last memory slot.
        fb.set_pixel(Point::new(2, 0), Color::RED);
        let col2 = map_index(2);
        assert!(fb.rows[15].pixels[TEST_PLANES - 1].data[col2].red1());
        assert!(!fb.rows[0].pixels[TEST_PLANES - 1].data[col2].red1());
    }

    #[test]
    fn erase_clears_only_color_bits() {
        let mut fb = TestBuffer::new();
        let oe_before =
            fb.rows[map_row_index::<16>(0)].pixels[0].data[map_index(1)].output_enable();
        fb.set_pixel(Point::new(0, 0), Color::WHITE);
        fb.erase();

        for row in &fb.rows {
            for plane in &row.pixels {
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
            fb.rows[map_row_index::<16>(0)].pixels[0].data[map_index(1)].output_enable(),
            oe_before
        );
    }

    #[test]
    fn erase_preserves_latch_and_oe_pattern() {
        let fb_fresh = TestBuffer::new();
        let mut fb = TestBuffer::new();
        fb.set_pixel(Point::new(0, 0), Color::WHITE);
        fb.erase();

        for row_idx in 0..16 {
            for plane_idx in 0..TEST_PLANES {
                for col in 0..TEST_COLS {
                    let entry = fb.rows[row_idx].pixels[plane_idx].data[col];
                    let fresh = fb_fresh.rows[row_idx].pixels[plane_idx].data[col];
                    assert_eq!(
                        entry.output_enable(),
                        fresh.output_enable(),
                        "OE mismatch at row {row_idx} plane {plane_idx} col {col}"
                    );
                    assert_eq!(
                        entry.latch(),
                        fresh.latch(),
                        "latch mismatch at row {row_idx} plane {plane_idx} col {col}"
                    );
                    assert_eq!(
                        entry.addr(),
                        fresh.addr(),
                        "addr mismatch at row {row_idx} plane {plane_idx} col {col}"
                    );
                }
            }
        }
    }

    #[test]
    fn erase_preserves_gap() {
        let fb_fresh = TestBuffer::new();
        let mut fb = TestBuffer::new();
        fb.set_pixel(Point::new(0, 0), Color::WHITE);
        fb.erase();

        for row_idx in 0..16 {
            assert_eq!(
                fb.rows[row_idx].gap, fb_fresh.rows[row_idx].gap,
                "gap mismatch at row {row_idx}"
            );
        }
    }

    #[test]
    fn gap_entries_hold_prev_addr_and_oe_blank() {
        let fb = TestBuffer::new();
        for (slot, row) in fb.rows.iter().enumerate() {
            let (prev_addr, _) = slot_addresses::<16>(slot);
            let prev_addr = u16::from(prev_addr);
            for entry in &row.gap {
                assert_eq!(
                    entry.addr(),
                    prev_addr,
                    "gap entry should hold prev_addr at slot {slot}"
                );
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_BLANK,
                    "gap entry should have OE blank at slot {slot}"
                );
                assert!(!entry.latch(), "gap entry must not latch at slot {slot}");
            }
        }
    }

    #[test]
    fn bcm_const_fns_return_expected_values() {
        assert_eq!(TestBuffer::bcm_chunk_count(), TEST_PLANES);
        assert_eq!(
            TestBuffer::bcm_chunk_bytes(),
            TEST_COLS * core::mem::size_of::<Entry>()
        );
        assert_eq!(TestBuffer::bcm_row_count(), 16);
        assert_eq!(TestBuffer::bcm_row_bytes(), GAP_BYTES + TRAILER_BYTES);
    }

    #[test]
    fn draw_target_iter_sets_pixels() {
        let mut fb = TestBuffer::new();
        let pixels = [Pixel(Point::new(1, 1), Color::RED)];
        let result = fb.draw_iter(pixels);
        assert!(result.is_ok());

        for plane_idx in 0..TEST_PLANES {
            let bit = 8 - TEST_PLANES + plane_idx;
            let entry = fb.rows[map_row_index::<16>(1)].pixels[plane_idx].data[map_index(1)];
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
        assert!(fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels enabled, this should be ignored
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should still be red (black write was skipped)
        assert!(fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].blu1());
    }

    #[test]
    #[cfg(not(feature = "skip-black-pixels"))]
    fn test_skip_black_pixels_disabled() {
        let mut fb = TestBuffer::new();

        // Set a red pixel first
        fb.set_pixel_internal(10, 5, Color::RED);

        // Verify it's red in the first plane
        let mapped_col_10 = map_index(10);
        assert!(fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].blu1());

        // Now set it to black - with skip-black-pixels disabled, this should overwrite
        fb.set_pixel_internal(10, 5, Color::BLACK);

        // The pixel should now be black (all bits false)
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].red1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].grn1());
        assert!(!fb.rows[map_row_index::<16>(5)].pixels[0].data[mapped_col_10].blu1());
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
        assert_eq!(len, TEST_COLS * core::mem::size_of::<Entry>());
        assert_eq!(ptr, (&raw const fb.rows[0].pixels[0]).cast::<u8>());
    }

    #[test]
    fn gap_ptr_len_returns_correct_pointers() {
        let fb = TestBuffer::new();
        let (ptr, len) = fb.gap_ptr_len(0);
        assert_eq!(len, GAP_BYTES);
        assert_eq!(ptr, (&raw const fb.rows[0].gap).cast::<u8>());
    }

    #[test]
    fn trailer_ptr_len_returns_correct_pointers() {
        let fb = TestBuffer::new();
        let (ptr, len) = fb.trailer_ptr_len(0);
        assert_eq!(len, TRAILER_BYTES);
        #[cfg(all(feature = "esp32-ordering", feature = "tail-closes-latch"))]
        assert_eq!(ptr, (&raw const fb.rows[0].padding).cast::<u8>());
        #[cfg(all(feature = "tail-closes-latch", not(feature = "esp32-ordering")))]
        assert_eq!(ptr, (&raw const fb.rows[0].tail).cast::<u8>());
        #[cfg(not(feature = "tail-closes-latch"))]
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
        let _ = fb.pixel_row_ptr_len(0, 8);
    }

    #[test]
    #[should_panic(expected = "out of range")]
    fn trailer_ptr_len_panics_for_invalid_row() {
        let fb = TestBuffer::new();
        let _ = fb.trailer_ptr_len(16);
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
                row.pixels, runtime_fb.rows[ri].pixels,
                "static vs runtime pixel mismatch at row {ri}"
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
                row.pixels, STATIC_FB.rows[ri].pixels,
                "re-formatted vs static pixel mismatch at row {ri}"
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
        assert!(fb.rows[map_row_index::<16>(5)].pixels[TEST_PLANES - 1].data[map_index(3)].grn1());

        FrameBufferOperations::erase(&mut fb);
        for row in &fb.rows {
            for plane in &row.pixels {
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
        let expected_pixels = core::mem::size_of::<PixelPlane<TEST_COLS>>() * TEST_PLANES;
        let expected_overhead = GAP_BYTES + TRAILER_BYTES;
        assert_eq!(
            core::mem::size_of::<RowData<TEST_COLS, 8>>(),
            expected_pixels + expected_overhead
        );
    }

    #[test]
    fn bcm_segment_count_equals_rows_times_planes_plus_gap_and_trailer() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let trailer = if HAS_TRAILER { 1 } else { 0 };
        assert_eq!(fb.bcm_segment_count(), 16 * (TEST_PLANES + gap + trailer));
    }

    #[test]
    fn bcm_segments_per_group_equals_planes_plus_gap_and_trailer() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let trailer = if HAS_TRAILER { 1 } else { 0 };
        assert_eq!(fb.bcm_segments_per_group(), TEST_PLANES + gap + trailer);
        assert_eq!(
            fb.bcm_segment_count() % fb.bcm_segments_per_group(),
            0,
            "segment count must be divisible by segments_per_group"
        );
    }

    #[test]
    fn bcm_segments_interleave_pixel_gap_and_trailer() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let trailer = if HAS_TRAILER { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap + trailer;

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

            // The end-of-row trailer (if any) is last.
            if HAS_TRAILER {
                let trailer_idx = row * segments_per_row + TEST_PLANES + gap;
                let seg = fb.bcm_segment(trailer_idx);
                let (ptr, len) = fb.trailer_ptr_len(row);
                assert_eq!(seg.ptr, ptr, "wrong trailer ptr at row {row}");
                assert_eq!(seg.len, len, "wrong trailer len at row {row}");
                assert_eq!(seg.reps, 1, "trailer reps must be 1 at row {row}");
            }
        }
    }

    #[test]
    fn bcm_segment_total_reps_per_row() {
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let trailer = if HAS_TRAILER { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap + trailer;

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
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let trailer = if HAS_TRAILER { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap + trailer;
        let plane_bytes = TEST_COLS * core::mem::size_of::<Entry>();

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
        use crate::FrameBuffer;
        let fb = TestBuffer::new();
        let gap = if HAS_GAP { 1 } else { 0 };
        let trailer = if HAS_TRAILER { 1 } else { 0 };
        let segments_per_row = TEST_PLANES + gap + trailer;
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
        use crate::FrameBuffer;

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
        let trailer = usize::from(HAS_TRAILER);
        // Physical slot holding logical row 0 (reversed by reverse-row-order).
        let row0_base = map_row_index::<32>(0) * (5 + gap + trailer);

        for plane_idx in 0..5usize {
            // Row 0: plane 0 is the slot's first segment, planes 1+ follow the
            // gap segment.
            let seg = fb.bcm_segment(if plane_idx == 0 {
                row0_base
            } else {
                row0_base + plane_idx + gap
            });

            let data = unsafe { core::slice::from_raw_parts(seg.ptr as *const u16, TRINITY_COLS) };
            let raw = data[col_idx];
            let entry = Entry::from_raw(raw);

            let bit = 8 - 5 + plane_idx; // LSB-first
            let expect_r = (0xFF >> bit) & 1 != 0;
            let expect_g = (0xFF >> bit) & 1 != 0;
            let expect_b = (0xFF >> bit) & 1 != 0;
            assert_eq!(
                entry.red1(),
                expect_r,
                "plane {plane_idx} (bit {bit}) R1 mismatch: raw={raw:#06x}"
            );
            assert_eq!(
                entry.grn1(),
                expect_g,
                "plane {plane_idx} (bit {bit}) G1 mismatch: raw={raw:#06x}"
            );
            assert_eq!(
                entry.blu1(),
                expect_b,
                "plane {plane_idx} (bit {bit}) B1 mismatch: raw={raw:#06x}"
            );
        }
    }

    #[test]
    fn bcm_segment_data_contains_gradient_pixels() {
        use crate::FrameBuffer;

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
        let trailer = usize::from(HAS_TRAILER);
        // Physical slot holding logical row 0 (reversed by reverse-row-order).
        let row0_base = map_row_index::<32>(0) * (5 + gap + trailer);
        for plane_idx in 0..5usize {
            // Row 0: plane 0 is the slot's first segment, planes 1+ follow the
            // gap segment.
            let seg = fb.bcm_segment(if plane_idx == 0 {
                row0_base
            } else {
                row0_base + plane_idx + gap
            });
            let data = unsafe { core::slice::from_raw_parts(seg.ptr as *const u16, TRINITY_COLS) };

            for x in 0..TRINITY_COLS {
                let col_idx = map_index(x);
                let entry = Entry::from_raw(data[col_idx]);
                let brightness = (x as u8).wrapping_mul(step);
                let bit = 8 - 5 + plane_idx; // LSB-first
                let expect_r = (brightness >> bit) & 1 != 0;
                assert_eq!(
                    entry.red1(),
                    expect_r,
                    "x={x} plane={plane_idx} bit={bit} brightness={brightness} \
                     raw={:#06x}",
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

    #[test]
    fn make_row_plane_template_both_false_blanks_only_latch() {
        let t = make_row_plane_template::<TEST_COLS>(5, false, false);
        for i in 0..TEST_COLS {
            let entry = t[map_index(i)];
            assert_eq!(entry.addr(), 5);
            if i == TEST_COLS - 1 {
                assert!(entry.latch(), "last entry must latch");
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_BLANK,
                    "latch should be OE blank"
                );
            } else {
                assert!(!entry.latch(), "col {i} must not latch");
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_ACTIVE,
                    "col {i} should be OE active"
                );
            }
        }
    }

    #[test]
    fn make_row_plane_template_trail_only() {
        let t = make_row_plane_template::<TEST_COLS>(5, true, false);
        for i in 0..TEST_COLS {
            let entry = t[map_index(i)];
            if i < TRAIL_BLANK_DELAY || i == TEST_COLS - 1 {
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_BLANK,
                    "col {i} should be OE blank"
                );
            } else {
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_ACTIVE,
                    "col {i} should be OE active"
                );
            }
        }
    }

    #[test]
    fn make_row_plane_template_lead_only() {
        let t = make_row_plane_template::<TEST_COLS>(5, false, true);
        for i in 0..TEST_COLS {
            let entry = t[map_index(i)];
            if i >= TEST_COLS.saturating_sub(LEAD_BLANK_DELAY + 1) {
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_BLANK,
                    "col {i} should be lead blank"
                );
            } else {
                assert_eq!(
                    entry.0 & 0b1_0000_0000,
                    OE_ACTIVE,
                    "col {i} should be OE active"
                );
            }
        }
    }
}
