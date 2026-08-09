//! Coordinate remapping for tiled and scan-pattern LED panel arrangements
//!
//! Use [`RemappedFrameBuffer`] to wrap any framebuffer type (`plain`, `latched`,
//! `bitplane::plain`, `bitplane::latched`) and remap pixel coordinates through a
//! [`PixelRemapper`] implementation.
//!
//! Available remappers:
//! - [`ChainTopRightDown`] — tiles multiple panels into a larger virtual display
//! - [`QuarterScan`] — remaps for 1/16-scan (quarter-scan) 64×64 panels, with
//!   pluggable wiring variants (see [`quarter_scan`])
//!
//! The older [`TiledFrameBuffer`] is still available but deprecated in favour of
//! [`RemappedFrameBuffer`], which has a simpler generic signature and works with
//! all framebuffer types.

use core::{convert::Infallible, marker::PhantomData};

use crate::{
    BcmSegment, Color, FrameBuffer, FrameBufferOperations, MutableFrameBuffer,
    BCM_SEGMENT_SHAPES_CAPACITY,
};
use embedded_dma::ReadBuffer;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, PixelColor, Point, Size};

/// Computes the number of columns needed if the displays are bing tiled together.
/// # Arguments
///
/// * `cols` - Number of columns per panel
/// * `num_panels_wide` - Number of panels tiled horizontally
/// * `num_panels_high` - Number of panels tiled vertically
///
/// # Returns
///
/// Number of columns needed internally for `DmaFrameBuffer`
#[must_use]
pub const fn compute_tiled_cols(
    cols: usize,
    num_panels_wide: usize,
    num_panels_high: usize,
) -> usize {
    cols * num_panels_wide * num_panels_high
}

/// Trait for pixel re-mappers
///
/// Implementors of this trait will remap x,y coordinates from a
/// virtual panel to the actual framebuffer used to drive the panels
///
/// # Type Parameters
///
/// * `PANEL_ROWS` - Number of rows in a single panel
/// * `PANEL_COLS` - Number of columns in a single panel
/// * `TILE_ROWS` - Number of panels stacked vertically
/// * `TILE_COLS` - Number of panels stacked horizontally
pub trait PixelRemapper {
    /// Number of rows in the virtual panel
    const VIRT_ROWS: usize;
    /// Number of columns in the virtual panel
    const VIRT_COLS: usize;
    /// Number of rows in the actual framebuffer
    const FB_ROWS: usize;
    /// Number of columns in the actual framebuffer
    const FB_COLS: usize;

    /// Remap a virtual pixel to a framebuffer pixel
    #[inline]
    fn remap<C: PixelColor>(mut pixel: embedded_graphics::Pixel<C>) -> embedded_graphics::Pixel<C> {
        pixel.0 = Self::remap_point(pixel.0);
        pixel
    }

    /// Remap a virtual point to a framebuffer point
    #[inline]
    #[must_use]
    fn remap_point(mut point: Point) -> Point {
        if point.x < 0 || point.y < 0 {
            // Skip remapping points which are off the screen
            return point;
        }
        let (re_x, re_y) = Self::remap_xy(point.x as usize, point.y as usize);
        // If larger than u16, it is fair to assume that the point will be off the screen
        point.x = i32::from(re_x as u16);
        point.y = i32::from(re_y as u16);
        point
    }

    /// Remap an x,y coordinate to a framebuffer pixel
    fn remap_xy(x: usize, y: usize) -> (usize, usize);

    /// Size of the virtual panel
    #[inline]
    #[must_use]
    fn virtual_size() -> (usize, usize) {
        (Self::VIRT_ROWS, Self::VIRT_COLS)
    }

    /// Size of the framebuffer that this remaps to
    #[inline]
    #[must_use]
    fn fb_size() -> (usize, usize) {
        (Self::FB_ROWS, Self::FB_COLS)
    }
}

/// Chaining strategy for tiled panels
///
/// This type should be provided to the [`TiledFrameBuffer`] as a type argument.
/// Take a look at its documentation for more details
///
/// When looking at the front, panels are chained together starting at the top right, chaining to the
/// left until the end of the column. Then wrapping down to the next row where panels are chained left to right.
/// This makes every second rows panels installed upside down.
/// This pattern repeats until all rows of panels are covered.
///
/// # Type Parameters
///
/// * `PANEL_ROWS` - Number of rows in a single panel
/// * `PANEL_COLS` - Number of columns in a single panel
/// * `TILE_ROWS` - Number of panels stacked vertically
/// * `TILE_COLS` - Number of panels stacked horizontally
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(core::fmt::Debug)]
pub struct ChainTopRightDown<
    const PANEL_ROWS: usize,
    const PANEL_COLS: usize,
    const TILE_ROWS: usize,
    const TILE_COLS: usize,
> {}

impl<
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
    > PixelRemapper for ChainTopRightDown<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
{
    const VIRT_ROWS: usize = PANEL_ROWS * TILE_ROWS;
    const VIRT_COLS: usize = PANEL_COLS * TILE_COLS;
    const FB_ROWS: usize = PANEL_ROWS;
    const FB_COLS: usize = PANEL_COLS * TILE_ROWS * TILE_COLS;

    fn remap_xy(x: usize, y: usize) -> (usize, usize) {
        // 0 = top row, 1 = next row, …
        let row = y / PANEL_ROWS;
        let base = (TILE_ROWS - 1 - row) * Self::VIRT_COLS;

        if row % 2 == 1 {
            // this row is mounted upside-down
            (
                base + Self::VIRT_COLS - 1 - x, // mirror x across the whole virtual row
                PANEL_ROWS - 1 - (y % PANEL_ROWS), // flip y within the panel
            )
        } else {
            (base + x, y % PANEL_ROWS) // normal orientation
        }
    }
}

/// Slot-assignment variants for [`QuarterScan`] panels
///
/// A quarter-scan panel's four row groups can be wired to the four
/// (channel, section) slots of the shift register in different orders
/// depending on the driver chip and manufacturer.  Each variant below
/// describes one common wiring; pick the one that matches your panel and
/// supply it as the third type parameter of `QuarterScan`, e.g.
/// `QuarterScan<64, 64, quarter_scan::Linear>`.
///
/// Panels with exotic wirings are supported without forking this crate:
/// implement [`Variant`](crate::tiling::quarter_scan::Variant) on a local
/// marker type.
pub mod quarter_scan {
    /// Group → (channel, section) slot assignment for a quarter-scan panel
    ///
    /// The table is indexed by row group (`y / (PANEL_ROWS / 4)`); each entry
    /// is `(channel, section)` where channel 0 is the top framebuffer band
    /// (framebuffer rows `0..PANEL_ROWS / 4`, channel 1 the bottom band) and
    /// section selects which `PANEL_COLS`-wide half of that channel's shift
    /// register the group is wired to.
    pub trait Variant {
        /// Group → (channel, section) slot table
        const SLOT: [(usize, usize); 4];
    }

    /// Naive row order: group 0 → ch1/sec0, 1 → ch1/sec1, 2 → ch2/sec0, 3 → ch2/sec1
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(core::fmt::Debug)]
    pub struct Linear;

    impl Variant for Linear {
        const SLOT: [(usize, usize); 4] = [(0, 0), (0, 1), (1, 0), (1, 1)];
    }

    /// Sections swapped within each half (the default): group 0 → ch1/sec1,
    /// 1 → ch1/sec0, 2 → ch2/sec1, 3 → ch2/sec0.  Verified on hardware.
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(core::fmt::Debug)]
    pub struct SectionsSwapped;

    impl Variant for SectionsSwapped {
        const SLOT: [(usize, usize); 4] = [(0, 1), (0, 0), (1, 1), (1, 0)];
    }

    /// Top and bottom halves exchanged: group 0 → ch2/sec0, 1 → ch2/sec1,
    /// 2 → ch1/sec0, 3 → ch1/sec1
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(core::fmt::Debug)]
    pub struct HalvesSwapped;

    impl Variant for HalvesSwapped {
        const SLOT: [(usize, usize); 4] = [(1, 0), (1, 1), (0, 0), (0, 1)];
    }

    /// Channel-interleaved groups: group 0 → ch1/sec0, 1 → ch2/sec0,
    /// 2 → ch1/sec1, 3 → ch2/sec1
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(core::fmt::Debug)]
    pub struct Alternating;

    impl Variant for Alternating {
        const SLOT: [(usize, usize); 4] = [(0, 0), (1, 0), (0, 1), (1, 1)];
    }
}

/// Pixel remapper for "quarter-scan" panels (1/16-scan on 64×64 panels)
///
/// Some LED panels — particularly 64×64 modules — use fewer row-address steps
/// than the row count would suggest.  A 1/16-scan 64×64 panel has only 16 row
/// addresses (A–D) instead of the 32 a 1/32-scan panel would use, and lights
/// four rows at a time: rows *n*, *n+16*, *n+32* and *n+48* for address *n*.
/// To make this work with the two HUB75 data channels, the panel internally
/// wires each channel's shift register as two side-by-side 64-column
/// sections, so the physical framebuffer is 2× wider and 2× shorter than the
/// logical display (32×128 for a 64×64 panel).
///
/// The mapping splits the panel into four groups of `PANEL_ROWS / 4`
/// consecutive rows and places them side by side.  How the four groups are
/// wired to the four (channel, section) slots varies by driver chip and
/// manufacturer (FM6126, ICN2037, MBI5153, etc.); the `V` type parameter
/// selects the wiring — see the [`quarter_scan`] module for the built-in
/// variants.  The default [`quarter_scan::SectionsSwapped`] (verified on
/// hardware) maps:
///
/// | virtual rows  | channel    | section (framebuffer columns) |
/// |---------------|------------|-------------------------------|
/// | `0..16`       | 1 (top)    | 1 (`64..128`)                 |
/// | `16..32`      | 1 (top)    | 0 (`0..64`)                   |
/// | `32..48`      | 2 (bottom) | 1 (`64..128`)                 |
/// | `48..64`      | 2 (bottom) | 0 (`0..64`)                   |
///
/// # Type Parameters
///
/// * `PANEL_ROWS` — Logical row count of the panel (e.g. 64)
/// * `PANEL_COLS` — Logical column count of the panel (e.g. 64)
/// * `V` — Group → slot wiring variant (see [`quarter_scan`]); defaults to
///   [`quarter_scan::SectionsSwapped`]
///
/// # Framebuffer geometry
///
/// The underlying framebuffer must be allocated with:
/// - rows = `PANEL_ROWS / 2`  (e.g. 32 for a 64-row panel — 16 addresses × 2 channels)
/// - cols = `PANEL_COLS * 2`  (e.g. 128 for a 64-column panel — 2 sections per channel)
///
/// For the bitplane framebuffers this means
/// `DmaFrameBuffer<{ PANEL_ROWS / 4 }, { PANEL_COLS * 2 }, PLANES>` because
/// their `NROWS` parameter counts row-address steps (row pairs).
#[derive(core::fmt::Debug)]
pub struct QuarterScan<
    const PANEL_ROWS: usize,
    const PANEL_COLS: usize,
    V: quarter_scan::Variant = quarter_scan::SectionsSwapped,
> {
    _variant: PhantomData<V>,
}

#[cfg(feature = "defmt")]
impl<const PANEL_ROWS: usize, const PANEL_COLS: usize, V: quarter_scan::Variant> defmt::Format
    for QuarterScan<PANEL_ROWS, PANEL_COLS, V>
{
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "QuarterScan");
    }
}

impl<const PANEL_ROWS: usize, const PANEL_COLS: usize, V: quarter_scan::Variant> PixelRemapper
    for QuarterScan<PANEL_ROWS, PANEL_COLS, V>
{
    const VIRT_ROWS: usize = PANEL_ROWS;
    const VIRT_COLS: usize = PANEL_COLS;
    const FB_ROWS: usize = PANEL_ROWS / 2;
    const FB_COLS: usize = PANEL_COLS * 2;

    fn remap_xy(x: usize, y: usize) -> (usize, usize) {
        // Clip points outside the virtual panel: embedded-graphics routinely
        // generates these (e.g. text with its baseline on the last row has
        // glyph cells extending below it).  Map them just past the end of
        // the framebuffer where the inner framebuffer silently discards them.
        if x >= Self::VIRT_COLS || y >= Self::VIRT_ROWS {
            return (Self::FB_COLS, Self::FB_ROWS);
        }

        // Each quarter of the panel (16 consecutive rows on a 64-row panel)
        // is wired to one 64-column section of the shift register; the
        // variant's table gives the group → (channel, section) wiring.
        let group_rows = PANEL_ROWS / 4;
        let group = y / group_rows;
        let addr = y % group_rows;
        let (channel, section) = V::SLOT[group];
        (section * PANEL_COLS + x, channel * group_rows + addr)
    }
}

/// Tile together multiple displays in a certain configuration to form a single larger display
///
/// This is a wrapper around an actual framebuffer implementation which can be used to tile multiple
/// LED matrices together by using a certain pixel remapping strategy.
///
/// # Type Parameters
/// - `F` - The type of the underlying framebuffer which will drive the display
/// - `M` - The pixel remapping strategy (see implementers of [`PixelRemapper`]) to use to map the virtual framebuffer to the actual framebuffer
/// - `PANEL_ROWS` - Number of rows in a single panel
/// - `PANEL_COLS` - Number of columns in a single panel
/// - `NROWS`: Number of rows per scan (typically half of ROWS)
/// - `BITS`: Color depth (1-8 bits)
/// - `FRAME_COUNT`: Number of frames used for Binary Code Modulation
/// * `TILE_ROWS` - Number of panels stacked vertically
/// * `TILE_COLS` - Number of panels stacked horizontally
/// * `FB_COLS` - Number of columns that the actual framebuffer must have to drive all display
///
/// # Example
/// ```rust
/// use hub75_framebuffer::{compute_frame_count, compute_rows};
/// use hub75_framebuffer::plain::DmaFrameBuffer;
/// use hub75_framebuffer::tiling::{TiledFrameBuffer, ChainTopRightDown, compute_tiled_cols};
///
/// const TILED_COLS: usize = 3;
/// const TILED_ROWS: usize = 3;
/// const ROWS: usize = 32;
/// const PANEL_COLS: usize = 64;
/// const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
/// const BITS: u8 = 2;
/// const NROWS: usize = compute_rows(ROWS);
/// const FRAME_COUNT: usize = compute_frame_count(BITS);
///
/// type FBType = DmaFrameBuffer<ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
/// type TiledFBType = TiledFrameBuffer<
///     FBType,
///     ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
///     ROWS,
///     PANEL_COLS,
///     NROWS,
///     BITS,
///     FRAME_COUNT,
///     TILED_ROWS,
///     TILED_COLS,
///     FB_COLS,
/// >;
///
/// let mut fb = TiledFBType::new();
///
/// // Now fb is ready to be used and can be treated like one big canvas (192*96 pixels in this example)
/// ```
#[deprecated(
    since = "0.11.0",
    note = "use RemappedFrameBuffer<F, M> instead -- it works with all framebuffer types and has a simpler signature"
)]
#[derive(core::fmt::Debug)]
pub struct TiledFrameBuffer<
    F,
    M: PixelRemapper,
    const PANEL_ROWS: usize,
    const PANEL_COLS: usize,
    const NROWS: usize,
    const BITS: u8,
    const FRAME_COUNT: usize,
    const TILE_ROWS: usize,
    const TILE_COLS: usize,
    const FB_COLS: usize,
>(F, PhantomData<M>);

#[allow(deprecated)]
impl<
        F: Default,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    >
    TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    /// Create a new "virtual display" that takes ownership of the underlying framebuffer
    /// and remaps any pixels written to it to the correct locations of the underlying framebuffer
    /// based on the given `PixelRemapper`
    #[must_use]
    pub fn new() -> Self {
        Self(F::default(), PhantomData)
    }
}

#[allow(deprecated)]
impl<
        F: Default,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > Default
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    fn default() -> Self {
        Self::new()
    }
}

#[allow(deprecated)]
impl<
        F: DrawTarget<Error = Infallible, Color = Color>,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > DrawTarget
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    type Color = Color;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        self.0.draw_iter(pixels.into_iter().map(M::remap))
    }
}

#[allow(deprecated)]
impl<
        F: DrawTarget<Error = Infallible, Color = Color>,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > OriginDimensions
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    fn size(&self) -> Size {
        Size::new(M::virtual_size().1 as u32, M::virtual_size().0 as u32)
    }
}

#[allow(deprecated)]
impl<
        F: FrameBufferOperations + FrameBuffer,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > FrameBufferOperations
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    #[inline]
    fn erase(&mut self) {
        self.0.erase();
    }

    #[inline]
    fn set_pixel(&mut self, p: Point, color: Color) {
        self.0.set_pixel(M::remap_point(p), color);
    }
}

#[allow(deprecated)]
///
/// # Deprecated
///
/// This implementation is deprecated since 0.11.0. The driver now uses `BcmSegment`
/// pointers instead of `ReadBuffer` for DMA transfers.
unsafe impl<
        T,
        F: ReadBuffer<Word = T>,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > ReadBuffer
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    type Word = T;

    unsafe fn read_buffer(&self) -> (*const T, usize) {
        self.0.read_buffer()
    }
}

#[allow(deprecated)]
impl<
        F: FrameBuffer,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > FrameBuffer
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    type Word = F::Word;

    const BCM_SEGMENT_SHAPES: [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] = F::BCM_SEGMENT_SHAPES;

    const BCM_SEQUENCE_LEN: usize = F::BCM_SEQUENCE_LEN;

    const BCM_SEQUENCE_COUNT: usize = F::BCM_SEQUENCE_COUNT;

    const BCM_SEGMENTS_PER_GROUP: usize = F::BCM_SEGMENTS_PER_GROUP;

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        self.0.bcm_segment(index)
    }
}

#[allow(deprecated)]
impl<
        F: MutableFrameBuffer,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > MutableFrameBuffer
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
}

#[cfg(feature = "defmt")]
#[allow(deprecated)]
impl<
        F: defmt::Format,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > defmt::Format
    for TiledFrameBuffer<
        F,
        M,
        PANEL_ROWS,
        PANEL_COLS,
        NROWS,
        BITS,
        FRAME_COUNT,
        TILE_ROWS,
        TILE_COLS,
        FB_COLS,
    >
{
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "TiledFrameBuffer({:?})", self.0);
    }
}

/// Coordinate-remapping wrapper for any framebuffer
///
/// This is a generic wrapper around any framebuffer implementation that remaps
/// pixel coordinates before forwarding them to the inner framebuffer. It can be
/// used for:
///
/// - **Tiling** multiple panels into a larger virtual display (see [`ChainTopRightDown`])
/// - **Scan-pattern remapping** for panels with non-standard row interleaving
///   (e.g. 1/16 scan on 64×64 panels — see [`QuarterScan`])
///
/// Unlike [`TiledFrameBuffer`], this wrapper uses only two type parameters and
/// works with all four framebuffer types (`plain`, `latched`, `bitplane::plain`,
/// `bitplane::latched`).
///
/// # Type Parameters
/// - `F` — The underlying framebuffer type
/// - `M` — The pixel remapping strategy (see implementors of [`PixelRemapper`])
///
/// # Example with tiled plain framebuffer
/// ```rust
/// use hub75_framebuffer::{compute_frame_count, compute_rows};
/// use hub75_framebuffer::plain::DmaFrameBuffer;
/// use hub75_framebuffer::tiling::{RemappedFrameBuffer, ChainTopRightDown, compute_tiled_cols};
///
/// const TILED_COLS: usize = 3;
/// const TILED_ROWS: usize = 3;
/// const ROWS: usize = 32;
/// const PANEL_COLS: usize = 64;
/// const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
/// const BITS: u8 = 2;
/// const NROWS: usize = compute_rows(ROWS);
/// const FRAME_COUNT: usize = compute_frame_count(BITS);
///
/// type FBType = DmaFrameBuffer<ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
/// type Remapper = ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>;
/// type Display = RemappedFrameBuffer<FBType, Remapper>;
///
/// let mut fb = Display::new();
/// // fb is a 192×96 virtual canvas
/// ```
///
/// # Example with tiled bitplane framebuffer
/// ```rust
/// use hub75_framebuffer::bitplane::plain::DmaFrameBuffer;
/// use hub75_framebuffer::tiling::{RemappedFrameBuffer, ChainTopRightDown, compute_tiled_cols};
///
/// const TILED_COLS: usize = 3;
/// const TILED_ROWS: usize = 3;
/// const ROWS: usize = 32;
/// const NROWS: usize = ROWS / 2;
/// const PANEL_COLS: usize = 64;
/// const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
/// const PLANES: usize = 8;
///
/// type FBType = DmaFrameBuffer<NROWS, FB_COLS, PLANES>;
/// type Remapper = ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>;
/// type Display = RemappedFrameBuffer<FBType, Remapper>;
///
/// let mut fb = Display::new();
/// // fb is a 192×96 virtual canvas backed by a bitplane framebuffer
/// ```
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(core::fmt::Debug)]
pub struct RemappedFrameBuffer<F, M: PixelRemapper>(F, PhantomData<M>);

impl<F: Default, M: PixelRemapper> RemappedFrameBuffer<F, M> {
    /// Create a new remapped framebuffer that takes ownership of a
    /// default-constructed inner framebuffer and remaps any pixel writes
    /// through the given [`PixelRemapper`].
    #[must_use]
    pub fn new() -> Self {
        Self(F::default(), PhantomData)
    }
}

impl<F: Default, M: PixelRemapper> Default for RemappedFrameBuffer<F, M> {
    fn default() -> Self {
        Self::new()
    }
}

impl<F: DrawTarget<Error = Infallible, Color = Color>, M: PixelRemapper> DrawTarget
    for RemappedFrameBuffer<F, M>
{
    type Color = Color;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        self.0.draw_iter(pixels.into_iter().map(M::remap))
    }
}

impl<F: DrawTarget<Error = Infallible, Color = Color>, M: PixelRemapper> OriginDimensions
    for RemappedFrameBuffer<F, M>
{
    fn size(&self) -> Size {
        Size::new(M::virtual_size().1 as u32, M::virtual_size().0 as u32)
    }
}

impl<F: FrameBufferOperations + FrameBuffer, M: PixelRemapper> FrameBufferOperations
    for RemappedFrameBuffer<F, M>
{
    #[inline]
    fn erase(&mut self) {
        self.0.erase();
    }

    #[inline]
    fn set_pixel(&mut self, p: Point, color: Color) {
        self.0.set_pixel(M::remap_point(p), color);
    }
}

///
/// # Deprecated
///
/// This implementation is deprecated since 0.11.0. The driver now uses `BcmSegment`
/// pointers instead of `ReadBuffer` for DMA transfers.
unsafe impl<T, F: ReadBuffer<Word = T>, M: PixelRemapper> ReadBuffer for RemappedFrameBuffer<F, M> {
    type Word = T;

    unsafe fn read_buffer(&self) -> (*const T, usize) {
        self.0.read_buffer()
    }
}

impl<F: FrameBuffer, M: PixelRemapper> FrameBuffer for RemappedFrameBuffer<F, M> {
    type Word = F::Word;

    const BCM_SEGMENT_SHAPES: [(usize, usize); BCM_SEGMENT_SHAPES_CAPACITY] = F::BCM_SEGMENT_SHAPES;

    const BCM_SEQUENCE_LEN: usize = F::BCM_SEQUENCE_LEN;

    const BCM_SEQUENCE_COUNT: usize = F::BCM_SEQUENCE_COUNT;

    const BCM_SEGMENTS_PER_GROUP: usize = F::BCM_SEGMENTS_PER_GROUP;

    fn bcm_segment(&self, index: usize) -> BcmSegment {
        self.0.bcm_segment(index)
    }
}

impl<F: MutableFrameBuffer, M: PixelRemapper> MutableFrameBuffer for RemappedFrameBuffer<F, M> {}

#[cfg(test)]
mod tests {
    extern crate std;

    use embedded_graphics::prelude::*;

    use super::*;
    use crate::MutableFrameBuffer;
    use core::convert::Infallible;

    #[test]
    fn test_virtual_size_function_with_equal_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 3>;
        let virt_size = PanelChain::virtual_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL * 3, COLS_IN_PANEL * 3));
    }

    #[test]
    fn test_virtual_size_function_with_uneven_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 5, 3>;
        let virt_size = PanelChain::virtual_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL * 5, COLS_IN_PANEL * 3));
    }

    #[test]
    fn test_virtual_size_function_with_single_column() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 1>;
        let virt_size = PanelChain::virtual_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL * 3, COLS_IN_PANEL));
    }

    #[test]
    fn test_fb_size_function_with_equal_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 3>;
        let virt_size = PanelChain::fb_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL, COLS_IN_PANEL * 9));
    }

    #[test]
    fn test_fb_size_function_with_uneven_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 5, 3>;
        let virt_size = PanelChain::fb_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL, COLS_IN_PANEL * 15));
    }

    #[test]
    fn test_fb_size_function_with_single_column() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 1>;
        let virt_size = PanelChain::fb_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL, COLS_IN_PANEL * 3));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_in_origin() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(0, 0), Color::RED));
        assert_eq!(pixel.0, Point::new(384, 0));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_in_bottom_left_corner() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(0, 95), Color::RED));
        assert_eq!(pixel.0, Point::new(0, 31));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_in_bottom_right_corner() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(191, 95), Color::RED));
        assert_eq!(pixel.0, Point::new(191, 31));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_on_x_right_edge_of_first_panel() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(63, 0), Color::RED));
        assert_eq!(pixel.0, Point::new(447, 0));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_on_x_left_edge_of_second_panel() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(64, 0), Color::RED));
        assert_eq!(pixel.0, Point::new(448, 0));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_on_y_bottom_edge_of_first_panel() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(0, 31), Color::RED));
        assert_eq!(pixel.0, Point::new(384, 31));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_on_y_top_edge_of_fourth_panel() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(0, 32), Color::RED));
        assert_eq!(pixel.0, Point::new(383, 31));
    }

    #[test]
    fn test_pixel_remap_top_right_down_point_slightly_to_the_top_middle() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(100, 40), Color::RED));
        assert_eq!(pixel.0, Point::new(283, 23));
    }

    #[test]
    fn test_pixel_remap_negative_pixel_does_not_remap() {
        type PanelChain = ChainTopRightDown<32, 64, 3, 3>;

        let pixel = PanelChain::remap(Pixel(Point::new(-5, 40), Color::RED));
        assert_eq!(pixel.0, Point::new(-5, 40));
    }

    #[test]
    fn test_compute_tiled_cols() {
        assert_eq!(192, compute_tiled_cols(32, 3, 2));
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiling_framebuffer_canvas_size() {
        use crate::plain::DmaFrameBuffer;
        use crate::tiling::{compute_tiled_cols, ChainTopRightDown, TiledFrameBuffer};
        use crate::{compute_frame_count, compute_rows};

        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
        const BITS: u8 = 2;
        const NROWS: usize = compute_rows(ROWS);
        const FRAME_COUNT: usize = compute_frame_count(BITS);

        type FBType = DmaFrameBuffer<ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
        type TiledFBType = TiledFrameBuffer<
            FBType,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            NROWS,
            BITS,
            FRAME_COUNT,
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >;

        let fb = TiledFBType::new();

        assert_eq!(fb.size(), Size::new(192, 96));
    }

    // Test helper framebuffer that records calls for verification
    struct TestFrameBuffer {
        calls: std::cell::RefCell<std::vec::Vec<Call>>,
        buf: [u8; 8],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Call {
        Erase,
        SetPixel { p: Point, color: Color },
        Draw(std::vec::Vec<(Point, Color)>),
    }

    impl TestFrameBuffer {
        fn new() -> Self {
            Self {
                calls: std::cell::RefCell::new(std::vec::Vec::new()),
                buf: [0; 8],
            }
        }

        fn take_calls(&self) -> std::vec::Vec<Call> {
            core::mem::take(&mut *self.calls.borrow_mut())
        }
    }

    impl Default for TestFrameBuffer {
        fn default() -> Self {
            Self::new()
        }
    }

    impl DrawTarget for TestFrameBuffer {
        type Color = Color;
        type Error = Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            let v = pixels.into_iter().map(|p| (p.0, p.1)).collect();
            self.calls.borrow_mut().push(Call::Draw(v));
            Ok(())
        }
    }

    impl OriginDimensions for TestFrameBuffer {
        fn size(&self) -> Size {
            Size::new(1, 1)
        }
    }

    impl FrameBuffer for TestFrameBuffer {
        type Word = u8;

        const BCM_SEGMENT_SHAPES: [(usize, usize); crate::BCM_SEGMENT_SHAPES_CAPACITY] = {
            let mut shapes = [(0usize, 0usize); crate::BCM_SEGMENT_SHAPES_CAPACITY];
            shapes[0] = (8, 1);
            shapes
        };

        const BCM_SEQUENCE_LEN: usize = 1;

        const BCM_SEQUENCE_COUNT: usize = 1;

        fn bcm_segment(&self, index: usize) -> BcmSegment {
            assert!(index == 0);
            BcmSegment {
                ptr: self.buf.as_ptr(),
                len: self.buf.len(),
                reps: 1,
            }
        }
    }

    impl FrameBufferOperations for TestFrameBuffer {
        fn erase(&mut self) {
            self.calls.borrow_mut().push(Call::Erase);
        }

        fn set_pixel(&mut self, p: Point, color: Color) {
            self.calls.borrow_mut().push(Call::SetPixel { p, color });
        }
    }

    impl MutableFrameBuffer for TestFrameBuffer {}

    #[allow(deprecated)]
    unsafe impl embedded_dma::ReadBuffer for TestFrameBuffer {
        type Word = u8;

        unsafe fn read_buffer(&self) -> (*const u8, usize) {
            (self.buf.as_ptr(), self.buf.len())
        }
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_draw_iter_forwards_with_remap() {
        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let mut fb = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >(TestFrameBuffer::new(), core::marker::PhantomData);

        let input = [
            Pixel(Point::new(0, 0), Color::RED),
            Pixel(Point::new(63, 0), Color::GREEN),
            Pixel(Point::new(64, 0), Color::BLUE),
            Pixel(Point::new(100, 40), Color::WHITE),
        ];

        fb.draw_iter(input.into_iter()).unwrap();

        let calls = fb.0.take_calls();
        assert_eq!(calls.len(), 1);
        match &calls[0] {
            Call::Draw(v) => {
                let expected =
                    [
                        ChainTopRightDown::<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>::remap(
                            Pixel(Point::new(0, 0), Color::RED),
                        ),
                        ChainTopRightDown::<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>::remap(
                            Pixel(Point::new(63, 0), Color::GREEN),
                        ),
                        ChainTopRightDown::<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>::remap(
                            Pixel(Point::new(64, 0), Color::BLUE),
                        ),
                        ChainTopRightDown::<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>::remap(
                            Pixel(Point::new(100, 40), Color::WHITE),
                        ),
                    ];
                let expected_points: std::vec::Vec<(Point, Color)> =
                    expected.iter().map(|p| (p.0, p.1)).collect();
                assert_eq!(v.as_slice(), expected_points.as_slice());
            }
            _ => panic!("expected a Draw call"),
        }
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_set_pixel_remaps_and_forwards() {
        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let mut fb = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >(TestFrameBuffer::new(), core::marker::PhantomData);

        let p = Point::new(100, 40);
        fb.set_pixel(p, Color::BLUE);

        let calls = fb.0.take_calls();
        assert_eq!(calls.len(), 1);
        match calls.into_iter().next().unwrap() {
            Call::SetPixel { p: rp, color } => {
                let expected =
                    ChainTopRightDown::<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>::remap_point(p);
                assert_eq!(rp, expected);
                assert_eq!(color, Color::BLUE);
            }
            _ => panic!("expected a SetPixel call"),
        }
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_erase_forwards() {
        const TILED_COLS: usize = 2;
        const TILED_ROWS: usize = 2;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let mut fb = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >(TestFrameBuffer::new(), core::marker::PhantomData);
        fb.erase();
        let calls = fb.0.take_calls();
        assert_eq!(calls, std::vec![Call::Erase]);
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_negative_coordinates_not_remapped() {
        const TILED_COLS: usize = 2;
        const TILED_ROWS: usize = 2;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let mut fb = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >(TestFrameBuffer::new(), core::marker::PhantomData);

        // set_pixel path
        let neg = Point::new(-3, 5);
        fb.set_pixel(neg, Color::GREEN);
        // draw_iter path
        fb.draw_iter(core::iter::once(Pixel(Point::new(10, -2), Color::RED)))
            .unwrap();

        let calls = fb.0.take_calls();
        assert_eq!(calls.len(), 2);
        assert!(matches!(calls[0], Call::SetPixel { p, .. } if p == neg));
        match &calls[1] {
            Call::Draw(v) => {
                assert_eq!(v.as_slice(), &[(Point::new(10, -2), Color::RED)]);
            }
            _ => panic!("expected a Draw call"),
        }
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_read_buffer_passthrough() {
        const TILED_COLS: usize = 2;
        const TILED_ROWS: usize = 2;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let fb = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >(TestFrameBuffer::new(), core::marker::PhantomData);

        let inner_ptr = fb.0.buf.as_ptr();
        let inner_len = fb.0.buf.len();

        let (ptr, len) = unsafe { fb.read_buffer() };
        assert_eq!(ptr, inner_ptr);
        assert_eq!(len, inner_len);
    }

    // Remapper that generates very large coordinates to trigger u16 truncation in remap_point
    struct Huge<
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
    >;

    impl<
            const PANEL_ROWS: usize,
            const PANEL_COLS: usize,
            const TILE_ROWS: usize,
            const TILE_COLS: usize,
        > PixelRemapper for Huge<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
    {
        const VIRT_ROWS: usize = PANEL_ROWS * TILE_ROWS;
        const VIRT_COLS: usize = PANEL_COLS * TILE_COLS;
        const FB_ROWS: usize = PANEL_ROWS;
        const FB_COLS: usize = PANEL_COLS * TILE_ROWS * TILE_COLS;

        fn remap_xy(x: usize, y: usize) -> (usize, usize) {
            (x + 70_000, y + 70_000)
        }
    }

    #[test]
    #[allow(deprecated)]
    fn test_remap_point_truncates_to_u16_range() {
        const TILED_COLS: usize = 1;
        const TILED_ROWS: usize = 1;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let mut fb = TiledFrameBuffer::<
            TestFrameBuffer,
            Huge<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >(TestFrameBuffer::new(), core::marker::PhantomData);
        fb.set_pixel(Point::new(1, 2), Color::RED);

        let calls = fb.0.take_calls();
        match calls.into_iter().next().unwrap() {
            Call::SetPixel { p, color } => {
                let (rx, ry) = (1usize + 70_000, 2usize + 70_000);
                let expected = Point::new(i32::from(rx as u16), i32::from(ry as u16));
                assert_eq!(p, expected);
                assert_eq!(color, Color::RED);
            }
            other => panic!("unexpected call recorded: {other:?}"),
        }
    }

    #[test]
    fn test_more_compute_tiled_cols_cases() {
        assert_eq!(compute_tiled_cols(64, 1, 4), 256);
        assert_eq!(compute_tiled_cols(64, 4, 1), 256);
        assert_eq!(compute_tiled_cols(32, 4, 5), 640);
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_default_and_new_construct() {
        const TILED_COLS: usize = 4;
        const TILED_ROWS: usize = 2;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let fb_default = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >::default();

        let fb_new = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >::new();

        // Size comes from OriginDimensions impl on TiledFrameBuffer (via M::virtual_size)
        let expected_size = Size::new((PANEL_COLS * TILED_COLS) as u32, (ROWS * TILED_ROWS) as u32);
        assert_eq!(fb_default.size(), expected_size);
        assert_eq!(fb_new.size(), expected_size);

        // No calls recorded yet on inner framebuffer
        assert!(fb_default.0.take_calls().is_empty());
        assert!(fb_new.0.take_calls().is_empty());
    }

    #[test]
    #[allow(deprecated)]
    fn test_tiled_origin_dimensions_matches_virtual_size() {
        const TILED_COLS: usize = 5;
        const TILED_ROWS: usize = 2;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);

        let fb = TiledFrameBuffer::<
            TestFrameBuffer,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
            ROWS,
            PANEL_COLS,
            { crate::compute_rows(ROWS) },
            2,
            { crate::compute_frame_count(2) },
            TILED_ROWS,
            TILED_COLS,
            FB_COLS,
        >::new();

        let (virt_rows, virt_cols) = <ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS> as PixelRemapper>::virtual_size();
        assert_eq!(fb.size(), Size::new(virt_cols as u32, virt_rows as u32));
    }

    // Expected mapping for ChainTopRightDown (current, correct behavior)
    fn expected_ctrdd_xy<const PR: usize, const PC: usize, const TR: usize, const TC: usize>(
        x: usize,
        y: usize,
    ) -> (usize, usize) {
        let row = y / PR;
        let base = (TR - 1 - row) * (PC * TC);
        if row % 2 == 1 {
            (base + (PC * TC) - 1 - x, PR - 1 - (y % PR))
        } else {
            (base + x, y % PR)
        }
    }

    #[test]
    fn test_chain_top_right_down_corners_2x3() {
        const PR: usize = 32;
        const PC: usize = 64;
        const TR: usize = 2;
        const TC: usize = 3;
        type M = ChainTopRightDown<PR, PC, TR, TC>;

        for r in 0..TR {
            for c in 0..TC {
                let x0 = c * PC;
                let y0 = r * PR;
                let corners = [
                    (x0, y0),                   // TL
                    (x0 + PC - 1, y0),          // TR
                    (x0, y0 + PR - 1),          // BL
                    (x0 + PC - 1, y0 + PR - 1), // BR
                ];

                for &(x, y) in &corners {
                    let got = <M as PixelRemapper>::remap_xy(x, y);
                    let exp = expected_ctrdd_xy::<PR, PC, TR, TC>(x, y);
                    assert_eq!(
                        got, exp,
                        "corner mismatch at panel (row={}, col={}), virtual=({}, {})",
                        r, c, x, y
                    );
                }
            }
        }
    }

    #[test]
    fn test_chain_top_right_down_corners_3x2() {
        const PR: usize = 32;
        const PC: usize = 64;
        const TR: usize = 3;
        const TC: usize = 2;
        type M = ChainTopRightDown<PR, PC, TR, TC>;

        for r in 0..TR {
            for c in 0..TC {
                let x0 = c * PC;
                let y0 = r * PR;
                let corners = [
                    (x0, y0),                   // TL
                    (x0 + PC - 1, y0),          // TR
                    (x0, y0 + PR - 1),          // BL
                    (x0 + PC - 1, y0 + PR - 1), // BR
                ];

                for &(x, y) in &corners {
                    let got = <M as PixelRemapper>::remap_xy(x, y);
                    let exp = expected_ctrdd_xy::<PR, PC, TR, TC>(x, y);
                    assert_eq!(
                        got, exp,
                        "corner mismatch at panel (row={}, col={}), virtual=({}, {})",
                        r, c, x, y
                    );
                }
            }
        }
    }

    // ---- RemappedFrameBuffer tests ----

    type TestRemapped = RemappedFrameBuffer<TestFrameBuffer, ChainTopRightDown<32, 64, 3, 3>>;

    #[test]
    fn test_remapped_default_and_new_construct() {
        let _fb = TestRemapped::new();
        let _fb2: TestRemapped = Default::default();
    }

    #[test]
    fn test_remapped_origin_dimensions_matches_virtual_size() {
        let fb = TestRemapped::new();
        assert_eq!(fb.size(), Size::new(192, 96));
    }

    #[test]
    fn test_remapped_draw_iter_forwards_with_remap() {
        let mut fb = RemappedFrameBuffer::<TestFrameBuffer, ChainTopRightDown<32, 64, 3, 3>>(
            TestFrameBuffer::new(),
            core::marker::PhantomData,
        );

        let input = [
            Pixel(Point::new(0, 0), Color::RED),
            Pixel(Point::new(63, 0), Color::GREEN),
            Pixel(Point::new(64, 0), Color::BLUE),
            Pixel(Point::new(100, 40), Color::WHITE),
        ];

        fb.draw_iter(input.into_iter()).unwrap();

        let calls = fb.0.take_calls();
        assert_eq!(calls.len(), 1);
        match &calls[0] {
            Call::Draw(v) => {
                let expected = [
                    ChainTopRightDown::<32, 64, 3, 3>::remap(Pixel(Point::new(0, 0), Color::RED)),
                    ChainTopRightDown::<32, 64, 3, 3>::remap(Pixel(
                        Point::new(63, 0),
                        Color::GREEN,
                    )),
                    ChainTopRightDown::<32, 64, 3, 3>::remap(Pixel(Point::new(64, 0), Color::BLUE)),
                    ChainTopRightDown::<32, 64, 3, 3>::remap(Pixel(
                        Point::new(100, 40),
                        Color::WHITE,
                    )),
                ];
                let expected_points: std::vec::Vec<(Point, Color)> =
                    expected.iter().map(|p| (p.0, p.1)).collect();
                assert_eq!(v.as_slice(), expected_points.as_slice());
            }
            _ => panic!("expected a Draw call"),
        }
    }

    #[test]
    fn test_remapped_set_pixel_remaps_and_forwards() {
        let mut fb = RemappedFrameBuffer::<TestFrameBuffer, ChainTopRightDown<32, 64, 3, 3>>(
            TestFrameBuffer::new(),
            core::marker::PhantomData,
        );

        let p = Point::new(100, 40);
        fb.set_pixel(p, Color::BLUE);

        let calls = fb.0.take_calls();
        assert_eq!(calls.len(), 1);
        match calls.into_iter().next().unwrap() {
            Call::SetPixel { p: rp, color } => {
                let expected = ChainTopRightDown::<32, 64, 3, 3>::remap_point(p);
                assert_eq!(rp, expected);
                assert_eq!(color, Color::BLUE);
            }
            _ => panic!("expected a SetPixel call"),
        }
    }

    #[test]
    fn test_remapped_erase_forwards() {
        let mut fb = RemappedFrameBuffer::<TestFrameBuffer, ChainTopRightDown<32, 64, 2, 2>>(
            TestFrameBuffer::new(),
            core::marker::PhantomData,
        );

        fb.erase();
        let calls = fb.0.take_calls();
        assert_eq!(calls, std::vec![Call::Erase]);
    }

    #[test]
    fn test_remapped_negative_coordinates_not_remapped() {
        let mut fb = RemappedFrameBuffer::<TestFrameBuffer, ChainTopRightDown<32, 64, 2, 2>>(
            TestFrameBuffer::new(),
            core::marker::PhantomData,
        );

        let neg = Point::new(-3, 5);
        fb.set_pixel(neg, Color::GREEN);
        fb.draw_iter(core::iter::once(Pixel(Point::new(10, -2), Color::RED)))
            .unwrap();

        let calls = fb.0.take_calls();
        assert_eq!(calls.len(), 2);
        assert!(matches!(calls[0], Call::SetPixel { p, .. } if p == neg));
        match &calls[1] {
            Call::Draw(v) => {
                assert_eq!(v.as_slice(), &[(Point::new(10, -2), Color::RED)]);
            }
            _ => panic!("expected a Draw call"),
        }
    }

    #[test]
    fn test_remapped_read_buffer_passthrough() {
        let fb = RemappedFrameBuffer::<TestFrameBuffer, ChainTopRightDown<32, 64, 2, 2>>(
            TestFrameBuffer::new(),
            core::marker::PhantomData,
        );

        let inner_ptr = fb.0.buf.as_ptr();
        let inner_len = fb.0.buf.len();

        let (ptr, len) = unsafe { fb.read_buffer() };
        assert_eq!(ptr, inner_ptr);
        assert_eq!(len, inner_len);
    }

    #[test]
    fn test_remapped_with_plain_framebuffer() {
        use crate::plain::DmaFrameBuffer;
        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
        const BITS: u8 = 2;
        const NROWS: usize = crate::compute_rows(ROWS);
        const FRAME_COUNT: usize = crate::compute_frame_count(BITS);

        type FBType = DmaFrameBuffer<ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
        type Display = RemappedFrameBuffer<
            FBType,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
        >;

        let fb = Display::new();
        assert_eq!(fb.size(), Size::new(192, 96));
    }

    #[test]
    fn test_remapped_with_latched_framebuffer() {
        use crate::latched::DmaFrameBuffer;
        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
        const BITS: u8 = 2;
        const NROWS: usize = crate::compute_rows(ROWS);
        const FRAME_COUNT: usize = crate::compute_frame_count(BITS);

        type FBType = DmaFrameBuffer<ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
        type Display = RemappedFrameBuffer<
            FBType,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
        >;

        let fb = Display::new();
        assert_eq!(fb.size(), Size::new(192, 96));
    }

    #[test]
    fn test_remapped_with_bitplane_plain_framebuffer() {
        use crate::bitplane::plain::DmaFrameBuffer;
        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const NROWS: usize = ROWS / 2;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
        const PLANES: usize = 8;

        type FBType = DmaFrameBuffer<NROWS, FB_COLS, PLANES>;
        type Display = RemappedFrameBuffer<
            FBType,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
        >;

        let fb = Display::new();
        assert_eq!(fb.size(), Size::new(192, 96));
    }

    #[test]
    fn test_remapped_with_bitplane_latched_framebuffer() {
        use crate::bitplane::latched::DmaFrameBuffer;
        const TILED_COLS: usize = 3;
        const TILED_ROWS: usize = 3;
        const ROWS: usize = 32;
        const NROWS: usize = ROWS / 2;
        const PANEL_COLS: usize = 64;
        const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
        const PLANES: usize = 8;

        type FBType = DmaFrameBuffer<NROWS, FB_COLS, PLANES>;
        type Display = RemappedFrameBuffer<
            FBType,
            ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
        >;

        let fb = Display::new();
        assert_eq!(fb.size(), Size::new(192, 96));
    }

    #[test]
    fn test_quarter_scan_associated_constants() {
        type QS = QuarterScan<64, 64>;
        assert_eq!(QS::VIRT_ROWS, 64);
        assert_eq!(QS::VIRT_COLS, 64);
        assert_eq!(QS::FB_ROWS, 32);
        assert_eq!(QS::FB_COLS, 128);
        assert_eq!(QS::virtual_size(), (64, 64));
        assert_eq!(QS::fb_size(), (32, 128));
    }

    #[test]
    fn test_quarter_scan_remap_xy_groups() {
        type QS = QuarterScan<64, 64>;
        // group 0 (virtual rows 0..16) → channel 1, section 1
        assert_eq!(QS::remap_xy(0, 0), (64, 0));
        assert_eq!(QS::remap_xy(63, 15), (127, 15));
        // group 1 (virtual rows 16..32) → channel 1, section 0
        assert_eq!(QS::remap_xy(0, 16), (0, 0));
        assert_eq!(QS::remap_xy(63, 31), (63, 15));
        // group 2 (virtual rows 32..48) → channel 2, section 1
        assert_eq!(QS::remap_xy(0, 32), (64, 16));
        assert_eq!(QS::remap_xy(63, 47), (127, 31));
        // group 3 (virtual rows 48..64) → channel 2, section 0
        assert_eq!(QS::remap_xy(0, 48), (0, 16));
        assert_eq!(QS::remap_xy(63, 63), (63, 31));
    }

    #[test]
    fn test_quarter_scan_remap_out_of_bounds_is_clipped() {
        type QS = QuarterScan<64, 64>;
        // points outside the 64×64 virtual panel map just past the inner
        // framebuffer bounds (128×32) so the inner framebuffer discards them
        assert_eq!(QS::remap_xy(64, 0), (128, 32));
        assert_eq!(QS::remap_xy(0, 64), (128, 32));
        assert_eq!(QS::remap_xy(100, 100), (128, 32));
        // the last valid pixel is unaffected
        assert_eq!(QS::remap_xy(63, 63), (63, 31));
    }

    #[test]
    fn test_quarter_scan_offscreen_pixels_do_not_panic() {
        // embedded-graphics generates pixels outside the canvas when drawing
        // text with its baseline on the last row (6×8 cell extending below
        // the baseline) — these must be clipped, not panic.
        use crate::bitplane::plain::DmaFrameBuffer;
        type InnerFB = DmaFrameBuffer<16, 128, 4>;
        type Display = RemappedFrameBuffer<InnerFB, QuarterScan<64, 64>>;

        let mut fb = Display::new();
        for x in 0..70 {
            for y in 57..70 {
                fb.set_pixel(Point::new(x, y), Color::WHITE);
            }
        }
    }

    #[test]
    fn test_quarter_scan_remap_is_bijective_and_in_bounds() {
        use super::quarter_scan::*;
        fn check<V: Variant>() {
            let mut seen = [false; 32 * 128];
            for y in 0..QuarterScan::<64, 64, V>::VIRT_ROWS {
                for x in 0..QuarterScan::<64, 64, V>::VIRT_COLS {
                    let (fb_x, fb_y) = QuarterScan::<64, 64, V>::remap_xy(x, y);
                    assert!(
                        fb_x < QuarterScan::<64, 64, V>::FB_COLS,
                        "fb_x {fb_x} out of bounds"
                    );
                    assert!(
                        fb_y < QuarterScan::<64, 64, V>::FB_ROWS,
                        "fb_y {fb_y} out of bounds"
                    );
                    let idx = fb_y * QuarterScan::<64, 64, V>::FB_COLS + fb_x;
                    assert!(!seen[idx], "duplicate mapping to ({fb_x}, {fb_y})");
                    seen[idx] = true;
                }
            }
            // every framebuffer cell is covered exactly once
            assert!(seen.iter().all(|&s| s));
        }
        check::<Linear>();
        check::<SectionsSwapped>();
        check::<HalvesSwapped>();
        check::<Alternating>();
    }

    #[test]
    fn test_quarter_scan_variant_tables_are_permutations() {
        use super::quarter_scan::*;
        fn assert_permutation<V: Variant>() {
            let mut seen = [false; 4];
            for &(channel, section) in V::SLOT.iter() {
                assert!(channel < 2 && section < 2, "slot out of range");
                let idx = channel * 2 + section;
                assert!(!seen[idx], "duplicate slot in variant table");
                seen[idx] = true;
            }
            assert!(
                seen.iter().all(|&s| s),
                "variant table must cover all four slots"
            );
        }
        assert_permutation::<Linear>();
        assert_permutation::<SectionsSwapped>();
        assert_permutation::<HalvesSwapped>();
        assert_permutation::<Alternating>();
    }

    #[test]
    fn test_quarter_scan_variant_mappings() {
        use super::quarter_scan::*;
        // Linear: naive row order
        assert_eq!(QuarterScan::<64, 64, Linear>::remap_xy(0, 0), (0, 0));
        assert_eq!(QuarterScan::<64, 64, Linear>::remap_xy(0, 16), (64, 0));
        assert_eq!(QuarterScan::<64, 64, Linear>::remap_xy(0, 32), (0, 16));
        assert_eq!(QuarterScan::<64, 64, Linear>::remap_xy(0, 48), (64, 16));
        // HalvesSwapped: top and bottom halves exchanged
        assert_eq!(
            QuarterScan::<64, 64, HalvesSwapped>::remap_xy(0, 0),
            (0, 16)
        );
        assert_eq!(
            QuarterScan::<64, 64, HalvesSwapped>::remap_xy(0, 16),
            (64, 16)
        );
        assert_eq!(
            QuarterScan::<64, 64, HalvesSwapped>::remap_xy(0, 32),
            (0, 0)
        );
        assert_eq!(
            QuarterScan::<64, 64, HalvesSwapped>::remap_xy(0, 48),
            (64, 0)
        );
        // Alternating: channel-interleaved groups
        assert_eq!(QuarterScan::<64, 64, Alternating>::remap_xy(0, 0), (0, 0));
        assert_eq!(QuarterScan::<64, 64, Alternating>::remap_xy(0, 16), (0, 16));
        assert_eq!(QuarterScan::<64, 64, Alternating>::remap_xy(0, 32), (64, 0));
        assert_eq!(
            QuarterScan::<64, 64, Alternating>::remap_xy(0, 48),
            (64, 16)
        );
    }

    #[test]
    fn test_quarter_scan_remap_matches_inner_bitplane_geometry() {
        // The bitplane DmaFrameBuffer<NROWS, COLS, PLANES> exposes NROWS * 2
        // rows (one per channel), so a 64x64 1/16-scan panel needs NROWS = 16
        // (16 addresses) and COLS = 128 (two 64-column sections per channel).
        use crate::bitplane::plain::DmaFrameBuffer;
        type QS = QuarterScan<64, 64>;
        type InnerFB = DmaFrameBuffer<16, 128, 4>;
        type Display = RemappedFrameBuffer<InnerFB, QS>;

        let mut fb = Display::new();
        assert_eq!(fb.size(), Size::new(64, 64));
        // one pixel in each group, verified through the remapped wrapper
        fb.set_pixel(Point::new(1, 2), Color::RED);
        fb.set_pixel(Point::new(1, 20), Color::GREEN);
        fb.set_pixel(Point::new(1, 40), Color::BLUE);
        fb.set_pixel(Point::new(1, 60), Color::WHITE);
    }
}
