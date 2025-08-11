//! For tiling multiple displays together in various grid arrangements
//! They have to be tiles together in some specific supported grid layouts.
//! Currently supported layouts:
//! - [`ChainTopRightDown`]
//!
//! To write to those panels the [`TiledFrameBuffer`] can be used.
//! A usage example can be found at that structs documentation.

use core::{convert::Infallible, marker::PhantomData};

use crate::{Color, FrameBuffer, FrameBufferOperations, WordSize};
#[cfg(not(feature = "esp-hal-dma"))]
use embedded_dma::ReadBuffer;
use embedded_graphics::prelude::{DrawTarget, OriginDimensions, PixelColor, Point, Size};
#[cfg(feature = "esp-hal-dma")]
use esp_hal::dma::ReadBuffer;

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
        let row = TILE_ROWS - y / PANEL_ROWS - 1;
        if row % 2 == 1 {
            // panel is upside down
            (
                Self::FB_COLS - x - (row * Self::VIRT_COLS) - 1,
                PANEL_ROWS - 1 - (y % PANEL_ROWS),
            )
        } else {
            ((row * Self::VIRT_COLS) + x, y % PANEL_ROWS)
        }
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

impl<
        F: FrameBufferOperations<PANEL_ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > FrameBufferOperations<PANEL_ROWS, PANEL_COLS, NROWS, BITS, FRAME_COUNT>
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

#[cfg(not(feature = "esp-hal-dma"))]
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

#[cfg(feature = "esp-hal-dma")]
unsafe impl<
        F: ReadBuffer,
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
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        self.0.read_buffer()
    }
}

impl<
        F: ReadBuffer,
        M: PixelRemapper,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const NROWS: usize,
        const BITS: u8,
        const FRAME_COUNT: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
        const FB_COLS: usize,
    > FrameBuffer<PANEL_ROWS, PANEL_COLS, NROWS, BITS, FRAME_COUNT>
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
    fn get_word_size(&self) -> WordSize {
        WordSize::Sixteen
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use embedded_graphics::prelude::*;

    use super::*;

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
}
