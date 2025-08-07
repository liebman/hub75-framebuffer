//! For tiling multiple displays together in various grid arrangements
//! They have to be tiles together

use core::convert::Infallible;

use crate::{Color};
use embedded_graphics::prelude::{PixelColor};

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
pub const fn compute_tiled_cols(cols: usize, num_panels_wide: usize, num_panels_high: usize) -> usize {
    cols * num_panels_wide * num_panels_high
}

/// Trait for pixel remappers
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
pub trait PixelRemapper<
    const PANEL_ROWS: usize,
    const PANEL_COLS: usize,
    const TILE_ROWS: usize,
    const TILE_COLS: usize,
>
{
    /// Number of rows in the virtual panel
    const VIRT_ROWS: usize = PANEL_ROWS * TILE_ROWS;
    /// Number of columns in the virtual panel
    const VIRT_COLS: usize = PANEL_COLS * TILE_COLS;
    /// Number of rows in the actual framebuffer
    const FB_ROWS: usize = PANEL_ROWS;
    /// Number of columns in the actual framebuffer
    const FB_COLS: usize = PANEL_COLS * TILE_COLS * TILE_ROWS;

    /// Remap a virtual pixel to a framebuffer pixel
    fn remap<C: PixelColor>(pixel: embedded_graphics::Pixel<C>) -> embedded_graphics::Pixel<C>;

    /// Size of the virtual panel
    #[inline]
    fn virtual_size() -> (usize, usize) {
        (Self::VIRT_ROWS, Self::VIRT_COLS)
    }

    /// Size of the framebuffer that this remaps to
    #[inline]
    fn fb_size() -> (usize, usize) {
        (Self::FB_ROWS, Self::FB_COLS)
    }
}

/// Chaining strategy for tiled panels
///
/// This type should be provided to the [TiledFrameBuffer] as a type argument.
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
    > ChainTopRightDown<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
{
    /// Create a new panel chain
    pub fn new() -> Self {
        Self {}
    }
}

impl<
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
    > PixelRemapper<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
    for ChainTopRightDown<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
{
    fn remap<C: PixelColor>(mut pixel: embedded_graphics::Pixel<C>) -> embedded_graphics::Pixel<C> {
        let row = TILE_ROWS as i32 -  pixel.0.y / PANEL_ROWS as i32 - 1;
        if row % 2 == 1 {
            // panel is upside down
            pixel.0.x = Self::FB_COLS as i32 - pixel.0.x - (row * Self::VIRT_COLS as i32) - 1;
            pixel.0.y = PANEL_ROWS as i32 - 1 - (pixel.0.y % PANEL_ROWS as i32);
        } else {
            pixel.0.x = (row * Self::VIRT_COLS as i32) + pixel.0.x;
            pixel.0.y = pixel.0.y % PANEL_ROWS as i32;
        }
        pixel
    }
}

/// Tile together multiple displays in a certain configuration to form a single larger display
///
/// This is a wrapper around an actual framebuffer implementation which can be used to tile multiple
/// LED matrices together by using a certain pixel remapping strategy.
/// # Example
/// ```rust
/// use hub75_framebuffer::compute_frame_count;
/// use hub75_framebuffer::compute_rows;
/// use hub75_framebuffer::plain::DmaFrameBuffer;
/// use hub75_framebuffer::tiling::{TiledFrameBuffer, ChainTopRightDown};
///
/// const TILED_COLS: usize = 3;
/// const TILED_ROWS: usize = 3;
/// const ROWS: usize = 32;
/// const PANEL_COLS: usize = 64;
/// const COLS: usize = PANEL_COLS * TILED_ROWS * TILED_COLS;
/// const BITS: u8 = 4;
/// const NROWS: usize = compute_rows(ROWS);
/// const FRAME_COUNT: usize = compute_frame_count(BITS);
///
/// type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
/// type PanelChain = ChainTopRightDown<ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>;
///
/// let mut fb = FBType::new();
/// let mut fb = TiledFrameBuffer::new(&mut fb, PanelChain::new());
///
/// // Now fb is ready to be used and can be treated like one big canvas (192*96 pixels in this example)
/// // The tiles framebuffer does intentionally not reimplement any functions of the underlying framebuffer
/// // If you need to access them you may use `fb.0`
/// ```
pub struct TiledFrameBuffer<'a,
    F,
    M,
    const PANEL_ROWS: usize,
    const PANEL_COLS: usize,
    const TILE_ROWS: usize,
    const TILE_COLS: usize,
>(pub &'a mut F, M);

impl<'a,
        F,
        M,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
    > TiledFrameBuffer<'a, F, M, PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
where
    F: embedded_graphics::draw_target::DrawTarget,
    M: PixelRemapper<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>,
{
    /// Create a new "virtual display" that takes ownership of the underlying framebuffer
    /// an remaps any pixels written to it to the correct locations of the underlying framebuffer
    /// based on the given PixelRemapper
    pub fn new(fb: &'a mut F, mapper: M) -> Self {
        Self(fb, mapper)
    }
}

impl<'a,
        F,
        M,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
    > embedded_graphics::draw_target::DrawTarget
    for TiledFrameBuffer<'a, F, M, PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
where
    F: embedded_graphics::draw_target::DrawTarget<Color = Color, Error = Infallible>,
    M: PixelRemapper<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>,
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

impl<'a,
        F,
        M,
        const PANEL_ROWS: usize,
        const PANEL_COLS: usize,
        const TILE_ROWS: usize,
        const TILE_COLS: usize,
    > embedded_graphics::prelude::OriginDimensions
    for TiledFrameBuffer<'a, F, M, PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>
where
    M: PixelRemapper<PANEL_ROWS, PANEL_COLS, TILE_ROWS, TILE_COLS>,
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        embedded_graphics::prelude::Size::new(
            M::virtual_size().1 as u32,
            M::virtual_size().0 as u32,
        )
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
        assert_eq!(virt_size, (ROWS_IN_PANEL*3, COLS_IN_PANEL*3));
    }

    #[test]
    fn test_virtual_size_function_with_uneven_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 5, 3>;
        let virt_size = PanelChain::virtual_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL*5, COLS_IN_PANEL*3));
    }

    #[test]
    fn test_virtual_size_function_with_single_column() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 1>;
        let virt_size = PanelChain::virtual_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL*3, COLS_IN_PANEL));
    }

    #[test]
    fn test_fb_size_function_with_equal_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 3>;
        let virt_size = PanelChain::fb_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL, COLS_IN_PANEL*9));
    }

    #[test]
    fn test_fb_size_function_with_uneven_rows_and_cols() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 5, 3>;
        let virt_size = PanelChain::fb_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL, COLS_IN_PANEL*15));
    }

    #[test]
    fn test_fb_size_function_with_single_column() {
        const ROWS_IN_PANEL: usize = 32;
        const COLS_IN_PANEL: usize = 64;
        type PanelChain = ChainTopRightDown<ROWS_IN_PANEL, COLS_IN_PANEL, 3, 1>;
        let virt_size = PanelChain::fb_size();
        assert_eq!(virt_size, (ROWS_IN_PANEL, COLS_IN_PANEL*3));
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
}