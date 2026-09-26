//! Compile-time mapping of the HUB75 signals onto the bits of the 16-bit
//! plain (`u16`) DMA word.
//!
//! The [`plain`](crate::plain) and [`bitplane::plain`](crate::bitplane::plain)
//! framebuffers store one `u16` per pixel-clock cycle, in which every HUB75
//! signal occupies a fixed bit. Which bit depends on the board — that is, on
//! the peripheral that clocks the buffer out — so the layout is selected by
//! Cargo feature. `interstate75` selects the Pimoroni Interstate 75 / 75 W
//! (RP2040 / RP2350) layout, which matches the board's contiguous GPIO
//! mapping and therefore lets a PIO `out pins, 16` stream drive the panel
//! with no run-time bit shuffling.
//!
//! ```text
//! default (ESP32 I2S / generic)          interstate75 (RP2040/RP2350 PIO)
//! 15 .. 13  spare (dummy2, B1, G1)       15 .. 14  spare
//!     12    R1      Red   – lower half       13    OE    Output-Enable / Blank
//!     11    B0      Blue  – upper half       12    LAT   Latch / STB
//!     10    G0      Green – upper half       11    CLK   (reserved; PIO side-set)
//!      9    R0      Red   – upper half    10..6    E D C B A   Row address
//!      8    OE      Output-Enable / Blank  5..3    B1 G1 R1    lower half
//!      7    spare                          2..0    B0 G0 R0    upper half
//!      6    spare
//!      5    LAT     Latch / STB
//!    4..0   A..E     Row address
//! ```
//!
//! Every item here is a `const`, so the selected layout is folded away during
//! compilation: the generated code is identical to using hand-written bit
//! literals, with no runtime branch and no lookup table.

// ---------------------------------------------------------------------------
// Layout selection: per-board signal bit positions
// ---------------------------------------------------------------------------
//
// The default layout matches any parallel-output peripheral whose wiring
// follows the historical HUB75 bit order used by the ESP32 I2S/PARLIO drivers.
// The `interstate75` feature selects the Pimoroni Interstate 75 / 75 W
// (RP2040 / RP2350) layout, which follows the board's contiguous GPIO mapping
// (`GPIO0..GPIO13`) so a PIO `out pins, 16` stream can drive the panel with no
// run-time bit shuffling:
//
//   default (ESP32 I2S / generic)          interstate75 (RP2040/RP2350 PIO)
//   15 .. 13  spare (dummy2, B1, G1)       15 .. 14  spare
//       12    R1      Red   - lower half       13    OE    Output-Enable / Blank
//       11    B0      Blue  - upper half       12    LAT   Latch / STB
//       10    G0      Green - upper half       11    CLK   (reserved; PIO side-set)
//        9    R0      Red   - upper half    10..6    E D C B A   Row address
//        8    OE      Output-Enable / Blank  5..3    B1 G1 R1    lower half
//        7    spare                          2..0    B0 G0 R0    upper half
//        6    spare
//        5    LAT     Latch / STB
//      4..0   A..E     Row address
//
// Every item is a `const`, so the selected layout is folded away during
// compilation: the generated code is identical to hand-written bit literals,
// with no runtime branch and no lookup table. `cfg_select!` keeps a single
// definition per signal no matter how many boards are supported.

// Shift of the `R0/G0/B0` (upper-half) colour triplet.
pub(crate) const COLOR0_SHIFT: u32 = core::cfg_select! {
    feature = "interstate75" => 0,
    _ => 9,
};

// Shift of the `R1/G1/B1` (lower-half) colour triplet.
pub(crate) const COLOR1_SHIFT: u32 = core::cfg_select! {
    feature = "interstate75" => 3,
    _ => 12,
};

// Shift of the 5-bit row address (`A..E`).
pub(crate) const ADDR_SHIFT: u32 = core::cfg_select! {
    feature = "interstate75" => 6,
    _ => 0,
};

// Single-bit mask of the `LATCH` / `STB` signal.
pub(crate) const LATCH_BIT: u16 = core::cfg_select! {
    feature = "interstate75" => 1 << 12,
    _ => 1 << 5,
};

// Single-bit mask of the output-enable signal.
pub(crate) const OE_BIT: u16 = core::cfg_select! {
    feature = "interstate75" => 1 << 13,
    _ => 1 << 8,
};

// ---------------------------------------------------------------------------
// Derived masks and levels
// ---------------------------------------------------------------------------

/// Mask covering the `R0/G0/B0` (upper-half) colour bits.
pub(crate) const COLOR0_MASK: u16 = 0b111 << COLOR0_SHIFT;
/// Mask covering the `R1/G1/B1` (lower-half) colour bits.
pub(crate) const COLOR1_MASK: u16 = 0b111 << COLOR1_SHIFT;
/// Mask covering every colour bit (`erase()` / `clear_colors()`).
pub(crate) const COLOR_MASK: u16 = COLOR0_MASK | COLOR1_MASK;

/// Every row-address bit set: the idle/sentinel address parked in the trailing
/// word so the final latch does not select the first row again.
#[cfg(feature = "tail-closes-latch")]
pub(crate) const ADDR_MASK: u16 = 0x1f << ADDR_SHIFT;

// The `OE` *level* constants (`OE_ACTIVE` / `OE_BLANK`) deliberately stay local
// to each framebuffer module: the deprecated `plain` framebuffer and
// `bitplane::plain` use opposite default polarities (see `plain.rs` and
// `bitplane/plain/mod.rs`). `pinmap` only owns the bit *position* (`OE_BIT`),
// so relocating `OE` for a board cannot silently flip either polarity.
