//! Compile-time mapping of the HUB75 signals onto the bits of the DMA word.
//!
//! The [`plain`](crate::plain) and [`bitplane::plain`](crate::bitplane::plain)
//! framebuffers store one 16-bit word per pixel-clock cycle, while
//! [`latched`](crate::latched) and [`bitplane::latched`](crate::bitplane::latched)
//! store 8-bit words. In every case each HUB75 signal occupies a fixed bit, and
//! *which* bit depends on the board (for the 16-bit layouts) or on the external
//! latch circuit (for the 8-bit layouts). This module is the single source of
//! truth for those positions.
//!
//! ## The layouts
//!
//! ```text
//! 16-bit plain / bitplane::plain
//! default (ESP32 I2S / generic)          interstate75 (RP2040/RP2350 PIO)
//! 15 .. 13  spare (dummy2, B1, G1)       15 .. 14  spare
//!     12    R1      Red   - lower half       13    OE    Output-Enable / Blank
//!     11    B0      Blue  - upper half       12    LAT   Latch / STB
//!     10    G0      Green - upper half       11    spare (unused)
//!      9    R0      Red   - upper half    10..6    E D C B A   Row address
//!      8    OE      Output-Enable / Blank  5..3    B1 G1 R1    lower half
//!      7    spare                          2..0    B0 G0 R0    upper half
//!      6    spare
//!      5    LAT     Latch / STB
//!    4..0   A..E     Row address
//!
//! 8-bit latched (fixed by the external latch circuit)
//!    7    OE   Output-Enable / Blank
//!    6    LAT  Latch / STB
//!    5    B2   Blue  - sub-pixel group 2
//!    4    G2   Green - sub-pixel group 2
//!    3    R2   Red   - sub-pixel group 2
//!    2    B1   Blue  - sub-pixel group 1
//!    1    G1   Green - sub-pixel group 1
//!    0    R1   Red   - sub-pixel group 1
//! ```
//!
//! Everything is a `const`, so the layout is folded away during compilation:
//! the generated code is identical to hand-written bit literals, with no
//! runtime branch and no lookup table. [`PINMAP`](crate::pinmap::PINMAP) and
//! [`LATCHED_PINMAP`](crate::pinmap::LATCHED_PINMAP) are
//! exported so a calling crate (the driver that owns the DMA/PIO peripheral)
//! can validate at compile time that it drives the panel with the same bit
//! assignments this crate assumes — e.g.
//!
//! ```
//! // The caller pins its own board definition to this crate's assumptions.
//! # #[cfg(not(feature = "interstate75"))]
//! const _: () = assert!(hub75_framebuffer::pinmap::PINMAP.oe == 8);
//! ```

// ---------------------------------------------------------------------------
// The signal -> bit table
// ---------------------------------------------------------------------------

/// Bit position of every HUB75 signal within one framebuffer word.
///
/// Naming note: `color0_*` (the `red0`/`grn0`/`blu0` fields) carry the
/// **upper-half** `R0/G0/B0` triplet and drive the `red1`/`grn1`/`blu1`
/// accessors, while `color1_*` (`red1`/`grn1`/`blu1` fields) carry the
/// **lower-half** `R1/G1/B1` triplet and drive `red2`/`grn2`/`blu2`.
///
/// The three colour signals of a triplet are assumed to be contiguous with red
/// as the least-significant bit (true of every supported layout); the exact
/// per-channel bits are recorded anyway so a driver can verify the wiring.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PinMap {
    /// Bit of the upper-half red signal (`R0`, `red1` accessor).
    pub red0: usize,
    /// Bit of the upper-half green signal (`G0`, `grn1` accessor).
    pub grn0: usize,
    /// Bit of the upper-half blue signal (`B0`, `blu1` accessor).
    pub blu0: usize,
    /// Bit of the lower-half red signal (`R1`, `red2` accessor).
    pub red1: usize,
    /// Bit of the lower-half green signal (`G1`, `grn2` accessor).
    pub grn1: usize,
    /// Bit of the lower-half blue signal (`B1`, `blu2` accessor).
    pub blu1: usize,
    /// Least-significant bit of the row-address field (`A`).
    pub addr_lsb: usize,
    /// Width of the row-address field in bits (`A..E` = 5).
    pub addr_bits: usize,
    /// Bit of the latch / strobe (`LAT`) signal.
    pub latch: usize,
    /// Bit of the output-enable (`OE`) signal.
    pub oe: usize,
    /// The unused ("spare") bits in the word. The 16-bit layouts expose these
    /// through the `dummy0`/`dummy1`/`dummy2` accessors; set to [`usize::MAX`]
    /// when unused.
    pub spare: [usize; 3],
}

impl PinMap {
    /// Mask covering the output-enable bit.
    #[must_use]
    #[inline]
    pub const fn oe_mask(&self) -> u16 {
        1u16 << self.oe
    }

    /// Mask covering the latch bit.
    #[must_use]
    #[inline]
    pub const fn latch_mask(&self) -> u16 {
        1u16 << self.latch
    }

    /// Mask covering the upper-half `R0/G0/B0` colour bits.
    #[must_use]
    #[inline]
    pub const fn color0_mask(&self) -> u16 {
        (1u16 << self.red0) | (1u16 << self.grn0) | (1u16 << self.blu0)
    }

    /// Mask covering the lower-half `R1/G1/B1` colour bits.
    #[must_use]
    #[inline]
    pub const fn color1_mask(&self) -> u16 {
        (1u16 << self.red1) | (1u16 << self.grn1) | (1u16 << self.blu1)
    }

    /// Mask covering every colour bit.
    #[must_use]
    #[inline]
    pub const fn color_mask(&self) -> u16 {
        self.color0_mask() | self.color1_mask()
    }

    /// Mask covering the whole row-address field.
    #[must_use]
    #[inline]
    pub const fn addr_mask(&self) -> u16 {
        (((1u32 << self.addr_bits) - 1) as u16) << self.addr_lsb
    }

    /// Most-significant bit of the row-address field.
    #[must_use]
    #[inline]
    pub const fn addr_msb(&self) -> usize {
        self.addr_lsb + self.addr_bits - 1
    }
}

/// Layout for the 16-bit [`plain`](crate::plain) and
/// [`bitplane::plain`](crate::bitplane::plain) framebuffers, selected per board.
///
/// The default matches any parallel-output peripheral wired in the historical
/// HUB75 bit order used by the ESP32 I2S/PARLIO drivers; the `interstate75`
/// feature selects the Pimoroni Interstate 75 / 75 W (RP2040 / RP2350) layout,
/// which follows the board's contiguous `GPIO0..GPIO13` mapping so a PIO
/// `out pins, 16` stream can drive the panel with no run-time bit shuffling.
pub const PINMAP: PinMap = core::cfg_select! {
    feature = "interstate75" => PinMap {
        red0: 0,
        grn0: 1,
        blu0: 2,
        red1: 3,
        grn1: 4,
        blu1: 5,
        addr_lsb: 6,
        addr_bits: 5,
        latch: 12,
        oe: 13,
        // spare[0] is the unused bit 11; spare[1]/spare[2] are the top two bits.
        spare: [11, 14, 15],
    },
    _ => PinMap {
        red0: 9,
        grn0: 10,
        blu0: 11,
        red1: 12,
        grn1: 13,
        blu1: 14,
        addr_lsb: 0,
        addr_bits: 5,
        latch: 5,
        oe: 8,
        spare: [6, 7, 15],
    },
};

/// Layout for the 8-bit [`latched`](crate::latched) and
/// [`bitplane::latched`](crate::bitplane::latched) framebuffers.
///
/// This layout is fixed by the external latch circuit, not by the board, so it
/// is not gated by a Cargo feature. It is exported for validation and to keep
/// every signal position in one place.
pub const LATCHED_PINMAP: PinMap = PinMap {
    red0: 0,
    grn0: 1,
    blu0: 2,
    red1: 3,
    grn1: 4,
    blu1: 5,
    addr_lsb: 0,
    addr_bits: 5,
    latch: 6,
    oe: 7,
    // Bit 5 is the only unused slot; the dummy accessors are not used here.
    spare: [usize::MAX; 3],
};

// ---------------------------------------------------------------------------
// 16-bit layout: derived consts
// ---------------------------------------------------------------------------

/// Shift of the upper-half `R0/G0/B0` colour triplet.
pub(crate) const COLOR0_SHIFT: u32 = PINMAP.red0 as u32;
/// Shift of the lower-half `R1/G1/B1` colour triplet.
pub(crate) const COLOR1_SHIFT: u32 = PINMAP.red1 as u32;
/// Shift of the 5-bit row address (`A..E`).
pub(crate) const ADDR_SHIFT: u32 = PINMAP.addr_lsb as u32;

/// Mask covering the `R0/G0/B0` (upper-half) colour bits.
pub(crate) const COLOR0_MASK: u16 = PINMAP.color0_mask();
/// Mask covering the `R1/G1/B1` (lower-half) colour bits.
pub(crate) const COLOR1_MASK: u16 = PINMAP.color1_mask();
/// Mask covering every colour bit (`erase()` / `clear_colors()`).
pub(crate) const COLOR_MASK: u16 = PINMAP.color_mask();

/// Single-bit mask of the `LATCH` / `STB` signal.
pub(crate) const LATCH_BIT: u16 = PINMAP.latch_mask();
/// Single-bit mask of the output-enable signal.
pub(crate) const OE_BIT: u16 = PINMAP.oe_mask();

/// Every row-address bit set: the idle/sentinel address parked in the trailing
/// word so the final latch does not select the first row again.
#[cfg(feature = "tail-closes-latch")]
pub(crate) const ADDR_MASK: u16 = PINMAP.addr_mask();

// ---------------------------------------------------------------------------
// 8-bit latched layout: derived consts
// ---------------------------------------------------------------------------

/// Single-bit mask of the latched layout's output-enable signal (bit position
/// only; the active/blank *levels* stay local to each module).
pub(crate) const LATCHED_OE_BIT: u8 = LATCHED_PINMAP.oe_mask() as u8;
/// Single-bit mask of the latched layout's latch signal.
pub(crate) const LATCHED_LATCH_BIT: u8 = LATCHED_PINMAP.latch_mask() as u8;
/// Mask covering the `R1/G1/B1` colour bits.
pub(crate) const LATCHED_COLOR0_MASK: u8 = LATCHED_PINMAP.color0_mask() as u8;
/// Mask covering the `R2/G2/B2` colour bits.
pub(crate) const LATCHED_COLOR1_MASK: u8 = LATCHED_PINMAP.color1_mask() as u8;
/// Mask covering every colour bit.
pub(crate) const LATCHED_COLOR_MASK: u8 = LATCHED_PINMAP.color_mask() as u8;

// The `OE` *level* constants (`OE_ACTIVE` / `OE_BLANK`) deliberately stay local
// to each framebuffer module: the deprecated `plain` framebuffer and
// `bitplane::plain` use opposite default polarities (see `plain.rs` and
// `bitplane/plain/mod.rs`), and the latched modules use a third fixed layout.
// `pinmap` only owns the bit *position* (`OE_BIT` / `LATCHED_OE_BIT`), so
// relocating `OE` for a board cannot silently flip any polarity.

// ---------------------------------------------------------------------------
// Compile-time sanity checks
// ---------------------------------------------------------------------------

const _: () = {
    assert!(PINMAP.addr_lsb + PINMAP.addr_bits <= 16);
    assert!(PINMAP.oe < 16);
    assert!(PINMAP.latch < 16);

    assert!(LATCHED_PINMAP.addr_lsb + LATCHED_PINMAP.addr_bits <= 8);
    assert!(LATCHED_PINMAP.oe < 8);
    assert!(LATCHED_PINMAP.latch < 8);
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn masks_match_positions() {
        for pm in [PINMAP, LATCHED_PINMAP] {
            assert_eq!(pm.oe_mask(), 1u16 << pm.oe);
            assert_eq!(pm.latch_mask(), 1u16 << pm.latch);
            assert_eq!(
                pm.color0_mask(),
                (1u16 << pm.red0) | (1u16 << pm.grn0) | (1u16 << pm.blu0)
            );
            assert_eq!(
                pm.color1_mask(),
                (1u16 << pm.red1) | (1u16 << pm.grn1) | (1u16 << pm.blu1)
            );
            assert_eq!(pm.color_mask(), pm.color0_mask() | pm.color1_mask());
            assert_eq!(pm.addr_mask(), ((1u16 << pm.addr_bits) - 1) << pm.addr_lsb);
            assert_eq!(pm.addr_msb(), pm.addr_lsb + pm.addr_bits - 1);
        }
    }
}
