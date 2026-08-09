//! Bitplane framebuffers for an 8-bit latched HUB75 interface.
//!
//! These framebuffers store colour data as separate bit-planes rather than
//! the threshold-based frames used by [`crate::latched::DmaFrameBuffer`].
//! Each plane holds one bit of every colour channel, giving `PLANES` planes
//! total (typically 8 for full 8-bit colour). Row addressing is carried by
//! four trailing `Address` bytes per row, identical to the non-bitplane
//! latched layout.
//!
//! Two layout variants are provided:
//!
//! - [`frame`] -- Plane-major: all rows for one bit-plane stored contiguously.
//!   The DMA chain sends each plane's data separately.
//! - [`row`] -- Row-major: all bit-planes for a single row stored contiguously.
//!   Optimized for row-by-row BCM where the inter-row gap appears only once
//!   per scan row instead of once per plane.
//!
//! Both variants share the same 8-bit `Entry`/`Address` formats and pin
//! mapping.
//!
//! # Hardware Requirements
//! Requires a parallel output peripheral capable of clocking 8 bits at a time,
//! plus an external latch circuit to hold the row address and gate the pixel
//! clock (same circuit as the non-bitplane latched variant).
//!
//! # HUB75 Signal Bit Mapping (8-bit words)
//! Two distinct 8-bit words are streamed to the panel:
//!
//! 1. **Address / Timing (`Address`)** -- row-select and latch control.
//! 2. **Pixel Data (`Entry`)** -- RGB bits for two sub-pixels plus OE/LAT
//!    shadow bits.
//!
//! ```text
//! Address word (row select & timing)
//! ┌──7─┬──6──┬─5─-┬─4─-┬─3-─┬─2-─┬─1-─┬─0-─┐
//! │ OE │ LAT │    │ E  │ D  │ C  │ B  │ A  │
//! └────┴─────┴───-┴───-┴───-┴───-┴───-┴───-┘
//! ```
//! ```text
//! Entry word (pixel data)
//! ┌──7─┬──6──┬─5──┬─4──┬─3──┬─2──┬─1──┬─0──┐
//! │ OE │ LAT │ B2 │ G2 │ R2 │ B1 │ G1 │ R1 │
//! └────┴─────┴────┴────┴────┴────┴────┴────┘
//! ```
//!
//! Bits 7-6 (OE/LAT) occupy the same positions in both words so the control
//! lines stay valid throughout the DMA stream.
//!
//! # Bitplane BCM Rendering
//! Each framebuffer is organised into `PLANES` bit-planes, one per colour
//! bit. To produce correct brightness via Binary Code Modulation, configure
//! the DMA descriptor chain so that each plane's data is output (scanned) a
//! number of times equal to its bit-weight (2^7 for the MSB plane down to
//! 2^0 for the LSB plane).
//!
//! Both layout variants store planes **LSB-first**: plane 0 carries bit 0
//! and is displayed once per scan, plane 7 carries bit 7 and is displayed
//! 128 times. Each BCM segment streams a contiguous *suffix* of planes,
//! repeated just enough times to bring each plane's total coverage to its
//! bit-weight — halving the number of DMA transfers per scan with identical
//! brightness. LSB-first ordering also means the first plane after an
//! address change displays stale data with minimal visual weight, avoiding
//! a separate primer segment. See the [`frame`] and [`row`] module
//! documentation for the exact scan order.
//!
//! See <https://www.batsocks.co.uk/readme/art_bcm_1.htm> for background on
//! BCM.
//!
//! # Memory Usage
//! Memory scales linearly with `PLANES`: unlike the threshold-based
//! [`crate::latched::DmaFrameBuffer`] whose frame count grows as
//! `2^BITS - 1`, these layouts use exactly `PLANES` planes regardless of
//! colour depth.
//!
//! - [`frame`]: `PLANES × NROWS × (COLS + 4 + INTER_ROW_BLANK)` bytes.
//! - [`row`]: `NROWS × (PLANES × (COLS + 4) + INTER_ROW_BLANK)` bytes.

use bitfield::bitfield;

pub mod frame;
pub mod row;

pub use frame::DmaFrameBuffer;

pub(crate) use crate::{INTER_ROW_BLANK, LEAD_BLANK_DELAY, TRAIL_BLANK_DELAY};

#[cfg(not(feature = "invert-oe"))]
pub(crate) const OE_ACTIVE: u8 = 0b1000_0000;
#[cfg(not(feature = "invert-oe"))]
pub(crate) const OE_BLANK: u8 = 0;

#[cfg(feature = "invert-oe")]
pub(crate) const OE_ACTIVE: u8 = 0;
#[cfg(feature = "invert-oe")]
pub(crate) const OE_BLANK: u8 = 0b1000_0000;

// ---------------------------------------------------------------------------
// Address / Entry — the 8-bit words that ride on the HUB75 bus
// ---------------------------------------------------------------------------

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq, Eq)]
    #[repr(transparent)]
    pub(crate) struct Address(u8);
    impl Debug;
    pub(crate) output_enable, set_output_enable: 7;
    pub(crate) latch, set_latch: 6;
    pub(crate) addr, set_addr: 4, 0;
}

impl Address {
    pub const fn new() -> Self {
        Self(0)
    }
}

bitfield! {
    #[derive(Clone, Copy, Default, PartialEq)]
    #[repr(transparent)]
    pub(crate) struct Entry(u8);
    impl Debug;
    pub(crate) output_enable, set_output_enable: 7;
    pub(crate) latch, set_latch: 6;
    pub(crate) blu2, set_blu2: 5;
    pub(crate) grn2, set_grn2: 4;
    pub(crate) red2, set_red2: 3;
    pub(crate) blu1, set_blu1: 2;
    pub(crate) grn1, set_grn1: 1;
    pub(crate) red1, set_red1: 0;
}

impl Entry {
    pub const fn new() -> Self {
        Self(0)
    }

    const COLOR0_MASK: u8 = 0b0000_0111;
    const COLOR1_MASK: u8 = 0b0011_1000;

    #[inline]
    fn set_color0_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::COLOR0_MASK) | (bits & Self::COLOR0_MASK);
    }

    #[inline]
    fn set_color1_bits(&mut self, bits: u8) {
        self.0 = (self.0 & !Self::COLOR1_MASK) | ((bits << 3) & Self::COLOR1_MASK);
    }
}

// ---------------------------------------------------------------------------
// Shared helper functions
// ---------------------------------------------------------------------------

#[inline]
pub(crate) const fn map_index(index: usize) -> usize {
    #[cfg(feature = "esp32-ordering")]
    {
        index ^ 2
    }
    #[cfg(not(feature = "esp32-ordering"))]
    {
        index
    }
}

pub(crate) const fn make_addr_table() -> [[Address; 4]; 32] {
    let mut tbl = [[Address::new(); 4]; 32];
    let mut addr = 0;
    while addr < 32 {
        tbl[addr][map_index(0)].0 = OE_BLANK | 1u8 << 6 | addr as u8;
        tbl[addr][map_index(1)].0 = OE_BLANK | 1u8 << 6 | addr as u8;
        tbl[addr][map_index(2)].0 = OE_BLANK | addr as u8;
        tbl[addr][map_index(3)].0 = OE_BLANK;
        addr += 1;
    }
    tbl
}

pub(crate) const ADDR_TABLE: [[Address; 4]; 32] = make_addr_table();

#[allow(clippy::absurd_extreme_comparisons)]
pub(crate) const fn make_data_template<const COLS: usize>() -> [Entry; COLS] {
    let mut data = [Entry::new(); COLS];
    let mut i = 0;
    while i < COLS {
        let mapped_i = map_index(i);
        data[mapped_i].0 =
            if i >= TRAIL_BLANK_DELAY && i < COLS.saturating_sub(LEAD_BLANK_DELAY + 1) {
                OE_ACTIVE
            } else {
                OE_BLANK
            };
        i += 1;
    }
    data
}
