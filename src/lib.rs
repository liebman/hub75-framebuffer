//! Framebuffer implementation for HUB75 LED matrix displays.
//!
//! ## How HUB75 LED Displays Work
//!
//! HUB75 RGB LED matrix panels are scanned, time-multiplexed displays that behave like a long
//! daisy-chained shift register rather than a random-access framebuffer.
//!
//! ### Signal names
//! - **R1 G1 B1 / R2 G2 B2** – Serial colour data for the upper and lower halves of the active scan line
//! - **CLK** – Shift-register clock; every rising edge pushes the six colour bits one pixel to the right
//! - **LAT / STB** – Latch; copies the shift-register contents to the LED drivers for the row currently selected by the address lines
//! - **OE** – Output-Enable (active LOW): LEDs are lit while OE is LOW and blanked when it is HIGH
//! - **A B C D (E)** – Row-address select lines (choose which pair of rows is lit)
//! - **VCC & GND** – 5 V power for panel logic and LED drivers
//!
//! ### Row-pair scanning workflow (e.g., 1/16-scan panel)
//! 1. While the panel is still displaying row pair N − 1, the controller shifts the six-bit colour data for row pair N into the chain (OE remains LOW so row N − 1 stays visible).
//! 2. After the last pixel is clocked in, the controller raises OE HIGH to blank the LEDs.
//! 3. With the panel blanked, it first changes the address lines to select row pair N, lets them settle for a few nanoseconds, and **then** pulses LAT to latch the freshly shifted data into the output drivers for that newly selected row.
//! 4. OE is immediately driven LOW again, lighting row pair N.
//! 5. Steps 1–4 repeat for every row pair fast enough (hundreds of Hz) that the human eye sees a steady image.
//!    - If the first row pair is being shifted, the panel continues showing the last row pair of the previous frame until the first blank-address-latch sequence occurs.
//!
//! ### Brightness and colour depth (Binary Code Modulation)
//! - Full colour is typically achieved using **Binary Code Modulation (BCM)**, also known as *Bit-Angle Modulation (BAM)*. Each bit-plane is displayed for a duration proportional to its binary weight (1, 2, 4, 8 …), yielding 2ⁿ intensity levels per channel. See [Batsocks – LED dimming using Binary Code Modulation](https://www.batsocks.co.uk/readme/art_bcm_1.htm) for a deeper explanation.
//! - Because each LED is on for only a fraction of the total frame time, the driver can use relatively high peak currents without overheating while average brightness is preserved.
//!
//! ### Implications for software / hardware drivers
//! - You don't simply "write a pixel" once; you must continuously stream the complete refresh data at MHz-range clock rates.
//! - Precise timing of CLK, OE, address lines, and LAT is critical—especially the order: blank (OE HIGH) → set address → latch → un-blank (OE LOW).
//! - Microcontrollers typically employ DMA, PIO, or parallel GPIO tricks, and FPGAs use dedicated logic, to sustain the data throughput while leaving processing resources free.
//!
//! In short: a HUB75 panel is a high-speed shift-register chain that relies on rapid row-pair scanning and **Binary Code Modulation (BCM)** to create a bright, full-colour image. Keeping OE LOW almost all the time—blanking only long enough to change the address and pulse LAT—maximises brightness without visible artefacts.
//!
//! ## Framebuffer Implementations
//!
//! Four framebuffer layouts are provided, covering two hardware variants and
//! two BCM strategies:
//!
//! | Module | Word size | External latch? | BCM strategy |
//! |--------|-----------|-----------------|--------------|
//! | [`bitplane::plain`] | 16-bit | No | True bitplane |
//! | [`bitplane::latched`] | 8-bit | Yes | True bitplane |
//! | [`plain`] *(deprecated)* | 16-bit | No | Threshold frames |
//! | [`latched`] *(deprecated)* | 8-bit | Yes | Threshold frames |
//!
//! ### Plain vs. Latched
//! - **Plain** packs all HUB75 signals (address, latch, OE, colour) into each
//!   16-bit word. No extra hardware beyond a parallel output peripheral.
//! - **Latched** uses 8-bit words with a separate external latch circuit to
//!   hold the row address and gate the pixel clock, halving per-entry memory.
//!
//! ### Threshold Frames vs. True Bitplane
//! The two BCM strategies differ in how they store colour data and how the
//! DMA stream must be driven to render it. **The bitplane layouts are
//! recommended for new designs**; the threshold layouts are deprecated.
//!
//! **True bitplane** (`bitplane::plain`, `bitplane::latched`) -- each of
//! `PLANES` planes (typically 8) stores one bit of every colour channel
//! directly. To render, stream each plane's data a number of times equal to
//! its bit-weight (the MSB plane 128 times, the LSB plane once), either with
//! a DMA descriptor chain or by reprogramming the DMA registers per segment.
//! Memory scales linearly with the number of planes. Each module offers a
//! plane-major (`frame`) and a row-major (`row`) layout — both LSB-first
//! with suffix-coalesced BCM segments; see their documentation for the exact
//! scan order.
//!
//! **Threshold frames** (`plain`, `latched`) -- **deprecated.** The driver
//! compares each channel's 8-bit value against per-frame thresholds and
//! stores the resulting on/off bits. For a colour depth of `BITS`, this
//! produces `2^BITS - 1` frames. Frame *n* is displayed for a duration
//! proportional to `2^n`. Memory grows exponentially with colour depth, but
//! the whole refresh is one contiguous buffer that even interrupt-less
//! circular DMA can stream.
//!
//! All four variants have configurable row and column dimensions, support
//! `embedded-graphics` via the `DrawTarget` trait, and expose their BCM
//! scan sequence through the [`FrameBuffer`] trait.
//!
//! ## Driving the Panel (DMA Integration)
//!
//! This crate is **buffer-only**: it builds the memory image and describes
//! exactly what to stream via the [`FrameBuffer`] trait — writing the
//! peripheral driver (the refresh loop) is up to you. A driver only needs
//! two things from a framebuffer:
//!
//! - [`FrameBuffer::Word`] — the stream element type: `u16` for the `plain`
//!   variants, `u8` for the `latched` ones. The peripheral must output this
//!   many bits in parallel and generates the pixel clock (`CLK`) itself.
//! - [`FrameBuffer::BCM_SEQUENCE`] and [`FrameBuffer::bcm_segment`] — the
//!   ordered `(ptr, len, reps)` segments that make up one complete panel
//!   refresh.
//!
//! The framebuffer must live in DMA-capable memory and outlive the transfer:
//! place it in a `static` (or a DMA RAM section on chips that require one)
//! and apply whatever alignment and cache maintenance your MCU needs.
//!
//! ### DMA with descriptor chains (least CPU)
//!
//! The bitplane stream is non-uniform: each segment must be output `reps`
//! times (the MSB plane up to `2^(PLANES-2)` times). This maps naturally
//! onto DMA descriptor chains (e.g. ESP32 GPDMA linked-list mode) — use
//! [`FrameBuffer::BCM_SEQUENCE`] / [`bcm_rep_count`] to size the descriptor
//! table at compile time and [`FrameBuffer::bcm_segment`] to populate it; a
//! segment whose `len` exceeds the DMA's maximum transfer size is simply
//! split across several descriptors. Link the chain into a loop and the
//! panel refreshes forever with zero CPU involvement. For free-running
//! loops, consider the `tail-closes-latch` feature so `LAT` is not left
//! asserted when the chain wraps.
//!
//! ### Simple DMA (registers only, no descriptors)
//!
//! If your DMA is just *source address + length* registers with a
//! transfer-complete interrupt, walk the segments in the ISR: on each
//! interrupt, program the address/length registers from
//! [`FrameBuffer::bcm_segment`], re-arm the transfer, and re-issue the same
//! segment until its `reps` count is exhausted before advancing. The cadence
//! is modest — the plane-major (`frame`) layouts raise only `PLANES`
//! interrupts per panel refresh (8 at full color depth); the row-major
//! (`row`) layouts raise one per segment of every row
//! ([`FrameBuffer::BCM_SEGMENT_COUNT`] per refresh).
//!
//! If the DMA cannot interrupt at all — truly circular-only hardware — the
//! deprecated threshold framebuffers ([`plain`], [`latched`]) remain an
//! option: their entire refresh is one contiguous buffer
//! ([`FrameBuffer::BCM_SEQUENCE_LEN`] is `1`, `reps == 1`) with all BCM
//! timing baked into the memory image, at an exponential memory cost.
//!
//! ## Multiple Panels / Scan-Pattern Remapping
//! Use [`tiling::RemappedFrameBuffer`] to drive several HUB75 panels as one
//! large virtual display, or to remap pixels for non-standard scan patterns
//! (e.g. 1/16-scan on 64×64 panels). It works with all four framebuffer types
//! and only requires two generic parameters (`F` and `M`).
//!
//! Combine it with a [`tiling::PixelRemapper`] implementation such as
//! [`tiling::ChainTopRightDown`] (tiling) or [`tiling::QuarterScan`] (1/16-scan).
//! The wrapper exposes a single `embedded-graphics` canvas, so for example a
//! 3 × 3 stack of 64 × 32 panels simply looks like a 192 × 96 screen while
//! all coordinate translation happens transparently.
//!
//! The older [`tiling::TiledFrameBuffer`] is still available but deprecated.
//!
//! ## Available Feature Flags
//!
//! ### `skip-black-pixels` Feature (disabled by default)
//! When enabled, calls to `set_pixel()` with `Color::BLACK` return early without
//! writing to the framebuffer. This provides a significant performance boost for
//! UI applications that frequently draw black pixels (backgrounds, clearing, etc.)
//! by assuming the framebuffer was already cleared.
//!
//! **Important**: This optimization assumes that black pixels represent "no change"
//! rather than "explicitly set to black". By default, black pixels are written
//! normally to ensure correct overwrite behavior. To enable the optimization:
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.11.0", features = ["skip-black-pixels"] }
//! ```
//!
//! ### `esp32-ordering` Feature (required for original ESP32 only)
//! **Required** when targeting the original ESP32 chip (not ESP32-S3 or other variants).
//! This feature adjusts byte ordering to accommodate the quirky requirements of the
//! ESP32's I²S peripheral in 8-bit and 16-bit modes. The original ESP32 has different
//! byte ordering requirements compared to other ESP32 variants (S2, S3, C3, etc.),
//! which do **not** need this feature.
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.11.0", features = ["esp32-ordering"] }
//! ```
//!
//! ### `tail-closes-latch` Feature (plain framebuffers only)
//! Appends a single extra "tail" word at the end of the DMA buffer that drives the
//! LATCH signal LOW (de-asserted) on the final clock edge. Without this feature the
//! last word in each row asserts LATCH HIGH to latch shifted data into the LED
//! drivers, and the GPIO pins remain in that state after the DMA transfer completes.
//! Some hardware configurations (e.g. free-running DMA loops or peripherals that
//! continue clocking after the descriptor chain ends) can re-latch stale data or
//! glitch if LATCH is left asserted.
//!
//! Enabling `tail-closes-latch` adds one 16-bit `Entry` (for `plain`) or one entry
//! per bit-plane (for `bitplane::plain`) that parks the bus with LATCH=0 and
//! OE=BLANK, cleanly terminating the transfer. The cost is a single extra word per
//! DMA chunk, which is negligible compared to the frame data.
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.11.0", features = ["tail-closes-latch"] }
//! ```
//!
//! ### `reverse-row-order` Feature (disabled by default)
//! Stores the rows of every framebuffer layout in reverse scan order so that
//! the DMA stream renders the last panel row first and row 0 last. The rows
//! are physically reversed in the buffer (row addresses are written
//! back-to-front while keeping the deferred address-change timing intact) and
//! the pixel-setting paths map logical rows to the reversed slots, so logical
//! coordinates are unchanged — only the scan order changes.
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.11.0", features = ["reverse-row-order"] }
//! ```
//!
//! ### Blanking delay features (`lead-blank-*` / `trail-blank-*`)
//!
//! Control the number of pixel-clock cycles of blanking (`OE` HIGH) inserted
//! around row-address changes. The lead blank controls how many cycles the
//! output is blanked *before* the row address is changed, and the trail blank
//! controls blanking *after* the row address is changed. Together they give
//! the address lines time to settle and prevent ghosting or "bleeding"
//! artifacts caused by the panel briefly displaying data on the wrong row
//! during the transition.
//!
//! | Feature          | Blanking cycles | Position              |
//! |------------------|-----------------|-----------------------|
//! | *(none)*         | 1 (default)     | before & after change |
//! | `lead-blank-1`   | 1               | before address change |
//! | `lead-blank-2`   | 2               | before address change |
//! | `lead-blank-4`   | 4               | before address change |
//! | `lead-blank-8`   | 8               | before address change |
//! | `lead-blank-16`  | 16              | before address change |
//! | `trail-blank-1`  | 1               | after address change  |
//! | `trail-blank-2`  | 2               | after address change  |
//! | `trail-blank-4`  | 4               | after address change  |
//! | `trail-blank-8`  | 8               | after address change  |
//! | `trail-blank-16` | 16              | after address change  |
//!
//! Higher values reduce ghosting at the cost of slightly less brightness (the
//! LEDs are on for less time per scan line). Start with the default and increase
//! only if you observe row-transition artifacts on your particular panel
//! hardware.
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.11.0", features = ["lead-blank-4", "trail-blank-2"] }
//! ```
//!
//! **Note:** At most one `lead-blank-*` and one `trail-blank-*` feature may be
//! enabled at a time. If multiple are enabled for the same edge, compile-time cfg
//! conflicts will result.
//!
//! ### Inter-row blanking features (`inter-row-blank-*`)
//!
//! Insert additional dead clock cycles at the row transition, between the
//! latch and the address-line change. In plain framebuffers the gap entries
//! hold the previous row address with `OE` HIGH (blank), deferring the
//! address change to the first pixel after the gap and giving slow panels
//! more time to finish blanking before the address lines move. In latched
//! framebuffers the latch and address change are inseparable in hardware,
//! so the gap simply adds extra blanked cycles after the address change.
//! Row-major bitplane framebuffers stream the gap between plane 0 (shifted
//! out before the address change) and plane 1 (shifted out at/after it);
//! all other framebuffers place it at the end of each row, between that
//! row's latch and the next row's first pixel. The gap entries are
//! invisible to all drawing primitives — they only appear in the DMA
//! stream.
//!
//! | Feature              | Gap cycles | RAM cost per row       |
//! |----------------------|------------|------------------------|
//! | *(none)*             | 0          | 0 bytes                |
//! | `inter-row-blank-4`  | 4          | 8 bytes (16-bit) / 4 bytes (8-bit) |
//! | `inter-row-blank-8`  | 8          | 16 bytes / 8 bytes     |
//! | `inter-row-blank-16` | 16         | 32 bytes / 16 bytes    |
//! | `inter-row-blank-32` | 32         | 64 bytes / 32 bytes    |
//!
//! ```toml
//! [dependencies]
//! hub75-framebuffer = { version = "0.11.0", features = ["inter-row-blank-8"] }
//! ```
//!
//! **Note:** At most one `inter-row-blank-*` feature may be enabled at a time.
//! These are independent of the `lead-blank-*` / `trail-blank-*` features and
//! can be combined with them.
//!
//! ### `defmt` Feature
//! Implements `defmt::Format` for framebuffer types so they can be emitted with
//! the `defmt` logging framework. No functional changes; purely adds a trait impl.
//!
//! ### `doc-images` Feature
//! Embeds documentation images when building docs on docs.rs. Not needed for
//! normal usage.
#![no_std]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::Point;

pub mod bitplane;
pub mod latched;
pub mod plain;
pub mod tiling;

/// Color type used in the framebuffer
pub type Color = Rgb888;

/// Word size configuration for the framebuffer
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordSize {
    /// 8-bit word size
    Eight,
    /// 16-bit word size
    Sixteen,
}

/// Computes the NROWS value from ROWS for `DmaFrameBuffer`
///
/// # Arguments
///
/// * `rows` - Total number of rows in the display
///
/// # Returns
///
/// Number of rows needed internally for `DmaFrameBuffer`
#[must_use]
pub const fn compute_rows(rows: usize) -> usize {
    rows / 2
}

/// Computes the number of frames needed for a given bit depth
///
/// This is used to determine how many frames are needed to achieve
/// the desired color depth through Binary Code Modulation (BCM).
///
/// # Arguments
///
/// * `bits` - Number of bits per color channel
///
/// # Returns
///
/// Number of frames required for the given bit depth
#[must_use]
pub const fn compute_frame_count(bits: u8) -> usize {
    (1usize << bits) - 1
}

/// Maps a logical row-pair index to its physical memory slot.
///
/// With the `reverse-row-order` feature the rows are stored back-to-front so
/// that the DMA stream renders the last panel row first and row 0 last; the
/// pixel-setting paths use this to compensate, keeping logical coordinates
/// unchanged. Without the feature this is the identity mapping.
#[inline]
pub(crate) const fn map_row_index<const NROWS: usize>(row_idx: usize) -> usize {
    #[cfg(feature = "reverse-row-order")]
    {
        NROWS - 1 - row_idx
    }
    #[cfg(not(feature = "reverse-row-order"))]
    {
        row_idx
    }
}

/// Returns the `(prev_addr, addr)` row addresses for the row stored in
/// physical memory slot `slot` (slots are streamed in order `0..NROWS`).
///
/// `addr` is the panel row address rendered from that slot and `prev_addr` is
/// the address of the row rendered immediately before it (the row displayed
/// while this slot's data is shifted in). With the `reverse-row-order`
/// feature the scan order is reversed: slot 0 renders row `NROWS - 1` first
/// and the last slot renders row 0 last, so each slot's `prev_addr` is the
/// *next* higher address (wrapping to 0 for slot 0, which follows the
/// previous frame's final slot rendering row 0).
#[inline]
pub(crate) const fn slot_addresses<const NROWS: usize>(slot: usize) -> (u8, u8) {
    #[cfg(feature = "reverse-row-order")]
    {
        let addr = (NROWS - 1 - slot) as u8;
        let prev_addr = if slot == 0 { 0 } else { (NROWS - slot) as u8 };
        (prev_addr, addr)
    }
    #[cfg(not(feature = "reverse-row-order"))]
    {
        let addr = slot as u8;
        let prev_addr = if slot == 0 {
            NROWS as u8 - 1
        } else {
            slot as u8 - 1
        };
        (prev_addr, addr)
    }
}

/// Static `(len, reps)` pair describing one BCM segment without a pointer.
///
/// [`FrameBuffer::BCM_SEQUENCE`] is a compile-time array of these, one per
/// segment in the repeating period. Drivers use it to size DMA descriptor
/// tables and compute timing without a framebuffer instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BcmLenReps {
    /// Byte length of the segment.
    pub len: usize,
    /// BCM repetition count: how many times the driver streams this segment
    /// before advancing to the next one.
    pub reps: usize,
}

impl BcmLenReps {
    /// A zero-valued entry used for padding unused slots in
    /// [`FrameBuffer::BCM_SEQUENCE`].
    pub const ZERO: Self = Self { len: 0, reps: 0 };
}

/// Capacity of the [`FrameBuffer::BCM_SEQUENCE`] array.
///
/// Sized for the worst case sequence: 8 bit-planes plus an inter-row gap
/// plus an end-of-row trailer.
pub const BCM_SEQUENCE_CAPACITY: usize = 10;

// ---------------------------------------------------------------------------
// Feature-gated blanking-delay constants (shared across all framebuffer types)
// ---------------------------------------------------------------------------

// Compile‑time assertion: at most one lead-blank-* feature enabled.
const _: () = assert!(
    (cfg!(feature = "lead-blank-1") as usize)
        + (cfg!(feature = "lead-blank-2") as usize)
        + (cfg!(feature = "lead-blank-4") as usize)
        + (cfg!(feature = "lead-blank-8") as usize)
        + (cfg!(feature = "lead-blank-16") as usize)
        + (cfg!(feature = "lead-blank-32") as usize)
        <= 1,
    "lead-blank-* features are mutually exclusive"
);

#[cfg(feature = "lead-blank-1")]
pub(crate) const LEAD_BLANK_DELAY: usize = 1;
#[cfg(feature = "lead-blank-2")]
pub(crate) const LEAD_BLANK_DELAY: usize = 2;
#[cfg(feature = "lead-blank-4")]
pub(crate) const LEAD_BLANK_DELAY: usize = 4;
#[cfg(feature = "lead-blank-8")]
pub(crate) const LEAD_BLANK_DELAY: usize = 8;
#[cfg(feature = "lead-blank-16")]
pub(crate) const LEAD_BLANK_DELAY: usize = 16;
#[cfg(feature = "lead-blank-32")]
pub(crate) const LEAD_BLANK_DELAY: usize = 32;

#[cfg(not(any(
    feature = "lead-blank-1",
    feature = "lead-blank-2",
    feature = "lead-blank-4",
    feature = "lead-blank-8",
    feature = "lead-blank-16",
    feature = "lead-blank-32"
)))]
pub(crate) const LEAD_BLANK_DELAY: usize = 0;

// Compile‑time assertion: at most one trail-blank-* feature enabled.
const _: () = assert!(
    (cfg!(feature = "trail-blank-1") as usize)
        + (cfg!(feature = "trail-blank-2") as usize)
        + (cfg!(feature = "trail-blank-4") as usize)
        + (cfg!(feature = "trail-blank-8") as usize)
        + (cfg!(feature = "trail-blank-16") as usize)
        + (cfg!(feature = "trail-blank-32") as usize)
        <= 1,
    "trail-blank-* features are mutually exclusive"
);

#[cfg(feature = "trail-blank-1")]
pub(crate) const TRAIL_BLANK_DELAY: usize = 1;
#[cfg(feature = "trail-blank-2")]
pub(crate) const TRAIL_BLANK_DELAY: usize = 2;
#[cfg(feature = "trail-blank-4")]
pub(crate) const TRAIL_BLANK_DELAY: usize = 4;
#[cfg(feature = "trail-blank-8")]
pub(crate) const TRAIL_BLANK_DELAY: usize = 8;
#[cfg(feature = "trail-blank-16")]
pub(crate) const TRAIL_BLANK_DELAY: usize = 16;
#[cfg(feature = "trail-blank-32")]
pub(crate) const TRAIL_BLANK_DELAY: usize = 32;

#[cfg(not(any(
    feature = "trail-blank-1",
    feature = "trail-blank-2",
    feature = "trail-blank-4",
    feature = "trail-blank-8",
    feature = "trail-blank-16",
    feature = "trail-blank-32"
)))]
pub(crate) const TRAIL_BLANK_DELAY: usize = 0;

// Compile‑time assertion: at most one inter-row-blank-* feature enabled.
const _: () = assert!(
    (cfg!(feature = "inter-row-blank-4") as usize)
        + (cfg!(feature = "inter-row-blank-8") as usize)
        + (cfg!(feature = "inter-row-blank-16") as usize)
        + (cfg!(feature = "inter-row-blank-32") as usize)
        <= 1,
    "inter-row-blank-* features are mutually exclusive"
);

#[cfg(feature = "inter-row-blank-4")]
pub(crate) const INTER_ROW_BLANK: usize = 4;
#[cfg(feature = "inter-row-blank-8")]
pub(crate) const INTER_ROW_BLANK: usize = 8;
#[cfg(feature = "inter-row-blank-16")]
pub(crate) const INTER_ROW_BLANK: usize = 16;
#[cfg(feature = "inter-row-blank-32")]
pub(crate) const INTER_ROW_BLANK: usize = 32;

#[cfg(not(any(
    feature = "inter-row-blank-4",
    feature = "inter-row-blank-8",
    feature = "inter-row-blank-16",
    feature = "inter-row-blank-32"
)))]
pub(crate) const INTER_ROW_BLANK: usize = 0;

/// A single segment of the BCM scan sequence.
///
/// The driver walks an ordered sequence of segments, sending each one `reps`
/// times, to produce correct BCM brightness weighting without needing to know
/// the framebuffer's internal memory layout. See the [`FrameBuffer`]
/// documentation for the segment/sequence terminology.
#[derive(Debug, Clone, Copy)]
pub struct BcmSegment {
    /// Pointer to the segment data.
    pub ptr: *const u8,
    /// Byte length of this segment.
    pub len: usize,
    /// BCM repetition count: how many times the driver streams this segment
    /// before advancing to the next one.
    pub reps: usize,
}

/// Framebuffer that exposes its BCM scan sequence as an ordered series of
/// segments.
///
/// # Terminology
///
/// - **Segment** — the atomic unit, a [`BcmSegment`]: `len` bytes at
///   `ptr`, streamed `reps` times before the driver advances to the next
///   segment.
/// - **Sequence** — the repeating period of the scan:
///   [`BCM_SEQUENCE_LEN`](Self::BCM_SEQUENCE_LEN) consecutive segments
///   whose `len` and `reps` are given by
///   [`BCM_SEQUENCE`](Self::BCM_SEQUENCE). One complete panel refresh
///   consists of this sequence repeated
///   [`BCM_SEQUENCE_COUNT`](Self::BCM_SEQUENCE_COUNT) times.
/// - **Group** — an optional driver hint:
///   [`BCM_SEGMENTS_PER_GROUP`](Self::BCM_SEGMENTS_PER_GROUP) consecutive
///   segments that a driver *may* chain into a single DMA transfer.
///   Groups never straddle a sequence boundary.
///
/// ```text
/// one refresh  = BCM_SEGMENT_COUNT segments
///              = BCM_SEQUENCE_COUNT repetitions of one sequence
/// one sequence = BCM_SEQUENCE_LEN segments (BCM_SEQUENCE)
/// one segment  = len bytes, streamed reps times
/// ```
///
/// | Layout | One sequence is… | `BCM_SEQUENCE_LEN` | `BCM_SEQUENCE_COUNT` |
/// |--------|------------------|--------------------|----------------------|
/// | Frame-major bitplane | the whole frame | `PLANES` | `1` |
/// | Row-major bitplane | one row's BCM cycle | `PLANES + has_gap + has_tail` | `NROWS` |
/// | Threshold (`plain` / `latched`) | the whole frame | `1` | `1` |
///
/// Row-major framebuffers repeat the same `(len, reps)` pattern `NROWS`
/// times, so `BCM_SEQUENCE` holds one row's worth of entries instead of
/// the whole frame's.
///
/// # Segment Ordering
///
/// **Frame-major** (bitplane) framebuffers describe the whole frame in a
/// single sequence of `PLANES` segments (`BCM_SEQUENCE_LEN == PLANES`,
/// `BCM_SEQUENCE_COUNT == 1`). Planes are **LSB-first** and stored
/// contiguously, so each segment streams a whole *suffix* of planes: the
/// segment for plane `k` starts at plane `k`, covers the remaining planes,
/// and is repeated just enough times to bring plane `k`'s total coverage
/// to `2^k`:
///
/// ```text
/// segment 0: (plane0_ptr, PLANES*plane_bytes, 1)      // LSB; all planes
/// segment 1: (plane1_ptr, (PLANES-1)*plane_bytes, 1)
/// segment 2: (plane2_ptr, (PLANES-2)*plane_bytes, 2)
/// …
/// segment N: (planeN_ptr, plane_bytes, 2^(PLANES-2))  // MSB
/// ```
///
/// **Row-major** framebuffers repeat one sequence per row
/// (`BCM_SEQUENCE_COUNT == NROWS`). Planes are **LSB-first** within each
/// row and stored contiguously. The inter-row gap segment sits between
/// plane 0 and plane 1 — the point where the row address changes; with a
/// gap enabled, plane 0 stands alone and plane 1's segment gets 2 reps:
///
/// ```text
/// sequence 0 (row 0):
///   (row0_plane0_ptr, PLANES*pixel_bytes, 1)
///   (row0_gap_ptr, gap_bytes, 1)                    // if enabled
///   (row0_plane1_ptr, (PLANES-1)*pixel_bytes, 1|2)  // 2 reps if gap
///   …
///   (row0_planeN_ptr, pixel_bytes, 2^(PLANES-2))
///   (row0_trailer_ptr, trailer_bytes, 1)            // if enabled
/// sequence 1 (row 1):
///   …
/// ```
///
/// **Threshold-based** framebuffers produce a single segment with
/// `reps = 1` covering the entire buffer.
pub trait FrameBuffer {
    /// The DMA word type used by this framebuffer (`u8` for latched, `u16`
    /// for direct-drive). Driver implementations can use this to enforce that pin
    /// configurations match the framebuffer at compile time.
    type Word;

    /// Static `(len, reps)` of the segments in one **sequence**, in scan
    /// order.
    ///
    /// This is instance-free data: `len` is the byte length of a segment
    /// and `reps` is how many times it is streamed before advancing to the
    /// next one. Drivers can use it at compile time to size transfer
    /// resources (for example DMA descriptor tables) without needing a
    /// framebuffer instance.
    ///
    /// Only the first [`BCM_SEQUENCE_LEN`](Self::BCM_SEQUENCE_LEN) entries
    /// are meaningful; the remaining entries are
    /// [`BcmLenReps::ZERO`] padding (stable Rust does not allow the array
    /// length to be computed from the implementor's const generics).
    const BCM_SEQUENCE: [BcmLenReps; BCM_SEQUENCE_CAPACITY];

    /// Number of BCM segments in one **sequence**, i.e. the number of
    /// meaningful entries in [`BCM_SEQUENCE`](Self::BCM_SEQUENCE).
    const BCM_SEQUENCE_LEN: usize;

    /// Number of times the sequence repeats in one complete panel refresh:
    /// `NROWS` for **row-major** framebuffers (one sequence per row), `1`
    /// for **frame-major** and threshold framebuffers (the sequence covers
    /// the whole frame).
    const BCM_SEQUENCE_COUNT: usize;

    /// Total number of BCM segments streamed for one complete panel
    /// refresh (all rows, all planes).
    ///
    /// The default implementation is
    /// `BCM_SEQUENCE_LEN * BCM_SEQUENCE_COUNT`.
    const BCM_SEGMENT_COUNT: usize = Self::BCM_SEQUENCE_LEN * Self::BCM_SEQUENCE_COUNT;

    /// Number of consecutive segments that a driver may chain into a
    /// single DMA transfer. Must divide
    /// [`BCM_SEQUENCE_LEN`](Self::BCM_SEQUENCE_LEN): groups never straddle
    /// a sequence boundary.
    ///
    /// - **Frame-major** framebuffers use `1` (each plane is its own group).
    /// - **Row-major** framebuffers use `PLANES + has_gap + has_tail` (all
    ///   segments for one row form a single transfer).
    const BCM_SEGMENTS_PER_GROUP: usize = 1;

    /// Total number of BCM segments streamed for one complete panel
    /// refresh.
    ///
    /// The default implementation returns [`Self::BCM_SEGMENT_COUNT`].
    fn bcm_segment_count(&self) -> usize {
        Self::BCM_SEGMENT_COUNT
    }

    /// Returns the byte pointer for the `index`-th BCM segment.
    ///
    /// The `len` and `reps` for this segment come from
    /// [`BCM_SEQUENCE`](Self::BCM_SEQUENCE)`[index %`
    /// [`BCM_SEQUENCE_LEN`](Self::BCM_SEQUENCE_LEN)`]`.
    ///
    /// # Panics
    /// Panics if `index >= BCM_SEGMENT_COUNT`.
    fn bcm_segment_ptr(&self, index: usize) -> *const u8;

    /// Returns the `index`-th BCM segment.
    ///
    /// The provided implementation combines
    /// [`bcm_segment_ptr`](Self::bcm_segment_ptr) with the static
    /// `(len, reps)` from [`BCM_SEQUENCE`](Self::BCM_SEQUENCE).
    ///
    /// # Panics
    /// Panics if `index >= BCM_SEGMENT_COUNT`.
    fn bcm_segment(&self, index: usize) -> BcmSegment {
        assert!(
            index < Self::BCM_SEGMENT_COUNT,
            "segment index {index} out of range for {} segments",
            Self::BCM_SEGMENT_COUNT
        );
        let BcmLenReps { len, reps } = Self::BCM_SEQUENCE[index % Self::BCM_SEQUENCE_LEN];
        BcmSegment {
            ptr: self.bcm_segment_ptr(index),
            len,
            reps,
        }
    }

    /// Number of consecutive segments that form one transfer group.
    ///
    /// The default implementation returns
    /// [`Self::BCM_SEGMENTS_PER_GROUP`].
    fn bcm_segments_per_group(&self) -> usize {
        Self::BCM_SEGMENTS_PER_GROUP
    }
}

/// Sum of `reps` across one complete panel refresh.
///
/// Useful for sizing DMA descriptor tables when the hardware uses one
/// descriptor per repetition (e.g. esp32 DMA and stm32 GPDMA linked-list mode).
pub const fn bcm_rep_count<FB: FrameBuffer>() -> usize {
    let mut total = 0;
    let mut seq = 0;
    while seq < FB::BCM_SEQUENCE_COUNT {
        let mut i = 0;
        while i < FB::BCM_SEQUENCE_LEN {
            total += FB::BCM_SEQUENCE[i].reps;
            i += 1;
        }
        seq += 1;
    }
    total
}

/// Trait for mutable framebuffers that support `embedded_graphics` drawing.
pub trait MutableFrameBuffer:
    FrameBuffer + DrawTarget<Color = Color, Error = core::convert::Infallible>
{
}

/// Trait for all operations a user may want to call on a framebuffer.
pub trait FrameBufferOperations: FrameBuffer {
    /// Erase pixel colors while preserving control bits.
    /// This is much faster than `format()` and is the typical way to clear the display.
    fn erase(&mut self);

    /// Set a pixel in the framebuffer.
    fn set_pixel(&mut self, p: Point, color: Color);
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::format;

    use super::*;
    use embedded_graphics::pixelcolor::RgbColor;

    #[test]
    fn test_compute_rows() {
        // Test typical panel sizes
        assert_eq!(compute_rows(32), 16);
        assert_eq!(compute_rows(64), 32);
        assert_eq!(compute_rows(16), 8);
        assert_eq!(compute_rows(128), 64);

        // Test edge cases
        assert_eq!(compute_rows(2), 1);
        assert_eq!(compute_rows(0), 0);

        // Test that it always divides by 2
        for rows in [8, 16, 24, 32, 48, 64, 96, 128, 256] {
            assert_eq!(compute_rows(rows), rows / 2);
        }
    }

    #[test]
    fn test_compute_frame_count() {
        // Test common bit depths
        assert_eq!(compute_frame_count(1), 1); // 2^1 - 1 = 1
        assert_eq!(compute_frame_count(2), 3); // 2^2 - 1 = 3
        assert_eq!(compute_frame_count(3), 7); // 2^3 - 1 = 7
        assert_eq!(compute_frame_count(4), 15); // 2^4 - 1 = 15
        assert_eq!(compute_frame_count(5), 31); // 2^5 - 1 = 31
        assert_eq!(compute_frame_count(6), 63); // 2^6 - 1 = 63
        assert_eq!(compute_frame_count(7), 127); // 2^7 - 1 = 127
        assert_eq!(compute_frame_count(8), 255); // 2^8 - 1 = 255

        // Test the formula: (2^bits) - 1
        for bits in 1..=8 {
            let expected = (1usize << bits) - 1;
            assert_eq!(compute_frame_count(bits), expected);
        }
    }

    #[test]
    #[cfg(not(feature = "reverse-row-order"))]
    fn test_map_row_index_forward() {
        for r in 0..16 {
            assert_eq!(map_row_index::<16>(r), r);
        }
    }

    #[test]
    #[cfg(feature = "reverse-row-order")]
    fn test_map_row_index_reversed() {
        for r in 0..16 {
            assert_eq!(map_row_index::<16>(r), 15 - r);
        }
    }

    #[test]
    #[cfg(not(feature = "reverse-row-order"))]
    fn test_slot_addresses_forward() {
        // Slot j renders panel row j; while it is shifted the panel displays
        // row j-1 (wrapping to the previous frame's last row for slot 0).
        for slot in 0..16usize {
            let (prev_addr, addr) = slot_addresses::<16>(slot);
            assert_eq!(addr, slot as u8);
            assert_eq!(
                prev_addr,
                if slot == 0 { 15 } else { slot as u8 - 1 },
                "prev_addr for slot {slot}"
            );
        }
    }

    #[test]
    #[cfg(feature = "reverse-row-order")]
    fn test_slot_addresses_reversed() {
        // Slot j renders panel row 15-j; while it is shifted the panel
        // displays the previously rendered row 15-(j-1) (row 0 from the
        // previous frame's final slot for slot 0).
        for slot in 0..16usize {
            let (prev_addr, addr) = slot_addresses::<16>(slot);
            assert_eq!(addr, 15 - slot as u8);
            assert_eq!(
                prev_addr,
                if slot == 0 { 0 } else { 16 - slot as u8 },
                "prev_addr for slot {slot}"
            );
        }
    }

    #[test]
    fn test_compute_frame_count_properties() {
        // Test that frame count grows exponentially
        assert!(compute_frame_count(2) > compute_frame_count(1));
        assert!(compute_frame_count(3) > compute_frame_count(2));
        assert!(compute_frame_count(4) > compute_frame_count(3));

        // Test doubling property: each additional bit approximately doubles frame count
        for bits in 1..=7 {
            let current_frames = compute_frame_count(bits);
            let next_frames = compute_frame_count(bits + 1);
            // next_frames should be approximately 2 * current_frames + 1
            assert_eq!(next_frames, 2 * current_frames + 1);
        }
    }

    #[test]
    fn test_word_size_enum() {
        // Test enum values
        let eight = WordSize::Eight;
        let sixteen = WordSize::Sixteen;

        assert_ne!(eight, sixteen);
        assert_eq!(eight, WordSize::Eight);
        assert_eq!(sixteen, WordSize::Sixteen);
    }

    #[test]
    fn test_word_size_debug() {
        let eight = WordSize::Eight;
        let sixteen = WordSize::Sixteen;

        let eight_debug = format!("{:?}", eight);
        let sixteen_debug = format!("{:?}", sixteen);

        assert_eq!(eight_debug, "Eight");
        assert_eq!(sixteen_debug, "Sixteen");
    }

    #[test]
    fn test_word_size_clone_copy() {
        let original = WordSize::Eight;
        let cloned = original.clone();
        let copied = original;

        assert_eq!(original, cloned);
        assert_eq!(original, copied);
        assert_eq!(cloned, copied);
    }

    #[test]
    fn test_color_type_alias() {
        // Test that Color is an alias for Rgb888
        let red_color: Color = Color::RED;
        let red_rgb888: Rgb888 = Rgb888::RED;

        assert_eq!(red_color, red_rgb888);
        assert_eq!(red_color.r(), 255);
        assert_eq!(red_color.g(), 0);
        assert_eq!(red_color.b(), 0);

        // Test various colors
        let colors = [
            (Color::RED, (255, 0, 0)),
            (Color::GREEN, (0, 255, 0)),
            (Color::BLUE, (0, 0, 255)),
            (Color::WHITE, (255, 255, 255)),
            (Color::BLACK, (0, 0, 0)),
            (Color::CYAN, (0, 255, 255)),
            (Color::MAGENTA, (255, 0, 255)),
            (Color::YELLOW, (255, 255, 0)),
        ];

        for (color, (r, g, b)) in colors {
            assert_eq!(color.r(), r);
            assert_eq!(color.g(), g);
            assert_eq!(color.b(), b);
        }
    }

    #[test]
    fn test_color_construction() {
        // Test Color construction from RGB values
        let custom_color = Color::new(128, 64, 192);
        assert_eq!(custom_color.r(), 128);
        assert_eq!(custom_color.g(), 64);
        assert_eq!(custom_color.b(), 192);

        // Test that it behaves like Rgb888
        let rgb888_color = Rgb888::new(128, 64, 192);
        assert_eq!(custom_color, rgb888_color);
    }

    #[test]
    fn test_helper_functions_const() {
        // Test that helper functions can be used in const contexts
        const ROWS: usize = 32;
        const COMPUTED_NROWS: usize = compute_rows(ROWS);
        const BITS: u8 = 4;
        const COMPUTED_FRAME_COUNT: usize = compute_frame_count(BITS);

        assert_eq!(COMPUTED_NROWS, 16);
        assert_eq!(COMPUTED_FRAME_COUNT, 15);
    }

    #[test]
    fn test_realistic_panel_configurations() {
        // Test common HUB75 panel configurations
        struct PanelConfig {
            rows: usize,
            cols: usize,
            bits: u8,
        }

        let configs = [
            PanelConfig {
                rows: 32,
                cols: 64,
                bits: 3,
            }, // 32x64 panel, 3-bit color
            PanelConfig {
                rows: 64,
                cols: 64,
                bits: 4,
            }, // 64x64 panel, 4-bit color
            PanelConfig {
                rows: 32,
                cols: 32,
                bits: 5,
            }, // 32x32 panel, 5-bit color
            PanelConfig {
                rows: 16,
                cols: 32,
                bits: 6,
            }, // 16x32 panel, 6-bit color
        ];

        for config in configs {
            let nrows = compute_rows(config.rows);
            let frame_count = compute_frame_count(config.bits);

            // Basic sanity checks for rows
            assert!(nrows > 0);
            assert!(nrows <= config.rows);
            assert_eq!(nrows * 2, config.rows);

            // Basic sanity checks for columns
            assert!(config.cols > 0);
            assert!(config.cols <= 256); // Reasonable upper limit for HUB75 panels

            // Frame count checks
            assert!(frame_count > 0);
            assert!(frame_count < 256); // Should be reasonable for typical bit depths

            // Frame count should grow with bit depth
            let prev_frame_count = compute_frame_count(config.bits - 1);
            assert!(frame_count > prev_frame_count);
        }
    }

    #[test]
    fn test_memory_calculations() {
        // Test that we can calculate memory requirements using helper functions
        const ROWS: usize = 64;
        const COLS: usize = 64;
        const BITS: u8 = 4;

        const NROWS: usize = compute_rows(ROWS);
        const FRAME_COUNT: usize = compute_frame_count(BITS);

        // These should be compile-time constants
        assert_eq!(NROWS, 32);
        assert_eq!(FRAME_COUNT, 15);

        // Verify the relationship between parameters
        assert_eq!(NROWS * 2, ROWS);
        assert_eq!(FRAME_COUNT, (1 << BITS) - 1);

        // Verify COLS is reasonable for memory calculations
        assert!(COLS > 0);
        assert!(COLS <= 256); // Reasonable limit for HUB75 panels
    }

    #[test]
    fn test_edge_cases() {
        // Test minimum values
        assert_eq!(compute_rows(2), 1);
        assert_eq!(compute_frame_count(1), 1);

        // Test maximum reasonable values
        assert_eq!(compute_rows(512), 256);
        assert_eq!(compute_frame_count(8), 255);

        // Test zero (though not practical)
        assert_eq!(compute_rows(0), 0);
    }

    // Note: We can't easily test the traits directly since they're abstract,
    // but they are thoroughly tested through their implementations in
    // the plain and latched modules.

    #[test]
    fn test_word_size_equality() {
        // Test all combinations of equality
        assert_eq!(WordSize::Eight, WordSize::Eight);
        assert_eq!(WordSize::Sixteen, WordSize::Sixteen);
        assert_ne!(WordSize::Eight, WordSize::Sixteen);
        assert_ne!(WordSize::Sixteen, WordSize::Eight);
    }

    #[test]
    fn test_bit_depth_limits() {
        // Test that our bit depth calculations work for the full range
        for bits in 1..=8 {
            let frame_count = compute_frame_count(bits);

            // Frame count should be positive
            assert!(frame_count > 0);

            // Frame count should be less than 2^bits
            assert!(frame_count < (1 << bits));

            // Frame count should be exactly (2^bits) - 1
            assert_eq!(frame_count, (1 << bits) - 1);
        }
    }

    #[test]
    fn test_documentation_examples() {
        // Test the example values from the documentation
        const ROWS: usize = 32;
        const COLS: usize = 64;
        const NROWS: usize = ROWS / 2;
        const BITS: u8 = 8;
        const FRAME_COUNT: usize = (1 << BITS) - 1;

        // Verify using our helper functions
        assert_eq!(compute_rows(ROWS), NROWS);
        assert_eq!(compute_frame_count(BITS), FRAME_COUNT);

        // Verify the values match documentation
        assert_eq!(ROWS, 32);
        assert_eq!(COLS, 64);
        assert_eq!(NROWS, 16);
        assert_eq!(FRAME_COUNT, 255);

        // Verify this matches typical panel dimensions
        assert!(COLS > 0);
        assert_eq!(NROWS * 2, ROWS);
    }
}
