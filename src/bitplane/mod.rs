//! Bitplane-oriented framebuffer implementations for HUB75 LED panels.
//!
//! These framebuffers store colour data as one bit-plane per colour bit
//! (typically 8 planes for full 8-bit colour). To render, output each plane a
//! number of times equal to its bit-weight (the MSB plane 2^7 times, the LSB
//! plane once) — either with a DMA descriptor chain or by reprogramming the
//! DMA address/length registers per segment from a transfer-complete
//! interrupt — so that the weighted repetition counts produce correct BCM
//! brightness.
//!
//! All layouts store planes **LSB-first** (plane 0 = LSB), and each BCM
//! segment streams a contiguous *suffix* of planes, repeated just enough
//! times to bring each plane's total coverage to its bit-weight — halving
//! the number of DMA transfers per scan with identical brightness. Drivers
//! should follow the [`FrameBuffer`](crate::FrameBuffer) segment sequence
//! rather than assuming an order; see its documentation for the
//! segment/group/sequence terminology.
//!
//! Two variants are provided:
//!
//! - [`plain`] -- 16-bit entries with all signals (address, latch, OE, colour)
//!   packed into each word. No external latch circuit needed.
//! - [`latched`] -- 8-bit entries with separate address bytes, requiring an
//!   external latch circuit but halving per-entry memory.

pub mod latched;
pub mod plain;
