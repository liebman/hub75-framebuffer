# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

### Added

* `lead-blank-32` and `trail-blank-32` features extending blanking delay options to 32 pixel-clock cycles.

* **Row-major bitplane framebuffer** (`bitplane::plain::row::DmaFrameBuffer`).
  Groups all bit-planes for a single row contiguously instead of storing entire
  planes together. Optimized for row-by-row BCM rendering where the driver
  replays each plane's pixel data for brightness weighting before moving to the
  next row, reducing ghosting on panels with slow row drivers. Planes are stored
  LSB-first and contiguously, so each BCM segment streams a whole suffix of
  planes: the segment for plane `k` covers planes `k..N` just enough times to
  reach `2^k` total displays, halving the number of DMA transfers per row
  (`2^(N-1)` instead of `2^N - 1`) with identical brightness. The
  `inter-row-blank-*` gap is streamed between plane 0 (shifted out with the
  previous row's address) and plane 1 (whose first pixel changes the address),
  holding `prev_addr` with `OE` blank.

* **Row-major latched bitplane framebuffer** (`bitplane::latched::row::DmaFrameBuffer`).
  Groups all bit-planes for a single row contiguously, each followed by its
  four address bytes, instead of storing entire planes together. Planes are
  stored LSB-first and every plane carries the current row address — the
  external latch holds the previous row's address during plane 0's shift, so
  no `prev_addr` handling is needed. As in the plain row-major layout, each
  BCM segment streams a suffix of planes, halving the number of DMA
  transfers per row (`2^(N-1)` instead of `2^N - 1`) with identical
  brightness. Blanking mirrors the plain row-major
  layout: the `lead-blank-*` delay applies to the first plane (whose address
  bytes change the row address) and the `trail-blank-*` delay to the second
  plane; all other planes run full-width. The `inter-row-blank-*` gap is
  streamed between plane 0 (whose address bytes latch the data and change
  the row address) and plane 1.

* `skip-black-pixels` support for all bitplane framebuffers
  (`bitplane::plain::frame`, `bitplane::plain::row`, and `bitplane::latched`).

* Compile-time validation of `NROWS` (1..=32) and `PLANES` (1..=8) for all
  bitplane framebuffers. Invalid configurations now panic in `const fn new()`
  — a compile-time error for `static` framebuffers — instead of silently
  corrupting the DMA stream.

* `tiling::QuarterScan` is now implemented for classic 1/16-scan 64×64 panels
  (16 row addresses, four rows lit at a time). The four quarters of the panel
  are mapped to side-by-side 64-column sections via a pluggable wiring variant
  (the third type parameter — see `tiling::quarter_scan`): built-in variants
  are `SectionsSwapped` (the default, verified on hardware), `Linear`,
  `HalvesSwapped` and `Alternating`, and custom wirings can be added by
  implementing `quarter_scan::Variant` downstream. The default places rows
  0–15 on channel 1 section 1, rows 16–31 on channel 1 section 0, rows 32–47
  on channel 2 section 1 and rows 48–63 on channel 2 section 0. The
  framebuffer geometry
  changed accordingly: `FB_ROWS` is now `PANEL_ROWS / 2` (32) and `FB_COLS`
  is `PANEL_COLS * 2` (128) — previously the stub declared a 16×256 geometry
  (8 addresses) that matched no real 1/16-scan panel and its `remap_xy`
  panicked with `todo!()`. Pair with a bitplane
  `DmaFrameBuffer<{ PANEL_ROWS / 4 }, { PANEL_COLS * 2 }, PLANES>`.
  Coordinates outside the virtual canvas are clipped (mapped just past the
  inner framebuffer's bounds) instead of panicking, matching the clipping
  behavior embedded-graphics draw operations rely on.

### ⚠️ Breaking

* `tiling::QuarterScan` gained a wiring-variant type parameter
  (`QuarterScan<ROWS, COLS, V = quarter_scan::SectionsSwapped>`) and is no
  longer value-constructible (private `PhantomData` field). Type-level usage
  such as `QuarterScan<64, 64>` is source-compatible.

* **`FrameBuffer` trait now exposes BCM segments instead of raw plane
  pointers.** The old methods `get_word_size()`, `plane_count()`, and
  `plane_ptr_len()` have been replaced with `bcm_segment_count()`,
  `bcm_segment()`, and `bcm_segments_per_group()`. The `Word` associated type
  is retained. All built-in framebuffers and tiling wrappers implement the new
  interface.

  **Migration:** if you implemented `FrameBuffer` on a custom type, replace
  `plane_count` / `plane_ptr_len` with the segment methods. Each former plane
  becomes one `BcmSegment { ptr, len, reps }`.

* **`bitplane::latched::Row` moved to `bitplane::latched::frame::Row`** as part
  of the module split. The `bitplane::latched::DmaFrameBuffer` path is
  unchanged via re-export.

* **Bitplane plane-major framebuffers now store planes LSB-first and emit
  suffix-coalesced BCM segments** (`bitplane::plain::frame::DmaFrameBuffer`
  and `bitplane::latched::frame::DmaFrameBuffer`). Plane 0 now carries the
  LSB (previously the MSB), and the segment for plane `k` spans planes
  `k..PLANES` with `2^(k-1)` repetitions (segment 0 spans all planes with 1
  repetition), halving the number of DMA transfers per frame (`2^(PLANES-1)`
  instead of `2^PLANES - 1`) with identical brightness. `bcm_segment_count()`
  and `bcm_segments_per_group()` are unchanged, but segment `len`/`reps`
  values changed.

  **Migration:** drivers must no longer assume one segment == one plane: a
  segment's `len` may exceed the platform's maximum DMA transfer size, so
  split each repetition into `div_ceil(len, max_chunk)` descriptors
  (`dma_descriptor_count(max_chunk)` now accounts for this). Per-group ISR
  cadence is unchanged (`PLANES` groups per frame), but inter-ISR intervals
  changed, and the longest contiguous single-plane (MSB) run per frame is
  now `2^(PLANES-2)` plane passes.

## [0.11.0] - 2026-08-02

### Added

* `inter-row-blank-4`, `inter-row-blank-8`, `inter-row-blank-16`, and `inter-row-blank-32` features that insert additional dead clock cycles between the latch and the address change at the end of each row. In plain framebuffers the gap entries hold the previous row address with `OE` blank, deferring the address change to the first pixel of the next row. Intended for panels with slower row drivers that need more time to finish blanking before the address lines change.

* `new()` is now `const` for all framebuffer types

### ⚠️ Breaking

* Deprecated the `plain` and `latched` `DmaFrameBuffer` implementations in favor of their bitplane counterparts. Users should migrate to `bitplane::plain::DmaFrameBuffer` and `bitplane::latched::DmaFrameBuffer` respectively. Bitplane framebuffers provide the same functionality with linearly-scaling memory usage (vs. exponential for threshold-based framebuffers), true BCM plane access via the `FrameBuffer` trait, and simpler multi-color-depth support by adding or removing planes.

## [0.10.0] - 2026-07-25

### ⚠️ Breaking

* Renamed blank delay features from `blank-delay-1/2/4/8` to separate `lead-blank-1/2/4/8/16` and `trail-blank-1/2/4/8/16` features. The lead blank delay controls how many clock cycles the output is blanked before the row address is changed, and the trail blank delay controls blanking after the row address is changed. The new `16` value is also available. Default is 1 for plain framebuffers and 0 for latched framebuffers (which handle timing via extra `Address` entries to manage the address change).

## [0.9.2] - 2026-07-05

### Fixed

* latched, bitplane/latched: fixed bug in OE setting in address table created in 0.9.1

## [0.9.1] - 2026-07-05 (YANKED)

### Changed

* latched, bitplane/latched: support `invert-oe`, `blank-delay-1`, `blank-delay-2`, `blank-delay-4`, and `blank-delay-8` features to control extra blanking time around row address changes, preventing ghosting artifacts on panels with slower address-line settling

## [0.9.0] - 2026-07-03

### ⚠️ Breaking

* `FrameBuffer` trait now requires a `Word` associated type (e.g. `type Word = u16;`) instead of manually implementing `get_word_size()`; `get_word_size()` has a default implementation derived from `size_of::<Self::Word>()`

### Added

* feature `tail-closes-latch` will include an extra entry in `plain` and `bitplane/plain` implementations to close the latch at the end of the the buffer (`plain`) and at the end of each plane (`bitplane/plain`)

## [0.8.1] - 2026-06-27

### Changed

* bitplane/latched: tweak latch timing & clear values after latch closed to prevent address "bleeding" into the first pixels in some cases

### Added

* plain, bitplane/plain: new `blank-delay-1`, `blank-delay-2`, `blank-delay-4`, and `blank-delay-8` features to control extra blanking time around row address changes, preventing ghosting artifacts on panels with slower address-line settling

## [0.8.0] - 2026-05-01

### ⚠️ Breaking

* removed unneeded const generics from the FrameBuffer, MutableFrameBuffer, and FrameBufferOperations traits

### Added

* add new bitplane framebuffers

### Removed

* remove dependency on `esp-hal`

## [0.7.0] - 2026-04-25

* bump `esp-hal` to `1.1.0`

## [0.6.0] - 2025-10-31

### Changed

* bump `esp-hal` to `1.0.0`

## [0.5.0] - 2025-10-14

### Changed

* bump `esp-hal` to `1.0.0-rc1`

## [0.4.2] - 2025-08-16

## [0.4.1] - 2025-08-14

## [0.4.0] - 2025-08-14

### Added

* New `tiling::TiledFrameBuffer` wrapper that lets you chain multiple HUB75
  panels into one large virtual canvas ([#10](https://github.com/liebman/hub75-framebuffer/pull/10))

## [0.3.0] - 2025-07-16

* update to `esp-hal` `1.0.0-rc.1` [#9](https://github.com/liebman/hub75-framebuffer/pull/9)

## [0.2.0] - 2025-06-20

### ⚠️ Breaking

* Renamed `DmaFrameBuffer::clear()` to `erase()`.  
  The new name avoids shadowing `embedded_graphics::DrawTarget::clear(Color)`.  
  Update your code: `fb.clear()` ➜ `fb.erase()`.  
  If you actually wanted the trait method, call `fb.clear(Color::BLACK)` instead.
* Removed feature flags `esp32`, `esp32s3`, `esp32c6`.
* Renamed feature `esp-dma` ➜ `esp-hal-dma`.

### Added

* `skip-black-pixels` feature that gives a performance boot in some cases (#2)
* Removed feature flags `esp32`, `esp32s3`, `esp32c6`.

### Changed

* almost double(!) performance of the set_pixel in the plain and latched
  `DmaFrameBuffers` (#2)
* Renamed feature `esp-dma` ➜ `esp-hal-dma`.

## [0.1.0] - 2025-06-14

* initial version

<!-- next-url -->
[Unreleased]: https://github.com/liebman/hub75-framebuffer/compare/v0.11.0...HEAD
[0.11.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.10.0...v0.11.0
[0.10.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.9.2...v0.10.0
[0.9.2]: https://github.com/liebman/hub75-framebuffer/compare/v0.9.1...v0.9.2
[0.9.1]: https://github.com/liebman/hub75-framebuffer/compare/v0.9.0...v0.9.1
[0.9.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.8.1...v0.9.0
[0.8.1]: https://github.com/liebman/hub75-framebuffer/compare/v0.8.0...v0.8.1
[0.8.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.4.2...v0.5.0
[0.4.2]: https://github.com/liebman/hub75-framebuffer/compare/v0.4.1...v0.4.2
[0.4.1]: https://github.com/liebman/hub75-framebuffer/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.1.0...v0.2.0
