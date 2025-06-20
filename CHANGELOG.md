# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

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

* almost double(!) performance of the set_pixel in the plain and latched `DmaFrameBuffers` (#2)
* Renamed feature `esp-dma` ➜ `esp-hal-dma`.

## [0.1.0] - 2025-06-14

* initial version

<!-- next-url -->
[Unreleased]: https://github.com/liebman/hub75-framebuffer/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/liebman/hub75-framebuffer/compare/v0.1.0...v0.2.0
