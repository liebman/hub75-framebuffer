# hub75-framebuffer

[![Crates.io](https://img.shields.io/crates/v/hub75-framebuffer.svg)](https://crates.io/crates/hub75-framebuffer)
[![Documentation](https://docs.rs/hub75-framebuffer/badge.svg)](https://docs.rs/hub75-framebuffer)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](README.md)
[![Coverage Status](https://coveralls.io/repos/github/liebman/hub75-framebuffer/badge.svg?branch=main)](https://coveralls.io/github/liebman/hub75-framebuffer?branch=main)

DMA-friendly framebuffer implementations for driving HUB75 RGB LED matrix
panels with Rust.  The crate focuses on **performance**, **correct timing**,
and **ergonomic drawing** by integrating tightly with the `embedded-graphics`
ecosystem.

---

## How HUB75 LED panels work (very short recap)

A HUB75 panel behaves like a long daisy-chained shift-register:

1. Color data for *one pair of rows* is shifted in serially on every cycle of `CLK`.
2. After the last pixel of the row pair has been clocked, the controller blanks
   the LEDs (`OE` HIGH), sets the address lines **A–E**, and produces a short
   pulse on `LAT` to latch the freshly-shifted data into the LED drivers.
3. `OE` goes LOW again and the row pair lights up while the next one is already
   being shifted.

Color depth is achieved with **Binary/Bit-Angle Code Modulation (BCM)**:
lower bit-planes are shown for shorter times, higher ones for longer, yielding
2^n intensity levels per channel while keeping peak currents low.

If you want a deeper explanation, have a look inside `src/lib.rs` — the crate
documentation contains an extensive primer.

---

## Two framebuffer flavors

| Module              | Extra hardware | Word size | Memory use | Pros / Cons |
|---------------------|----------------|-----------|------------|-------------|
| `plain`             | none           | 16 bit (14 used) | high       | Simplest, wires exactly like a standard HUB75 matrix. |
| `latched`           | **external latch gate** (see below) | 8 bit | ×½ of `plain` | Lower memory footprint, but needs a tiny glue-logic board. |

### The latch circuit

The *latched* implementation assumes a small external circuit that holds the
row address while gating the pixel clock.  A typical solution uses a 74xx373
latch along with a few NAND gates:

![Latch circuit block diagram](images/latch-circuit.png)

The latch IC stores the address bits whilst one NAND gate blocks the `CLK`
signal during the latch interval.  The remaining spare gate can be employed
to combine a global PWM signal with `OE` for fine-grained brightness control
as shown.

---

## Getting started

Add the dependency to your `Cargo.toml`:

```toml
[dependencies]
hub75-framebuffer = "0.2.0"
```

### Choose your parameters

```rust
use hub75_framebuffer::{compute_frame_count, compute_rows};
use hub75_framebuffer::latched::DmaFrameBuffer; 
// or ::plain::DmaFrameBuffer

const ROWS:       usize = 32;              // panel height
const COLS:       usize = 64;              // panel width
const BITS:       u8    = 3;               // colour depth ⇒ 7 BCM frames
const NROWS:      usize = compute_rows(ROWS);          // 16
const FRAME_COUNT:usize = compute_frame_count(BITS);   // (1<<BITS)-1 = 7

// Create a framebuffer (already initialized/cleared)
let mut framebuffer = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>
    ::new();
```

You can now draw using any `embedded-graphics` primitive:

```rust
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Rectangle, PrimitiveStyle};
use hub75_framebuffer::Color;

Rectangle::new(Point::new(0, 0), Size::new(COLS as u32, ROWS as u32))
    .into_styled(PrimitiveStyle::with_fill(Color::BLACK))
    .draw(&mut framebuffer)
    .unwrap();

Circle::new(Point::new(20, 10), 8)
    .into_styled(PrimitiveStyle::with_fill(Color::GREEN))
    .draw(&mut framebuffer)
    .unwrap();
```

Finally hand the raw DMA buffer off to your MCU's parallel peripheral.

---

## Crate features

### `esp-hal-dma` (required when using `esp-hal`)

**Required** when using the `esp-hal` crate for ESP32 development. This
feature switches the `ReadBuffer` trait implementation from `embedded-dma`
to `esp-hal::dma`. If you're targeting ESP32 devices with `esp-hal`, you
**must** enable this feature for DMA compatibility.

```toml
[dependencies]
hub75-framebuffer = { version = "0.2.0", features = ["esp-hal-dma"] }
```

### `esp32-ordering` (required for original ESP32 only)

**Required** when targeting the original ESP32 chip (not ESP32-S3 or other
variants). This feature adjusts byte ordering to accommodate the quirky
requirements of the ESP32's I²S peripheral in 8-bit and 16-bit modes. Other
ESP32 variants (S2, S3, C3, etc.) do **not** need this feature.

```toml
[dependencies]
hub75-framebuffer = { version = "0.2.0", features = ["esp32-ordering"] }
```

### `skip-black-pixels`

Skip drawing black pixels for performance boost in UI applications. When
enabled, calls to `set_pixel()` with `Color::BLACK` return early without
writing to the framebuffer, assuming the framebuffer was already cleared.

### `defmt`

Implement the `defmt::Format` trait so framebuffer types can be logged with
the [`defmt`](https://github.com/knurling-rs/defmt) ecosystem.

### `doc-images`

Embed documentation images when building docs on docs.rs. Not needed for
normal usage.

Enable features in your `Cargo.toml`:

```toml
[dependencies]
hub75-framebuffer = { version = "0.2.0", 
                      features = ["esp-hal-dma", "esp32-ordering"] }
```

---

## Running tests

```shell
cargo test
```

All logic including bitfields, address mapping, brightness modulation and
the `embedded-graphics` integration is covered by a comprehensive test-suite
(≈ 300 tests).

---

## License

Licensed under either of

* Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
* MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.
