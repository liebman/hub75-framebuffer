// Run with: cargo bench --bench fill_rect_latched

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use embedded_graphics::{
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use hub75_framebuffer::latched::DmaFrameBuffer;
use hub75_framebuffer::{compute_frame_count, compute_rows, Color};
use std::hint::black_box;
use std::time::Duration;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type TestFrameBuffer = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;

// Configure Criterion for better statistical rigor and consistency
fn configure_criterion() -> Criterion {
    Criterion::default()
        .sample_size(200) // More samples for better accuracy
        .measurement_time(Duration::from_secs(10)) // Longer measurement time
        .warm_up_time(Duration::from_secs(3)) // Adequate warm-up
        .confidence_level(0.95) // Higher confidence level
        .significance_level(0.02) // More sensitive to changes
}

// Test colors: focused set without black pixel complexity
const TEST_COLORS: &[(&str, Color)] = &[
    ("red", Color::RED),                 // High intensity (255, 0, 0)
    ("gray", Color::new(128, 128, 128)), // Medium intensity, typical UI color
];

// Comprehensive rectangle test cases covering different sizes, positions, and aspect ratios
fn get_test_rectangles() -> Vec<(&'static str, Rectangle)> {
    vec![
        // Original test cases
        (
            "full_panel",
            Rectangle::new(Point::zero(), Size::new(COLS as u32, ROWS as u32)),
        ),
        (
            "half_panel",
            Rectangle::new(Point::zero(), Size::new(COLS as u32, (ROWS / 2) as u32)),
        ),
        (
            "medium_rect",
            Rectangle::new(Point::new(16, 8), Size::new(32, 16)),
        ),
        (
            "small_rect",
            Rectangle::new(Point::new(28, 12), Size::new(8, 8)),
        ),
        // New test cases for better coverage
        (
            "tiny_rect",
            Rectangle::new(Point::new(30, 14), Size::new(4, 4)),
        ), // Cache line effects
        (
            "square_medium",
            Rectangle::new(Point::new(24, 8), Size::new(16, 16)),
        ), // Square aspect ratio
        (
            "wide_rect",
            Rectangle::new(Point::new(0, 14), Size::new(64, 4)),
        ), // Row-dominant access
        (
            "tall_rect",
            Rectangle::new(Point::new(30, 0), Size::new(4, 32)),
        ), // Column-dominant access
        // Position variations
        (
            "corner_topleft",
            Rectangle::new(Point::new(0, 0), Size::new(16, 16)),
        ), // Top-left corner
        (
            "corner_center",
            Rectangle::new(Point::new(24, 8), Size::new(16, 16)),
        ), // Center position
        (
            "corner_bottomright",
            Rectangle::new(Point::new(48, 16), Size::new(16, 16)),
        ), // Bottom-right
        (
            "span_boundary",
            Rectangle::new(Point::new(24, 14), Size::new(16, 8)),
        ), // Crosses upper/lower boundary
    ]
}

// Baseline implementation for comparison - simple pixel-by-pixel without optimizations
fn draw_rect_baseline(fb: &mut TestFrameBuffer, rect: &Rectangle, color: Color) {
    // Use draw_iter directly to bypass any fill_contiguous optimizations
    let pixels: Vec<_> = rect
        .points()
        .map(|point| embedded_graphics::Pixel(point, color))
        .collect();

    fb.draw_iter(pixels.into_iter()).unwrap();
}

// Optimized implementation using fill_contiguous
fn draw_rect_optimized(fb: &mut TestFrameBuffer, rect: &Rectangle, color: Color) {
    rect.into_styled(PrimitiveStyle::with_fill(color))
        .draw(fb)
        .unwrap();
}

fn fill_rect_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("fill_rect_latched");

    let rectangles = get_test_rectangles();

    for (rect_name, rect) in rectangles.iter() {
        let pixel_count = (rect.size.width * rect.size.height) as u64;
        let memory_bytes = pixel_count * FRAME_COUNT as u64; // Approximate memory access

        group.throughput(Throughput::Elements(pixel_count));
        group.throughput(Throughput::Bytes(memory_bytes));

        for (color_name, color) in TEST_COLORS.iter() {
            // Baseline benchmark
            group.bench_with_input(
                BenchmarkId::new(format!("{}_baseline", rect_name), color_name),
                &(rect, *color),
                |b, (rect, color)| {
                    let mut fb = TestFrameBuffer::new();
                    b.iter(|| {
                        fb.clear();
                        black_box(draw_rect_baseline(
                            black_box(&mut fb),
                            black_box(rect),
                            black_box(*color),
                        ));
                    });
                },
            );

            // Optimized benchmark
            group.bench_with_input(
                BenchmarkId::new(format!("{}_optimized", rect_name), color_name),
                &(rect, *color),
                |b, (rect, color)| {
                    let mut fb = TestFrameBuffer::new();
                    b.iter(|| {
                        fb.clear();
                        black_box(draw_rect_optimized(
                            black_box(&mut fb),
                            black_box(rect),
                            black_box(*color),
                        ));
                    });
                },
            );
        }
    }

    group.finish();
}

criterion_group!(name = benches; config = configure_criterion(); targets = fill_rect_benchmark);
criterion_main!(benches);
