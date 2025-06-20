// Run with: cargo bench --bench render_text_plain

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    prelude::*,
    text::{Baseline, Text},
};
use hub75_framebuffer::plain::DmaFrameBuffer;
use hub75_framebuffer::{compute_frame_count, compute_rows, Color};
use std::{hint::black_box, time::Duration};

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type TestFrameBuffer = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;

// Three representative strings of different lengths
const TEST_STRINGS: &[(&str, &str)] = &[
    ("short", "HELLO"),
    ("medium", "THE QUICK BROWN FOX"),
    ("long", "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"),
];

// Calculate appropriate iteration count based on text length to target ~1-5ms per measurement
fn get_iteration_count(text: &str) -> usize {
    match text.len() {
        0..=10 => 1000, // Short text: more iterations
        11..=25 => 500, // Medium text: moderate iterations
        _ => 200,       // Long text: fewer iterations
    }
}

fn configure_criterion() -> Criterion {
    Criterion::default()
        .sample_size(100)
        .measurement_time(Duration::from_secs(10)) // Longer measurement time
        .warm_up_time(Duration::from_secs(3))
        .confidence_level(0.95)
        .significance_level(0.05)
}

// Baseline: draw each character separately (higher function-call overhead)
fn draw_text_baseline(
    fb: &mut TestFrameBuffer,
    origin: Point,
    text: &str,
    style: MonoTextStyle<Color>,
) {
    let mut x = origin.x;
    for ch in text.chars() {
        // Draw one-character string
        let s = ch.to_string();
        Text::with_baseline(&s, Point::new(x, origin.y), style, Baseline::Top)
            .draw(fb)
            .unwrap();
        x += FONT_6X10.character_size.width as i32;
    }
}

// Optimised: draw the whole string in one call
fn draw_text_optimised(
    fb: &mut TestFrameBuffer,
    origin: Point,
    text: &str,
    style: MonoTextStyle<Color>,
) {
    Text::with_baseline(text, origin, style, Baseline::Top)
        .draw(fb)
        .unwrap();
}

fn render_text_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("render_text_plain");
    let style = MonoTextStyle::new(&FONT_6X10, Color::WHITE);

    for (case, text) in TEST_STRINGS {
        let pixel_count = (text.len() as u32
            * FONT_6X10.character_size.width
            * FONT_6X10.character_size.height) as u64;
        let iterations = get_iteration_count(text);

        group.throughput(Throughput::Elements(pixel_count * iterations as u64));

        // Baseline
        group.bench_with_input(
            BenchmarkId::new("baseline", case),
            &(text, iterations),
            |b, &(text, iterations)| {
                let origin = Point::new(0, 0);
                b.iter(|| {
                    let mut fb = TestFrameBuffer::new();
                    for _ in 0..iterations {
                        fb.erase();
                        black_box(draw_text_baseline(
                            black_box(&mut fb),
                            black_box(origin),
                            black_box(text),
                            black_box(style),
                        ));
                    }
                });
            },
        );

        // Optimised
        group.bench_with_input(
            BenchmarkId::new("optimised", case),
            &(text, iterations),
            |b, &(text, iterations)| {
                let origin = Point::new(0, 0);
                b.iter(|| {
                    let mut fb = TestFrameBuffer::new();
                    for _ in 0..iterations {
                        fb.erase();
                        black_box(draw_text_optimised(
                            black_box(&mut fb),
                            black_box(origin),
                            black_box(text),
                            black_box(style),
                        ));
                    }
                });
            },
        );
    }

    group.finish();
}

criterion_group!(name = benches; config = configure_criterion(); targets = render_text_benchmark);
criterion_main!(benches);
