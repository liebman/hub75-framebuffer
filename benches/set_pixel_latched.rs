// Run with:  cargo bench --bench set_pixel_latched

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::Point;
use hub75_framebuffer::{latched::DmaFrameBuffer, Color};
use std::hint::black_box;
use std::time::Duration;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = hub75_framebuffer::compute_rows(ROWS);
const FRAME_COUNT: usize = hub75_framebuffer::compute_frame_count(BITS);

// Number of iterations to target ~1-5ms per measurement
const ITERATIONS: usize = 100;

fn configure_criterion() -> Criterion {
    Criterion::default()
        .sample_size(100)
        .measurement_time(Duration::from_secs(10)) // Longer measurement time
        .warm_up_time(Duration::from_secs(3))
        .confidence_level(0.95)
        .significance_level(0.05)
}

fn set_pixel_latched(c: &mut Criterion) {
    let mut group = c.benchmark_group("set_pixel_latched");
    group.throughput(Throughput::Elements((ROWS * COLS * ITERATIONS) as u64));

    group.bench_function("latched_dma_framebuffer", |b| {
        let mut fb = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();

        b.iter(|| {
            for _ in 0..ITERATIONS {
                for y in 0..ROWS {
                    for x in 0..COLS {
                        black_box(&mut fb).set_pixel(
                            black_box(Point::new(x as i32, y as i32)),
                            black_box(Color::RED),
                        );
                    }
                }
            }
        });
    });

    group.finish();
}

criterion_group!(name = benches; config = configure_criterion(); targets = set_pixel_latched);
criterion_main!(benches);
