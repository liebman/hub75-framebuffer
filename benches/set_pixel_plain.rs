// Run with:  cargo bench --bench set_pixel_plain

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use embedded_graphics::pixelcolor::RgbColor;
use embedded_graphics::prelude::Point;
use hub75_framebuffer::{plain::DmaFrameBuffer, Color};
use std::hint::black_box;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = hub75_framebuffer::compute_rows(ROWS);
const FRAME_COUNT: usize = hub75_framebuffer::compute_frame_count(BITS);

fn set_pixel_plain(c: &mut Criterion) {
    let mut group = c.benchmark_group("set_pixel_plain");
    group.throughput(Throughput::Elements((ROWS * COLS) as u64));

    group.bench_function("plain_dma_framebuffer", |b| {
        let mut fb = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();
        fb.clear();

        b.iter(|| {
            for y in 0..ROWS {
                for x in 0..COLS {
                    black_box(&mut fb).set_pixel(
                        black_box(Point::new(x as i32, y as i32)),
                        black_box(Color::RED),
                    );
                }
            }
        });
    });

    group.finish();
}

criterion_group!(benches, set_pixel_plain);
criterion_main!(benches);
