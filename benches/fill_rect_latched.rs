// Run with: cargo bench --bench fill_rect_latched

use criterion::{criterion_group, criterion_main, Criterion, Throughput, BenchmarkId};
use embedded_graphics::{
    primitives::{PrimitiveStyle, Rectangle},
    prelude::*,
};
use hub75_framebuffer::latched::DmaFrameBuffer;
use hub75_framebuffer::{compute_frame_count, compute_rows, Color};
use std::hint::black_box;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type TestFrameBuffer = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;

fn fill_rect_benchmark(c: &mut Criterion) {
    let mut group = c.benchmark_group("fill_rect_latched");

    let test_cases = [
        ("full_panel", Rectangle::new(Point::zero(), Size::new(COLS as u32, ROWS as u32))),
        ("half_panel", Rectangle::new(Point::zero(), Size::new(COLS as u32, (ROWS / 2) as u32))),
        ("medium_rect", Rectangle::new(Point::new(16, 8), Size::new(32, 16))),
        ("small_rect", Rectangle::new(Point::new(28, 12), Size::new(8, 8))),
    ];

    for (case_name, rect) in test_cases.iter() {
        group.throughput(Throughput::Elements((rect.size.width * rect.size.height) as u64));
        group.bench_with_input(BenchmarkId::new(*case_name, "red"), rect, |b, rect| {
            let mut fb = TestFrameBuffer::new();
            b.iter(|| {
                fb.clear();
                black_box(
                    Rectangle::new(rect.top_left, rect.size)
                        .into_styled(PrimitiveStyle::with_fill(Color::RED)),
                )
                .draw(black_box(&mut fb))
                .unwrap();
            });
        });
        group.bench_with_input(BenchmarkId::new(*case_name, "black"), rect, |b, rect| {
            let mut fb = TestFrameBuffer::new();
            b.iter(|| {
                fb.clear();
                black_box(
                    Rectangle::new(rect.top_left, rect.size)
                        .into_styled(PrimitiveStyle::with_fill(Color::BLACK)),
                )
                .draw(black_box(&mut fb))
                .unwrap();
            });
        });
    }
    group.finish();
}

criterion_group!(benches, fill_rect_benchmark);
criterion_main!(benches); 