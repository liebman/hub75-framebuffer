// Run with:  cargo bench --bench clear_plain

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use hub75_framebuffer::plain::DmaFrameBuffer;
use std::hint::black_box;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = hub75_framebuffer::compute_rows(ROWS);
const FRAME_COUNT: usize = hub75_framebuffer::compute_frame_count(BITS);

fn clear_plain(c: &mut Criterion) {
    let mut group = c.benchmark_group("clear_plain");
    // Each call to `clear` formats every frame. Account for all modified elements.
    group.throughput(Throughput::Elements((ROWS * COLS * FRAME_COUNT) as u64));

    group.bench_function("plain_dma_framebuffer_clear", |b| {
        // Allocate the framebuffer once outside the measured loop.
        let mut fb = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();

        b.iter(|| {
            // Measure the time taken to clear the entire framebuffer.
            black_box(&mut fb).clear();
        });
    });

    group.finish();
}

criterion_group!(benches, clear_plain);
criterion_main!(benches);
