// Run with:  cargo bench --bench clear_latched

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use hub75_framebuffer::latched::DmaFrameBuffer;
use std::hint::black_box;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = hub75_framebuffer::compute_rows(ROWS);
const FRAME_COUNT: usize = hub75_framebuffer::compute_frame_count(BITS);

fn clear_latched(c: &mut Criterion) {
    let mut group = c.benchmark_group("clear_latched");
    group.throughput(Throughput::Elements((ROWS * COLS * FRAME_COUNT) as u64));

    group.bench_function("latched_dma_framebuffer_clear", |b| {
        // Create a formatted framebuffer once
        let mut fb = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();

        b.iter(|| {
            // Benchmark the fast clear operation that users call frequently
            black_box(&mut fb).clear();
        });
    });

    group.finish();
}

criterion_group!(benches, clear_latched);
criterion_main!(benches);
