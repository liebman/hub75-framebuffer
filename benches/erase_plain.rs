// Run with:  cargo bench --bench clear_plain

use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use hub75_framebuffer::plain::DmaFrameBuffer;
use std::hint::black_box;
use std::time::Duration;

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 3;
const NROWS: usize = hub75_framebuffer::compute_rows(ROWS);
const FRAME_COUNT: usize = hub75_framebuffer::compute_frame_count(BITS);

// Number of iterations to target ~1-5ms per measurement
const ITERATIONS: usize = 1000;

fn configure_criterion() -> Criterion {
    Criterion::default()
        .sample_size(100)
        .measurement_time(Duration::from_secs(10)) // Longer measurement time
        .warm_up_time(Duration::from_secs(3))
        .confidence_level(0.95)
        .significance_level(0.05)
}

fn erase_plain(c: &mut Criterion) {
    let mut group = c.benchmark_group("Plain Implementation");
    group.throughput(Throughput::Elements(
        (ROWS * COLS * FRAME_COUNT * ITERATIONS) as u64,
    ));

    group.bench_function("erase", |b| {
        // Create a formatted framebuffer once
        let mut fb = DmaFrameBuffer::<ROWS, COLS, NROWS, BITS, FRAME_COUNT>::new();

        b.iter(|| {
            black_box(&mut fb).erase();
        });
    });

    group.finish();
}

criterion_group!(name = benches; config = configure_criterion(); targets = erase_plain);
criterion_main!(benches);
