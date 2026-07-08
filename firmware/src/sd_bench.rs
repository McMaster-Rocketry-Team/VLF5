//! Shared SD latency statistics for `sd_test` benchmarks.

use defmt::info;
use heapless::Vec;

/// Worst-case SD write stall observed on hardware (~37 ms). The write queue and
/// flight-data channel are sized to absorb at least this without dropping.
pub const WORST_CASE_WRITE_US: u32 = 40_000;

pub const MAX_LATENCY_SAMPLES: usize = 8192;

pub struct LatencyStats {
    pub count: u32,
    pub min: u32,
    pub max: u32,
    pub sum: u64,
    samples: Vec<u32, MAX_LATENCY_SAMPLES>,
    dropped_samples: u32,
}

impl LatencyStats {
    pub fn new() -> Self {
        Self {
            count: 0,
            min: u32::MAX,
            max: 0,
            sum: 0,
            samples: Vec::new(),
            dropped_samples: 0,
        }
    }

    pub fn record(&mut self, us: u32) {
        self.count += 1;
        self.min = self.min.min(us);
        self.max = self.max.max(us);
        self.sum += us as u64;
        if self.samples.push(us).is_err() {
            self.dropped_samples += 1;
        }
    }

    pub fn summarize(self, name: &str) {
        if self.count == 0 {
            info!("{}: no samples", name);
            return;
        }
        let avg = (self.sum / self.count as u64) as u32;
        let mut sorted = self.samples;
        sorted.sort_unstable();
        let p50 = percentile(&sorted, 50);
        let p95 = percentile(&sorted, 95);
        let p99 = percentile(&sorted, 99);
        let p999 = percentile_milli(&sorted, 999);
        info!(
            "{}: n={} min={}us avg={}us p50={}us p95={}us p99={}us p999={}us max={}us dropped_hist={}",
            name,
            self.count,
            self.min,
            avg,
            p50,
            p95,
            p99,
            p999,
            self.max,
            self.dropped_samples
        );
    }
}

fn percentile(sorted: &[u32], pct: u8) -> u32 {
    if sorted.is_empty() {
        return 0;
    }
    let idx = ((sorted.len() - 1) as u32 * pct as u32 / 100) as usize;
    sorted[idx]
}

fn percentile_milli(sorted: &[u32], permille: u16) -> u32 {
    if sorted.is_empty() {
        return 0;
    }
    let idx = ((sorted.len() - 1) as u32 * permille as u32 / 1000) as usize;
    sorted[idx]
}
