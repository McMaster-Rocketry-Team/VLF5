#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(never_type)]
#![feature(try_blocks)]

mod clock_config;
mod sd_bench;

use {defmt_rtt_pipe as _, panic_probe as _};

use clock_config::vlf5_clock_config;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::sdmmc::sd::{CmdBlock, DataBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_stm32::{
    bind_interrupts, peripherals,
    rng::{self, Rng},
};
use embassy_stm32::{
    crc::{Config as CrcConfig, Crc, InputReverseConfig, PolySize},
    sdmmc::{self, Sdmmc},
};
use embassy_time::{Duration, Instant, Timer};
use firmware_common_new::flight_storage::{BLOCK_SIZE, RECORD_LEN, USABLE_PER_BLOCK};
use heapless::Deque;
use sd_bench::{LatencyStats, WORST_CASE_WRITE_US};

const SD_CLOCK: u32 = 25_000_000;
const BENCH_BLOCKS: usize = 8192;
const LOGGING_DURATION: Duration = Duration::from_secs(60);
const FLUSH_INTERVAL: Duration = Duration::from_millis(250);
const SD_WRITE_QUEUE_DEPTH: usize = 16;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

#[derive(Clone, Copy)]
enum SdWriteJob {
    Data { index: u32, block: [u8; BLOCK_SIZE] },
    Superblock { block: [u8; BLOCK_SIZE] },
}

struct QueuedLogger {
    cur: [u8; BLOCK_SIZE],
    cur_offset: usize,
    write_index: u32,
    last_persisted_offset: usize,
    pending: Deque<SdWriteJob, SD_WRITE_QUEUE_DEPTH>,
}

impl QueuedLogger {
    fn new(start_block: u32) -> Self {
        Self {
            cur: [0u8; BLOCK_SIZE],
            cur_offset: 0,
            write_index: start_block,
            last_persisted_offset: 0,
            pending: Deque::new(),
        }
    }

    fn append(&mut self, record: &[u8; RECORD_LEN]) -> Result<(), ()> {
        if self.cur_offset + RECORD_LEN > USABLE_PER_BLOCK {
            self.enqueue_current()?;
            self.write_index += 1;
            self.cur = [0u8; BLOCK_SIZE];
            self.cur_offset = 0;
            self.last_persisted_offset = 0;
        }
        self.cur[self.cur_offset..self.cur_offset + RECORD_LEN].copy_from_slice(record);
        self.cur_offset += RECORD_LEN;
        Ok(())
    }

    fn enqueue_current(&mut self) -> Result<(), ()> {
        self.pending
            .push_back(SdWriteJob::Data {
                index: self.write_index,
                block: self.cur,
            })
            .map_err(|_| ())?;
        self.last_persisted_offset = self.cur_offset;
        Ok(())
    }

    fn enqueue_flush(&mut self, superblock: &[u8; BLOCK_SIZE]) -> Result<(), ()> {
        if self.cur_offset > self.last_persisted_offset {
            self.enqueue_current()?;
        }
        self.pending
            .push_back(SdWriteJob::Superblock { block: *superblock })
            .map_err(|_| ())
    }

    fn pop_job(&mut self) -> Option<SdWriteJob> {
        self.pending.pop_front()
    }

    fn queue_depth(&self) -> usize {
        self.pending.len()
    }
}

async fn bench_sequential_writes(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    start_block: u32,
    block: &DataBlock,
) -> LatencyStats {
    let mut stats = LatencyStats::new();
    let t0 = Instant::now();
    for i in 0..BENCH_BLOCKS {
        let t_start = Instant::now();
        storage
            .write_block(start_block + i as u32, block)
            .await
            .unwrap();
        stats.record((Instant::now() - t_start).as_micros() as u32);
    }
    let elapsed_ms = (Instant::now() - t0).as_millis();
    let bytes = BENCH_BLOCKS * BLOCK_SIZE;
    let kb_per_s = bytes as u64 * 1000 / elapsed_ms.max(1) / 1024;
    info!(
        "sequential: {} blocks in {}ms -> {} KiB/s",
        BENCH_BLOCKS,
        elapsed_ms,
        kb_per_s
    );
    stats
}

async fn drain_one(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    job: SdWriteJob,
    stats: &mut LatencyStats,
) {
    let t_start = Instant::now();
    match job {
        SdWriteJob::Data { index, block } => {
            let mut data = DataBlock::new();
            data.copy_from_slice(&block);
            storage.write_block(index, &data).await.unwrap();
        }
        SdWriteJob::Superblock { block } => {
            let mut data = DataBlock::new();
            data.copy_from_slice(&block);
            storage.write_block(0, &data).await.unwrap();
        }
    }
    stats.record((Instant::now() - t_start).as_micros() as u32);
}

/// Production-shaped logger: RAM append + write queue + periodic flush.
async fn bench_logging_pattern(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    start_block: u32,
    record: &[u8; RECORD_LEN],
    superblock: &[u8; BLOCK_SIZE],
    duration: Duration,
) -> LatencyStats {
    let mut stats = LatencyStats::new();
    let mut logger = QueuedLogger::new(start_block);
    let mut flush_at = Instant::now() + FLUSH_INTERVAL;
    let deadline = Instant::now() + duration;
    let mut record_count = 0u32;
    let mut max_queue = 0usize;

    while Instant::now() < deadline {
        while let Some(job) = logger.pop_job() {
            drain_one(storage, job, &mut stats).await;
        }

        if let Err(()) = logger.append(record) {
            while let Some(job) = logger.pop_job() {
                drain_one(storage, job, &mut stats).await;
            }
            let _ = logger.append(record);
        }
        record_count += 1;

        if Instant::now() >= flush_at {
            let _ = logger.enqueue_flush(superblock);
            flush_at += FLUSH_INTERVAL;
        }

        max_queue = max_queue.max(logger.queue_depth());
        Timer::after_micros(4_680).await;
    }

    let _ = logger.enqueue_flush(superblock);
    while let Some(job) = logger.pop_job() {
        drain_one(storage, job, &mut stats).await;
    }

    info!(
        "logging pattern: {} records, {} writes, max_queue={} (budget={})",
        record_count,
        stats.count,
        max_queue,
        SD_WRITE_QUEUE_DEPTH
    );
    stats
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(vlf5_clock_config());

    info!(
        "VLF5 SD benchmark ({} MHz SDIO, {} blocks)",
        SD_CLOCK / 1_000_000,
        BENCH_BLOCKS
    );

    let mut sdmmc = Sdmmc::new_4bit(
        p.SDMMC1,
        Irqs,
        p.PC12,
        p.PD2,
        p.PC8,
        p.PC9,
        p.PC10,
        p.PC11,
        Default::default(),
    );

    let mut cmd_block = CmdBlock::new();
    let mut storage = StorageDevice::new_sd_card(&mut sdmmc, &mut cmd_block, mhz(25))
        .await
        .unwrap();

    let mut rng = Rng::new(p.RNG, Irqs);
    let crc_config =
        CrcConfig::new(InputReverseConfig::None, false, PolySize::Width8, 69, 69).unwrap();
    let mut crc = Crc::new(p.CRC, crc_config);

    let mut block = DataBlock::new();
    rng.async_fill_bytes(&mut block[..USABLE_PER_BLOCK])
        .await
        .unwrap();
    crc.reset();
    crc.feed_bytes(&block[..USABLE_PER_BLOCK]);
    block[USABLE_PER_BLOCK..BLOCK_SIZE].copy_from_slice(&crc.read().to_le_bytes());

    let mut record = [0u8; RECORD_LEN];
    rng.async_fill_bytes(&mut record).await.unwrap();

    let mut superblock = [0u8; BLOCK_SIZE];
    superblock[..16].copy_from_slice(b"VLF5bench0000000");

    info!(
        "worst-case write budget: {}us, queue depth {}",
        WORST_CASE_WRITE_US,
        SD_WRITE_QUEUE_DEPTH
    );

    const BENCH_BASE: u32 = 50_000;

    let seq = bench_sequential_writes(&mut storage, BENCH_BASE, &block).await;
    seq.summarize("sequential write latency");

    let log = bench_logging_pattern(
        &mut storage,
        BENCH_BASE + BENCH_BLOCKS as u32,
        &record,
        &superblock,
        LOGGING_DURATION,
    )
    .await;
    log.summarize("logging-pattern write latency");

    info!("SD benchmark complete");
}
