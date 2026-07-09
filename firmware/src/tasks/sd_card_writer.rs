use crate::{SetTargetWatch, VLStatusMutex, tasks::data_logger::FlightDataChannel};

use defmt::{error, info, warn};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{self, PC8, PC9, PC10, PC11, PC12, PD2, SDMMC1},
    sdmmc::{self, Sdmmc},
    sdmmc::sd::{Addressable, CmdBlock, DataBlock, StorageDevice},
    time::mhz,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware_common_new::flight_storage::{
    AvionicsConfig, BLOCK_SIZE, DATA_START_BLOCK, DEFAULT_TARGET_APOGEE_AGL, MAX_WIRE_LEN,
    STORAGE_VERSION, SUPERBLOCK_INDEX, USABLE_PER_BLOCK, count_records_in_bytes, decode_config_block,
    decode_superblock, encode_config_block, encode_response_header, encode_superblock,
    finalize_data_block, serialize_log_record,
};
use heapless::Deque;

/// A command from the USB host (see [`firmware_common_new::vlp::usb::CliRequest`])
/// or from avionics (persist target apogee).
#[derive(Clone, Copy)]
pub enum StorageCommand {
    /// Report log metadata only (record / block counts).
    List,
    /// Stream the whole log back over USB.
    Download,
    /// Erase the log.
    Clear,
    /// Persist target apogee AGL (m) to the dedicated config block.
    SaveTargetApogee(f32),
}

/// A piece of a USB response, produced by the SD task and drained by the USB
/// task onto the bulk-IN endpoint.
pub enum StorageResponse {
    /// `len` valid bytes at the front of `buf` to write to the endpoint.
    Chunk { buf: [u8; BLOCK_SIZE], len: usize },
    /// End of this response — the USB task sends a zero-length packet.
    End,
}

impl StorageResponse {
    fn chunk(bytes: &[u8]) -> Self {
        let mut buf = [0u8; BLOCK_SIZE];
        buf[..bytes.len()].copy_from_slice(bytes);
        StorageResponse::Chunk {
            buf,
            len: bytes.len(),
        }
    }
}

/// Set by the USB control handler, awaited by the SD task.
pub type StorageCmdSignal = Signal<NoopRawMutex, StorageCommand>;
/// Filled by the SD task, drained by the USB task onto the bulk-IN endpoint.
pub type StorageRespChannel = embassy_sync::channel::Channel<NoopRawMutex, StorageResponse, 8>;

/// How often the superblock is updated so a power cut never loses more than this
/// much metadata. Partial data blocks are only rewritten when they have new records.
const FLUSH_INTERVAL: Duration = Duration::from_millis(250);

/// Pending SD jobs while the card is busy. Sized for a worst-case ~40 ms stall at
/// ~54 block commits/s plus periodic superblock updates.
const SD_WRITE_QUEUE_DEPTH: usize = 16;

/// Warn when the queue is deeper than this — sustained pressure means SDIO cannot
/// keep up with the producer.
const SD_QUEUE_WARN_DEPTH: usize = 8;

/// Worst-case single `write_block` latency seen on hardware (microseconds).
const WORST_CASE_WRITE_US: u32 = 40_000;

/// SDIO calls that exceed this are treated as a dead card so the task never hangs.
const SD_IO_TIMEOUT: Duration = Duration::from_secs(2);

/// Transient driver errors before the card is declared offline.
const SD_IO_ERROR_LIMIT: u8 = 3;

/// SD card init timeout per attempt.
const SD_INIT_TIMEOUT: Duration = Duration::from_secs(2);

/// SD card init attempts before giving up.
const SD_INIT_ATTEMPTS: u8 = 5;

#[derive(Clone, Copy)]
enum SdWriteJob {
    Data {
        index: u32,
        block: [u8; BLOCK_SIZE],
    },
    Superblock {
        block: [u8; BLOCK_SIZE],
    },
}

enum SdIoError {
    Timeout,
    Driver,
}

enum DrainResult {
    Ok,
    Failed,
    Dead,
}

enum AppendError {
    QueueFull,
    CardFull,
}

fn card_max_block_index(storage: &StorageDevice<'_, 'static, sdmmc::sd::Card>) -> u32 {
    (storage.card().size() / BLOCK_SIZE as u64).saturating_sub(1) as u32
}

/// Last block is reserved for [`AvionicsConfig`]; flight log must not use it.
fn flight_log_max_block_index(storage: &StorageDevice<'_, 'static, sdmmc::sd::Card>) -> u32 {
    card_max_block_index(storage).saturating_sub(1)
}

fn config_block_index(storage: &StorageDevice<'_, 'static, sdmmc::sd::Card>) -> u32 {
    card_max_block_index(storage)
}

async fn load_avionics_config(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
) -> AvionicsConfig {
    let index = config_block_index(storage);
    let mut block = DataBlock::new();
    if read_block_timed(storage, index, &mut block).await.is_ok()
        && let Some(cfg) = decode_config_block(&*block)
    {
        info!(
            "Loaded target apogee from SD: {} m AGL",
            cfg.target_apogee_agl
        );
        return cfg;
    }
    info!(
        "No SD config block; using default target apogee {} m AGL",
        DEFAULT_TARGET_APOGEE_AGL
    );
    AvionicsConfig::default()
}

async fn save_avionics_config(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    config: &AvionicsConfig,
    io_failures: &mut u8,
) -> bool {
    let index = config_block_index(storage);
    let raw = encode_config_block(config);
    let mut block = DataBlock::new();
    block.copy_from_slice(&raw);
    match write_block_timed(storage, index, &block).await {
        Ok(()) => {
            *io_failures = 0;
            info!(
                "Saved target apogee to SD: {} m AGL (block {})",
                config.target_apogee_agl, index
            );
            true
        }
        Err(SdIoError::Timeout) => {
            error!("SD config write timed out");
            false
        }
        Err(SdIoError::Driver) => {
            *io_failures = io_failures.saturating_add(1);
            error!("SD config write failed");
            *io_failures < SD_IO_ERROR_LIMIT
        }
    }
}

async fn write_block_timed(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    index: u32,
    block: &DataBlock,
) -> Result<(), SdIoError> {
    match select(
        Timer::after(SD_IO_TIMEOUT),
        storage.write_block(index, block),
    )
    .await
    {
        Either::First(_) => Err(SdIoError::Timeout),
        Either::Second(Ok(())) => Ok(()),
        Either::Second(Err(_)) => Err(SdIoError::Driver),
    }
}

async fn read_block_timed(
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    index: u32,
    block: &mut DataBlock,
) -> Result<(), SdIoError> {
    match select(
        Timer::after(SD_IO_TIMEOUT),
        storage.read_block(index, block),
    )
    .await
    {
        Either::First(_) => Err(SdIoError::Timeout),
        Either::Second(Ok(())) => Ok(()),
        Either::Second(Err(_)) => Err(SdIoError::Driver),
    }
}

/// In-RAM logging state. SDIO is decoupled via a pending write queue.
struct FlightLogger {
    cur: [u8; BLOCK_SIZE],
    cur_offset: usize,
    cur_records: u32,
    write_index: u32,
    record_count: u32,
    /// Bytes of `cur` already committed to the card at `write_index`.
    last_persisted_offset: usize,
    pending: Deque<SdWriteJob, SD_WRITE_QUEUE_DEPTH>,
    queue_warned: bool,
}

impl FlightLogger {
    fn block_count(&self) -> u32 {
        (self.write_index - DATA_START_BLOCK) + if self.cur_records > 0 { 1 } else { 0 }
    }

    fn empty() -> Self {
        Self {
            cur: [0u8; BLOCK_SIZE],
            cur_offset: 0,
            cur_records: 0,
            write_index: DATA_START_BLOCK,
            record_count: 0,
            last_persisted_offset: 0,
            pending: Deque::new(),
            queue_warned: false,
        }
    }

    fn drop_pending(&mut self) {
        self.pending.clear();
        self.queue_warned = false;
    }

    fn push_job(&mut self, job: SdWriteJob) -> Result<(), ()> {
        self.pending.push_back(job).map_err(|_| {
            error!("SD write queue full ({} jobs)", SD_WRITE_QUEUE_DEPTH);
        })?;
        if self.pending.len() >= SD_QUEUE_WARN_DEPTH && !self.queue_warned {
            warn!(
                "SD write queue depth {} (worst-case write {}us)",
                self.pending.len(),
                WORST_CASE_WRITE_US
            );
            self.queue_warned = true;
        }
        Ok(())
    }

    async fn new(storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>) -> Self {
        let mut empty = Self::empty();

        let mut sb = DataBlock::new();
        if read_block_timed(storage, SUPERBLOCK_INDEX, &mut sb)
            .await
            .is_ok()
            && let Some(info) = decode_superblock(&*sb)
            && info.storage_version == STORAGE_VERSION
            && info.block_count > 0
        {
            let last_index = DATA_START_BLOCK + info.block_count - 1;
            let mut last = DataBlock::new();
            if read_block_timed(storage, last_index, &mut last)
                .await
                .is_ok()
            {
                empty.cur.copy_from_slice(&*last);
                empty.cur_offset = info.last_block_offset as usize;
                empty.cur_records = count_records_in_bytes(&empty.cur, empty.cur_offset);
                empty.write_index = last_index;
                empty.record_count = info.record_count;
                empty.last_persisted_offset = empty.cur_offset;
                info!(
                    "Resuming log: {} records across {} blocks",
                    empty.record_count, info.block_count
                );
                return empty;
            }
        }

        info!("Starting fresh flight log");
        empty
    }

    /// Buffer a record. Full blocks are queued for SDIO — never written inline.
    fn has_space(&self, bytes_len: usize, max_block_index: u32) -> bool {
        let next_index = if self.cur_offset + bytes_len > USABLE_PER_BLOCK {
            self.write_index.saturating_add(1)
        } else {
            self.write_index
        };
        next_index <= max_block_index
    }

    fn append(&mut self, bytes: &[u8], max_block_index: u32) -> Result<(), AppendError> {
        if !self.has_space(bytes.len(), max_block_index) {
            return Err(AppendError::CardFull);
        }
        if self.cur_offset + bytes.len() > USABLE_PER_BLOCK {
            self.enqueue_current_block().map_err(|_| AppendError::QueueFull)?;
            self.write_index += 1;
            self.cur = [0u8; BLOCK_SIZE];
            self.cur_offset = 0;
            self.cur_records = 0;
            self.last_persisted_offset = 0;
        }
        self.cur[self.cur_offset..self.cur_offset + bytes.len()].copy_from_slice(bytes);
        self.cur_offset += bytes.len();
        self.cur_records += 1;
        self.record_count += 1;
        Ok(())
    }

    fn enqueue_current_block(&mut self) -> Result<(), ()> {
        let mut block = self.cur;
        finalize_data_block(&mut block);
        self.push_job(SdWriteJob::Data {
            index: self.write_index,
            block,
        })?;
        self.last_persisted_offset = self.cur_offset;
        Ok(())
    }

    fn enqueue_superblock(&mut self) -> Result<(), ()> {
        let raw = encode_superblock(
            self.record_count,
            self.block_count(),
            self.cur_offset as u32,
        );
        let mut block = [0u8; BLOCK_SIZE];
        block.copy_from_slice(&raw);
        self.push_job(SdWriteJob::Superblock { block })
    }

    /// Queue any dirty in-RAM data and a superblock update for crash safety.
    fn enqueue_flush(&mut self) -> Result<(), ()> {
        if self.cur_offset > self.last_persisted_offset {
            self.enqueue_current_block()?;
        }
        self.enqueue_superblock()?;
        self.queue_warned = false;
        Ok(())
    }

    fn note_io_result(io_failures: &mut u8, result: &Result<(), SdIoError>) -> DrainResult {
        match result {
            Ok(()) => {
                *io_failures = 0;
                DrainResult::Ok
            }
            Err(SdIoError::Timeout) => DrainResult::Dead,
            Err(SdIoError::Driver) => {
                *io_failures = io_failures.saturating_add(1);
                if *io_failures >= SD_IO_ERROR_LIMIT {
                    DrainResult::Dead
                } else {
                    DrainResult::Failed
                }
            }
        }
    }

    async fn drain_one(
        &mut self,
        storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
        io_failures: &mut u8,
    ) -> DrainResult {
        let Some(job) = self.pending.pop_front() else {
            return DrainResult::Ok;
        };

        let t0 = Instant::now();
        let result = match job {
            SdWriteJob::Data { index, block } => {
                let mut data = DataBlock::new();
                data.copy_from_slice(&block);
                write_block_timed(storage, index, &data).await
            }
            SdWriteJob::Superblock { block } => {
                let mut data = DataBlock::new();
                data.copy_from_slice(&block);
                write_block_timed(storage, SUPERBLOCK_INDEX, &data).await
            }
        };

        let elapsed_us = (Instant::now() - t0).as_micros() as u32;
        if elapsed_us > WORST_CASE_WRITE_US {
            warn!("SD write took {}us (queue depth {})", elapsed_us, self.pending.len());
        }

        let outcome = Self::note_io_result(io_failures, &result);
        if matches!(outcome, DrainResult::Failed | DrainResult::Dead) {
            match result {
                Err(SdIoError::Timeout) => {
                    error!("SD write timed out after {}ms", SD_IO_TIMEOUT.as_millis())
                }
                Err(SdIoError::Driver) => {
                    error!("SD write failed ({} consecutive errors)", *io_failures)
                }
                _ => {}
            }
        }
        outcome
    }

    async fn drain_all(
        &mut self,
        storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
        io_failures: &mut u8,
    ) -> bool {
        loop {
            match self.drain_one(storage, io_failures).await {
                DrainResult::Ok => {}
                DrainResult::Failed => return true,
                DrainResult::Dead => return false,
            }
            if self.pending.is_empty() {
                return true;
            }
        }
    }

    async fn send_header(&self, resp: &StorageRespChannel) {
        let header = encode_response_header(self.record_count, self.block_count());
        resp.send(StorageResponse::chunk(&header)).await;
    }

    async fn finish_response(&self, resp: &StorageRespChannel) {
        resp.send(StorageResponse::End).await;
    }

    async fn handle_command_offline(&mut self, cmd: StorageCommand, resp: &StorageRespChannel) {
        match cmd {
            StorageCommand::List => {
                warn!("USB: list while SD offline ({} records in RAM)", self.record_count);
                self.send_header(resp).await;
                self.finish_response(resp).await;
            }
            StorageCommand::Download => {
                warn!("USB: download while SD offline");
                self.send_header(resp).await;
                self.finish_response(resp).await;
            }
            StorageCommand::Clear => {
                info!("USB: clear in-RAM log (SD offline)");
                *self = Self::empty();
                self.send_header(resp).await;
                self.finish_response(resp).await;
            }
            StorageCommand::SaveTargetApogee(agl) => {
                warn!("SD offline; cannot persist target apogee {} m", agl);
            }
        }
    }

    async fn handle_command(
        &mut self,
        storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
        cmd: StorageCommand,
        resp: &StorageRespChannel,
        io_failures: &mut u8,
    ) -> bool {
        match cmd {
            StorageCommand::SaveTargetApogee(agl) => {
                return save_avionics_config(
                    storage,
                    &AvionicsConfig {
                        target_apogee_agl: agl,
                    },
                    io_failures,
                )
                .await;
            }
            _ => {}
        }

        let _ = self.enqueue_flush();
        if !self.drain_all(storage, io_failures).await {
            return false;
        }

        match cmd {
            StorageCommand::List => {
                info!("USB: list ({} records)", self.record_count);
                self.send_header(resp).await;
            }
            StorageCommand::Download => {
                let block_count = self.block_count();
                info!(
                    "USB: download {} records, {} blocks",
                    self.record_count, block_count
                );
                self.send_header(resp).await;
                for i in 0..block_count {
                    let mut blk = DataBlock::new();
                    if read_block_timed(storage, DATA_START_BLOCK + i, &mut blk)
                        .await
                        .is_ok()
                    {
                        resp.send(StorageResponse::chunk(&*blk)).await;
                    } else {
                        error!("SD read_block {} failed", DATA_START_BLOCK + i);
                        return false;
                    }
                }
            }
            StorageCommand::Clear => {
                info!("USB: clear storage");
                *self = Self::empty();
                let _ = self.enqueue_superblock();
                if !self.drain_all(storage, io_failures).await {
                    return false;
                }
                self.send_header(resp).await;
            }
            StorageCommand::SaveTargetApogee(_) => unreachable!(),
        }
        self.finish_response(resp).await;
        true
    }
}

fn mark_card_offline(
    card_alive: &mut bool,
    logger: &mut FlightLogger,
    vl_status: &VLStatusMutex,
) {
    if !*card_alive {
        return;
    }
    error!("SD card offline; flight logging isolated from avionics");
    logger.drop_pending();
    *card_alive = false;
    vl_status.lock(|s| s.borrow_mut().sd_ok = false);
}

async fn mark_card_full(
    log_full: &mut bool,
    logger: &mut FlightLogger,
    storage: &mut StorageDevice<'_, 'static, sdmmc::sd::Card>,
    io_failures: &mut u8,
    vl_status: &VLStatusMutex,
) {
    if *log_full {
        return;
    }
    warn!(
        "SD card full ({} records, {} blocks); stopping flight logging",
        logger.record_count,
        logger.block_count()
    );
    let _ = logger.enqueue_flush();
    let _ = logger.drain_all(storage, io_failures).await;
    *log_full = true;
    vl_status.lock(|s| s.borrow_mut().sd_ok = false);
}

async fn run_offline_loop(
    channel: &'static FlightDataChannel,
    storage_cmd: &'static StorageCmdSignal,
    storage_resp: &'static StorageRespChannel,
    vl_status: &'static VLStatusMutex,
) -> ! {
    let mut logger = FlightLogger::empty();
    loop {
        match select3(storage_cmd.wait(), channel.receive(), Timer::after(FLUSH_INTERVAL)).await {
            Either3::First(cmd) => logger.handle_command_offline(cmd, storage_resp).await,
            Either3::Second(_) => {}
            Either3::Third(_) => {}
        }
    }
}

#[embassy_executor::task]
pub async fn sd_card_writer(
    sdmmc1: Peri<'static, SDMMC1>,
    clk_pin: Peri<'static, PC12>,
    cmd_pin: Peri<'static, PD2>,
    d0_pin: Peri<'static, PC8>,
    d1_pin: Peri<'static, PC9>,
    d2_pin: Peri<'static, PC10>,
    d3_pin: Peri<'static, PC11>,

    channel: &'static FlightDataChannel,
    storage_cmd: &'static StorageCmdSignal,
    storage_resp: &'static StorageRespChannel,
    vl_status: &'static VLStatusMutex,
    target_agl_watch: &'static SetTargetWatch,
) {
    bind_interrupts!(struct Irqs {
        SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
    });

    vl_status.lock(|s| s.borrow_mut().sd_ok = false);

    let mut sdmmc = Sdmmc::new_4bit(
        sdmmc1,
        Irqs,
        clk_pin,
        cmd_pin,
        d0_pin,
        d1_pin,
        d2_pin,
        d3_pin,
        Default::default(),
    );

    Timer::after_millis(50).await;

    let mut storage = 'init: loop {
        for attempt in 1..=SD_INIT_ATTEMPTS {
            let mut cmd_block = CmdBlock::new();
            match select(
                Timer::after(SD_INIT_TIMEOUT),
                StorageDevice::new_sd_card(&mut sdmmc, &mut cmd_block, mhz(25)),
            )
            .await
            {
                Either::Second(Ok(s)) => {
                    info!("SD card initialised (attempt {})", attempt);
                    break 'init s;
                }
                Either::Second(Err(e)) => error!("SD init attempt {} failed: {}", attempt, e),
                Either::First(_) => warn!("SD init attempt {} timed out", attempt),
            }
            Timer::after_millis(200).await;
        }
        error!("SD card unavailable; avionics continues without SD logging");
        target_agl_watch.sender().send(DEFAULT_TARGET_APOGEE_AGL);
        run_offline_loop(channel, storage_cmd, storage_resp, vl_status).await;
    };

    let config = load_avionics_config(&mut storage).await;
    target_agl_watch.sender().send(config.target_apogee_agl);

    vl_status.lock(|s| s.borrow_mut().sd_ok = true);
    let max_block_index = flight_log_max_block_index(&storage);
    let mut logger = FlightLogger::new(&mut storage).await;
    let mut card_alive = true;
    let mut log_full = !logger.has_space(MAX_WIRE_LEN, max_block_index);

    if log_full {
        warn!("SD card log already occupies all available blocks");
        vl_status.lock(|s| s.borrow_mut().sd_ok = false);
    } else {
        info!(
            "SD log capacity: blocks {}..={} (config block {})",
            DATA_START_BLOCK,
            max_block_index,
            config_block_index(&storage)
        );
    }

    let mut flush_ticker = Ticker::every(FLUSH_INTERVAL);
    let mut io_failures = 0u8;

    loop {
        let mut go_offline = false;

        if card_alive {
            loop {
                match logger.drain_one(&mut storage, &mut io_failures).await {
                    DrainResult::Ok => {
                        if logger.pending.is_empty() {
                            break;
                        }
                    }
                    DrainResult::Failed => break,
                    DrainResult::Dead => {
                        go_offline = true;
                        break;
                    }
                }
            }
        }

        if go_offline {
            mark_card_offline(&mut card_alive, &mut logger, vl_status);
            log_full = true;
            continue;
        }

        match select3(storage_cmd.wait(), channel.receive(), flush_ticker.next()).await {
            Either3::First(cmd) => {
                if card_alive {
                    if !logger
                        .handle_command(&mut storage, cmd, storage_resp, &mut io_failures)
                        .await
                    {
                        mark_card_offline(&mut card_alive, &mut logger, vl_status);
                        log_full = true;
                    } else if matches!(cmd, StorageCommand::Clear) {
                        log_full = false;
                        vl_status.lock(|s| s.borrow_mut().sd_ok = true);
                    }
                } else {
                    logger.handle_command_offline(cmd, storage_resp).await;
                }
            }
            Either3::Second(record) => {
                if !card_alive || log_full {
                    continue;
                }
                let (bytes, len) = serialize_log_record(&record);
                match logger.append(&bytes[..len], max_block_index) {
                    Ok(()) => {}
                    Err(AppendError::CardFull) => {
                        mark_card_full(
                            &mut log_full,
                            &mut logger,
                            &mut storage,
                            &mut io_failures,
                            vl_status,
                        )
                        .await;
                    }
                    Err(AppendError::QueueFull) => {
                        warn!("SD queue full, dropping flight record");
                    }
                }
            }
            Either3::Third(_) => {
                if card_alive && !log_full {
                    if logger.enqueue_flush().is_err() {
                        warn!("SD flush enqueue failed (queue full)");
                    }
                }
            }
        }
    }
}
