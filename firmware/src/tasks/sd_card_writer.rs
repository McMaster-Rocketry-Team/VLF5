use crate::{VLStatusMutex, tasks::data_logger::FlightDataChannel};

use defmt::{error, info, warn};
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{self, PC8, PC9, PC10, PC11, PC12, PD2, SDMMC1},
    sdmmc::{self, DataBlock, Sdmmc},
    time::mhz,
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Ticker, Timer};
use firmware_common_new::flight_storage::{
    BLOCK_SIZE, DATA_START_BLOCK, RECORDS_PER_BLOCK, RECORD_LEN, SUPERBLOCK_INDEX, USABLE_PER_BLOCK,
    decode_superblock, encode_response_header, encode_superblock, finalize_data_block,
    serialize_record,
};

/// A command from the USB host (see [`firmware_common_new::vlp::usb::CliRequest`]).
#[derive(Clone, Copy)]
pub enum StorageCommand {
    /// Report log metadata only (record / block counts).
    List,
    /// Stream the whole log back over USB.
    Download,
    /// Erase the log.
    Clear,
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
/// Depth gives the SD reader a little runway ahead of the slower USB writer.
pub type StorageRespChannel = Channel<NoopRawMutex, StorageResponse, 8>;

/// How often the in-progress (partial) block is flushed to the card so a power
/// cut never loses more than this much data.
const FLUSH_INTERVAL: Duration = Duration::from_millis(250);

/// Owns the SD peripheral and all logging state. Records are packed into
/// 512-byte blocks in RAM and written to raw SD sectors; block 0 holds a
/// superblock so the log survives a reboot.
struct FlightLogger {
    sdmmc: Sdmmc<'static, SDMMC1>,
    /// The block currently being filled (records at the front, CRC stamped on
    /// write).
    cur: [u8; BLOCK_SIZE],
    /// Bytes of `cur` occupied by records.
    cur_offset: usize,
    /// Records held in `cur`.
    cur_records: u32,
    /// SD block index that `cur` will be written to.
    write_index: u32,
    /// Total records in the log.
    record_count: u32,
    /// `cur` has records not yet persisted to the card.
    dirty: bool,
}

impl FlightLogger {
    /// Number of live data blocks the host should read back.
    fn block_count(&self) -> u32 {
        (self.write_index - DATA_START_BLOCK) + if self.cur_records > 0 { 1 } else { 0 }
    }

    /// Resume from the existing superblock if present, otherwise start fresh.
    async fn new(sdmmc: Sdmmc<'static, SDMMC1>) -> Self {
        let mut empty = Self {
            cur: [0u8; BLOCK_SIZE],
            cur_offset: 0,
            cur_records: 0,
            write_index: DATA_START_BLOCK,
            record_count: 0,
            dirty: false,
            sdmmc,
        };

        let mut sb = DataBlock([0u8; BLOCK_SIZE]);
        if empty.sdmmc.read_block(SUPERBLOCK_INDEX, &mut sb).await.is_ok()
            && let Some(info) = decode_superblock(&sb.0)
            && info.block_count > 0
        {
            let per_block = RECORDS_PER_BLOCK as u32;
            let full_blocks = info.block_count - 1;
            let mut records_in_last = info.record_count.saturating_sub(full_blocks * per_block);
            if records_in_last == 0 || records_in_last > per_block {
                // Counts disagree (corruption / interrupted write) — treat the
                // last block as full so the "only the last block is partial"
                // invariant the host relies on still holds.
                records_in_last = per_block;
            }
            let last_index = DATA_START_BLOCK + info.block_count - 1;
            let mut last = DataBlock([0u8; BLOCK_SIZE]);
            if empty.sdmmc.read_block(last_index, &mut last).await.is_ok() {
                empty.cur = last.0;
                empty.cur_records = records_in_last;
                empty.cur_offset = records_in_last as usize * RECORD_LEN;
                empty.write_index = last_index;
                empty.record_count = full_blocks * per_block + records_in_last;
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

    /// Append one serialised record, persisting the previous block when this one
    /// fills up. The partial tail is flushed separately by [`Self::persist`].
    async fn append(&mut self, bytes: &[u8]) {
        if self.cur_offset + bytes.len() > USABLE_PER_BLOCK {
            // `cur` is full — persist it before reusing the buffer, then advance.
            self.write_current_block().await;
            self.write_index += 1;
            self.cur = [0u8; BLOCK_SIZE];
            self.cur_offset = 0;
            self.cur_records = 0;
        }
        self.cur[self.cur_offset..self.cur_offset + bytes.len()].copy_from_slice(bytes);
        self.cur_offset += bytes.len();
        self.cur_records += 1;
        self.record_count += 1;
        self.dirty = true;
    }

    async fn write_current_block(&mut self) {
        let mut block = self.cur;
        finalize_data_block(&mut block);
        if let Err(e) = self.sdmmc.write_block(self.write_index, &DataBlock(block)).await {
            error!("SD write_block {} failed: {}", self.write_index, e);
        }
    }

    async fn write_superblock(&mut self) {
        let block = encode_superblock(self.record_count, self.block_count());
        if let Err(e) = self.sdmmc.write_block(SUPERBLOCK_INDEX, &DataBlock(block)).await {
            error!("SD superblock write failed: {}", e);
        }
    }

    /// Persist the in-progress block and superblock if anything changed.
    async fn persist(&mut self) {
        if self.dirty {
            self.write_current_block().await;
            self.write_superblock().await;
            self.dirty = false;
        }
    }

    async fn send_header(&self, resp: &StorageRespChannel) {
        let header = encode_response_header(self.record_count, self.block_count());
        resp.send(StorageResponse::chunk(&header)).await;
    }

    /// Handle a host command, pushing the reply onto `resp`.
    async fn handle_command(&mut self, cmd: StorageCommand, resp: &StorageRespChannel) {
        // Make the card consistent with RAM so a download sees every record.
        self.persist().await;

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
                    let mut blk = DataBlock([0u8; BLOCK_SIZE]);
                    match self.sdmmc.read_block(DATA_START_BLOCK + i, &mut blk).await {
                        Ok(()) => resp.send(StorageResponse::chunk(&blk.0)).await,
                        Err(e) => {
                            error!("SD read_block {} failed: {}", DATA_START_BLOCK + i, e);
                            break;
                        }
                    }
                }
            }
            StorageCommand::Clear => {
                info!("USB: clear storage");
                self.record_count = 0;
                self.write_index = DATA_START_BLOCK;
                self.cur = [0u8; BLOCK_SIZE];
                self.cur_offset = 0;
                self.cur_records = 0;
                self.dirty = false;
                self.write_superblock().await;
                self.send_header(resp).await;
            }
        }
        resp.send(StorageResponse::End).await;
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
) {
    bind_interrupts!(struct Irqs {
        SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
    });

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

    info!("SDMMC bus init clock: {}", sdmmc.clock().0);

    // Let the soldered SD-NAND's supply settle before talking to it.
    Timer::after_millis(50).await;

    // Bound each attempt with a timeout so a stuck transfer can't silently stall
    // the whole logger; if the card never comes up, keep the rest of the system
    // alive by draining the queue instead of spamming "backlogged" warnings.
    const INIT_TIMEOUT: Duration = Duration::from_secs(2);
    const INIT_ATTEMPTS: u8 = 5;
    let mut init_ok = false;
    for attempt in 1..=INIT_ATTEMPTS {
        // 25 MHz requested → ~20 MHz actual with the 40 MHz SDMMC kernel clock
        // (clkdiv=1). ~20x faster than the old 1 MHz for both logging and USB
        // readback. The SD-NAND traces are short; drop this if CRC errors appear.
        match select(Timer::after(INIT_TIMEOUT), sdmmc.init_sd_card(mhz(25))).await {
            Either::Second(Ok(())) => {
                info!("SD card initialised (attempt {})", attempt);
                init_ok = true;
                break;
            }
            Either::Second(Err(e)) => error!("SD init attempt {} failed: {}", attempt, e),
            Either::First(_) => warn!("SD init attempt {} timed out", attempt),
        }
        Timer::after_millis(200).await;
    }

    if !init_ok {
        error!("SD card unavailable; flight logging disabled");
        vl_status.lock(|s| s.borrow_mut().sd_ok = false);
        // Drain so the producer side doesn't back up and spam warnings.
        loop {
            let _ = channel.receive().await;
        }
    }
    vl_status.lock(|s| s.borrow_mut().sd_ok = true);

    let mut logger = FlightLogger::new(sdmmc).await;
    let mut flush_ticker = Ticker::every(FLUSH_INTERVAL);

    loop {
        // `select` is biased toward its first future, so the USB command goes
        // first: at ~416 Hz the record branch is almost always ready and would
        // otherwise starve host commands. A command only becomes ready when the
        // host actually sends one, so logging is unaffected in the common case.
        match select3(
            storage_cmd.wait(),
            channel.receive(),
            flush_ticker.next(),
        )
        .await
        {
            Either3::First(cmd) => {
                logger.handle_command(cmd, storage_resp).await;
            }
            Either3::Second(record) => {
                let bytes = serialize_record(&record);
                logger.append(&bytes).await;
            }
            Either3::Third(_) => {
                logger.persist().await;
            }
        }
    }
}
