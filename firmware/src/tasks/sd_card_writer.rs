use crate::{VLStatusMutex, tasks::data_logger::FlightDataChannel};

use defmt::{error, info};
use embassy_stm32::{
    Peri, bind_interrupts,
    peripherals::{self, PC8, PC9, PC10, PC11, PC12, PD2, SDMMC1},
    sdmmc::{self, DataBlock, Sdmmc},
    time::mhz,
};
use firmware_common_new::flight_data_record::FlightDataRecord;
use rkyv::{
    api::low::to_bytes_in_with_alloc,
    rancor::Failure,
    ser::{allocator::SubAllocator, writer::Buffer},
};

const RECORD_LEN: usize = size_of::<<FlightDataRecord as rkyv::Archive>::Archived>();

/// rkyv requires this
#[repr(C, align(16))]
struct Serialized([u8; RECORD_LEN]);

/// Last 4 bytes of every 512-byte block are reserved for a CRC (written at
const USABLE: usize = 508;

/// Accumulates serialized records into 512-byte SD blocks. Records never
/// straddle a block boundary (the tail is zero-padded).
struct BlockBuffer {
    buf: DataBlock,
    offset: usize,
}

impl BlockBuffer {
    const fn new() -> Self {
        Self {
            buf: DataBlock([0u8; 512]),
            offset: 0,
        }
    }

    /// Append a record's bytes. Returns the completed 512-byte block when the
    /// record didn't fit and a fresh block was started.
    fn push(&mut self, bytes: &[u8]) -> Option<[u8; 512]> {
        if self.offset + bytes.len() > USABLE {

            // clear buffer
            self.buf = DataBlock([0u8; 512]);

            self.buf[..bytes.len()].copy_from_slice(bytes);
            self.offset = bytes.len();
            return Some(self.buf.0);
        }
        self.buf[self.offset..self.offset + bytes.len()].copy_from_slice(bytes);
        self.offset += bytes.len();
        None
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

    info!("SDMMC clock: {}", sdmmc.clock().0);
    match sdmmc.init_sd_card(mhz(1)).await {
        Ok(()) => {
            info!("SD card initialised!");
            vl_status.lock(|s| s.borrow_mut().sd_ok = true);
        }
        Err(e) => {
            error!("SD card init failed: {}", e);
            vl_status.lock(|s| s.borrow_mut().sd_ok = false);
            return;
        }
    }

    // Blocks 0 & 1 are reserved for the ping-pong superblock (Phase 2).
    let mut block_index: u32 = 2;
    let mut block_buf = BlockBuffer::new();

    loop {
        let record = channel.receive().await;

        let mut scratch = Serialized([0u8; RECORD_LEN]);
        to_bytes_in_with_alloc::<_, _, Failure>(
            &record,
            Buffer::from(&mut scratch.0[..]),
            SubAllocator::empty(),
        )
        .unwrap();

        if let Some(_full_block) = block_buf.push(&scratch.0) {
            // TODO(phase 2): CRC `_full_block`'s last 4 bytes, then
            //   sdmmc.write_block(block_index, &DataBlock(_full_block)).await
            // plus a capacity guard and periodic superblock updates.
            block_index = block_index.wrapping_add(1);
        }
    }
}
