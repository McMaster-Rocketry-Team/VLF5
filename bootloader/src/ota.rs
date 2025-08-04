use core::cell::RefCell;

use binary_macros::base64;
use embassy_stm32::{
    Peri,
    flash::{Flash, WRITE_SIZE},
    peripherals::FLASH,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Timer;
use firmware_common_new::{
    bootloader::verify_firmware,
    can_bus::{
        messages::{CanBusMessageEnum, ack::AckMessage, data_transfer::DataType},
        receiver::CanReceiver,
        sender::CanSender,
    },
};
use heapless::Vec;
use salty::Sha512;

use crate::{BootOption, app_address, configure_next_boot};

static PUBLIC_KEY: &[u8] = base64!("file:../../Rust_Monorepo/bootloader/public.key");

#[embassy_executor::task]
pub async fn ota_task(
    ota_started: &'static RefCell<bool>,
    can_sender: &'static CanSender<NoopRawMutex, 4>,
    can_receiver: &'static CanReceiver<NoopRawMutex, 4, 2>,
    flash: Peri<'static, FLASH>,
) {
    // writing to flash stalls the cpu anyways, so its fine to use blocking api here
    let mut flash = Flash::new_blocking(flash);
    const PAGE_SIZE: u32 = 2 * 1024;
    let mut sha512 = Sha512::new();
    let mut signature = Vec::<u8, 64>::new();
    let firmware_start_address = app_address() - 0x08000000;
    let mut firmware_write_offset = firmware_start_address;
    let mut last_sequence_number = 0u8;
    let mut write_buffer = Vec::<u8, WRITE_SIZE>::new();
    let mut first_page_data = Vec::<u8, { PAGE_SIZE as usize }>::new();
    let mut can_sub = can_receiver.subscriber().unwrap();

    loop {
        let message = can_sub.next_message_pure().await.data;

        if let CanBusMessageEnum::DataTransfer(data_transfer) = message.message
            && data_transfer.data_type == DataType::Firmware
            && data_transfer.destination_node_id == can_receiver.self_node_id()
        {
            if data_transfer.start_of_transfer {
                {
                    let mut ota_started = ota_started.borrow_mut();
                    *ota_started = true;
                }
                sha512 = Sha512::new();
                signature.clear();
                firmware_write_offset = firmware_start_address;
                last_sequence_number = data_transfer.sequence_number;
                write_buffer.clear();
            } else {
                last_sequence_number = last_sequence_number.wrapping_add(1);
                if data_transfer.sequence_number != last_sequence_number {
                    log_warn!(
                        "out of order message detected, expected sequence number {}, received {}",
                        last_sequence_number,
                        data_transfer.sequence_number
                    );
                    continue;
                }
            }

            let mut data = data_transfer.data();
            while data.len() > 0 {
                let copy_len = (WRITE_SIZE - write_buffer.len()).min(data.len());
                write_buffer.extend_from_slice(&data[..copy_len]).unwrap();
                data = &data[copy_len..];

                if write_buffer.len() == WRITE_SIZE {
                    if signature.len() < signature.capacity() {
                        // first 64 byte of the "firmware" is the signature
                        log_info!("write signature");
                        signature.extend_from_slice(&write_buffer).unwrap();
                    } else {
                        if firmware_write_offset % PAGE_SIZE == 0 {
                            log_info!("erase flash");
                            flash
                                .blocking_erase(
                                    firmware_write_offset,
                                    firmware_write_offset + PAGE_SIZE,
                                )
                                .unwrap();
                        }

                        sha512.update(&write_buffer);

                        if !first_page_data.is_full() {
                            log_info!("write first page");
                            first_page_data.extend_from_slice(&write_buffer).unwrap();
                        } else {
                            log_info!("write flash");
                            flash
                                .blocking_write(firmware_write_offset, &write_buffer)
                                .unwrap();
                        }
                        firmware_write_offset += WRITE_SIZE as u32;
                    }
                    write_buffer.clear();
                }
            }

            can_sender.send(
                AckMessage {
                    crc: message.crc,
                    node_id: message.id.node_id,
                }
                .into(),
            );

            if data_transfer.end_of_transfer {
                log_info!("firmware received");

                let sha512_bytes = sha512.finalize();
                sha512 = Sha512::new();

                if !signature.is_full() {
                    log_warn!("signature not full");
                    continue;
                }

                if !first_page_data.is_full() {
                    log_warn!("first_page_data not full");
                    continue;
                }

                match verify_firmware(
                    &sha512_bytes,
                    signature.as_slice().try_into().unwrap(),
                    PUBLIC_KEY.try_into().unwrap(),
                ) {
                    Ok(()) => {
                        log_info!("Firmware verification succeeded, writing the first page");

                        let mut address = firmware_start_address + PAGE_SIZE - WRITE_SIZE as u32;
                        let mut first_page_data = first_page_data.as_slice();
                        while !first_page_data.is_empty() {
                            flash
                                .blocking_write(
                                    address,
                                    &first_page_data[first_page_data.len() - 8..],
                                )
                                .unwrap();
                            first_page_data = &first_page_data[..first_page_data.len() - 8];
                            address -= WRITE_SIZE as u32;
                        }

                        log_info!("Rebooting to application");
                        // wait 50ms to make sure the ack message is sent
                        Timer::after_millis(50).await;
                        configure_next_boot(BootOption::Application);
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    Err(_e) => {
                        // FIXME: even if firmware verification failed it
                        // is still possible to go into the firmware after reboot
                        // TODO do not write the first page until verification succeed
                        log_warn!("Firmware verification failed: {}", _e)
                    }
                };
            }
        }
    }
}
