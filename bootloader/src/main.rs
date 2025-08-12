#![no_std]
#![no_main]
#![feature(slice_as_array)]
#![feature(impl_trait_in_assoc_type)]

mod fmt;

mod can;
mod clock_config;
mod ota;

use core::{cell::RefCell, mem::MaybeUninit, ptr::write_volatile};

use crate::clock_config::vlf5_clock_config;
use can::start_can_bus_tasks;
use cortex_m::singleton;
use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
    peripherals::{PA7, PB1},
    wdg::IndependentWatchdog,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware_common_new::can_bus::{
    messages::{
        CanBusMessageEnum,
        node_status::{NodeHealth, NodeMode, NodeStatusMessage},
        reset::ResetMessage,
    },
    receiver::CanReceiver,
    sender::CanSender,
};
use ota::ota_task;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

/// # BACKUP_RAM\[0\]:
///
/// 0x69426942 to indicate the next boot should go into bootloader.
///
/// # Trial boots
///
/// Prior to loading the main application, the bootloader will set BACKUP_RAM\[0\]
/// to 0x69426942 and start a watchdog that if not refreshed in 1 second, will reset
/// the device.
///
/// After the main application is started, it should reset BACKUP_RAM\[0\] to 0 and
/// refresh or disable the watchdog.
///
/// If the main application failed to start, the watchdog will reset the device and
/// due to the magic number in BACKUP_RAM\[0\], the device will stay in bootloader.
#[unsafe(link_section = ".backup_ram")]
static mut BACKUP_RAM: MaybeUninit<[u32; 2]> = MaybeUninit::uninit();

pub enum BootOption {
    Bootloader,
    Application,
}

pub fn configure_next_boot(boot_option: BootOption) {
    let backup_ram = unsafe {
        #[allow(static_mut_refs)]
        BACKUP_RAM.assume_init_mut()
    };
    let magic = match boot_option {
        BootOption::Bootloader => 0x69426942,
        BootOption::Application => 0,
    };
    unsafe {
        write_volatile(backup_ram.as_mut_ptr(), magic);
    }
}

pub fn app_address() -> u32 {
    unsafe extern "C" {
        static __app_address: u32;
    }
    unsafe { &__app_address as *const u32 as u32 }
}

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(vlf5_clock_config());

    let backup_ram = unsafe {
        #[allow(static_mut_refs)]
        BACKUP_RAM.assume_init_mut()
    };

    if backup_ram[0] != 0x69426942 {
        // no magic number found, boot normally
        configure_next_boot(BootOption::Bootloader);
        // let mut wdt = IndependentWatchdog::new(p.IWDG1, 1_000_000);
        // wdt.unleash();

        unsafe {
            let mut p = cortex_m::Peripherals::steal();
            p.SCB.invalidate_icache();
            p.SCB.clean_dcache(&mut p.CPUID);
            p.SCB.vtor.write(app_address());
            cortex_m::asm::bootload(app_address() as *const u32);
        }
    }

    // enter DFU only if magic number detected
    log_info!("DFU!");
    let executor = singleton!(: Executor = Executor::new()).unwrap();
    let ota_started = singleton!(: RefCell<bool> = RefCell::new(false)).unwrap();
    executor.run(|spawner| {
        spawner.must_spawn(status_led_task(p.PB1, p.PA7));
        let (can_sender, can_receiver) = start_can_bus_tasks(&spawner, p.FDCAN2, p.PB5, p.PB6);
        spawner.must_spawn(node_status_task(can_sender));
        spawner.must_spawn(can_reset_task(can_receiver));
        spawner.must_spawn(ota_task(ota_started, can_sender, can_receiver, p.FLASH));
        spawner.must_spawn(ota_timeout_task(ota_started));
    });
}

#[embassy_executor::task]
async fn status_led_task(red_led: Peri<'static, PB1>, green_led: Peri<'static, PA7>) {
    let mut red_led = Output::new(red_led, Level::High, Speed::Low);
    let mut green_led = Output::new(green_led, Level::High, Speed::Low);

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        red_led.set_low();
        green_led.set_low();
        Timer::after_millis(50).await;
        red_led.set_high();
        green_led.set_high();
        Timer::after_millis(50).await;
        red_led.set_low();
        green_led.set_low();
        Timer::after_millis(50).await;
        red_led.set_high();
        green_led.set_high();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn node_status_task(can_sender: &'static CanSender<NoopRawMutex, 4>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        can_sender
            .send(
                NodeStatusMessage::new_no_custom_status(
                    Instant::now().as_secs() as u32,
                    NodeHealth::Healthy,
                    NodeMode::Maintenance,
                )
                .into(),
            )
            ;
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn can_reset_task(can_receiver: &'static CanReceiver<NoopRawMutex, 4, 2>) {
    let mut subscriber = can_receiver.subscriber().unwrap();
    loop {
        let can_message = subscriber.next_message_pure().await.data.message;

        if let CanBusMessageEnum::Reset(ResetMessage {
            node_id,
            reset_all,
            into_bootloader,
        }) = can_message
            && (node_id == can_receiver.self_node_id() || reset_all)
        {
            configure_next_boot(if into_bootloader {
                BootOption::Bootloader
            } else {
                BootOption::Application
            });
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

#[embassy_executor::task]
async fn ota_timeout_task(ota_started: &'static RefCell<bool>) {
    Timer::after_secs(2).await;
    if !*ota_started.borrow() {
        configure_next_boot(BootOption::Application);
        cortex_m::peripheral::SCB::sys_reset();
    }
}

#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
#[cfg(not(feature = "defmt"))]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[cortex_m_rt::exception]
#[cfg(not(feature = "defmt"))]
unsafe fn DefaultHandler(_: i16) -> ! {
    panic!();
}

#[panic_handler]
#[cfg(not(feature = "defmt"))]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}
