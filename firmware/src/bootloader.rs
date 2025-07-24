use core::{mem::MaybeUninit, ptr::write_volatile};

use embassy_stm32::{peripherals::IWDG1, wdg::IndependentWatchdog, Peri};
use embassy_time::{Duration, Ticker};

/// # BACKUP_RAM\[0\]:
///
/// 0x69426942 to indicate the next boot should go into bootloader.
///
/// # Trial boots
///
/// Prior to loading the main application, the bootloader will keep BACKUP_RAM\[0\]
/// at 0x69426942 and start a watchdog that if not refreshed in 1 second, will reset
/// the device.
///
/// After the main application is started, it should reset BACKUP_RAM\[0\] to 0 and
/// refresh or disable the watchdog.
///
/// If the main application failed to start, the watchdog will reset the device and
/// due to the magic number in BACKUP_RAM\[0\], the device will stay in bootloader.
#[unsafe(link_section = ".backup_ram")]
static mut BACKUP_RAM: MaybeUninit<[u32; 256]> = MaybeUninit::uninit();

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

#[embassy_executor::task]
pub async fn watchdog_task(wdt: Peri<'static, IWDG1>) {
    configure_next_boot(BootOption::Application);
    let mut wdt = IndependentWatchdog::new(wdt, 500_000);
    wdt.unleash();

    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop {
        wdt.pet();
        ticker.next().await;
    }
}