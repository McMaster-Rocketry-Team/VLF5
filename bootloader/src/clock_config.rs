use embassy_stm32::rcc::mux::*;
use embassy_stm32::rcc::*;
use embassy_stm32::time::mhz;
use embassy_stm32::Config;

pub fn vlf5_clock_config() -> Config {
    let mut config = embassy_stm32::Config::default();
    let rcc = &mut config.rcc;

    rcc.hsi = None;
    rcc.hse = Some(Hse {
        freq: mhz(16),
        mode: HseMode::Oscillator,
    });
    rcc.csi = false;

    rcc.hsi48 = None;
    rcc.sys = Sysclk::PLL1_P;

    rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL60,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV20),
        divr: None,
    });
    rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL10,
        divp: Some(PllDiv::DIV4),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV4),
    });
    rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL10,
        divp: None,
        divq: None,
        divr: Some(PllDiv::DIV8),
    });

    rcc.d1c_pre = AHBPrescaler::DIV1;
    rcc.ahb_pre = AHBPrescaler::DIV2;
    rcc.apb1_pre = APBPrescaler::DIV2;
    rcc.apb2_pre = APBPrescaler::DIV2;
    rcc.apb3_pre = APBPrescaler::DIV2;
    rcc.apb4_pre = APBPrescaler::DIV2;

    rcc.timer_prescaler = TimerPrescaler::DefaultX2;
    rcc.voltage_scale = VoltageScale::Scale0;

    rcc.ls = LsConfig::default_lsi();
    rcc.mux.spi123sel = Saisel::PLL2_P;
    rcc.mux.usart16910sel = Usart16910sel::PLL2_Q;
    rcc.mux.rngsel = Rngsel::PLL1_Q;
    rcc.mux.i2c1235sel = I2c1235sel::PLL3_R;
    rcc.mux.spi45sel = Spi45sel::PLL2_Q;
    rcc.mux.adcsel = Adcsel::PLL2_P;
    rcc.mux.usbsel = Usbsel::PLL1_Q;
    rcc.mux.fdcansel = Fdcansel::PLL2_Q;
    rcc.mux.sdmmcsel = Sdmmcsel::PLL2_R;

    config
}
