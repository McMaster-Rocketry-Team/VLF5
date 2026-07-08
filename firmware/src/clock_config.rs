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
    rcc.sys = Sysclk::Pll1P;

    rcc.pll1 = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul60,
        fracn: None,
        divp: Some(PllDiv::Div2),
        divq: Some(PllDiv::Div20),
        divr: None,
    });
    rcc.pll2 = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul10,
        fracn: None,
        divp: Some(PllDiv::Div4),
        divq: Some(PllDiv::Div4),
        divr: Some(PllDiv::Div4),
    });
    rcc.pll3 = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul10,
        fracn: None,
        divp: None,
        divq: None,
        divr: Some(PllDiv::Div8),
    });

    rcc.d1c_pre = AHBPrescaler::Div1;
    rcc.ahb_pre = AHBPrescaler::Div2;
    rcc.apb1_pre = APBPrescaler::Div2;
    rcc.apb2_pre = APBPrescaler::Div2;
    rcc.apb3_pre = APBPrescaler::Div2;
    rcc.apb4_pre = APBPrescaler::Div2;

    rcc.timer_prescaler = TimerPrescaler::DefaultX2;
    rcc.voltage_scale = VoltageScale::Scale0;

    rcc.ls = LsConfig::default_lsi();
    rcc.mux.spi123sel = Saisel::Pll2P;
    rcc.mux.usart16910sel = Usart16910sel::Pll2Q;
    rcc.mux.rngsel = Rngsel::Pll1Q;
    rcc.mux.i2c1235sel = I2c1235sel::Pll3R;
    rcc.mux.spi45sel = Spi45sel::Pll2Q;
    rcc.mux.adcsel = Adcsel::Pll2P;
    rcc.mux.usbsel = Usbsel::Pll1Q;
    rcc.mux.fdcansel = Fdcansel::Pll2Q;
    rcc.mux.sdmmcsel = Sdmmcsel::Pll2R;

    config
}
