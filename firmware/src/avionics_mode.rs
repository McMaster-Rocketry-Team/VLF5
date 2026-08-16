#[derive(Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum AvionicsMode {
    Armed,
    SelfTest,
    LowPower,
    Landed,
    Demo,
}

impl From<firmware_common_new::vlp::packets::change_mode::Mode> for AvionicsMode {
    fn from(value: firmware_common_new::vlp::packets::change_mode::Mode) -> Self {
        match value {
            firmware_common_new::vlp::packets::change_mode::Mode::LowPower => AvionicsMode::LowPower,
            firmware_common_new::vlp::packets::change_mode::Mode::SelfTest => AvionicsMode::SelfTest,
            firmware_common_new::vlp::packets::change_mode::Mode::Armed => AvionicsMode::Armed,
            firmware_common_new::vlp::packets::change_mode::Mode::Landed => AvionicsMode::Landed,
            firmware_common_new::vlp::packets::change_mode::Mode::Demo => AvionicsMode::Demo,
        }
    }
}

impl AvionicsMode {
    /// Flight-data logging runs only while `Armed` — the mode covers the entire
    /// ascent/coast/deploy/descent until the auto-switch to `Landed`. Every other
    /// mode (SelfTest / LowPower / Demo / Landed) doesn't log, so pre-flight checks
    /// and time on the ground after landing don't fill the SD card.
    pub fn should_log(&self) -> bool {
        matches!(self, AvionicsMode::Armed)
        // true
    }
}
