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
    /// Flight-data logging runs only from Arm through Landed — the whole flight — so
    /// pre-flight modes (SelfTest / LowPower / Demo) don't fill the SD card. The mode
    /// stays `Armed` for the entire ascent/coast/deploy/descent, then auto-switches to
    /// `Landed`, so `{Armed, Landed}` covers "armed all the way to landed".
    pub fn should_log(&self) -> bool {
        matches!(self, AvionicsMode::Armed | AvionicsMode::Landed)
    }
}
