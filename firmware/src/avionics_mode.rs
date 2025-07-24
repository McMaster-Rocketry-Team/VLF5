#[derive(Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum AvionicsMode {
    Armed,
    SelfTest,
    LowPower,
    Landed,
}

impl From<firmware_common_new::vlp::packets::change_mode::Mode> for AvionicsMode {
    fn from(value: firmware_common_new::vlp::packets::change_mode::Mode) -> Self {
        match value {
            firmware_common_new::vlp::packets::change_mode::Mode::LowPower => AvionicsMode::LowPower,
            firmware_common_new::vlp::packets::change_mode::Mode::SelfTest => AvionicsMode::SelfTest,
            firmware_common_new::vlp::packets::change_mode::Mode::Armed => AvionicsMode::Armed,
            firmware_common_new::vlp::packets::change_mode::Mode::Landed => AvionicsMode::Landed,
        }
    }
}

impl AvionicsMode {
    pub fn sensors_active(&self) -> bool {
        matches!(self, AvionicsMode::Armed | AvionicsMode::SelfTest)
    }
}
