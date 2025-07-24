#[derive(Clone, Copy, defmt::Format)]
pub enum AvionicsMode {
    Armed,
    SelfTest,
    LowPower,
    Landed,
}

impl AvionicsMode {
    pub fn sensors_active(&self) -> bool {
        matches!(self, AvionicsMode::Armed | AvionicsMode::SelfTest)
    }
}
