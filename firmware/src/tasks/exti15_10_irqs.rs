use embassy_stm32::{bind_interrupts, exti, interrupt};

bind_interrupts!(pub struct ExtI15_10Irqs {
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
});
