use core::cell::RefCell;

use cortex_m::singleton;
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri, bind_interrupts,
    can::{
        self, CanConfigurator, CanRx, CanTx, Frame,
        enums::{BusError, FrameCreateError},
        frame::Envelope,
    },
    peripherals::{FDCAN2, PB5, PB6},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
    pubsub,
};
use embassy_time::{Duration, Instant, Ticker};
use firmware_common_new::{
    can_bus::{
        CanBusFrame, CanBusRX, CanBusTX,
        id::{CanBusExtendedId, can_node_id_from_serial_number},
        messages::{
            CanBusMessageEnum, PRE_UNIX_TIME_MESSAGE_TYPE, UNIX_TIME_MESSAGE_TYPE,
            node_status::{NodeHealth, NodeMode, NodeStatusMessage},
            vl_status::VLStatusMessage,
        },
        node_types::VOID_LAKE_NODE_TYPE,
        receiver::{CanReceiver, ReceivedCanBusMessage},
        sender::{CanSender, create_unix_time_frame_data},
    },
    sensor_reading::SensorReading,
    time::BootTimestamp,
};
use stm32_device_signature::device_id;

use crate::{
    FlightStageMutex, VLStatusMutex,
    bootloader::{BootOption, configure_next_boot},
    can_central::CanCentral,
    tasks::{sensor_tasks::BatteryVWatch, unix_clock::UnixClock},
};

pub type CanReceiverSub = pubsub::Subscriber<
    'static,
    NoopRawMutex,
    SensorReading<BootTimestamp, ReceivedCanBusMessage>,
    4,
    3,
    1,
>;

pub fn init_can_bus(
    fdcan2: Peri<'static, FDCAN2>,
    pb5: Peri<'static, PB5>,
    pb6: Peri<'static, PB6>,
) -> (
    &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    CanRx<'static>,
) {
    bind_interrupts!(struct Irqs {
        FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
        FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
    });

    let mut can = CanConfigurator::new(fdcan2, pb5, pb6, Irqs);
    can.set_bitrate(1_000_000);
    let can = can.into_normal_mode();
    let (tx, rx, _) = can.split();
    let tx = singleton!(: Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>> = Mutex::new(RefCell::new(tx))).unwrap();

    (tx, rx)
}

#[embassy_executor::task]
pub async fn can_bus_broadcast_unix_time_task(
    tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    unix_clock: &'static UnixClock,
) {
    let can_node_id = can_node_id_from_serial_number(device_id());
    let pre_unix_frame_id: u32 = CanBusExtendedId::new(
        1,
        PRE_UNIX_TIME_MESSAGE_TYPE,
        VOID_LAKE_NODE_TYPE,
        can_node_id,
    )
    .into();
    let unix_frame_id: u32 =
        CanBusExtendedId::new(1, UNIX_TIME_MESSAGE_TYPE, VOID_LAKE_NODE_TYPE, can_node_id).into();

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        ticker.next().await;
        if !unix_clock.is_ready() {
            continue;
        }

        let tx = tx.lock().await;
        let mut tx = tx.borrow_mut();

        tx.write(&Frame::new_extended(pre_unix_frame_id, &[]).unwrap())
            .await;
        tx.write(
            &Frame::new_extended(
                unix_frame_id,
                &create_unix_time_frame_data(unix_clock.now_us().unwrap()),
            )
            .unwrap(),
        )
        .await;
    }
}

pub async fn start_can_bus_low_prio_tasks(
    spawner: &Spawner,
    tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
    rx: CanRx<'static>,
    flight_stage: &'static FlightStageMutex,
    battery_v_watch: &'static BatteryVWatch,
    vl_status: &'static VLStatusMutex,
) -> (
    &'static CanSender<NoopRawMutex>,
    &'static CanReceiver<NoopRawMutex, 4, 3>,
    &'static CanCentral<NoopRawMutex>,
) {
    let can_node_id = can_node_id_from_serial_number(device_id());
    info!("CAN Device ID: {}", can_node_id);

    let can_sender =
        singleton!(: CanSender<NoopRawMutex> = CanSender::new(VOID_LAKE_NODE_TYPE, can_node_id, Some(&defmt_rtt_pipe::PIPE)))
            .unwrap();
    let can_receiver =
        singleton!(: CanReceiver<NoopRawMutex, 4, 3> = CanReceiver::new(can_node_id)).unwrap();
    let can_central = singleton!(: CanCentral<NoopRawMutex> = CanCentral::new()).unwrap();

    spawner.must_spawn(can_bus_tx_task(can_sender, tx));
    spawner.must_spawn(can_bus_rx_task(can_receiver, rx, vl_status));
    spawner.must_spawn(node_status_task(can_sender, flight_stage, battery_v_watch, vl_status));
    let can_receiver_sub = can_receiver.subscriber().unwrap();
    spawner.must_spawn(can_message_receive_task(
        can_node_id,
        can_central,
        can_receiver_sub,
    ));

    (can_sender, can_receiver, can_central)
}

#[embassy_executor::task]
async fn can_bus_tx_task(
    can_sender: &'static CanSender<NoopRawMutex>,
    tx: &'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>,
) {
    struct TxWrapper(&'static Mutex<CriticalSectionRawMutex, RefCell<CanTx<'static>>>);
    impl CanBusTX for TxWrapper {
        type Error = FrameCreateError;

        async fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
            let frame = Frame::new_extended(id, data)?;

            let tx = self.0.lock().await;
            let mut tx = tx.borrow_mut();
            tx.write(&frame).await;
            // FIXME: not a big problem for now, but will be a problem if we implement OTA
            // it needs to flush all the can frames before rebooting
            // tx.flush_all().await;

            Ok(())
        }
    }

    let mut tx_wrapper = TxWrapper(tx);
    can_sender.run_daemon(&mut tx_wrapper).await;
}

#[embassy_executor::task]
async fn can_bus_rx_task(
    can_receiver: &'static CanReceiver<NoopRawMutex, 4, 3>,
    rx: CanRx<'static>,
    vl_status: &'static VLStatusMutex,
) {
    struct RxWrapper(CanRx<'static>, &'static VLStatusMutex);
    struct EnvelopeWrapper(Envelope);

    impl CanBusFrame for EnvelopeWrapper {
        fn timestamp_us(&self) -> u64 {
            self.0.ts.as_micros()
        }

        fn id(&self) -> u32 {
            match self.0.frame.id() {
                embedded_can::Id::Standard(standard_id) => standard_id.as_raw() as u32,
                embedded_can::Id::Extended(extended_id) => extended_id.as_raw() as u32,
            }
        }

        fn data(&self) -> &[u8] {
            self.0.frame.data()
        }
    }

    impl CanBusRX for RxWrapper {
        type Error = BusError;
        type Frame = EnvelopeWrapper;

        async fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
            let result = self.0.read().await.map(EnvelopeWrapper);

            self.1.lock(|s| {
                let mut s = s.borrow_mut();
                s.can_bus_ok = result.is_ok();
            });

            result
        }
    }

    let mut rx_wrapper = RxWrapper(rx, vl_status);
    can_receiver.run_daemon::<_, 8>(&mut rx_wrapper).await;
}

#[embassy_executor::task]
async fn node_status_task(
    can_sender: &'static CanSender<NoopRawMutex>,
    flight_stage: &'static FlightStageMutex,
    battery_v_watch: &'static BatteryVWatch,
    vl_status: &'static VLStatusMutex,
) {
    let mut battery_v_reading = battery_v_watch.receiver().unwrap();
    battery_v_reading.get().await;

    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        can_sender.send(
            NodeStatusMessage::new(
                Instant::now().as_secs() as u32,
                NodeHealth::Healthy,
                NodeMode::Operational,
                vl_status.lock(|s| s.borrow().clone()),
            )
            .into(),
        );
        can_sender.send(
            VLStatusMessage {
                flight_stage: flight_stage.borrow().clone().into_inner(),
                battery_mv: (battery_v_reading.try_get().unwrap().data * 1000.0) as u16,
            }
            .into(),
        );
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn can_message_receive_task(
    self_can_id: u16,
    can_central: &'static CanCentral<NoopRawMutex>,
    mut can_receiver_sub: CanReceiverSub,
) {
    loop {
        let SensorReading {
            timestamp_us, data, ..
        } = can_receiver_sub.next_message_pure().await;

        match data.message {
            CanBusMessageEnum::Reset(reset_message) => {
                if reset_message.node_id == self_can_id || reset_message.reset_all {
                    configure_next_boot(if reset_message.into_bootloader {
                        BootOption::Bootloader
                    } else {
                        BootOption::Application
                    });
                    cortex_m::peripheral::SCB::sys_reset();
                }
            }
            CanBusMessageEnum::NodeStatus(node_status_message) => {
                can_central.on_receive_node_status(timestamp_us, &data.id, &node_status_message)
            }
            _ => {}
        }
    }
}
