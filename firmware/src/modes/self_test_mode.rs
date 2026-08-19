use defmt::info;
use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::PubSubBehavior};
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware_common_new::{
    can_bus::{
        messages::{
            CanBusMessageEnum,
            amp_control::AmpControlMessage,
            amp_status::{AmpStatusMessage, PowerOutputStatus},
            vl_status::FlightStage,
        },
        node_types::{AMP_NODE_TYPE, ICARUS_NODE_TYPE, OZYS_NODE_TYPE, PAYLOAD_SDRM_NODE_TYPE},
    },
    vlp::{
        client::VLPAvionics,
        packets::{
            VLPDownlinkPacket,
            self_test_result::{NodeStatus, SelfTestResultPacketBuilder},
        },
    },
};

use crate::{
    AvionicsModeWatch, ContinuityWatch, FlightStageMutex, VLStatusMutex,
    avionics_mode::AvionicsMode,
    can::CanReceiverSub,
    can_central::CanCentral,
    tasks::{
        amp_control_task::AmpControlWatch,
        buzzer_task::{BuzzerPubSub, BuzzerTone},
        sensor_tasks::{SENSOR_INIT_ATTEMPTS, SENSOR_INIT_TIMEOUT_MS},
    },
    utils::run_with_timeout,
};

pub async fn self_test_mode(
    vlp_avionics_client: &'static VLPAvionics<NoopRawMutex>,
    avionics_mode_watch: &'static AvionicsModeWatch,
    can_central: &'static CanCentral<NoopRawMutex>,
    vl_status: &'static VLStatusMutex,
    amp_control_watch: &'static AmpControlWatch,
    flight_stage: &'static FlightStageMutex,
    continuity_watch: &'static ContinuityWatch,
    mut can_receiver_sub: CanReceiverSub,
    buzzer_pubsub: &'static BuzzerPubSub,
) {
    info!("enter self test mode");
    flight_stage.lock(|r| {
        *r.borrow_mut() = FlightStage::SelfTest;
    });

    let self_test_fut = async {
        let mut self_test_failed = false;
        let mut self_test_partial_failure = false;
        let packet_builder = SelfTestResultPacketBuilder::<NoopRawMutex>::new();

        amp_control_watch.sender().send(AmpControlMessage {
            out1_enable: false,
            out2_enable: false,
            out3_enable: false,
        });
        // Two separate waits, both still needed as they are: the first lets the
        // AMP outputs actually de-power and settle before anything is measured,
        // the second gives the bus nodes a window to re-report after
        // `can_central.clear()` so the AMP block below is reading this session's
        // heartbeat rather than an empty table.
        Timer::after_millis(1000).await;
        can_central.clear();
        Timer::after_millis(1000).await;

        // test vl
        wait_for_sensors_ready(vl_status).await;
        let continuity = continuity_watch.receiver().unwrap().get().await;
        packet_builder.update(|packet| {
            vl_status.lock(|s| {
                let status = s.borrow();
                packet.imu_ok = status.imu_ok;
                packet.baro_ok = status.baro_ok;
                packet.mag_ok = status.mag_ok;
                packet.gps_ok = status.gps_ok;
                packet.sd_ok = status.sd_ok;
                packet.can_bus_ok = status.can_bus_ok;
                packet.main_continuity = continuity.pyro_main_continuity;
                packet.drogue_continuity = continuity.pyro_drogue_continuity;

                if !packet.imu_ok || !packet.baro_ok || !packet.gps_ok {
                    self_test_failed = true;
                }
                if !packet.mag_ok || !packet.sd_ok || !packet.can_bus_ok {
                    self_test_partial_failure = true;
                }
            });
        });

        // test amp
        packet_builder.update(|packet| {
            if let Some(amp) = can_central.get_nodes::<1>(AMP_NODE_TYPE).first() {
                packet.amp = NodeStatus::from_message(&amp.status);
            } else {
                packet.amp = NodeStatus::offline();
            }

            if !packet.amp.healthy() {
                self_test_partial_failure = true;
            }
        });

        // test amp out 2 (air brakes, camera, VL-Mini, VLF6)
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: false,
                out2_enable: true,
                out3_enable: false,
            });
            Timer::after_millis(15000).await; // longer time for payload sdrm to connect to payload, also air brakes need to home.
            // A timeout and an answer of "not PowerGood" are the same result to
            // the operator listening at the pad: the output did not come up. Both
            // land as `false` here, and `false` is a partial failure.
            let out2_power_good = get_amp_status_message(&mut can_receiver_sub)
                .await
                .is_some_and(|amp_status_message| {
                    amp_status_message.out2.status == PowerOutputStatus::PowerGood
                });
            if !out2_power_good {
                self_test_partial_failure = true;
            }
            packet_builder.update(|packet| {
                packet.amp_out2_power_good = out2_power_good;
            });

            // These three nodes need a freshness check that the AMP block
            // above does not.
            //
            // `can_central` keeps a node's last `NodeStatusMessage` for ever,
            // so `from_message` on its own reports whatever the node last said,
            // however long ago that was. For AMP that cannot go wrong here:
            // `can_central.clear()` runs at t=1 s and the AMP block at t=2 s,
            // comfortably inside the 5 s `is_online()` window, so anything in
            // the table there is current by construction.
            //
            // These three sit on AMP out 2, energised at t~7-9 s and read at
            // t~22-26 s. A stale entry only needs the node to have gone silent
            // any time before t~17-21 s — roughly 8-12 s of the ~14 s they are
            // powered. The 15 s wait above exists because the air brakes have
            // to home, a high-current mechanical operation, and a brownout
            // during it is precisely what this test is looking for: a node that
            // reported Healthy on its way up and then dropped off the bus is
            // the failure, not the noise. `SelfTestResultPacket` has no
            // `*_online` fields either, so unlike armed mode there is no
            // out-of-band channel to tell the ground about it.
            packet_builder.update(|packet| {
                if let Some(ozys) = can_central.get_nodes::<1>(OZYS_NODE_TYPE).first() {
                    packet.ozys = if ozys.is_online() {
                        NodeStatus::from_message(&ozys.status)
                    } else {
                        NodeStatus::offline()
                    };
                } else {
                    packet.ozys = NodeStatus::offline();
                }
                // OZYS is reported to the ground but deliberately does not vote:
                // it carries no flight-critical function, so a missing or
                // unhealthy OZYS still chimes success.

                if let Some(payload_sdrm) =
                    can_central.get_nodes::<1>(PAYLOAD_SDRM_NODE_TYPE).first()
                {
                    packet.payload_sdrm = if payload_sdrm.is_online() {
                        NodeStatus::from_message(&payload_sdrm.status)
                    } else {
                        NodeStatus::offline()
                    };
                } else {
                    packet.payload_sdrm = NodeStatus::offline();
                }
                if !packet.payload_sdrm.healthy() {
                    self_test_partial_failure = true;
                }

                if let Some(icarus) = can_central.get_nodes::<1>(ICARUS_NODE_TYPE).first() {
                    packet.icarus = if icarus.is_online() {
                        NodeStatus::from_message(&icarus.status)
                    } else {
                        NodeStatus::offline()
                    };
                } else {
                    packet.icarus = NodeStatus::offline();
                }
                if !packet.icarus.healthy() {
                    self_test_partial_failure = true;
                }
            });
        }

        Timer::after_millis(3000).await;

        // test amp out 1
        // recovery beacon is on battery 1
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: true,
                out2_enable: false,
                out3_enable: false,
            });
            Timer::after_millis(5000).await; // keep this long for any longer and you shall deafen people
            // A timeout and an answer of "not PowerGood" are the same result to
            // the operator listening at the pad: the output did not come up. Both
            // land as `false` here, and `false` is a partial failure.
            let out1_power_good = get_amp_status_message(&mut can_receiver_sub)
                .await
                .is_some_and(|amp_status_message| {
                    amp_status_message.out1.status == PowerOutputStatus::PowerGood
                });
            if !out1_power_good {
                self_test_partial_failure = true;
            }
            packet_builder.update(|packet| {
                packet.amp_out1_power_good = out1_power_good;
            });
        }

        // test amp out 3
        {
            amp_control_watch.sender().send(AmpControlMessage {
                out1_enable: false,
                out2_enable: false,
                out3_enable: true,
            });
            Timer::after_millis(2000).await;
            // A timeout and an answer of "not PowerGood" are the same result to
            // the operator listening at the pad: the output did not come up. Both
            // land as `false` here, and `false` is a partial failure.
            let out3_power_good = get_amp_status_message(&mut can_receiver_sub)
                .await
                .is_some_and(|amp_status_message| {
                    amp_status_message.out3.status == PowerOutputStatus::PowerGood
                });
            if !out3_power_good {
                self_test_partial_failure = true;
            }
            packet_builder.update(|packet| {
                packet.amp_out3_power_good = out3_power_good;
            });
        }

        amp_control_watch.sender().send(AmpControlMessage {
            out1_enable: false,
            out2_enable: false,
            out3_enable: false,
        });

        if self_test_failed {
            info!("self test failed");
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
        } else if self_test_partial_failure {
            info!("self test partial failure");
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
        } else {
            info!("self test success");
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::Low(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
            buzzer_pubsub.publish_immediate(BuzzerTone::High(250, 100));
        }

        let packet: VLPDownlinkPacket = packet_builder.create_packet().into();
        info!("self test done: {}", packet);
        let mut ticker = Ticker::every(Duration::from_secs(2));

        loop {
            ticker.next().await;
            vlp_avionics_client.send(packet.clone());
        }
    };

    let wait_self_test_mode_end_fut = async {
        let mut receiver = avionics_mode_watch.receiver().unwrap();
        receiver.changed_and(|m| *m != AvionicsMode::SelfTest).await;
    };

    select(self_test_fut, wait_self_test_mode_end_fut).await;
}

/// How often the sensor health flags are re-checked while waiting for them.
/// Short against the deadline below, so the wait ends essentially the moment
/// the last sensor comes good.
const SENSOR_READY_POLL_MS: u64 = 50;

/// Allowance on top of the init budget for the first successful READ to land
/// and raise the flag. The init budget bounds the transfers, not the sampling
/// that follows them: the barometer's slowest loop runs at 5 Hz (200 ms per
/// sample), and the three `reset()` / `power_up()` calls carry ~90 ms of
/// deliberate settling delays inside them that the budget does not count.
const SENSOR_FIRST_READ_ALLOWANCE_MS: u64 = 500;

/// Wait until the IMU, barometer and magnetometer have each produced one good
/// read, or until they have provably run out of retries — whichever comes
/// first.
///
/// The health flags are fail-safe: false at boot, latched true by the first
/// successful read. That is the right default, but it means a fixed wait before
/// the `vl_status` snapshot is a race the flags can lose. `imu_baro_task` makes
/// three bounded init transfers before its first read — `imu.reset()`,
/// `baro.reset()` and `imu.power_up()` — each allowed
/// [`SENSOR_INIT_ATTEMPTS`] × [`SENSOR_INIT_TIMEOUT_MS`] = 1.5 s, so its worst
/// case is 4.5 s, while the two 1 s timers in [`self_test_mode`] put the
/// snapshot at ~2 s. Hardware that stalled on a couple of init transfers and
/// then recovered would be snapshotted as failed and chime a real error for it.
/// (`mag_task` spends the same budget on `mag.reset()` and `mag.power_up()`,
/// and then reads at 100 Hz.)
///
/// So wait for the answer rather than guessing when it will exist. The deadline
/// is derived from those two constants instead of written out, so changing
/// either moves this with it and the two cannot drift.
///
/// This costs nothing on healthy hardware, where the flags are already up
/// before the first poll, and stretches only for hardware that is genuinely
/// retrying. It does not mask a real failure: the deadline is the entire budget
/// a sensor could ever spend, so anything still false at the end has run out of
/// attempts and is reported failed exactly as before.
async fn wait_for_sensors_ready(vl_status: &'static VLStatusMutex) {
    let deadline = Instant::now()
        + Duration::from_millis(
            3 * SENSOR_INIT_ATTEMPTS as u64 * SENSOR_INIT_TIMEOUT_MS
                + SENSOR_FIRST_READ_ALLOWANCE_MS,
        );

    loop {
        let all_ok = vl_status.lock(|s| {
            let s = s.borrow();
            s.imu_ok && s.baro_ok && s.mag_ok
        });
        if all_ok {
            return;
        }
        if Instant::now() >= deadline {
            info!("self test: sensor health flags still down at the init deadline");
            return;
        }
        Timer::after_millis(SENSOR_READY_POLL_MS).await;
    }
}

async fn get_amp_status_message(can_receiver_sub: &mut CanReceiverSub) -> Option<AmpStatusMessage> {
    can_receiver_sub.clear();
    let wait_message_fut = async {
        loop {
            let message = can_receiver_sub.next_message_pure().await;
            if let CanBusMessageEnum::AmpStatus(message) = message.data.message {
                break message;
            }
        }
    };

    run_with_timeout(2000, wait_message_fut).await.ok()
}
