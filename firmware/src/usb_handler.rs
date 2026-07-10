use cortex_m::singleton;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::Driver;
use embassy_stm32::{Peri, bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_usb::control::{OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Endpoint, EndpointAddress, EndpointIn};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler, UsbDevice};

use firmware_common_new::vlp::client::VLPAvionics;
#[cfg(all(feature = "hil-replay", not(feature = "hil-radio")))]
use firmware_common_new::vlp::packets::{MAX_VLP_PACKET_SIZE, VLPUplinkPacket};
use firmware_common_new::vlp::usb::CliRequest;
use panic_probe as _;
use stm32_device_signature::device_id_hex;

use crate::tasks::sd_card_writer::{
    StorageCmdSignal, StorageCommand, StorageRespChannel, StorageResponse,
};

pub type HandlerMutex = Mutex<CriticalSectionRawMutex, ControlHandler>;

/// Host→device VLP uplinks arrive as control-OUT data and are queued here for the
/// bridge task (which runs in the same executor as the VLP client) to inject.
pub type VlpUplinkChannel =
    embassy_sync::channel::Channel<NoopRawMutex, heapless::Vec<u8, 100>, 4>;

// Windows needs this to bind WinUSB without a custom driver.
const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"];
bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

/// Bulk-IN endpoint address the host reads flight-log downloads from (EP 1 IN -> 0x81).
const EP_IN_ADDR: u8 = 1;
/// Bulk-IN endpoint the host reads VLP downlink frames from in HIL builds (EP 2 IN -> 0x82).
#[cfg(all(feature = "hil-replay", not(feature = "hil-radio")))]
const EP_VLP_IN_ADDR: u8 = 2;

#[embassy_executor::task]
pub async fn setup_usb_handler(
    usb_otg_fs: Peri<'static, USB_OTG_FS>,
    pa12: Peri<'static, PA12>,
    pa11: Peri<'static, PA11>,
    storage_cmd: &'static StorageCmdSignal,
    storage_resp: &'static StorageRespChannel,
    vlp_client: &'static VLPAvionics<NoopRawMutex>,
    uplink_cmd: &'static VlpUplinkChannel,
    spawner: Spawner,
) {
    let _ = vlp_client; // used only in HIL builds (VLP-over-USB bridge)

    // Buffers for building the USB descriptors.
    let ep_out_buffer = singleton!(: [u8;256] = [0u8; 256]).unwrap();
    let config_descriptor = singleton!(: [u8;256] = [0u8; 256]).unwrap();
    let bos_descriptor = singleton!(: [u8;256] = [0u8; 256]).unwrap();
    let msos_descriptor = singleton!(: [u8;256] = [0u8; 256]).unwrap();
    let control_buf = singleton!(: [u8;64] = [0u8; 64]).unwrap();

    let mut config = embassy_stm32::usb::Config::default();
    config.vbus_detection = false;

    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, ep_out_buffer, config);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("MacRocketry");
    config.product = Some("VLF5");
    config.serial_number = Some(device_id_hex());

    let handler = singleton!(: HandlerMutex =
        HandlerMutex::new(ControlHandler::new(storage_cmd, uplink_cmd)))
    .unwrap();
    let control_handler = handler.get_mut();

    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );

    // Windows WinUSB binding.
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);

    // Full-speed bulk endpoints have a 64-byte max packet size.
    let mut ep_in = alt.endpoint_bulk_in(Some(EndpointAddress::from(EP_IN_ADDR)), 64);
    // HIL: a second bulk-IN carries VLP downlink frames (independent of flight-log).
    #[cfg(all(feature = "hil-replay", not(feature = "hil-radio")))]
    let mut ep_vlp_in = alt.endpoint_bulk_in(Some(EndpointAddress::from(EP_VLP_IN_ADDR)), 64);

    control_handler.interface_num = interface.interface_number();

    drop(function);

    builder.handler(control_handler);

    let usb = builder.build();
    info!("USB handler initialised.");
    spawner.spawn(run_usb(usb).unwrap());

    ep_in.wait_enabled().await;

    // Flight builds and Option B (`hil-radio`) drive VLP over the real radio, so USB only
    // carries flight-log traffic here.
    #[cfg(any(not(feature = "hil-replay"), feature = "hil-radio"))]
    storage_drain(&mut ep_in, storage_resp).await;

    // Plain HIL builds bridge VLP over USB alongside the flight-log stream.
    #[cfg(all(feature = "hil-replay", not(feature = "hil-radio")))]
    embassy_futures::join::join3(
        storage_drain(&mut ep_in, storage_resp),
        vlp_downlink_drain(&mut ep_vlp_in, vlp_client),
        vlp_uplink_drain(uplink_cmd, vlp_client),
    )
    .await;
}

/// Drain flight-log responses produced by the SD task onto the bulk-IN endpoint. A
/// response is a sequence of `Chunk`s terminated by `End`, which lets the host know
/// one logical transfer is finished.
async fn storage_drain(ep_in: &mut impl EndpointIn, storage_resp: &'static StorageRespChannel) {
    // If the host disconnects mid-transfer, the first write fails; rather than
    // hammer a failed write per remaining chunk (thousands, for a big download),
    // drop the rest of this transfer and resume cleanly at the next `End`.
    let mut aborted = false;
    loop {
        match storage_resp.receive().await {
            StorageResponse::Chunk { buf, len } => {
                if aborted {
                    continue;
                }
                // embassy's OTG `write` rejects any buffer larger than the
                // endpoint's max packet size (64 on full-speed), so a 512-byte
                // block must go out as 64-byte USB packets.
                'chunk: for packet in buf[..len].chunks(64) {
                    if ep_in.write(packet).await.is_err() {
                        // One retry across a transient endpoint disable — the
                        // host issues a SET_INTERFACE when it (re)claims the
                        // interface between CLI commands, briefly disabling the
                        // endpoint. Bound the wait so a truly-gone host aborts.
                        let reenabled = matches!(
                            select(
                                embassy_time::Timer::after_millis(500),
                                ep_in.wait_enabled()
                            )
                            .await,
                            Either::Second(())
                        );
                        if !reenabled || ep_in.write(packet).await.is_err() {
                            warn!("USB bulk write failed — host gone, aborting transfer");
                            aborted = true;
                            break 'chunk;
                        }
                    }
                }
            }
            StorageResponse::End => {
                // No terminating zero-length packet: the host already knows the
                // exact byte count from the response header and stops on its
                // own. Writing a ZLP here blocks whenever the host has already
                // stopped reading, which wedges the endpoint for the next
                // command (the bug that made `download` fail right after `list`).
                aborted = false;
            }
        }
    }
}

/// HIL: forward each VLP downlink the avionics app emits to the host over EP2 as a
/// fixed 101-byte frame `[len: u8][payload (100)]`. Fixed size means the transfer
/// always ends on a short packet, so the host reads exactly one frame per read.
#[cfg(all(feature = "hil-replay", not(feature = "hil-radio")))]
async fn vlp_downlink_drain(
    ep: &mut impl EndpointIn,
    vlp: &'static VLPAvionics<NoopRawMutex>,
) {
    ep.wait_enabled().await;
    let mut frame = [0u8; MAX_VLP_PACKET_SIZE + 1];
    loop {
        let packet = vlp.wait_downlink().await;
        let len = packet.serialize(&mut frame[1..]);
        frame[0] = len as u8;
        for chunk in frame.chunks(64) {
            if ep.write(chunk).await.is_err() {
                // Host not reading EP2 yet (or reconnecting): wait for it, drop this
                // frame, and resume with the next downlink.
                ep.wait_enabled().await;
                break;
            }
        }
    }
}

/// HIL: deserialize host uplinks queued from control-OUT and inject them into the
/// avionics VLP client as if received+verified over the air (no signing on USB).
#[cfg(all(feature = "hil-replay", not(feature = "hil-radio")))]
async fn vlp_uplink_drain(
    chan: &'static VlpUplinkChannel,
    vlp: &'static VLPAvionics<NoopRawMutex>,
) {
    loop {
        let bytes = chan.receive().await;
        match VLPUplinkPacket::deserialize(&bytes) {
            Some(packet) => {
                info!("USB VLP uplink injected ({} bytes)", bytes.len());
                vlp.inject_uplink(packet);
            }
            None => warn!("USB VLP: bad uplink packet ({} bytes)", bytes.len()),
        }
    }
}

#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, Driver<'static, USB_OTG_FS>>) {
    usb.run().await;
}

pub struct ControlHandler {
    interface_num: InterfaceNumber,
    storage_cmd: &'static StorageCmdSignal,
    uplink_cmd: &'static VlpUplinkChannel,
}

impl ControlHandler {
    fn new(storage_cmd: &'static StorageCmdSignal, uplink_cmd: &'static VlpUplinkChannel) -> Self {
        Self {
            interface_num: InterfaceNumber(0),
            storage_cmd,
            uplink_cmd,
        }
    }
}

// We are the USB device; the host (rocket-cli) drives us with vendor control
// transfers carrying a `CliRequest` in `wValue` (and, for `Uplink`, the packet in
// the data stage).
impl Handler for ControlHandler {
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        // Only handle vendor requests aimed at our interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }
        if req.index != self.interface_num.0 as u16 {
            return None;
        }

        match req.value.into() {
            CliRequest::List => self.storage_cmd.signal(StorageCommand::List),
            CliRequest::Download => self.storage_cmd.signal(StorageCommand::Download),
            CliRequest::Clear => self.storage_cmd.signal(StorageCommand::Clear),
            CliRequest::Uplink => {
                // Copy the serialized uplink out of the control buffer and queue it;
                // the bridge task injects it into the VLP client.
                let mut packet: heapless::Vec<u8, 100> = heapless::Vec::new();
                if packet.extend_from_slice(buf).is_ok() {
                    let _ = self.uplink_cmd.try_send(packet);
                } else {
                    warn!("USB VLP: uplink too large ({} bytes)", buf.len());
                }
            }
            CliRequest::Invalid => {
                warn!("USB: invalid CLI request value {}", req.value);
                return Some(OutResponse::Rejected);
            }
        }
        Some(OutResponse::Accepted)
    }
}
