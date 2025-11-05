use defmt::*;
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::Driver;
use embassy_stm32::{Peri, bind_interrupts, peripherals, usb};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Endpoint, EndpointAddress, EndpointIn};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

// use {defmt_rtt as _, panic_probe as _};
use panic_probe as _; 

use crate::SendDataSignal; 

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"]; // windows needs this
bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[embassy_executor::task]
pub async fn setup_usb_handler(
    usb_otg_fs: Peri<'static, USB_OTG_FS>,
    pa12: Peri<'static, PA12>,
    pa11: Peri<'static, PA11>,
) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    config.vbus_detection = false;

    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-raw example");
    config.serial_number = Some("12345678");

    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut handler = ControlHandler {
        interface_num: InterfaceNumber(0),
    };

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Windows stuff
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
    ));

    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);

    let ep_in = alt.endpoint_bulk_in(Some(EndpointAddress::from(1)), 10); // I cannot tell you for the life of me what type this variable is meant to have

    handler.interface_num = interface.interface_number();

    drop(function); // why??

    builder.handler(&mut handler);

    // Build the builder.
    let mut usb = builder.build();
    // Run the USB device.

    info!("USB handler initialised.");

    usb.run().await;
}

struct ControlHandler {
    interface_num: InterfaceNumber,
}

// (we are the device)
impl Handler for ControlHandler {
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        info!("Got control_out, request={}, buf={:a}", req, buf);

        None
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Got control_in, request={}", req);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.interface_num.0 as u16 {
            return None;
        }

        // TODO define a protocol for this

        if req.request == 101 && req.value == 201
        // list
        {
            buf[..8].copy_from_slice(b"data.csv");
            Some(InResponse::Accepted(&buf[..7]))
        } else if req.value == 202
        // clear
        {
            buf[..16].copy_from_slice(b"clearing data...");
            Some(InResponse::Accepted(&buf[..7]))
        } else {
            Some(InResponse::Rejected)
        }
    }
}

// The plan is to add this task in low_prio_main and have it wait on two signals, one which contains whether it should write or not
// and another that tells it what files to write over the endpoint (this one could be a watch)
// The rust compiler keeps complaining about EndpointIn<'static> and its messages arent very helpful.

// #[embassy_executor::task]
// async fn send_data_task(mut ep: EndpointIn<'static>, send_signal: SendDataSignal) {
//     let payload: &[u8] = b"some sample data"; // fixed for now.
//     loop {
//         send_signal.wait().await;

//         // Send data on bulk endpoint

//         embassy_time::Timer::after_millis(1).await;
//     }
// }
