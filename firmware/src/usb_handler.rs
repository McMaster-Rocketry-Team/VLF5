use core::str::FromStr;

use cortex_m::singleton;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::Driver;
use embassy_stm32::{Peri, bind_interrupts, peripherals, usb};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::signal::Signal;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{EndpointAddress, EndpointIn};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler, UsbDevice};

use firmware_common_new::usb_encoder::{self, encode_float, encode_string};

use firmware_common_new::vlp::usb::CliRequest;
// use {defmt_rtt as _, panic_probe as _};
use panic_probe as _;

use heapless::String;

pub struct DataParams {
    should_send: bool,
    len: usize,
    // TODO add members that describe which data to send. (this depends on file layout)
}

pub type SendDataSignal = Signal<NoopRawMutex, DataParams>;
pub type HandlerMutex = Mutex<CriticalSectionRawMutex, ControlHandler>;

// It needs some buffers for building the descriptors.

const DEVICE_INTERFACE_GUIDS: &[&str] = &["{DAC2087C-63FA-458D-A55D-827C0762DEC7}"]; // windows needs this
bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[embassy_executor::task]
pub async fn setup_usb_handler(
    usb_otg_fs: Peri<'static, USB_OTG_FS>,
    pa12: Peri<'static, PA12>,
    pa11: Peri<'static, PA11>,
    spawner: Spawner,
) {
    // Create the driver, from the HAL.
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
    config.serial_number = Some("4206980085");

    let send_signal: &'static SendDataSignal =
        singleton!(:SendDataSignal = SendDataSignal::new()).unwrap();

    let handler =
        singleton!(: HandlerMutex = HandlerMutex::new(ControlHandler::new(send_signal))).unwrap();

    let control_handler = handler.get_mut();
    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
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

    let mut ep_in = alt.endpoint_bulk_in(Some(EndpointAddress::from(1)), 1024);

    control_handler.interface_num = interface.interface_number();

    drop(function); // why??

    builder.handler(control_handler);

    // Build the builder.
    let usb = builder.build();

    info!("USB handler initialised.");
    // Run the USB device.
    spawner.must_spawn(run_usb(usb));

    let mut buf = [0u8; 1024];

    // heres where I will have to put the file reading code

    let demo: String<1024> = String::from_str("HelloCLI").unwrap();

    match encode_string(&mut buf, 0, demo, "HelloCLI".len()) {
        Ok(_) => {
            info!("succesfully encoded string")
        }
        Err(_) => {
            info!("error encoding string")
        }
    }
    loop {
        let signal = send_signal.wait().await;
        if !signal.should_send {
            embassy_time::Timer::after_millis(1).await;
            continue;
        }

        // TODO use channels for file buffering
        let write_buf = &buf[0..signal.len - 1];
        ep_in.write(write_buf).await.unwrap();
        buf = [0u8; 1024];
        send_signal.reset();

        // Send data on bulk endpoint
    }
}

#[embassy_executor::task]

async fn run_usb(mut usb: UsbDevice<'static, Driver<'static, USB_OTG_FS>>) {
    usb.run().await;
}

pub struct ControlHandler {
    interface_num: InterfaceNumber,
    send_signal: &'static SendDataSignal,
}

impl ControlHandler {
    fn new(send_data_signal: &'static SendDataSignal) -> Self {
        Self {
            interface_num: InterfaceNumber(0),
            send_signal: send_data_signal,
        }
    }
}

// Default is intentionally not implemented for ControlHandler because it requires a 'static SendDataSignal.
// If a default is needed, construct the ControlHandler with a valid &'static SendDataSignal at call sites.

// (we are the device)
impl Handler for ControlHandler {
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        info!("Got control_out, request={}, buf={:a}", req, buf);

        // Only handle Vendor request types to an Interface.
        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return None;
        }

        // Ignore requests to other interfaces.
        if req.index != self.interface_num.0 as u16 {
            return None;
        }

        let cli_req = req.value.try_into().unwrap();

        match cli_req {
            CliRequest::List => {
                info!("List files requested");
                Some(OutResponse::Accepted)
            }
            CliRequest::Clear => {
                info!("Clear files requested");
                Some(OutResponse::Accepted)
            }
            CliRequest::Download => {
                info!("Download files requested");

                self.send_signal.signal(DataParams {
                    should_send: true,
                    len: "stime".len(),
                });

                Some(OutResponse::Accepted)
            }
            _ => {
                info!("Invalid request received through USB!");
                Some(OutResponse::Rejected)
            }
        }
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, _buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Got control_in, request={}", req);
        None
    }
}

#[allow(dead_code)]
fn rkyv_example() {
    use rkyv::{
        api::low::{from_bytes_unchecked, to_bytes_in_with_alloc},
        rancor::{Failure, Infallible},
        ser::{allocator::SubAllocator, writer::Buffer},
    };

    #[derive(rkyv::Archive, rkyv::Deserialize, rkyv::Serialize)]
    struct Data {
        a: f32,
    }

    const DATA_SERIALIZED_LENGTH: usize = size_of::<<Data as rkyv::Archive>::Archived>();

    let original_data = Data { a: 1.0 };

    // serialization start
    let mut buffer = [0u8; DATA_SERIALIZED_LENGTH];
    to_bytes_in_with_alloc::<_, _, Failure>(
        &original_data,
        Buffer::from(&mut buffer), // when buffer has a different size, get a slice of the buffer first
        SubAllocator::empty(),
    )
    .unwrap();
    // `buffer` contains the serialized data now

    // deserialization start
    let deserialized_data = unsafe { from_bytes_unchecked::<Data, Infallible>(&buffer).unwrap() };
}
