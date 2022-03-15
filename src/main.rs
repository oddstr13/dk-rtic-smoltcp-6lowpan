//! In here all hardware dependent code is kept, and to run the independent parts the firmware crate
//! is called.

#![no_main]
#![no_std]

use crate::monotonic_nrf52::MonoTimer;
use fugit::{self, ExtU32};
use nrf52840_hal as hal;
use panic_rtt_target as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

// USB Serial
use hal::clocks::{self, Clocks};
use hal::usbd::{UsbPeripheral, Usbd};

use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usb_device::prelude::UsbDevice;

use usbd_serial::{SerialPort, USB_CLASS_CDC};

mod monotonic_nrf52;

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [SWI0_EGU0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        serial: SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
    }

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
    }

    #[monotonic(binds = TIMER1, default = true)]
    type Tonic = MonoTimer<nrf52840_hal::pac::TIMER1>;

    #[init(local = [EP_MEMORY: [u32; 1024] = [0; 1024], usb_bus: Option<UsbBusAllocator<Usbd<UsbPeripheral<'static>>>> = None])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = MonoTimer::new(cx.device.TIMER1);

        rtt_init_print!();
        rprintln!("init");

        static mut CLOCKS: Option<
            Clocks<clocks::ExternalOscillator, clocks::ExternalOscillator, clocks::LfOscStarted>,
        > = None;

        //let core = cx.core;

        let clocks = Clocks::new(cx.device.CLOCK);
        let clocks = clocks.enable_ext_hfosc();
        let clocks = clocks.set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass);
        let clocks = clocks.start_lfclk();
        let _clocks = clocks.enable_ext_hfosc();
        //let board = hal::init().unwrap();
        let clocks = &*unsafe { CLOCKS.get_or_insert(_clocks) };

        cx.local
            .usb_bus
            .replace(Usbd::new(UsbPeripheral::new(cx.device.USBD, &clocks)));

        let mut serial = SerialPort::new(cx.local.usb_bus.as_ref().unwrap());

        let mut usb_dev = UsbDeviceBuilder::new(
            cx.local.usb_bus.as_ref().unwrap(),
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("OpenShell.no")
        .product("dk-rtic-smoltcp-6lowpan-test")
        .serial_number("abcdef")
        .device_class(USB_CLASS_CDC)
        .device_release(0x0001)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();

        task1::spawn().ok();

        (Shared { serial }, Local { usb_dev }, init::Monotonics(mono))
    }

    #[task(shared=[serial])]
    fn task1(mut cx: task1::Context) {
        rprintln!("task1");

        cx.shared.serial.lock(|serial| {
            serial.write(b"task1\r\n").ok();
            serial.flush().ok();
        });

        task1::spawn_after(5000.millis()).ok();
    }
    ///*
    #[task(binds=USBD, local=[usb_dev], shared=[serial])]
    fn usb_handler(mut cx: usb_handler::Context) {
        rprintln!("usb_worker");
        let usb_dev = cx.local.usb_dev;

        cx.shared.serial.lock(|serial| {
            let mut buf = [0u8; 64];
            if usb_dev.poll(&mut [serial]) {
                match serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                       rprintln!("Data received '{:?}'", buf);
                    },
                    Ok(_) => {},
                    Err(_) => {},
                }
            }
        });
    } //*/
}
