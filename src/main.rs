#![no_std]
#![no_main]

use rtic::app;
//use panic_rtt_target as _;
use panic_halt as _;
mod monotonic_timer0;

#[app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [RTC2])]
mod app {
    use core::str;

    use super::monotonic_timer0::{ExtU32, MonoTimer};
    use embedded_hal::{digital::v2::OutputPin, Pwm as EHPwm};
    use nrf52840_hal as hal;
    use nrf52840_hal::usbd::{UsbPeripheral, Usbd};
    use nrf52840_hal::{
        clocks::{self, Clocks},
        gpio::{Level, Output, PushPull},
        pac::{PWM1, TIMER0, TWIM0},
        pwm::{self, Pwm}, // Pulse Width Modulation interface"
        spi::{self, Spi}, // "Serial peripheral interface"
        time::Hertz,
        twim::{self, Twim}, // "Two-wire interface master"
    };
    use rtt_target::{rprintln, rtt_init_print};
    /* use sh1107::{prelude::*, Builder};
    use embedded_graphics::{
        primitives::{Rectangle, PrimitiveStyle},
        mono_font::{ascii::FONT_6X9, MonoTextStyle},
        pixelcolor::BinaryColor,
        prelude::*,
        text::Text,
    };
    use smart_leds::{SmartLedsWrite, RGB8};
    use ws2812_spi::{MODE as NeoPixel_SPI_MODE, Ws2812};
    */

    // USB Serial
    use usb_device::bus::UsbBusAllocator;
    use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
    use usb_device::prelude::UsbDevice;
    use usbd_serial::{SerialPort, USB_CLASS_CDC};

    static HEX_CHARS: &[u8; 16] = b"0123456789ABCDEF";
    const HEX_SIZE: usize = 24;

    fn hexify(mut n: u64) -> [u8; HEX_SIZE] {
        let mut i = 0;
        let mut res: [u8; HEX_SIZE] = [b'\0'; HEX_SIZE];

        while n > 0 {
            if i != 0 {
                res[HEX_SIZE - i - 1] = b':';
                i += 1;
            }
            let cur = n & 0xff;

            res[HEX_SIZE - i - 1] = HEX_CHARS[(cur & 0xf) as usize];
            i += 1;

            res[HEX_SIZE - i - 1] = HEX_CHARS[(cur >> 4) as usize];
            i += 1;

            n >>= 8;
        }
        //res.reverse();
        res.copy_within(HEX_SIZE - i..HEX_SIZE, 0);
        res[i..HEX_SIZE].fill(b'\0');
        res
    }

    #[monotonic(binds = TIMER0, default = true)]
    type MyMono = MonoTimer<TIMER0>;

    #[shared]
    struct DataCommon {
        serial: &'static mut SerialPort<'static, Usbd<UsbPeripheral<'static>>>,
        usb_dev: &'static mut usb_device::device::UsbDevice<'static, Usbd<UsbPeripheral<'static>>>,
        usb_bus: &'static mut UsbBusAllocator<Usbd<UsbPeripheral<'static>>>,
    }

    #[local]
    struct DataLocal {
        led: hal::gpio::p0::P0_13<Output<PushPull>>,
        state: bool,
        //display: GraphicsMode<I2cInterface<hal::twim::Twim<TWIM0>>>,
        //neopixel: Ws2812<hal::spi::Spi<SPI1>>,
        //pwm: Pwm<PWM1>,
        //val: u16,
        //dir: bool,
    }

    #[init]
    fn init(cx: init::Context) -> (DataCommon, DataLocal, init::Monotonics) {
        rtt_init_print!();
        rprintln!("entered init...");

        static mut USB_BUS: Option<UsbBusAllocator<Usbd<UsbPeripheral>>> = None;
        static mut SERIAL: Option<SerialPort<Usbd<UsbPeripheral>>> = None;
        static mut USB_DEV: Option<UsbDevice<Usbd<UsbPeripheral>>> = None;

        let peripherals: nrf52840_hal::pac::Peripherals = cx.device;
        let core = cx.core;
        let _clocks = Clocks::new(peripherals.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass)
            .start_lfclk();

        static mut CLOCKS: Option<
            Clocks<clocks::ExternalOscillator, clocks::ExternalOscillator, clocks::LfOscStarted>,
        > = None;
        let clocks = &*unsafe { CLOCKS.get_or_insert(_clocks) };

        //*USB_BUS = Some(usb::UsbBus::new(usb));
        *USB_BUS = Some(Usbd::new(UsbPeripheral::new(peripherals.USBD, &clocks)));
        unsafe {
            USB_BUS.replace();
            //*USB_BUS = Some();
            SERIAL.replace(SerialPort::new(USB_BUS.as_ref().unwrap()));
        }

        // UsbBusAllocator<Usbd<UsbPeripheral>>
        rprintln!("got peripherals");

        let mono = MonoTimer::new(peripherals.TIMER0);

        // setup GPIO
        let port0 = hal::gpio::p0::Parts::new(peripherals.P0);

        // setup PWM
        let pwm_pin = port0.p0_17.into_push_pull_output(Level::Low).degrade();
        let mut pwm = Pwm::new(peripherals.PWM1);
        // get metrics
        let max_duty = pwm.get_max_duty();
        rprintln!("pwm max duty: {}", max_duty);

        pwm.set_period(Hertz(500_u32))
            .set_output_pin(pwm::Channel::C0, pwm_pin);
        pwm.enable();
        pwm.set_duty_on(pwm::Channel::C0, 0);
        rprintln!("pwm init duty cycle: {}", pwm.duty_on(pwm::Channel::C0));

        // setup I2C and OLED display
        let scl = port0.p0_26.into_floating_input().degrade();
        let sda = port0.p0_25.into_floating_input().degrade();
        let i2c_pins = twim::Pins { scl, sda };
        let i2c = Twim::new(peripherals.TWIM0, i2c_pins, twim::Frequency::K400);
        //let display_size = DisplaySize::Display64x128;
        //let display_rot  = DisplayRotation::Rotate270;
        /*   let mut display: GraphicsMode<_> = Builder::new()
        .with_size(display_size)
        .with_rotation(display_rot)
        .connect_i2c(i2c)
        .into(); */
        /* setup SPI and neopixel
        let sck = port0.p0_12.into_push_pull_output(Level::Low).degrade();
        let mosi = port0.p0_13.into_push_pull_output(Level::Low).degrade();
        let spi_pins = spi::Pins {sck, mosi: Some(mosi), miso: None};
        let spi = Spi::new(
            peripherals.SPI1,
            spi_pins,
            spi::Frequency::M2,
            NeoPixel_SPI_MODE
        );
        let mut neopixel = Ws2812::new(spi);
        let pixels = [RGB8::new(0, 0, 0)];
        neopixel.write(pixels.iter().cloned());
        */

        //rprintln!("init display...\n");
        //display.init().unwrap();
        // define header text
        //let style = MonoTextStyle::new(&FONT_6X9, BinaryColor::On);
        //let mut text = Text::new("RTIC testing", Point::new(3, 0), style);
        // get it's size and shift it down appropriately
        //let bb: Rectangle = text.bounding_box();
        //let (tw, th) = (bb.size.width as i32, bb.size.height as i32);
        //text.position.y = th + 3;
        //text.draw(&mut display);
        //display.flush().unwrap();

        let mut led_pin = port0.p0_13.into_push_pull_output(Level::Low);
        led_pin.set_high().unwrap();

        rprintln!("Spawning blink task...\n");
        blink::spawn().unwrap();

        //rprintln!("Spawning pwm_change task...\n");
        //pwm_change::spawn().unwrap();

        let _id = (peripherals.FICR.deviceid[0].read().bits() as u64)
            | (peripherals.FICR.deviceid[1].read().bits() as u64) << 32;
        let id = hexify(_id);
        let short_id: u16 = (_id & 0xffff) as u16;
        // TODO: Hard-coded last two bytes of the hardware ID / IP address of the two ends
        let target_short: u16;
        if short_id == 0xC352 {
            // DK
            target_short = 0x3805;
        } else {
            target_short = 0xC352;
        }

        let _addr = (peripherals.FICR.deviceaddr[0].read().bits() as u64)
            | ((peripherals.FICR.deviceaddr[1].read().bits() & 0xffff) as u64) << 32;
        let addr = hexify(_addr);
        let addr_str = str::from_utf8(&addr).unwrap_or("");

        unsafe {
            USB_DEV.replace(
                UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                    .manufacturer("OpenShell.no")
                    .product("usb-dongle-smoltcp-6lowpan-test")
                    .serial_number(addr_str)
                    .device_class(USB_CLASS_CDC)
                    .device_release(0x0001)
                    .max_packet_size_0(64) // (makes control transfers 8x faster)
                    .build(),
            );
        }

        // UsbDevice<Usbd<UsbPeripheral>>

        rprintln!("Created usb_dev.");

        // return resources
        (
            DataCommon {
                serial: SERIAL.as_ref().unwrap(),
                usb_dev: USB_DEV.unwrap(),
                usb_bus: USB_BUS.unwrap(),
            },
            DataLocal {
                led: led_pin,
                state: false,
                //display: display,
                //neopixel: neopixel,
                //pwm: pwm,
                //val: 0,
                //dir: true
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led, state])]
    fn blink(cx: blink::Context) {
        rprintln!("blink!");
        let v = !*cx.local.state;
        match v {
            true => cx.local.led.set_high().unwrap(),
            false => cx.local.led.set_low().unwrap(),
        }
        *cx.local.state = v;
        rprintln!("Value of cx.local.state: {}", cx.local.state);
        blink::spawn_after(1_000_000.micros()).unwrap();
    }

    /*
    #[task(local = [pwm, val, dir])]
    fn pwm_change(cx: pwm_change::Context) {
        let pwm = cx.local.pwm;
        // get current val and check it for out-of-bounds
        let mut val = *cx.local.val;
        let mut dir = *cx.local.dir;
        if val >= pwm.get_max_duty() {
            dir = false;
        } else if val <= 0 {
            dir = true;
        }
        let delta: i16 = match dir {
            true  =>  0xF,
            false => -0xF,
        };
        val = ((val as i16) + delta) as u16;
        *cx.local.val = val;
        *cx.local.dir = dir;
        pwm.set_duty_on(pwm::Channel::C0, val);
        rprintln!("pwm val:      {}", val);
        rprintln!("pwm duty on:  {}", pwm.duty_on(pwm::Channel::C0));
        rprintln!("pwm duty off: {}", pwm.duty_off(pwm::Channel::C0));
        pwm_change::spawn_after(700.micros()).unwrap();
    }
    */

    /*
    #[task(local = [display])]
    fn screen_clear(cx: screen_clear::Context) {
        //rprintln!("screen_clear!");
        let mut d = cx.local.display;
        d.clear();
        d.flush().unwrap();
        //rprintln!("leaving screen_clear..");
    }
    */
}
