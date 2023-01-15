//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    usb,
    watchdog::Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16C0, 0x27DD))
        .manufacturer("fake company")
        .product("serial port")
        .serial_number("TEST")
        .device_class(2)
        .build();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    loop {
        delay_with_poll_ms(&mut delay, &mut usb_dev, &mut [&mut serial], 500);
        while !usb_dev.poll(&mut [&mut serial]) {}
        serial.write(b"LED ON\r\n").unwrap();
        led_pin.set_high().unwrap();

        delay_with_poll_ms(&mut delay, &mut usb_dev, &mut [&mut serial], 500);
        while !usb_dev.poll(&mut [&mut serial]) {}
        serial.write(b"LED OFF\r\n").unwrap();
        led_pin.set_low().unwrap();
    }
}

fn delay_with_poll_ms<A: UsbBus>(
    delay: &mut cortex_m::delay::Delay,
    usb_dev: &mut UsbDevice<A>,
    classes: &mut [&mut dyn UsbClass<A>],
    ms: u32,
) {
    const DELAY_MS: u32 = 1;
    let count = ms / DELAY_MS;

    for _ in 0..count {
        let _ = usb_dev.poll(classes);
        delay.delay_ms(DELAY_MS);
    }
}

// End of file
