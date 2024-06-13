#![no_std]
#![no_main]

use fugit::RateExtU32;
use hal::pac;
use hal::uart::{DataBits, StopBits, UartConfig};
use rp2040_hal::spi::Enabled;
use rp2040_hal::Clock;
use rp_pico::entry;

use embedded_hal::digital::v2::OutputPin;

use panic_halt as _;
use rp2040_hal as hal;

use rp2040_lib::my_macro::UART_TRANSMITTER;
use rp2040_lib::print;
use rp2040_lib::println;

use rp2040_lib::bme280::spi::BME280;

use rp2040_hal::gpio::bank0::Gpio4;
use rp2040_hal::gpio::bank0::Gpio5;
use rp2040_hal::gpio::bank0::Gpio6;
use rp2040_hal::gpio::bank0::Gpio7;
use rp2040_hal::gpio::FunctionSio;
use rp2040_hal::gpio::Pin;
use rp2040_hal::gpio::PullDown;
use rp2040_hal::gpio::{FunctionSpi, SioOutput};

use crate::pac::SPI0;
//use embedded_hal::spi::FullDuplex;
use rp2040_hal::Spi;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        // (8-6-7)
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (pins.gpio0.reconfigure(), pins.gpio1.reconfigure());
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let (_, uart_tx) = uart.split();

    critical_section::with(|_| unsafe {
        UART_TRANSMITTER = Some(uart_tx);
    });

    let mut led_pin = pins.led.into_push_pull_output();

    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    let cs = pins.gpio5.into_push_pull_output();

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        embedded_hal::spi::MODE_0,
    );

    let mut bme280 = BME280::<
        Spi<
            Enabled,
            SPI0,
            (
                Pin<Gpio7, FunctionSpi, PullDown>,
                Pin<Gpio4, FunctionSpi, PullDown>,
                Pin<Gpio6, FunctionSpi, PullDown>,
            ),
        >,
        Pin<Gpio5, FunctionSio<SioOutput>, PullDown>,
    >::new(spi, cs);

    // DeviceのIDコード(0x60)を正しく読めれば成功としている
    if bme280.init() {
        println!("BME280 initialization successful.");
        println!("BME280 ID = 0x60.\r\n");
    } else {
        println!("BME280 initialization failed.\r\n");
    }

    loop {
        bme280.read_data();

        let (temp, humi, pres) = bme280.get_elements();

        println!("T = {:.2} ℃", temp);
        println!("H = {:.2} %", humi);
        println!("P = {:.2} hPa\r\n", pres);

        led_pin.set_high().unwrap();
        delay.delay_ms(200);
        led_pin.set_low().unwrap();
        delay.delay_ms(800);
    }
}
