#![feature(start, core_intrinsics)]
#![no_std]

#[macro_use]
extern crate zinc;

#[macro_use]
extern crate log;

extern crate teensy;
use teensy::ncsprng::Prng;

//use core::intrinsics::volatile_load;

use core::option::Option::Some;
use zinc::hal::uart::Parity;
use zinc::util::support::wfi;

static mut systicks: u32 = 0;

#[allow(dead_code)]
#[no_mangle]
pub unsafe extern "C" fn isr_systick() {
    systicks += 1;
}

use zinc::hal::k20::{watchdog, uart, spi};
use zinc::hal::k20::spi::SPITransmit;
use zinc::hal::k20::uart::UARTPeripheral::UART0;
use zinc::hal::k20::uart_logger;
use zinc::hal::k20::pin::Port::*;
use zinc::hal::k20::pin::Pin;
use zinc::hal::k20::pin::Function;
use zinc::hal::pin::GpioDirection::*;
use zinc::hal::pin::Gpio;

use core::fmt::Write;
use core::str::FromStr;

use zinc::drivers::apa102::*;

/*
impl SPITransmit for LedString {
    fn transmit(&self, &SPI) -> SPITxResult {
        let preamble_len = 1;
        let postamble_len = self.len() / 8;
        let data: [u32;] = 
    }
}
 */

pub fn main() {
    // Pins for Teensy 3.1 (http://www.pjrc.com/)
    // Pin				Arduino
    //  0	B16			RXD
    //  1	B17			TXD
    //  2	D0
    //  3	A12	FTM1_CH0
    //  4	A13	FTM1_CH1
    //  5	D7	FTM0_CH7	OC0B/T1
    //  6	D4	FTM0_CH4	OC0A
    //  7	D2
    //  8	D3			ICP1
    //  9	C3	FTM0_CH2	OC1A
    // 10	C4	FTM0_CH3	SS/OC1B
    // 11	C6			MOSI/OC2A
    // 12	C7			MISO
    // 13	C5			SCK         LED
    // 14	D1
    // 15	C0
    // 16	B0	(FTM1_CH0)
    // 17	B1	(FTM1_CH1)
    // 18	B3			SDA
    // 19	B2			SCL
    // 20	D5	FTM0_CH5
    // 21	D6	FTM0_CH6
    // 22	C1	FTM0_CH0
    // 23	C2	FTM0_CH1
    let led = Pin::new(PortC,
                       5,
                       Function::Gpio,
                       Some(Out));

    let mut uart = uart::UART::new(UART0, 115200, 8, Parity::Disabled, 1,
                                   Pin::new(PortB, 16, Function::Gpio, Some(In)),
                                   Pin::new(PortB, 17, Function::Gpio, Some(Out)));

    uart_logger::init(uart);
    info!("Boot!");

    // info!("I setup logging!");
    // info!("Apparently it's {:?} times.", zinc::hal::k20::rtc::time());
    
    // info!("\r\nOhai, we're in main(). fBUS is kicking at a cool {} Hz, and our UART is running - obvi.", zinc::hal::k20::clocks::bus_clock().expect("Bus clock not set"));

    // let _ = write!(&mut uart, "\r\nAbout to setup SPI...");
    let spi = &spi::SPI::write_only(spi::SPIPeripheral::SPI0,
                                    // SCK
                                    Pin::new(PortD, 1, Function::Gpio, Some(Out)),
                                    // SOUT
                                    Pin::new(PortC, 6, Function::Gpio, Some(Out)));
    // let _ = writeln!(&mut uart, " done.\r");
    info!("About to send a bunch of stuff to the dotstar at {} Hz.", spi.baud_rate());

    // let preamble: &[u32] = &[0][..];
    // let preamble_result = preamble.transmit(spi);
    // trace!("Preamble {:?}", preamble_result);
    
    let mut ledstring: [APA102;72] = [UNLIT;72];
    ledstring[0] = BLUE;
    ledstring[1] = GREEN;
    ledstring[2] = RED;
    ledstring[3] = PINK;
    
    info!("Wrote the ledstring: {:?}", (&ledstring[..]).transmit(spi));

    info!("Welp. It should be running...");

    let mut old_time = zinc::hal::k20::rtc::time().unwrap_or(0);
    let mut prng = Prng::seed(old_time);
    
    loop {
        let time = zinc::hal::k20::rtc::time().unwrap_or(0);
        if time == old_time {
            continue;
        }
        old_time = time;
        {
            rotate_strip(&mut ledstring);
            ledstring[0] = random_pixel(&mut prng);
                
            // Write out new strip.
            let _ = (&ledstring[..]).transmit(spi);
        }
            
        match time % 2 == 0 {
            true => {
                led.set_high();
                info!("[{:>9}] Blinky.", time);
            },
            false => {
                led.set_low();
                info!("[{:>9}] Blanky.", time);
            }
        }
        // Sleep for interrupts
        wfi();
    }
}

fn random_pixel(prng: &mut Prng) -> APA102 {
    APA102::bgr(prng.scaled_next(0,8) as u8,
                prng.scaled_next(0,8) as u8,
                prng.scaled_next(0,8) as u8)
}

#[start]
fn start(_: isize, _: *const *const u8) -> isize {
    watchdog::init(watchdog::State::Disabled);
    zinc::hal::k20::init::startup(96_000_000);
    zinc::hal::k20::clocks::init_systick(1000);

    option_env!("RTC_TIME")
        .and_then(|timestr| u32::from_str(timestr).ok())
        .and_then(|time| {
            zinc::hal::k20::rtc::set(time);
            zinc::hal::k20::rtc::enable();
            Some(time)
        });

    main();
    0
}
