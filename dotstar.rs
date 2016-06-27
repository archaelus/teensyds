#![feature(start, core_intrinsics)]
#![no_std]

#[macro_use]
extern crate zinc;

use core::intrinsics::volatile_load;

use core::option::Option::Some;
use zinc::hal::uart::Parity;
use zinc::util::support::wfi;

static mut ticks: u32 = 0;
static mut global_on: u32 = 0;

const TICKS_PER_TOGGLE: u32 = 2;
const TICKS_PER_SECOND: u32 = 1;

#[allow(dead_code)]
#[no_mangle]
pub unsafe extern "C" fn isr_systick() {
    ticks += 1;
    if ticks % TICKS_PER_TOGGLE == 0 {
        global_on = !global_on;
    }
}

use zinc::hal::k20::{watchdog, uart, spi};
use zinc::hal::k20::spi::SPITransmit;
use zinc::hal::k20::uart::UARTPeripheral::UART0;
use zinc::hal::k20::pin::Port::*;
use zinc::hal::k20::pin::Pin;
use zinc::hal::k20::pin::Function;
use zinc::hal::pin::GpioDirection::*;
use zinc::hal::pin::Gpio;

use core::fmt::Write;

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
struct APA102 {
    intensity: u8,
    blue: u8,
    green: u8,
    red: u8
}

use core::convert::From;

impl From<APA102> for u32 {
    fn from(val: APA102) -> u32 {
        // magic start of led value
        0xE0_00_00_00 &
            ((val.intensity & 0x1F) as u32).wrapping_shl(24) &
            (val.blue as u32).wrapping_shl(16) &
            (val.green as u32).wrapping_shl(8) &
            (val.red as u32)
    }
}

impl From<u32> for APA102 {
    fn from (val: u32) -> APA102 {
        APA102 { intensity: (0x1F_00_00_00 & val).wrapping_shr(24) as u8,
                 blue: (0x00_FF_00_00 & val).wrapping_shr(16) as u8,
                 green: (0x00_00_FF_00 & val).wrapping_shr(8) as u8,
                 red: (0x00_00_00_FF & val) as u8 }
    }
}

use core::cmp::{min, max};
impl APA102 {
    fn from_u32(val: u32) -> APA102 {
        APA102 { intensity: (0x1F_00_00_00 & val).wrapping_shr(24) as u8,
                 blue: (0x00_FF_00_00 & val).wrapping_shr(16) as u8,
                 green: (0x00_00_FF_00 & val).wrapping_shr(8) as u8,
                 red: (0x00_00_00_FF & val) as u8 }
    }

    fn to_u32(&self) -> u32 {
        // magic start of led value
        0xE0_00_00_00 &
            ((self.intensity & 0x1F) as u32).wrapping_shl(24) &
            (self.blue as u32).wrapping_shl(16) &
            (self.green as u32).wrapping_shl(8) &
            (self.red as u32)
    }
    
    fn alt_blue(&mut self, delta: i8, lower: u8, upper:u8) -> &mut APA102 {
        self.blue = alt(self.blue, delta, lower, upper);
        self
    }

    fn alt_green(&mut self, delta: i8, lower: u8, upper:u8) -> &mut APA102 {
        self.green = alt(self.green, delta, lower, upper);
        self
    }

    fn alt_red(&mut self, delta: i8, lower: u8, upper:u8) -> &mut APA102 {
        self.green = alt(self.red, delta, lower, upper);
        self
    }
}

fn alt(val: u8, delta:i8, lower: u8, upper: u8) -> u8 {
    match delta.signum() {
        -1 => clamp(val.saturating_sub(delta.abs() as u8), lower, upper),
        0 => val,
        _ => clamp(val.saturating_add(delta as u8), lower, upper)
    }
}

fn clamp(val: u8, lower: u8, upper: u8) -> u8 {
    min(max(val, lower), upper)
}

type LedString = [APA102;72];

use zinc::hal::k20::spi::SPITxResult;

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

    writeln!(&mut uart, "\r\n\r\nOhai, we're in main(). fBUS is kicking at a cool {} Hz, and our UART is running - obvi.\r", zinc::hal::k20::clocks::bus_clock().expect("Bus clock not set"));
    //uart.format("\r\nOhai, we're in main() and our UART is running.\r\n");

    write!(&mut uart, "About to setup SPI...");
    let spi = &spi::SPI::write_only(spi::SPIPeripheral::SPI0,
                                    // SCK
                                    Pin::new(PortD, 1, Function::Gpio, Some(Out)),
                                    // SOUT
                                    Pin::new(PortC, 6, Function::Gpio, Some(Out)));
    writeln!(&mut uart, " done.\r");
    writeln!(&mut uart, "About to send a bunch of stuff to the dotstar at {} Hz.\r", spi.baud_rate());

    let preamble: &[u32] = &[0x00_00_00_00;1][..];
    let mut ledstring: [u32;72] = [0xFE_0F_0F_04;72];
    ledstring[15] = APA102::from_u32(ledstring[15]).alt_green(16, 0, 16).to_u32();
    let postamble: &[u32] = &[0x00_00_00_00;9][..];

    write!(&mut uart, "{} bit Preamble...", preamble.len() * 32);
    match preamble.transmit(&spi) {
        Ok(frames) => {writeln!(&mut uart, " {} frames.\r", frames);},
        _ => {writeln!(&mut uart, " didn't transmit correctly?\r");}
    }
    writeln!(&mut uart, "{} LED Frames.\r", ledstring.len());
    match (&ledstring[..]).transmit(&spi) {
        Ok(frames) => {writeln!(&mut uart, " {} frames.\r", frames);},
        _ => {writeln!(&mut uart, " didn't transmit correctly?\r");}
    }
    writeln!(&mut uart, "{} bit Postamble.\r", postamble.len() * 32);
    postamble.transmit(&spi);

    writeln!(&mut uart, "Welp. It should be running...\r");

    loop {
        let on: bool = unsafe { volatile_load(&global_on as *const u32) == 0 };
        match on {
            true => {
                led.set_high();
                writeln!(&mut uart, "[{:>9}] Blinky.\r",
                         unsafe { volatile_load(&ticks as *const u32) });
            }
            false => {
                led.set_low();
                writeln!(&mut uart, "[{:>9}] Blanky.\r",
                         unsafe { volatile_load(&ticks as *const u32) });                
            }
        }
        wfi();
    }
}

#[start]
fn start(_: isize, _: *const *const u8) -> isize {
    watchdog::init(watchdog::State::Disabled);
    zinc::hal::k20::init::startup(96_000_000);
    zinc::hal::k20::clocks::init_systick(1000 / TICKS_PER_SECOND);
    //zinc::hal::k20::rtc::tick_isr(true);
    main();
    0
}
