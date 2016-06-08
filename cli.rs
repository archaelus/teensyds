#![feature(start, core_intrinsics)]
#![no_std]

#[macro_use]
extern crate zinc;

use core::intrinsics::volatile_load;

use core::option::Option::Some;
use zinc::hal::uart::Parity;
use zinc::drivers::chario::CharIO;
use zinc::util::support::wfi;

static mut ticks: u32 = 0;
static mut global_on: u32 = 0;

const TICKS_PER_TOGGLE: u32 = 1;
const TICKS_PER_SECOND: u32 = 1;

#[allow(dead_code)]
#[no_mangle]
pub unsafe extern "C" fn isr_systick() {
    ticks += 1;
    if (ticks % TICKS_PER_TOGGLE == 0) {
        global_on = !global_on;
    }
}

use zinc::hal::k20::{watchdog, uart};
use zinc::hal::k20::uart::UARTPeripheral::UART0;
use zinc::hal::k20::pin::Port::*;
use zinc::hal::k20::pin::Pin;
use zinc::hal::k20::pin::Function;
use zinc::hal::pin::GpioDirection::*;
use zinc::hal::pin::Gpio;


pub fn main() {
    // Pins for Teensy 3.1 (http://www.pjrc.com/)
    let led1 = Pin::new(PortC,
                        5,
                        Function::Gpio,
                        Some(Out));
    let uart = uart::UART::new(UART0, 115200, 8, Parity::Disabled, 1,
                               Pin::new(PortB, 16, Function::Gpio, Some(In)),
                               Pin::new(PortB, 17, Function::Gpio, Some(Out)));
    
    // uart.puts("Ehrmergherd. We're uartin.\n");

    loop {
        let on: bool = unsafe { volatile_load(&global_on as *const u32) == 0 };
        match on {
            true => {
                led1.set_high();
                uart.puti(unsafe { volatile_load(&ticks as *const u32) });
                uart.puts(" Blinky.\r\n");
            }
            false => {
                led1.set_low();
                uart.puti(unsafe { volatile_load(&ticks as *const u32) });
                uart.puts(" Blanky.\r\n");
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
