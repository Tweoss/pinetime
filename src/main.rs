#![no_std] // don't link the Rust standard library
#![no_main] // disable all Rust-level entry points
#![allow(asm_sub_register)]

use core::{
    arch::{asm, global_asm},
    marker::PhantomData,
    panic::PanicInfo,
};

use cortex_m::peripheral::{syst::SystClkSource, SYST};
use embassy_nrf::{
    bind_interrupts,
    config::Config,
    gpio::{Level, Output, OutputDrive},
    interrupt::typelevel::Handler,
    pac::clock::Clock,
    peripherals,
    spim::{self, Instance, InterruptHandler},
    timer::{Frequency, Timer},
    Peripherals,
};

// use embedded_hal::digital::OutputPin;

/// See nrf52 page 25
const STACK_ADDR: usize = 0x81_0000 - 4;

global_asm!(r#"
.section ".text.start"
.globl _start
_start:
    @ @ force the mode to be SUPER.
    @ mov r0,  
    @ orr r0,r0,#(1<<7)    @ disable interrupts.
    @ msr cpsr, r0

    @ @ prefetch flush
    @ mov r1, #0;
    @ mcr p15, 0, r1, c7, c5, 4
    @ ldr r0, =0x50000518
    @ mvn r1, #0
    @ str r1, [r0]
    @ ldr r0, =0x5000050C
    @ mvn r1, #0
    @ str r1, [r0]

    @ mov sp,         @ initialize stack pointer
    ldr sp, _stack_addr @ initialize stack pointer
    @ mov fp, #0          @ clear frame pointer reg.  don't think needed.
    bl rsstart          @ we could jump right to rsstart (notmain)
    @ bl _cstart        @ call our code to do initialization.
    _stack_addr: .word {}

    @ _interrupt_table_end:   @ end of the table.
"#
, const STACK_ADDR);

extern "C" {
    pub fn _start();
    pub static mut __code_start__: u8;
    // pub static mut _start: u8;
    pub static mut __code_end__: u8;
    pub static mut __data_start__: u8;
    pub static mut __data_end__: u8;
    pub static mut __bss_start__: u8;
    pub static mut __bss_end__: u8;
}

#[no_mangle]
pub unsafe extern "C" fn rsstart() -> ! {
    // // TODO: move bss somewhere else
    // // Safety: I *believe* this is sufficient to prevent compiler reorderings.
    // // https://stackoverflow.com/questions/72823056/how-to-build-a-barrier-by-rust-asm
    // asm!("");
    // // Not sure if this is sound.
    // // Was unable to observe nonzeroed BSS before, so saw no change.
    // let count = (&raw const __bss_end__).byte_offset_from(&raw const __bss_start__);

    // for index in 0..count {
    //     // Use assembly instead of a slice copy because rust/LLVM believes that
    //     // is guaranteed to be undefined => unreachable after.
    //     let dest = (&raw mut __bss_start__).byte_offset(index * (size_of::<u32>() as isize));
    //     let source = 0_u32;
    //     asm!("str {}, [{}]", in(reg) source, in(reg) dest);
    // }
    // asm!("");
    // interrupt_init();

    // //     // now setup timer interrupts.
    // //     //  - Q: if you change 0x100?
    // //     //  - Q: if you change 16?
    // assert!(!timer_initialized());
    // // // interrupts::timer_init(1, 0x100);
    // // interrupts::timer_init(16, 0x1000);
    // // assert!(timer_initialized());
    // gpio_interrupts_init();

    // pi0_lib::debug::setup();

    // cycle_counter::init();

    // let mut peripherals = unsafe { Peripherals::steal() };
    // let pins = unsafe { get_pins() };
    // let (p14, pins): (Pin<14, { PinFsel::Unset }>, _) = pins.pluck();
    // let (p15, _pins): (Pin<15, { PinFsel::Unset }>, _) = pins.pluck();
    // let w = setup_uart(p14, p15, &mut peripherals);
    // store_uart(w);

    // pi0_lib::virtual_memory::setup();

    main();
    #[allow(clippy::empty_loop)]
    loop {}
    // rpi_reboot();
}

struct SpimInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> Handler<T::Interrupt> for SpimInterruptHandler<T> {
    #[doc = r" Interrupt handler function."]
    #[doc = r""]
    #[doc = r" Must be called every time the `I` interrupt fires, synchronously from"]
    #[doc = r" the interrupt handler context."]
    #[doc = r""]
    #[doc = r" # Safety"]
    #[doc = r""]
    #[doc = r" This function must ONLY be called from the interrupt handler for `I`."]
    unsafe fn on_interrupt() {
        todo!()
    }
}

bind_interrupts!(struct Irqs {
    SPI2 => InterruptHandler<peripherals::SPI2>;
});

fn main() {
    // 0x50000000 GPIO P0 General purpose input and output
    // - P0.16/TRACEDATA1 VIBRATOR OUT OUT
    // OUTSET 0x508 Set individual bits in GPIO port
    // OUTCLR 0x50C Clear individual bits in GPIO port
    // DIRSET 0x518 DIR set register
    // unsafe { ((0x5000_0000 + 0x518) as *mut u32).write_volatile(!0x0) };
    // loop {
    //     unsafe {
    //         ((0x50000000 + 0x50C) as *mut u32).write_volatile(!0x0);
    //     }
    // }
    //
    //    uint32_t args[3];
    // args[0] = (uint32_t)fh;
    // args[1] = (uint32_t)buffer;
    // args[2] = (uint32_t)length;
    // return __semihost(SYS_WRITE, args);
    // unsafe { ((0x50000000 + 0x518) as *mut u32).write_volatile(0x10000) };
    // unsafe { ((0x50000000 + 0x518) as *mut u32).write_volatile(0x1 << 14) };

    // let peripherals = embassy_nrf::init(Config::default());
    let peripherals = unsafe { embassy_nrf::Peripherals::steal() };
    let cm_peripherals = unsafe { cortex_m::Peripherals::steal() };
    // let cm_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut system_timer = cm_peripherals.SYST;
    system_timer.set_clock_source(SystClkSource::Core);

    let pin = peripherals.P0_16;
    let mut vibrator = Output::new(pin, Level::High, OutputDrive::Standard);
    let mut screen = Output::new(peripherals.P0_14, Level::High, OutputDrive::Standard);

    vibrator.set_low();

    for i in 0..100 {
        screen.set_low();
        for _ in 0..50 {
            system_timer.set_reload(0xF1111118);
            system_timer.clear_current();
            system_timer.enable_counter();
            while !system_timer.has_wrapped() {}
        }
        screen.set_high();
        system_timer.disable_counter();
        for _ in 0..50 {
            system_timer.set_reload(0xF1111118);
            system_timer.clear_current();
            system_timer.enable_counter();
            while !system_timer.has_wrapped() {}
        }
        screen.set_low();
        system_timer.disable_counter();
    }
    println!("continued");
    // Clock::start
    // https://github.com/embassy-rs/embassy/blob/9d672c44d1dccaac039c656bc2986c4fcf9823c9/embassy-nrf/src/lib.rs#L886
    // https://github.com/nrf-rs/nrf-hal/blob/f132dd7966e297ba2132943ad487c68cbd88c6fb/nrf-hal-common/src/clocks.rs#L52
    // let r = embassy_nrf::pac::CLOCK;
    // r.tasks_hfclkstart().write_value(1);
    // while r.events_hfclkstarted().read() == 0 {}
    // r.events_hfclkstarted().write_value(0);

    // r.lfclksrc()
    //     .write(|w| w.set_src(embassy_nrf::pac::clock::vals::Lfclksrc::RC));
    // r.events_lfclkstarted().write_value(0);
    // r.tasks_lfclkstart().write_value(1);
    // while r.events_lfclkstarted().read() == 0 {}

    // let timer = Timer::new_counter(peripherals.TIMER0);
    // timer.set_frequency(Frequency::F8MHz);
    // timer.set_frequency(Frequency::F4MHz);
    // // peripherals.SPI2;
    // let mut config = spim::Config::default();
    // // https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
    // config.frequency = spim::Frequency::M8;
    // config.mode = spim::MODE_3;
    // config.orc = 122;

    // let mut spi = spim::Spim::new(
    //     peripherals.SPI2,
    //     Irqs,
    //     peripherals.P0_02,
    //     peripherals.P0_04,
    //     peripherals.P0_03,
    //     config,
    // );

    // spi.blocking_write(&[0u8]);

    // timer.clear();
    // timer.start();

    // let cc = timer.cc(0);
    // while cc.capture() == 0 {
    //     // timer.task_count().trigger();
    //     println!("{}", cc.capture());
    // }
    // // for _ in 0..8_000_000 {
    // //     unsafe { asm!("nop") }
    // // }
    // // for _ in 0..8_000_000 {
    // //     unsafe { asm!("nop") }
    // // }
    // // timer.clear();
    // while timer.cc(0).read() < 125_000 {
    //     unsafe { asm!("nop") }
    // }
    // // vibrator.set_high();
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    #[allow(clippy::empty_loop)]
    loop {}
}

pub static mut SEMIHOSTING_WRITER: SemihostingWriter = SemihostingWriter {};

pub struct SemihostingWriter {}
//
// https://github.com/apache/mynewt-core/blob/dadb17412b2973f18b061b10eee275081d610392/hw/drivers/semihosting/src/mbed_semihost_api.c#L51
fn write_debug_str(file_handle: u32, buffer: &[u8]) {
    // https://github.com/apache/mynewt-core/blob/dadb17412b2973f18b061b10eee275081d610392/hw/drivers/semihosting/src/mbed_semihost_api.c#L26
    const SYS_WRITE: u32 = 0x5;

    let args = [file_handle, buffer.as_ptr() as u32, buffer.len() as u32];
    unsafe { semihost(SYS_WRITE, &args) };
}
// https://github.com/lupyuen/pinetime-rust-mynewt/blob/master/libs/semihosting_console/src/semihosting_console.c#L97
/// We normally set the file handle to 2 to write to the debugger's stderr output.
const SEMIHOST_HANDLE: u32 = 2;

// https://github.com/apache/mynewt-core/blob/master/hw/drivers/semihosting/include/semihosting/mbed_semihost_api.h#L46
unsafe extern "C" fn semihost<T>(reason: u32, arg: &T) -> u32 {
    let arg = (arg as *const T) as u32;
    let out;
    // https://github.com/lupyuen/pinetime-rust-mynewt/blob/eab7a6460957e3759b956f8a9232e17081dabd56/libs/semihosting_console/src/semihosting_console.c#L64
    asm!(
        "
            mov r0, {CMD}
            mov r1, {MSG}
            bkpt #0xAB
            mov {OUTPUT}, r0
            ",
        CMD = in(reg) reason,
        MSG = in(reg) arg,
        OUTPUT = out(reg) out,
    );
    out
}

impl core::fmt::Write for SemihostingWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write_debug_str(SEMIHOST_HANDLE, s.as_bytes());
        Ok(())
    }
}
/// This will error if the args cause an interrupt (like software interrupt).
#[macro_export]
macro_rules! dbg {
    ($( $args:expr),* ) => {
        unsafe {
            let w = &raw mut $crate::SEMIHOSTING_WRITER;
            $(
                core::fmt::Write::write_fmt(w.as_mut().unwrap(), format_args!("[{}:{}:{}] ", file!(), line!(), column!())).unwrap();
                core::fmt::Write::write_fmt(w.as_mut().unwrap(), format_args!("{} = {:?}", stringify!($args), $args)).unwrap();
                core::fmt::Write::write_str(w.as_mut().unwrap(), "\n").unwrap();
            )*
        }
    };
}

/// This will error if the args cause an interrupt (like software interrupt).
#[macro_export]
macro_rules! print {
    ($( $args:tt)* ) => {
        unsafe {
            let w = &raw mut $crate::SEMIHOSTING_WRITER;
            core::fmt::Write::write_fmt(w.as_mut().unwrap(), format_args!($($args)*)).unwrap();
        }
    };
}

/// This will error if the args cause an interrupt (like software interrupt).
#[macro_export]
macro_rules! println {
    ($( $args:tt)* ) => {
        unsafe {
            let w = &raw mut $crate::SEMIHOSTING_WRITER;
            core::fmt::Write::write_fmt(w.as_mut().unwrap(), format_args!($($args)*)).unwrap();
            core::fmt::Write::write_str(w.as_mut().unwrap(), "\n").unwrap();
        }
    };
}
