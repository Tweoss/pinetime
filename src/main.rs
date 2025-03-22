#![no_std] // don't link the Rust standard library
#![no_main] // disable all Rust-level entry points
#![allow(asm_sub_register)]

use core::{
    arch::{asm, global_asm},
    f32,
    marker::PhantomData,
    panic::PanicInfo,
};

use cortex_m::peripheral::{syst::SystClkSource, SYST};
use display::Orientation;
use embassy_nrf::{
    bind_interrupts,
    config::Config,
    gpio::{Level, Output, OutputDrive, Pin},
    interrupt::typelevel::Handler,
    pac::{clock::Clock, SPI0, SPIM0},
    peripherals,
    spim::{self, Instance, InterruptHandler},
    spis::Polarity,
    timer::{Frequency, Timer},
    Peripherals,
};
use timer::Delay;

global_asm!(
    r#"
.section .text.reset
.globl Reset
Reset:
    @ initialize BSS to zero.
    ldr r0, =__bss_start__
    ldr r1, =__bss_end__
    movs r2, #0
    0:
    @ if we've reached the end of bss, exit (jump forward).
    cmp r1, r0
    beq 1f
    @ otherwise, store 0 into address at r0 and increment r0.
    stm r0!, {{r2}}
    @ then loop (jump back).
    b 0b
    1:

    @ copy data from flash to RAM.
    ldr r0, =__data_start__
    ldr r1, =__data_end__
    ldr r2, =__la_data_start__
    0:
    cmp r1, r0
    beq 1f
    ldm r2!, {{r3}}
    stm r0!, {{r3}}
    b 0b
    1:

    @ Enable the floating point unit (NRF52 has one) by enabling CP10 and CP11
    @ coprocessors.
    @ See arm cortex-m4 (7-71) for enabling.
    ldr r0, =0xE000ED88
    ldr r1, =(0xF << 20)
    ldr r2, [r0]
    orr r2, r2, r1
    str r2, [r0]
    @ cortex-m-rt has these barriers.
    dsb
    isb

    @ rsstart shouldn't return.
    bl rsstart

    udf #0

@ Put the address of the reset handler into the vector table.
.section .vector_table.reset_vector
    .word Reset + 1
"#
);

extern "C" {
    pub fn _start();
    pub static mut RAM_START: u32;
    pub static mut RAM_SIZE: u32;
    pub static mut __code_start__: u8;
    pub static mut __code_end__: u8;
    pub static mut __data_start__: u8;
    pub static mut __data_end__: u8;
    pub static mut __bss_start__: u8;
    pub static mut __bss_end__: u8;
}

#[no_mangle]
pub extern "C" fn rsstart() -> ! {
    main();
    #[allow(clippy::empty_loop)]
    loop {}
}

bind_interrupts!(struct Irqs {
    SPI2 => InterruptHandler<peripherals::SPI2>;
});

fn main() {
    // TODO: get nrf timer (not systimer) working

    let peripherals = unsafe { embassy_nrf::Peripherals::steal() };
    let cm_peripherals = unsafe { cortex_m::Peripherals::steal() };
    // let cm_peripherals = cortex_m::Peripherals::take().unwrap();
    let mut system_timer = cm_peripherals.SYST;
    system_timer.set_clock_source(SystClkSource::Core);
    let mut system_timer = timer::SystemTimer { syst: system_timer };

    let pin = peripherals.P0_16;
    let mut vibrator = Output::new(pin, Level::High, OutputDrive::Standard);
    let mut screen = Output::new(peripherals.P0_23, Level::High, OutputDrive::Standard);

    // println!("calib: {:?}", system_timer.calib.read());

    // https://github.com/embassy-rs/embassy/blob/9d672c44d1dccaac039c656bc2986c4fcf9823c9/embassy-nrf/src/lib.rs#L886
    // https://github.com/nrf-rs/nrf-hal/blob/f132dd7966e297ba2132943ad487c68cbd88c6fb/nrf-hal-common/src/clocks.rs#L52
    let r = embassy_nrf::pac::CLOCK;
    r.tasks_hfclkstart().write_value(1);
    while r.events_hfclkstarted().read() != 1 {}
    r.events_hfclkstarted().write_value(0);

    // r.lfclksrc()
    //     .write(|w| w.set_src(embassy_nrf::pac::clock::vals::Lfclksrc::RC));
    // r.events_lfclkstarted().write_value(0);
    // r.tasks_lfclkstart().write_value(1);
    // while r.events_lfclkstarted().read() == 0 {}

    let mut delay_timer = timer::DelayTimer1Mhz::new(peripherals.TIMER0);
    for _ in 0..1 {
        screen.set_low();
        delay_timer.delay_ms(100);
        screen.set_high();
        delay_timer.delay_ms(100);
    }
    delay_timer.delay_ms(400);
    screen.set_low();

    // https://github.com/tstellanova/cst816s/blob/master/examples/touchpad.rs
    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    config.mode = spim::MODE_3;
    config.orc = 122;

    let spi = spim::Spim::new(
        peripherals.SPI2,
        Irqs,
        peripherals.P0_02,
        peripherals.P0_04,
        peripherals.P0_03,
        config,
    );

    let mut disp = display::Display::new(
        spi,
        peripherals.P0_18,
        peripherals.P0_25,
        peripherals.P0_26,
        &mut delay_timer,
    );
    disp.set_orientation(Orientation::Portrait);

    #[allow(clippy::unusual_byte_groupings)]
    for i in 0..250 {
        for j in 0..250 {
            disp.set_pixel((i, j), ((i - 125) * j * 12 + 1));
        }
    }

    delay_timer.delay_ms(3000);
}

#[link_section = ".vector_table.exceptions"]
#[no_mangle]
#[used]
// TODO: have exceptions
pub static __EXCEPTIONS__: [u32; 14] = [1; 14];

mod timer {
    use cortex_m::peripheral::SYST;
    use embassy_nrf::{
        timer::{Frequency, Instance, Timer},
        Peripheral,
    };

    pub trait Delay {
        /// Delay for at least this many microseconds.
        fn delay_us(&mut self, micros: u32);

        /// Delay for at least this many milliseconds.
        fn delay_ms(&mut self, mut millis: u32) {
            const MAX_MICROS_AS_MILLIS: u32 = u32::MAX / 1000;
            while millis > MAX_MICROS_AS_MILLIS {
                self.delay_us(MAX_MICROS_AS_MILLIS * 1000);
                millis -= MAX_MICROS_AS_MILLIS;
            }
            self.delay_us(millis * 1000);
        }
    }

    pub struct SystemTimer {
        pub syst: SYST,
    }

    pub const HFCLK_FREQ: u32 = 64_000_000;

    impl Delay for SystemTimer {
        fn delay_us(&mut self, micros: u32) {
            // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
            const MAX_RVR: u32 = 0x00FF_FFFF;

            let mut total_rvr = micros * (HFCLK_FREQ / 1_000_000);

            while total_rvr != 0 {
                let current_rvr = total_rvr.min(MAX_RVR);

                self.syst.set_reload(current_rvr);
                self.syst.clear_current();
                self.syst.enable_counter();

                total_rvr -= current_rvr;

                while !self.syst.has_wrapped() {}

                self.syst.disable_counter();
            }
        }
    }

    /// Uses the 0'th capture compare register.
    pub struct DelayTimer1Mhz<'d, T: Instance> {
        timer: Timer<'d, T>,
    }

    impl<'d, T: Instance> DelayTimer1Mhz<'d, T> {
        pub fn new(timer: impl Peripheral<P = T> + 'd) -> Self {
            let timer = Timer::new(timer);
            timer.set_frequency(Frequency::F1MHz);
            Self { timer }
        }
    }

    impl<T: Instance> Delay for DelayTimer1Mhz<'_, T> {
        /// Uses the specified capture/compare register.
        fn delay_us(&mut self, micros: u32) {
            self.timer.clear();
            self.timer.start();
            let cc = self.timer.cc(0);
            while cc.capture() < micros {}
            self.timer.stop()
        }
    }
}

mod display {
    use embassy_nrf::{
        gpio::{Flex, Level, Output, OutputDrive, Pin},
        peripherals::{P0_18, P0_25, P0_26},
        spim::{Instance, Spim},
    };

    use crate::timer::Delay;

    pub struct Display<'d, T: Instance> {
        spi: Spim<'d, T>,
        // Low for command, high for data.
        dc_pin: Output<'d>,
        cs_pin: Output<'d>,
        reset_pin: Output<'d>,
    }
    impl<'d, T: Instance> Display<'d, T> {
        // let mut display = ST7789::new(di, display_rst, SCREEN_WIDTH as u16, SCREEN_HEIGHT as u16);
        //
        // Init sequence pulled from:
        // https://github.com/sajattack/st7735-lcd-rs/blob/ecf4f96d86130661eb17395f81dec7e6cb41c3fb/src/lib.rs#L122
        // https://github.com/adafruit/Adafruit-ST7735-Library/blob/62112b90eddcb2ecc51f474e9fe98b68eb26cb2a/Adafruit_ST7789.cpp#L53
        pub fn new(
            spi: Spim<'d, T>,
            data_command_pin: P0_18,
            chip_select_pin: P0_25,
            reset_pin: P0_26,
            delay_source: &mut impl Delay,
        ) -> Self {
            let cs_pin = Output::new(chip_select_pin, Level::Low, OutputDrive::Standard);
            let dc_pin = Output::new(data_command_pin, Level::Low, OutputDrive::Standard);
            let mut reset_pin = Output::new(reset_pin, Level::Low, OutputDrive::Standard);

            #[no_mangle]
            #[inline(never)]
            pub extern "C" fn target_break() {
                unsafe { core::arch::asm!("") }
            }

            // Hard reset.
            reset_pin.set_high();
            delay_source.delay_us(10);
            reset_pin.set_low();
            delay_source.delay_us(10);
            reset_pin.set_high();
            delay_source.delay_us(10);
            target_break();

            let mut d = Self {
                spi,
                dc_pin,
                cs_pin,
                reset_pin,
            };
            let mut delay_ms = |ms: u32| {
                delay_source.delay_ms(ms);
            };

            d.write_cmd(Instruction::SWRESET); // Software reset.
            delay_ms(150);
            d.write_cmd(Instruction::SLPOUT); // out of sleep mode.
            delay_ms(10);
            d.write_cmd(Instruction::COLMOD); // set color mode.
            d.write_data(&[0b0101_0101]); // 16-bit color.
            delay_ms(10);

            // d.write_cmd(Instruction::VSCRDER); // vertical scroll definition
            // d.write_data(&[0u8, 0u8, 0x14u8, 0u8, 0u8, 0u8]); // 0 TSA, 320 VSA, 0 BSA

            // the two sources differ on madctl
            d.write_cmd(Instruction::MADCTL); // left -> right, bottom -> top RGB
            d.write_data(&[0b0000_0000]);
            d.write_cmd(Instruction::CASET); // column addr set
            d.write_data(&[0x0, 0, 0, 240]); // x start 0, xend 240
            d.write_cmd(Instruction::RASET); // column addr set
            d.write_data(&[0x0, 0, (320_u32 >> 8) as u8, (320_u32 & 0xFF) as u8]); // y start 0, y end 320

            // TODO:  VSCRDER?

            d.write_cmd(Instruction::INVON); // supposedly a hack: turn on invert?
            delay_ms(10);
            d.write_cmd(Instruction::NORON); // turn on normal display.
            delay_ms(10);
            d.write_cmd(Instruction::DISPON); // turn on main screen.
            delay_ms(10);

            d
        }

        fn write_cmd(&mut self, instr: Instruction) {
            // TODO: unnecessary cs_pin?
            // self.cs_pin.set_low();
            self.dc_pin.set_low();
            let _ = self.spi.blocking_write(&[instr as u8]);
            // self.cs_pin.set_high();
        }

        fn write_data(&mut self, data: &[u8]) {
            self.cs_pin.set_low();
            self.dc_pin.set_high();
            let _ = self.spi.blocking_write(data);
            self.cs_pin.set_high();
        }

        pub fn set_orientation(&mut self, orientation: Orientation) {
            self.write_cmd(Instruction::MADCTL);
            self.write_data(&[orientation as u8]);
        }

        pub fn set_pixel(&mut self, (x, y): (u16, u16), color: u16) {
            self.set_address_window((x, y), (x, y));
            self.write_cmd(Instruction::RAMWR);
            self.write_data(&color.to_be_bytes());
        }

        pub fn set_pixels(
            &mut self,
            start: (u16, u16),
            end: (u16, u16),
            colors: impl IntoIterator<Item = u16>,
        ) {
            self.set_address_window(start, end);
            self.write_cmd(Instruction::RAMWR);
            // self.write_data(data);
            todo!()
            // self.write_data(&color.to_be_bytes());
        }

        fn set_address_window(&mut self, start: (u16, u16), end: (u16, u16)) {
            self.write_cmd(Instruction::CASET);
            self.write_data(&start.0.to_be_bytes());
            self.write_data(&end.0.to_be_bytes());
            self.write_cmd(Instruction::RASET);
            self.write_data(&start.1.to_be_bytes());
            self.write_data(&end.1.to_be_bytes());
        }
    }

    #[repr(u8)]
    pub enum Orientation {
        Portrait = 0b0000_0000,         // no inverting
        Landscape = 0b0110_0000,        // invert column and page/column order
        PortraitSwapped = 0b1100_0000,  // invert page and column order
        LandscapeSwapped = 0b1010_0000, // invert page and page/column order
    }

    /// ST7789 instructions. https://github.com/sajattack/st7735-lcd-rs/blob/ecf4f96d86130661eb17395f81dec7e6cb41c3fb/src/instruction.rs#L3
    #[repr(u8)]
    enum Instruction {
        NOP = 0x00,
        SWRESET = 0x01,
        RDDID = 0x04,
        RDDST = 0x09,
        SLPIN = 0x10,
        SLPOUT = 0x11,
        PTLON = 0x12,
        NORON = 0x13,
        INVOFF = 0x20,
        INVON = 0x21,
        DISPOFF = 0x28,
        DISPON = 0x29,
        CASET = 0x2A,
        RASET = 0x2B,
        RAMWR = 0x2C,
        RAMRD = 0x2E,
        PTLAR = 0x30,
        VSCRDER = 0x33,
        COLMOD = 0x3A,
        MADCTL = 0x36,
        VSCAD = 0x37,
        VCMOFSET = 0xC5,
    }
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
