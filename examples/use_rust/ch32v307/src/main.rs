#![no_std]
#![no_main]

mod keymap;
mod logger;
mod vial;

use core::mem::MaybeUninit;
use core::panic::PanicInfo;

use ch32_hal::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use ch32_hal::i2c::I2c;
use ch32_hal::mode::Blocking;
use ch32_hal::otg_fs::{self, Driver};
use ch32_hal::peripherals::{I2C2, USART1};
use ch32_hal::time::Hertz;
use ch32_hal::usart::UartTx;
use ch32_hal::usb::EndpointDataBuffer;
use ch32_hal::{self as hal, bind_interrupts, peripherals, rcc, usart, Config, Peripherals};
use defmt::{info, println, unwrap};
use embassy_executor::Spawner;
use embassy_time::Timer;
use logger::set_logger;
use rmk::channel::EVENT_CHANNEL;
use rmk::config::{BehaviorConfig, ControllerConfig, KeyboardUsbConfig, RmkConfig, VialConfig};
use rmk::debounce::default_debouncer::DefaultDebouncer;
use rmk::futures::future::join3;
use rmk::input_device::Runnable;
use rmk::keyboard::Keyboard;
use rmk::light::LightController;
use rmk::matrix::Matrix;
use rmk::{initialize_keymap, k, run_devices, run_rmk};
use static_cell::StaticCell;
use vial::{VIAL_KEYBOARD_DEF, VIAL_KEYBOARD_ID};

use crate::keymap::{COL, ROW};

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
});

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    critical_section::with(|_| {
        if unsafe { !crate::logger::has_logger() } {
            let uart1_config = usart::Config::default();
            unsafe {
                // SAFETY: PANICCCCCCCC
                let p = Peripherals::steal();
                LOGGER_UART =
                    MaybeUninit::new(UartTx::<'static, _, _>::new_blocking(p.USART1, p.PA9, uart1_config).unwrap());
            };
            set_logger(&|data| unsafe {
                #[allow(unused_must_use, static_mut_refs)]
                LOGGER_UART.assume_init_mut().blocking_write(data);
            });
        }
        let info = unsafe { core::ptr::read_volatile(&raw const info) };
        if let Some(location) = info.location() {
            println!(
                "panic occurred in file '{}' at line {}",
                location.file(),
                location.line()
            );
        } else {
            println!("panic occurred but can't get location information...");
        }
        loop {}
    })
}

static mut LOGGER_UART: MaybeUninit<UartTx<'static, USART1, Blocking>> = MaybeUninit::uninit();

fn fusb(mut i2c: I2c<'static, I2C2, Blocking>) {
    let mut buf = [0u8; 1];
    unwrap!(i2c.blocking_write(0x31, &[0x5, 0b0111_1011]));
    unwrap!(i2c.blocking_write_read(0x31, &[0x5], &mut buf));
    println!("0x31 0x5 reg: {:#b}", buf[0]);
    unwrap!(i2c.blocking_write_read(0x31, &[0x03], &mut buf));
    println!("0x31 0x03 reg: {:#b}", buf[0]);
    unwrap!(i2c.blocking_write(0x31, &[0x03, buf[0] | 0x2]));

    unwrap!(i2c.blocking_write(0x21, &[0x5, 0b0111_1011]));
    unwrap!(i2c.blocking_write_read(0x21, &[0x5], &mut buf));
    println!("0x21 0x5 reg: {:#b}", buf[0]);
    unwrap!(i2c.blocking_write_read(0x21, &[0x03], &mut buf));
    println!("0x21 0x03 reg: {:#b}", buf[0]);
    unwrap!(i2c.blocking_write(0x21, &[0x03, 0b0001_0010]));
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let rcc_cfg: rcc::Config = {
        use rcc::*;

        Config {
            hse: Some(Hse {
                freq: Hertz(16_000_000),
                mode: HseMode::Oscillator,
            }),
            sys: Sysclk::PLL,
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV2,
                mul: PllMul::MUL18,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: LsConfig::default_lsi(),
            hspll_src: HsPllSource::HSE,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV4,
            }),
        }
    };

    // setup clocks
    let cfg = Config {
        rcc: rcc_cfg,
        ..Default::default()
    };
    let p = hal::init(cfg);
    // Setup the printer
    let uart1_config = usart::Config::default();
    unsafe {
        LOGGER_UART = MaybeUninit::new(UartTx::<'static, _, _>::new_blocking(p.USART1, p.PA9, uart1_config).unwrap());
    };
    set_logger(&|data| unsafe {
        #[allow(unused_must_use, static_mut_refs)]
        LOGGER_UART.assume_init_mut().blocking_write(data);
    });

    // wait for serial-cat
    Timer::after_millis(2069).await;
    info!("rmk");

    // Setup I2C
    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    let i2c = I2c::new_blocking(p.I2C2, i2c_scl, i2c_sda, Hertz::khz(100), Default::default());
    fusb(i2c);

    /* USB DRIVER SECION */
    static BUFFER: StaticCell<[EndpointDataBuffer; 8]> = StaticCell::new();
    // let mut buffer = ;
    let driver = Driver::new(
        p.OTG_FS,
        p.PA12,
        p.PA11,
        BUFFER.init(core::array::from_fn(|_| EndpointDataBuffer::default())),
    );

    let cols: [AnyPin; COL] = [p.PC6.degrade(), p.PC7.degrade(), p.PA4.degrade()];
    let rows: [AnyPin; ROW] = [p.PB1.degrade(), p.PC4.degrade(), p.PC5.degrade()];

    let output_pins: [Output; COL] = cols.map(|c| Output::new(c, Level::Low, ch32_hal::gpio::Speed::High));
    let input_pins: [Input; ROW] = rows.map(|r| Input::new(r, Pull::Down));

    let mut default_keymap = keymap::get_default_keymap();
    let keymap = initialize_keymap(&mut default_keymap, BehaviorConfig::default()).await;

    // Initialize the matrix and keyboard
    let debouncer = DefaultDebouncer::<ROW, COL>::new();
    let mut matrix = Matrix::<_, _, _, ROW, COL>::new(input_pins, output_pins, debouncer);
    // let mut matrix = rmk::matrix::TestMatrix::<ROW, COL>::new();
    let mut keyboard = Keyboard::new(&keymap); // Initialize the light controller

    let keyboard_usb_config = KeyboardUsbConfig {
        vid: 0x4c4b,
        pid: 0x4643,
        manufacturer: "Haobo",
        product_name: "Ch32 RMK Keyboard",
        serial_number: "vial:f64c2b3c:000001",
    };
    let vial_config = VialConfig::new(VIAL_KEYBOARD_ID, VIAL_KEYBOARD_DEF);

    let rmk_config = RmkConfig {
        usb_config: keyboard_usb_config,
        vial_config,
        ..Default::default()
    };

    // Initialize the light controller
    let mut light_controller: LightController<Output> = LightController::new(ControllerConfig::default().light_config);

    join3(
        run_devices! (
            (matrix) => EVENT_CHANNEL,
        ),
        keyboard.run(), // Keyboard is special
        run_rmk(&keymap, driver, &mut light_controller, rmk_config),
    )
    .await;
}
