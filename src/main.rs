#![no_std]
#![no_main]

mod e220;
mod errors;
mod pelcod;
mod protocols;
mod ptzdriver;
mod relays;

use core::cell::RefCell;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Input, Level, Output, Speed};
use embassy_stm32::peripherals::PC13;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv,
    PllSource, Sysclk,
};
use embassy_stm32::spi;
use embassy_stm32::time::mhz;
use embassy_stm32::usart::BufferedUart;
use embassy_stm32::{Config, bind_interrupts, dma, peripherals, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::{Read, Write};

use {defmt_rtt as _, panic_probe as _};

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X12},
    pixelcolor::{Rgb565, raw::ToBytes},
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle, Triangle},
    text::Text,
};
use heapless::{Vec, format};
use ssd1331_async::{BitDepth, Framebuffer, Ssd1331, WritePixels};
use static_cell::{ConstStaticCell, StaticCell};

use e220::*;
use errors::Error;
use pelcod::*;
use protocols::*;
use ptzdriver::*;
use relays::*;

// адрес модуля - для компиляции сервера =1
// для исполнительных модулей 2, 3 ...
//const MODULE_ADDRESS: u8 = 2;

const MODULE_ADDRESS_H: u8 = 44;
const MODULE_ADDRESS_L: u8 = 79;
const MASTER_ADDRESS_H: u8 = 78;
const MASTER_ADDRESS_L: u8 = 6;
const CRYPT_H: u8 = 50;
const CRYPT_L: u8 = 19;

// состояние приложения для обмена между потоками
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct AppContext {
    target_az: AzAngle,
    target_el: ElAngle,
    current_az: AzAngle,
    current_el: ElAngle,
    noise: u8,
    signal: u8,
    seq: u32,
    power_control: Relays,
}

static APP_CONTEXT: Mutex<CriticalSectionRawMutex, RefCell<AppContext>> =
    Mutex::new(RefCell::new(AppContext {
        target_az: AzAngle(0),
        target_el: ElAngle(0),
        current_az: AzAngle(0),
        current_el: ElAngle(0),
        noise: 0,
        signal: 0,
        seq: 0,
        power_control: Relays::new(),
    }));

static LCD_REDRAW: Signal<CriticalSectionRawMutex, ()> = Signal::new();

const FRAME_BUFFER_SIZE: usize = 32 * 40 * 2;
static PIXEL_DATA: ConstStaticCell<[u8; FRAME_BUFFER_SIZE]> =
    ConstStaticCell::new([0; FRAME_BUFFER_SIZE]);

// таск для того, чтоб чип не засыпал и номально прошивался без ресет
#[embassy_executor::task]
async fn idle() {
    loop {
        embassy_futures::yield_now().await;
    }
}

// таск мигания светодиодом
#[embassy_executor::task]
async fn blinky(led: Peri<'static, PC13>) {
    let mut led = Output::new(led, Level::High, Speed::Low);
    loop {
        led.toggle();
        Timer::after_millis(300).await;
    }
}

// таск управления PTZ
#[embassy_executor::task]
async fn ptzrotor(uart: BufferedUart<'static>) {
    let mut ptz = PTZDriver::new(1, uart).await;
    loop {
        Timer::after_millis(300).await; // заменить после настройки чтения!!!
        //info!("calling ptz...");

        match ptz.runner().await {
            Ok(_) => {}
            Err(e) => {
                //println!("Error UART {:?}", e);
                info!("ptz error...");
                continue;
            }
        }
    }
}

bind_interrupts!(struct Irqs {
    DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
//    DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
});

bind_interrupts!(struct IrqsUART2 {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

bind_interrupts!(struct IrqsUART1 {
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // настраиваем тактирование всего
    let mut config = embassy_stm32::Config::default();
    // Configure HSE (25 MHz Crystal)
    config.rcc.hse = Some(Hse {
        freq: mhz(25),
        mode: HseMode::Oscillator,
    });

    // Configure PLL for 96 MHz System Clock
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,  // PLLM 25MHz / 25 = 1 MHz
        mul: PllMul::MUL384,       // PLLN 1MHz * 384 = 384 MHz (VCO)
        divp: Some(PllPDiv::DIV4), // 384MHz / 4 = 96 MHz (SYSCLK)
        divq: Some(PllQDiv::DIV8), // 384MHz / 8 = 48 MHz (USB CLK)
        divr: None,
    });

    // Set System Clock Source
    config.rcc.sys = Sysclk::PLL1_P;

    // Bus Prescalers (Critical for Stability)
    // AHB  = 96 MHz (Max 100)
    // APB1 = 48 MHz  (Max 50) -> Must be DIV2 or higher
    // APB2 = 96 MHz (Max 100)
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    let p = embassy_stm32::init(config);

    info!("Starting...");

    let mut display = {
        // настройка SPI
        let mut spi_config = spi::Config::default();
        spi_config.frequency = embassy_stm32::time::mhz(50);
        let spi_bus = spi::Spi::new_txonly(p.SPI1, p.PA5, p.PA7, p.DMA2_CH2, Irqs, spi_config);
        let cs = Output::new(p.PA4, Level::Low, Speed::VeryHigh);
        let spi_dev = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

        let rst = Output::new(p.PB0, Level::Low, Speed::VeryHigh);
        let dc = Output::new(p.PB1, Level::Low, Speed::VeryHigh);
        Ssd1331::new(
            ssd1331_async::Config::default(),
            rst,
            dc,
            spi_dev,
            &mut Delay {},
        )
        .await
        .unwrap()
    };

    // USART2 - порт для передачи команд pelcod
    let usart2 = {
        let mut config_usart2 = usart::Config::default();
        config_usart2.baudrate = 2400;
        static TX_BUF2: StaticCell<[u8; 32]> = StaticCell::new();
        let tx_buf2 = &mut TX_BUF2.init([0; 32])[..];
        static RX_BUF2: StaticCell<[u8; 32]> = StaticCell::new();
        let rx_buf2 = &mut RX_BUF2.init([0; 32])[..];
        BufferedUart::new(
            p.USART2,
            p.PA3,
            p.PA2,
            tx_buf2,
            rx_buf2,
            IrqsUART2,
            config_usart2,
        )
        .unwrap()
    };

    // USART1 - порт для передачи команд e220
    let mut usart1 = {
        let mut config_usart1 = usart::Config::default();
        config_usart1.baudrate = 9600;
        static TX_BUF1: StaticCell<[u8; 32]> = StaticCell::new();
        let tx_buf1 = &mut TX_BUF1.init([0; 32])[..];
        static RX_BUF1: StaticCell<[u8; 32]> = StaticCell::new();
        let rx_buf1 = &mut RX_BUF1.init([0; 32])[..];
        BufferedUart::new(
            p.USART1,
            p.PB7,
            p.PB6,
            tx_buf1,
            rx_buf1,
            IrqsUART1,
            config_usart1,
        )
        .unwrap()
    };
    /*
    PB4 - M0
    PB3 - M1
    PA15 - AUX
     */
    let m0 = Output::new(p.PB4, Level::Low, Speed::VeryHigh);
    let m1 = Output::new(p.PB3, Level::Low, Speed::VeryHigh);
    let aux = Input::new(p.PA15, embassy_stm32::gpio::Pull::None);

    let ptz_pwr = Output::new(p.PB12, Level::Low, Speed::Low);
    let lna_pwr = Output::new(p.PB13, Level::Low, Speed::Low);

    //let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    spawner.spawn(idle().unwrap()); // бесконечный цикл для предотвращения сна
    spawner.spawn(blinky(p.PC13).unwrap()); // мигание светодиодом
    spawner.spawn(ptzrotor(usart2).unwrap()); // управление повороткой
    spawner.spawn(process_e220(usart1, m0, m1, aux, ptz_pwr, lna_pwr).unwrap()); // работа в эфире

    // Use the first 12x6x2 bytes of the static buffer to render text
    // character by character and transfer it to the screen. If we couldn't
    // spare 144 bytes, we could do this in even smaller chunks.
    let pixel_data = PIXEL_DATA.take();
    let font = TextRenderer::new(include_bytes!("./font_6x12.bin"), Size::new(6, 12));
    //let start = Instant::now();
    font.render_text(
        "PTZ",
        Point::zero(),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "REQ",
        Point::new(0, 12),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "SEQ",
        Point::new(0, 24),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "R",
        Point::new(64, 0),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;
    font.render_text(
        "L",
        Point::new(64, 12),
        Rgb565::WHITE,
        Rgb565::BLACK,
        pixel_data,
        &mut display,
    )
    .await;

    // дефолтные значения после включения
    {
        let ctx = APP_CONTEXT.lock().await;
        let mut inner = ctx.borrow_mut();
        inner.target_az = AzAngle::try_from(1000).unwrap();
        inner.target_el = ElAngle::try_from(4500).unwrap();
    }

    let mut az2print = AzAngle(0);
    let mut el2print = ElAngle(0);
    let mut azr2print = AzAngle(0);
    let mut elr2print = ElAngle(0);
    let mut s2print = 0;
    let mut n2print = 0;
    let mut seq2print = 0;
    let mut relay2print = Relays::new();

    /*info!("test uart1...");
    const ANS: &[u8] = b"OK";
    usart1.write_all(ANS).await.ok();
    let mut read_buf: [u8; 10] = [0; 10]; // Читаем по одному символу
    usart1.read(&mut read_buf).await.ok();
    info!("uart1 read {}", read_buf);*/

    loop {
        //Timer::after_millis(500).await;
        //const ERR_ANS: &[u8] = b"?\n";
        //usart2.write_all(ERR_ANS).await.ok();
        //embassy_futures::yield_now().await;
        /*info!("high");
         led.set_high();
        Timer::after_millis(300).await;

        info!("low");
        led.set_low();
        Timer::after_millis(300).await;*/
        {
            let ctx = APP_CONTEXT.lock().await;
            let inner = ctx.borrow();
            az2print = inner.current_az;
            el2print = inner.current_el;
            azr2print = inner.target_az;
            elr2print = inner.target_el;
            s2print = rssi_to_dbm(inner.signal);
            n2print = rssi_to_dbm(inner.noise);
            seq2print = inner.seq;
            relay2print = inner.power_control;
        }
        let ptz_str = format!(12;"{:03} {:02}", az2print, el2print).unwrap();
        font.render_text(
            &ptz_str,
            Point::new(24, 0),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let req_str = format!(12;"{:03} {:02}", azr2print, elr2print).unwrap();
        font.render_text(
            &req_str,
            Point::new(24, 12),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let seq_str = format!(12;"{}", seq2print).unwrap();
        font.render_text(
            &seq_str,
            Point::new(24, 24),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;
        let rssi_str = format!(20;"S:{:04} N:{:04}", s2print, n2print).unwrap();
        font.render_text(
            &rssi_str,
            Point::new(0, 36),
            Rgb565::GREEN,
            Rgb565::BLACK,
            pixel_data,
            &mut display,
        )
        .await;

        if relay2print.is_ptz_on() {
            font.render_text(
                "ON ",
                Point::new(76, 0),
                Rgb565::RED,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        } else {
            font.render_text(
                "OFF",
                Point::new(76, 0),
                Rgb565::BLUE,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        }
        if relay2print.is_lna_on() {
            font.render_text(
                "ON ",
                Point::new(76, 12),
                Rgb565::RED,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        } else {
            font.render_text(
                "OFF",
                Point::new(76, 12),
                Rgb565::BLUE,
                Rgb565::BLACK,
                pixel_data,
                &mut display,
            )
            .await;
        }

        LCD_REDRAW.wait().await; // Wait for signal
        LCD_REDRAW.reset();
    }
}

struct TextRenderer {
    data: &'static [u8],
    char_size: Size,
    char_byte_count: usize,
}

impl TextRenderer {
    pub fn new(data: &'static [u8], char_size: Size) -> Self {
        let char_bit_count = char_size.width as usize * char_size.height as usize;
        //assert!(char_bit_count % 8 == 0);
        Self {
            data,
            char_size,
            char_byte_count: char_bit_count / 8,
        }
    }

    fn unpack(&self, c: char, buf: &mut [u8], fc: &[u8], bc: &[u8]) {
        //assert!(fc.len() == bc.len());
        let color_len = fc.len();
        let idx = c as usize - ' ' as usize;
        let start = idx * self.char_byte_count;
        let mut i = 0;
        for b in &self.data[start..start + self.char_byte_count] {
            let mut code = *b;
            for _ in 0..8 {
                buf[i..i + color_len].copy_from_slice(if code & 1 == 1 { fc } else { bc });
                code >>= 1;
                i += color_len;
            }
        }
    }

    pub async fn render_text(
        &self,
        text: &str,
        top_left: Point,
        fc: Rgb565,
        bc: Rgb565,
        buf: &mut [u8],
        display: &mut impl WritePixels,
    ) {
        let buf_size = self.char_size.width as usize * self.char_size.height as usize * 2;
        let buf = &mut buf[..buf_size];
        for (i, c) in text.chars().enumerate() {
            self.unpack(c, buf, fc.to_be_bytes().as_ref(), bc.to_be_bytes().as_ref());
            display
                .write_pixels(
                    buf,
                    BitDepth::Sixteen,
                    Rectangle::new(
                        top_left + Point::new(i as i32 * self.char_size.width as i32, 0),
                        self.char_size,
                    ),
                )
                .await;
        }
    }
}
