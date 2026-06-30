// протоеол обмена с исполнительным модулем певоротки
use super::*;

use embassy_stm32::usart::BufferedUart;
use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};

use crc::{CRC_8_LTE, Crc};
use heapless::Vec;

use crate::e220::*;
use crate::errors::*;
use crate::pelcod::*;

//const MASTER_ADDRESS: u8 = 1;
const CMD_PACKET_SIZE: usize = 13;

pub struct Protocol {
    seq: u32, // последовательный номер пакета для контроля эфира
}

impl Protocol {
    pub fn new() -> Self {
        Protocol { seq: 0 }
    }

    pub fn set_seq(&mut self, seq: u32) {
        self.seq = seq;
    }

    pub fn get_seq(&self) -> u32 {
        self.seq
    }

    // расчёт контрольной суммы
    pub fn crc8(&mut self, block: &[u8]) -> u8 {
        let crc_provider = Crc::<u8>::new(&CRC_8_LTE);
        let mut digest = crc_provider.digest();
        digest.update(block);
        digest.finalize()
    }

    // Пакет ответа по команде от контроллера
    // 1 - байт 0xFF
    // 2,3,4,5 - последовательный номер как в пришедшей команде
    // 6,7 - u16 AngleAz - фактические значения
    // 8,9 - u16 AngleEl
    // 10 - signal rssi
    // 11 - noise rssi
    // 12 - состояние реле
    // 13 - CRC
    pub fn make_answer(
        &mut self,
        az: &AzAngle,
        el: &ElAngle,
        s_rssi: u8,
        n_rssi: u8,
        rly: &Relays,
    ) -> Result<Vec<u8, CMD_PACKET_SIZE>, Error> {
        let mut payload: Vec<u8, CMD_PACKET_SIZE> = Vec::new();
        let seq_bytes_be: [u8; 4] = self.seq.to_be_bytes();

        let azu: u16 = az.into();
        let az_bytes_be: [u8; 2] = azu.to_be_bytes();

        let elu: u16 = el.into();
        let el_bytes_be: [u8; 2] = elu.to_be_bytes();

        // формируем пакет
        payload.push(0xFF).unwrap();
        payload.extend_from_slice(&seq_bytes_be).unwrap();
        payload.extend_from_slice(&az_bytes_be).unwrap();
        payload.extend_from_slice(&el_bytes_be).unwrap();
        payload.push(s_rssi).unwrap();
        payload.push(n_rssi).unwrap();
        payload.push(rly.into()).unwrap();

        // считаем его CRC
        let crc8 = self.crc8(&payload);
        // добавляем CRC в коне пакета
        payload.push(crc8).unwrap();
        Ok(payload)
    }
}

// таск обработки команд e220 приёма и передачи в эфир
#[embassy_executor::task]
pub async fn process_e220(
    mut usart: BufferedUart<'static>,
    m0: Output<'static>,
    m1: Output<'static>,
    aux: Input<'static>,
    mut relay_ptz: Output<'static>,
    mut relay_lna: Output<'static>,
) {
    let mut e220 = E220Module::new(usart, m0, m1, aux);

    // инициализация радио модуля E220
    let mut init_count = 1;
    // после старта он должен поднять уровень AUX
    info!("Init e220...");
    e220.aux_wait().await;
    loop {
        info!("Init attempt {}", init_count);
        let r = e220.module_init(MODULE_ADDRESS_H, MODULE_ADDRESS_L).await;
        match r {
            Err(_) => {
                info!("Error init");
                init_count += 1;
                if init_count > 10 {
                    loop {}
                }
                continue;
            }
            Ok(_) => {
                info!("Init e220 OK");
                break;
            }
        }
    }
    e220.set_mode(E220Mode::default()).await;

    let mut protocol = Protocol::new();

    // в ответе текущие значения углов!
    let mut az_cmd = AzAngle(0);
    let mut el_cmd = ElAngle(0);

    loop {
        //Timer::after_millis(3000).await;
        /*protocol.next_seq();
        {
            let ctx = APP_CONTEXT.lock().await;
            let mut inner = ctx.borrow_mut();
            inner.seq = protocol.get_seq();
        }*/

        // слушаем эфир
        info!("Waiting for packet...");
        let res = e220.get_packet().await;
        match res {
            Err(_) => {
                info!("Command rcv failed");
                Timer::after_millis(1000).await;
                continue;
            }
            Ok(_) => {
                info!("Got packet {:?}", e220.rcv_buf);
            }
        }

        // валидация и парсинг пакета - группа операций
        if e220.rcv_buf.len() != 12 {
            info!("ERR packet len");
            continue;
        }
        if e220.rcv_buf[0] != 0xFF {
            info!("ERR 0xFF");
            continue;
        }
        // считаем и проверяем его CRC
        let crc8 = protocol.crc8(&e220.rcv_buf[0..10]);
        if e220.rcv_buf[10] != crc8 {
            info!("ERR CRC");
            continue;
        }
        // проверяем, что номер пакета всегда растет
        let seq_bytes = &e220.rcv_buf[1..=4];
        let new_seq = u32::from_be_bytes(seq_bytes.try_into().unwrap());
        if protocol.get_seq() > new_seq || (protocol.get_seq() + 100 < new_seq && protocol.get_seq() != 0) {
            info!("ERR Seq");
            continue;
        } else {
            protocol.set_seq(new_seq);
        }
        //info!("Seq {}", protocol.get_seq());
        let az_bytes = &e220.rcv_buf[5..=6];
        let new_azu = u16::from_be_bytes(az_bytes.try_into().unwrap());
        let new_az = if let Ok(a) = AzAngle::try_from(new_azu) {
            a
        } else {
            info!("ERR Az");
            continue;
        };

        let el_bytes = &e220.rcv_buf[7..=8];
        let new_elu = u16::from_be_bytes(el_bytes.try_into().unwrap());
        let new_el = if let Ok(a) = ElAngle::try_from(new_elu) {
            a
        } else {
            info!("ERR El");
            continue;
        };

        let mut rly = if let Ok(a) = Relays::try_from(e220.rcv_buf[9]) {
            a
        } else {
            info!("ERR Rly");
            continue;
        };

        let rssi = e220.rcv_buf[11];

        // получаем уровень местного шума
        let rssi_n = if let Ok(r) = e220.try_get_noise_dbm(3).await {
            r
        } else {
            info!("Err read noise dbm");
            continue;
        };

        // обновляем включение реле
        if rly.is_ptz_on() {
            relay_ptz.set_high();
        } else {
            relay_ptz.set_low();
        }
        if rly.is_lna_on() {
            relay_lna.set_high();
        } else {
            relay_lna.set_low();
        }

        // обновляем значения углов
        {
            let ctx = APP_CONTEXT.lock().await;
            let mut inner = ctx.borrow_mut();
            az_cmd = inner.current_az;
            el_cmd = inner.current_el;
            inner.target_az = new_az;
            inner.target_el = new_el;
            inner.signal = rssi;
            inner.noise = rssi_n;
            inner.seq = protocol.get_seq();
            inner.power_control = rly;
        }
        LCD_REDRAW.signal(());

        //Timer::after_millis(1000).await;

        // здесь послать ответ с текущими координатами
        // формируем команду для поворотки
        let cmd = protocol
            .make_answer(&az_cmd, &el_cmd, rssi, rssi_n, &rly)
            .unwrap();
        // отправляем в эфир
        let res = e220
            .send_packet(MASTER_ADDRESS_H, MASTER_ADDRESS_L, &cmd)
            .await;
        match res {
            Err(_) => {
                info!("Answer send failed");
            }
            Ok(_) => {
                info!("Answer sent successfully");
            }
        }
    }
}
