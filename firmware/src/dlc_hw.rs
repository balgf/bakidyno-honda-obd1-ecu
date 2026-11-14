//! `USART1` (`PA9` `DLC_TX` / `PA10` `DLC_RX`, `AF7`) `9600 bps 8N1` Honda `D7 DLC` driver.
//!
//! Configures `PA9` with hardware TX inversion (`invert_tx: true`) to match the external
//! `BSS138` low-side open-drain pull-down stage (MCU TX idle `0 V` -> `BSS138` OFF -> `D7` held
//! at `+5 V` idle) and enables an internal pull-up on `PA10` (`DLC_RX`) so an unconnected wire
//! never generates phantom framing errors.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::warn;
use ecu_core::dlc::{DLC_BAUDRATE_BPS, DlcParser, encode_dlc_response};
use embassy_stm32::Peri;
use embassy_stm32::gpio::Pull;
use embassy_stm32::peripherals::{PA9, PA10, USART1};
use embassy_stm32::usart::{BufferedUart, Config as UsartConfig};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::{ECU_STATE, Irqs};

static DLC_TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static DLC_RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static DLC_FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

/// Returns number of valid Honda `D7 DLC` request frames answered since boot.
pub fn answered_frame_count() -> u32 {
    DLC_FRAME_COUNT.load(Ordering::Relaxed)
}

/// Initializes `USART1` (`PA9` TX / `PA10` RX) at `9600 bps 8N1` for Honda `D7 DLC`.
pub fn init_dlc_uart(
    usart1: Peri<'static, USART1>,
    pa10_rx: Peri<'static, PA10>,
    pa9_tx: Peri<'static, PA9>,
) -> Option<BufferedUart<'static>> {
    let mut cfg = UsartConfig::default();
    cfg.baudrate = DLC_BAUDRATE_BPS;
    cfg.rx_pull = Pull::Up;
    cfg.invert_tx = true;
    cfg.eager_reads = Some(1);

    let tx_buf = DLC_TX_BUF.init([0u8; 64]);
    let rx_buf = DLC_RX_BUF.init([0u8; 64]);
    match BufferedUart::new(usart1, pa10_rx, pa9_tx, tx_buf, rx_buf, Irqs, cfg) {
        Ok(uart) => Some(uart),
        Err(_) => {
            warn!("USART1 D7 DLC init failed");
            None
        }
    }
}

/// Async Honda `D7 DLC` `9600 bps` diagnostic responder loop.
#[embassy_executor::task]
pub async fn dlc_uart_task(mut uart: BufferedUart<'static>) -> ! {
    let mut parser = DlcParser::new();
    let mut byte = [0u8; 1];
    let mut echo_skip_buf: [u8; 40] = [0; 40];
    let mut echo_expected_len: usize = 0;
    let mut echo_matched: usize = 0;

    loop {
        if uart.read(&mut byte).await.is_err() {
            parser.reset();
            echo_expected_len = 0;
            echo_matched = 0;
            continue;
        }

        // Discard 1-wire self-echo bytes immediately following our own transmission while passing
        // through any mismatched byte (normal 2-wire UART or host contention) straight to parser.
        if echo_matched < echo_expected_len {
            if byte[0] == echo_skip_buf[echo_matched] {
                echo_matched += 1;
                if echo_matched >= echo_expected_len {
                    echo_expected_len = 0;
                    echo_matched = 0;
                }
                continue;
            }
            echo_expected_len = 0;
            echo_matched = 0;
        }

        if let Some(cmd) = parser.push_byte(byte[0]) {
            let (snap, aux) = {
                let state = ECU_STATE.lock().await;
                (state.ecu.snapshot(), state.aux_telemetry)
            };
            let resp = encode_dlc_response(cmd, &snap, &aux);
            let slice = resp.as_slice();
            if uart.write_all(slice).await.is_ok() {
                let _ = uart.flush().await;
                let n = slice.len().min(echo_skip_buf.len());
                echo_skip_buf[..n].copy_from_slice(&slice[..n]);
                echo_expected_len = n;
                echo_matched = 0;
                DLC_FRAME_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}
