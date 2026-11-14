//! `FDCAN1` (`PD0` RX / `PD1` TX, `AF9`, `120 MHz` `PLL1_Q`) hardware driver with
//! startup silicon internal-loopback self-test and non-blocking `20 Hz` telemetry broadcast.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use defmt::{info, warn};
use ecu_core::can_bus::{
    CAN_ID_SELF_TEST, CAN_NOMINAL_BITRATE_BPS, CAN_SELF_TEST_PAYLOAD, encode_telemetry_frames,
};
use embassy_stm32::Peri;
use embassy_stm32::can::filter::{StandardFilter, StandardFilterSlot};
use embassy_stm32::can::{Can, CanConfigurator, CanRx, CanTx, Frame, Properties};
use embassy_stm32::peripherals::{FDCAN1, PD0, PD1};
use embassy_time::{Duration, Ticker, with_timeout};

use crate::{ECU_STATE, Irqs};

static CAN_SELF_TEST_PASSED: AtomicBool = AtomicBool::new(false);
static CAN_TX_OK_COUNT: AtomicU32 = AtomicU32::new(0);
static CAN_TX_DROP_COUNT: AtomicU32 = AtomicU32::new(0);
static CAN_RX_COUNT: AtomicU32 = AtomicU32::new(0);
static CAN_LAST_TEC: AtomicU8 = AtomicU8::new(0);

/// Summary status snapshot reported in `1 Hz` RTT diagnostics.
#[derive(Debug, Clone, Copy)]
pub struct CanHwStats {
    pub self_test_passed: bool,
    pub tx_ok: u32,
    pub tx_drop: u32,
    pub rx_count: u32,
    pub tec: u8,
}

/// Reads current `FDCAN1` counters without locking async state.
pub fn stats() -> CanHwStats {
    CanHwStats {
        self_test_passed: CAN_SELF_TEST_PASSED.load(Ordering::Relaxed),
        tx_ok: CAN_TX_OK_COUNT.load(Ordering::Relaxed),
        tx_drop: CAN_TX_DROP_COUNT.load(Ordering::Relaxed),
        rx_count: CAN_RX_COUNT.load(Ordering::Relaxed),
        tec: CAN_LAST_TEC.load(Ordering::Relaxed),
    }
}

/// Runs an internal-loopback frame roundtrip across the `FDCAN1` silicon core (no external
/// transceiver needed), then configures `FDCAN1` in normal `500 kbps` mode and splits TX/RX.
pub async fn init_and_selftest(
    mut fdcan1: Peri<'static, FDCAN1>,
    mut pd0_rx: Peri<'static, PD0>,
    mut pd1_tx: Peri<'static, PD1>,
) -> (CanTx<'static>, CanRx<'static>, Properties) {
    let passed =
        run_internal_loopback(fdcan1.reborrow(), pd0_rx.reborrow(), pd1_tx.reborrow()).await;
    CAN_SELF_TEST_PASSED.store(passed, Ordering::Relaxed);
    if passed {
        info!(
            "FDCAN1 internal loopback self-test PASS ({} bps)",
            CAN_NOMINAL_BITRATE_BPS
        );
    } else {
        warn!("FDCAN1 internal loopback self-test FAILED");
    }

    let mut cfg = CanConfigurator::new(fdcan1, pd0_rx, pd1_tx, Irqs);
    cfg.set_bitrate(CAN_NOMINAL_BITRATE_BPS);
    let can = cfg.into_normal_mode();
    can.properties().set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );
    can.split()
}

async fn run_internal_loopback(
    fdcan1: Peri<'_, FDCAN1>,
    pd0_rx: Peri<'_, PD0>,
    pd1_tx: Peri<'_, PD1>,
) -> bool {
    let mut cfg = CanConfigurator::new(fdcan1, pd0_rx, pd1_tx, Irqs);
    cfg.set_bitrate(CAN_NOMINAL_BITRATE_BPS);
    let mut can = cfg.into_internal_loopback_mode();
    can.properties().set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );

    let Ok(tx_frame) = Frame::new_standard(CAN_ID_SELF_TEST, &CAN_SELF_TEST_PAYLOAD) else {
        return false;
    };

    if with_timeout(Duration::from_millis(20), can.write(&tx_frame))
        .await
        .is_err()
    {
        return false;
    }

    match with_timeout(Duration::from_millis(50), can.read()).await {
        Ok(Ok(envelope)) => envelope.frame.data() == CAN_SELF_TEST_PAYLOAD,
        _ => false,
    }
}

/// Broadcasts canonical `0x640..=0x642` classic CAN frames at `20 Hz` (`50 ms` period).
///
/// Uses bounded TX backpressure timeouts so an open/unterminated CAN header without a peer ACK
/// never stalls the ECU scheduler and automatically resumes streaming once a node connects.
#[embassy_executor::task]
pub async fn can_tx_task(mut tx: CanTx<'static>, props: Properties) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        ticker.next().await;

        let (snap, aux) = {
            let state = ECU_STATE.lock().await;
            (state.ecu.snapshot(), state.aux_telemetry)
        };

        let frames = encode_telemetry_frames(&snap, &aux);
        for pkt in frames {
            let Ok(frame) = Frame::new_standard(pkt.id, &pkt.data) else {
                continue;
            };
            if with_timeout(Duration::from_millis(5), tx.write(&frame))
                .await
                .is_ok()
            {
                CAN_TX_OK_COUNT.fetch_add(1, Ordering::Relaxed);
            } else {
                CAN_TX_DROP_COUNT.fetch_add(1, Ordering::Relaxed);
            }
        }
        CAN_LAST_TEC.store(props.tx_error_count(), Ordering::Relaxed);
    }
}

/// Drains incoming `FDCAN1` RX frames and increments the live RX counter.
#[embassy_executor::task]
pub async fn can_rx_task(mut rx: CanRx<'static>) -> ! {
    loop {
        match rx.read().await {
            Ok(_env) => {
                CAN_RX_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            Err(_bus_err) => {
                embassy_time::Timer::after(Duration::from_millis(20)).await;
            }
        }
    }
}

#[allow(dead_code)]
fn _keep_can_type_ref(_: Can<'static>) {}
