//! Hardware `EXTI` + `TIM2->CNT` (`1 MHz`) pulse/duty capture for `PB7` (`D9 ALTF*`)
//! and `PE15` (`Aux Header FLEX`).

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use ecu_core::aux_io::EdgePulseCapture;
use embassy_stm32::Peri;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt::typelevel::{Binding, EXTI9_5, EXTI15_10};
use embassy_stm32::mode::Async;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{EXTI7, EXTI15, PB7, PE15};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};

type SampleSlot = Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, u32)>>>;

static FLEX_SAMPLE: SampleSlot = Mutex::new(RefCell::new(None));
static ALTF_SAMPLE: SampleSlot = Mutex::new(RefCell::new(None));
static FLEX_ACTIVE_LOW_PULSE: AtomicBool = AtomicBool::new(false);
static ALTF_ACTIVE_LOW_DUTY: AtomicBool = AtomicBool::new(false);

const EDGE_FLOOD_WINDOW_US: u32 = 200;
const EDGE_FLOOD_BURST_LIMIT: u8 = 32;

/// Updates capture polarity flags from `AuxIoConfig` at runtime.
pub fn set_capture_polarity(flex_active_low_pulse: bool, altf_active_low_duty: bool) {
    FLEX_ACTIVE_LOW_PULSE.store(flex_active_low_pulse, Ordering::Relaxed);
    ALTF_ACTIVE_LOW_DUTY.store(altf_active_low_duty, Ordering::Relaxed);
}

/// Drains the latest captured `(period_us, pulse_us)` sample for `PE15` (`FLEX`).
pub fn take_flex_sample() -> Option<(u32, u32)> {
    FLEX_SAMPLE.lock(|cell| cell.borrow_mut().take())
}

/// Drains the latest captured `(period_us, pulse_us)` sample for `PB7` (`ALTF*`).
pub fn take_altf_sample() -> Option<(u32, u32)> {
    ALTF_SAMPLE.lock(|cell| cell.borrow_mut().take())
}

/// Configures `PB7` (`EXTI7` `ALTF*`) with internal pull-up.
pub fn init_altf_input(
    pb7: Peri<'static, PB7>,
    exti7: Peri<'static, EXTI7>,
    irq: impl Binding<EXTI9_5, exti::InterruptHandler<EXTI9_5>> + 'static,
) -> ExtiInput<'static, Async> {
    ExtiInput::new(pb7, exti7, Pull::Up, irq)
}

/// Configures `PE15` (`EXTI15` `FLEX`) with internal pull-up.
pub fn init_flex_input(
    pe15: Peri<'static, PE15>,
    exti15: Peri<'static, EXTI15>,
    irq: impl Binding<EXTI15_10, exti::InterruptHandler<EXTI15_10>> + 'static,
) -> ExtiInput<'static, Async> {
    ExtiInput::new(pe15, exti15, Pull::Up, irq)
}

async fn run_edge_capture_loop(
    mut pin: ExtiInput<'static, Async>,
    slot: &'static SampleSlot,
    active_low_flag: &'static AtomicBool,
) -> ! {
    let mut capture = EdgePulseCapture::new();
    let mut last_us = pac::TIM2.cnt().read();
    let mut rapid_burst: u8 = 0;

    loop {
        pin.wait_for_any_edge().await;
        let now_us = pac::TIM2.cnt().read();
        let is_rising = pin.is_high();

        if let Some(pair) =
            capture.on_edge(is_rising, now_us, active_low_flag.load(Ordering::Relaxed))
        {
            slot.lock(|cell| {
                *cell.borrow_mut() = Some(pair);
            });
        }

        // Rate-limit floating/chattering jumper noise bursts so EXTI never starves Thread mode.
        if now_us.wrapping_sub(last_us) < EDGE_FLOOD_WINDOW_US {
            rapid_burst = rapid_burst.saturating_add(1);
            if rapid_burst >= EDGE_FLOOD_BURST_LIMIT {
                rapid_burst = 0;
                Timer::after(Duration::from_micros(500)).await;
            }
        } else {
            rapid_burst = 0;
        }
        last_us = now_us;
    }
}

/// Async edge-capture task for `PE15` (`Aux Header FLEX`, Continental `50–150 Hz`).
#[embassy_executor::task]
pub async fn exti15_flex_task(pin: ExtiInput<'static, Async>) -> ! {
    run_edge_capture_loop(pin, &FLEX_SAMPLE, &FLEX_ACTIVE_LOW_PULSE).await
}

/// Async edge-capture task for `PB7` (`D9 ALTF*`, Honda alternator field duty).
#[embassy_executor::task]
pub async fn exti7_altf_task(pin: ExtiInput<'static, Async>) -> ! {
    run_edge_capture_loop(pin, &ALTF_SAMPLE, &ALTF_ACTIVE_LOW_DUTY).await
}
