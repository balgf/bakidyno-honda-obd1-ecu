//! Interrupt-domain `TIM2` (`1.000 MHz`) trigger decoder & Compare Scheduler.
//!
//! Runs directly inside `TIM2` ISR (`Priority::P1`), preempting Thread Mode (`SDMMC`, `USB`,
//! `1 kHz` control ticks, `knock_dsp_task`, `CAN`, `UART`) so a slow SD card write or `ECU_STATE`
//! lock can never overwrite `TIM2->CCR1` (`CC1OF`), drop a `CKP` tooth, or delay spark/fuel arming.

use core::cell::RefCell;

use ecu_core::{RtControlTargets, RtTriggerEngine, RtTriggerStatus};
use embassy_stm32::Peri;
use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::peripherals::{PA1, PA15, PB10, PB11, TIM2};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::input_capture::CapturePin;
use embassy_stm32::timer::low_level::{CountingMode, InputCaptureMode, InputTISelection, Timer};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4, Channel};
use embassy_stm32::{pac, rcc};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use crate::knock_hw;
use crate::outputs::HardwareOutputs;

struct RtDomain {
    engine: RtTriggerEngine,
    vss_kmh: f32,
    last_vss_ts_us: Option<u32>,
}

impl RtDomain {
    const fn new() -> Self {
        Self {
            engine: RtTriggerEngine::new(),
            vss_kmh: 0.0,
            last_vss_ts_us: None,
        }
    }
}

static RT_DOMAIN: BlockingMutex<CriticalSectionRawMutex, RefCell<RtDomain>> =
    BlockingMutex::new(RefCell::new(RtDomain::new()));

/// Owns the initialized `TIM2` peripheral and its 4 alternate-function capture pins for the
/// lifetime of `main()` so [`Timer::drop`] never gates `TIM2` RCC clocks off.
pub struct Tim2TriggerHw {
    tim2: Timer<'static, TIM2>,
    _ckp_pin: CapturePin<'static, TIM2, Ch1>,
    _tdc_pin: CapturePin<'static, TIM2, Ch2>,
    _cyp_pin: CapturePin<'static, TIM2, Ch3>,
    _vss_pin: CapturePin<'static, TIM2, Ch4>,
}

impl Tim2TriggerHw {
    /// Initializes free-running 32-bit `TIM2` at `1.000 MHz` (`1 µs` resolution) with hardware
    /// rising-edge latching on `CH1..CH4` while leaving `CC1IE` (`CKP`) masked until [`arm_irq`].
    pub fn init(
        tim2_peri: Peri<'static, TIM2>,
        pa15_ckp: Peri<'static, PA15>,
        pa1_tdc: Peri<'static, PA1>,
        pb10_cyp: Peri<'static, PB10>,
        pb11_vss: Peri<'static, PB11>,
    ) -> Self {
        // Explicit RCC enable ensures `TIM2` clocks before pin/register config even across warm reset.
        rcc::enable_and_reset::<TIM2>();

        let ckp_pin = CapturePin::new(pa15_ckp, Pull::Down);
        let tdc_pin = CapturePin::new(pa1_tdc, Pull::Down);
        let cyp_pin = CapturePin::new(pb10_cyp, Pull::Down);
        let vss_pin = CapturePin::new(pb11_vss, Pull::Down);

        let mut tim2 = Timer::new(tim2_peri);
        tim2.set_counting_mode(CountingMode::EdgeAlignedUp);
        tim2.set_tick_freq(Hertz(1_000_000));
        tim2.set_max_compare_value(u32::MAX);

        for ch in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            tim2.enable_input_interrupt(ch, /* enable= */ false);
            tim2.set_input_ti_selection(ch, InputTISelection::Normal);
            tim2.set_input_capture_mode(ch, InputCaptureMode::Rising);
            tim2.enable_channel(ch, /* enable= */ true);
        }

        pac::TIM2.sr().write(|_| {});
        tim2.start();

        Self {
            tim2,
            _ckp_pin: ckp_pin,
            _tdc_pin: tdc_pin,
            _cyp_pin: cyp_pin,
            _vss_pin: vss_pin,
        }
    }

    /// Unmasks `TIM2` (`Priority::P1`) `CH1` (`CKP`) interrupts once initial calibration is loaded.
    pub fn arm_irq(&self) {
        pac::TIM2.sr().write(|_| {});
        interrupt::typelevel::TIM2::set_priority(interrupt::Priority::P1);
        interrupt::typelevel::TIM2::unpend();
        // SAFETY: `Tim2TriggerInterruptHandler` is bound to `TIM2` in `Irqs` and synchronizes
        // shared trigger state exclusively via `critical_section::with`.
        unsafe {
            interrupt::typelevel::TIM2::enable();
        }
        self.tim2
            .enable_input_interrupt(Channel::Ch1, /* enable= */ true);
    }
}

/// Custom `TIM2` interrupt handler bound via `bind_interrupts!`.
pub struct Tim2TriggerInterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::TIM2> for Tim2TriggerInterruptHandler {
    /// # Safety
    ///
    /// Invoked exclusively by the hardware `TIM2` NVIC vector (`Priority::P1`); synchronizes
    /// `RT_DOMAIN` and output compare registers via `critical_section::with`.
    unsafe fn on_interrupt() {
        let regs = pac::TIM2;
        let sr = regs.sr().read();

        // Read `SR` before `CCR1..4` because reading `CCRx` clears `CCxIF` in hardware.
        let ckp_ts_us = sr.ccif(0).then(|| regs.ccr(0).read());
        let tdc_ts_us = sr.ccif(1).then(|| regs.ccr(1).read());
        let cyp_ts_us = sr.ccif(2).then(|| regs.ccr(2).read());
        let vss_ts_us = sr.ccif(3).then(|| regs.ccr(3).read());

        regs.sr().modify(|w| {
            for ch in 0..4 {
                w.set_ccof(ch, /* val= */ false);
            }
        });

        let (outputs, knock_plan, synced) = critical_section::with(|cs| {
            let mut dom = RT_DOMAIN.borrow(cs).borrow_mut();

            if let Some(vss_ts) = vss_ts_us {
                if let Some(prev_vss) = dom.last_vss_ts_us {
                    let dt_vss = vss_ts.wrapping_sub(prev_vss);
                    if dt_vss > 1_000 {
                        dom.vss_kmh = 1_448_000.0 / (dt_vss as f32);
                    }
                }
                dom.last_vss_ts_us = Some(vss_ts);
            }

            if let Some(cyp_ts) = cyp_ts_us {
                dom.engine.on_cyp_capture(cyp_ts);
            }
            if let Some(tdc_ts) = tdc_ts_us {
                dom.engine.on_tdc_capture(tdc_ts);
            }

            let (outputs, knock_plan) = if let Some(ckp_ts) = ckp_ts_us {
                dom.engine.on_ckp_capture(ckp_ts)
            } else {
                (Default::default(), None)
            };

            (outputs, knock_plan, dom.engine.is_full_synced())
        });

        // Program `TIM1`/`TIM3`/`TIM7` outside `RT_DOMAIN` lock so higher-priority `TIM1_CC`/`TIM3`
        // (`P0`) compare ISRs are never masked during unrelated tooth work.
        if let Some(ckp_ts) = ckp_ts_us {
            if outputs.event.is_some() {
                HardwareOutputs::schedule_tooth_outputs(ckp_ts, &outputs.cop, &outputs.inj);
            }
            if let Some(plan) = knock_plan {
                knock_hw::schedule_window(plan);
            }
        }
        HardwareOutputs::enforce_safety(synced);
    }
}

/// Runs the `400 ms` stall timeout + `1 s` VSS timeout and returns an atomic snapshot for `1 kHz`.
pub fn sync_1khz(now_us: u32) -> (RtTriggerStatus, f32) {
    critical_section::with(|cs| {
        let mut dom = RT_DOMAIN.borrow(cs).borrow_mut();
        if let Some(last_vss) = dom.last_vss_ts_us
            && now_us.wrapping_sub(last_vss) > 1_000_000
        {
            dom.vss_kmh = 0.0;
        }
        let status = dom.engine.sync_1khz(now_us);
        (status, dom.vss_kmh)
    })
}

/// Pushes fresh `1 kHz` control targets (`spark_advance`, `dwell`, `inj_pw`, `knock_cfg`) into ISR.
pub fn update_targets(targets: RtControlTargets) {
    critical_section::with(|cs| {
        RT_DOMAIN
            .borrow(cs)
            .borrow_mut()
            .engine
            .update_targets(targets);
    });
}
