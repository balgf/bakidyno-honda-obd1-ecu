//! Hard real-time hardware timer output drivers (`TIM1`, `TIM3`, `TIM16`, `TIM4`).
//!
//! Hardware allocation (`_DOCS/stm32h743_pinout.md`):
//! - `TIM1` (`1.000 MHz`): Sequential Smart COP `COP1..COP4` on `PE9, PE11, PE13, PE14` (`1-3-4-2`)
//! - `TIM3` (`1.000 MHz`): Sequential Injectors `INJ1..INJ4` (`PC6, PC7, PB0, PB1`, `1..90,000 µs`)
//! - `TIM16` (`1.000 MHz` One-Pulse): `1.200 ms` `TACH_OUT` pulse on `PB8` fired on every spark
//! - `TIM4` (`450 Hz` PWM): `PD14` (`IACV_PWM`) and `PD15` (`O2HTC_PWM`)

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::CriticalSection;
use defmt::warn;
use ecu_core::fuel::InjScheduleBatch;
use ecu_core::ignition::ToothScheduleBatch;
use ecu_types::{Cylinder, DiagStatusReport, OutputEnableMask, OutputTestCommand};
use embassy_stm32::Peri;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::interrupt::typelevel;
use embassy_stm32::pac;
use embassy_stm32::pac::timer::TimGp16;
use embassy_stm32::pac::timer::regs::SrGp16;
use embassy_stm32::pac::timer::vals::{CcmrOutputCcs, OcmGp, OcmLimited, Urs};
use embassy_stm32::peripherals::{
    PB0, PB1, PB8, PC6, PC7, PD14, PD15, PE9, PE11, PE13, PE14, TIM1, TIM3, TIM4, TIM16,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{CountingMode, Timer};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Ch4, CoreInstance};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// `240 MHz` timer kernel clock / `240` = exact `1.000 MHz` (`1 µs` per timer count).
const TIMER_1MHZ_PRESCALER: u16 = 239;
/// Hardware dwell watchdog cutoff (`9.5 ms`) to prevent Smart COP overheating on trigger stalls.
const MAX_COP_DWELL_US: u32 = 9_500;
/// Intermediate chunk for `TIM3` 16-bit timer extension when cold-crank `pw_us > 50 ms`.
const INJ_EXTEND_CHUNK_US: i32 = 30_000;
/// Duration of each `PB8 TACH_OUT` hardware one-pulse (`1.200 ms`).
const TACH_PULSE_WIDTH_US: u16 = 1_200;

#[derive(Clone, Copy, Default)]
struct CopSlot {
    dwell_high: bool,
    dwell_started_tim2_us: u32,
    pending_spark_tim2_us: Option<u32>,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum InjPhase {
    #[default]
    Closed,
    WaitOpen,
    OpenExtending,
    OpenWaitClose,
}

#[derive(Clone, Copy, Default)]
struct InjSlot {
    phase: InjPhase,
    off_target_tim2_us: u32,
}

static COP_SLOTS: Mutex<CriticalSectionRawMutex, RefCell<[CopSlot; 4]>> = Mutex::new(RefCell::new(
    [CopSlot {
        dwell_high: false,
        dwell_started_tim2_us: 0,
        pending_spark_tim2_us: None,
    }; 4],
));

static INJ_SLOTS: Mutex<CriticalSectionRawMutex, RefCell<[InjSlot; 4]>> = Mutex::new(RefCell::new(
    [InjSlot {
        phase: InjPhase::Closed,
        off_target_tim2_us: 0,
    }; 4],
));

#[derive(Clone, Copy)]
struct DiagTestState {
    active_cmd: Option<OutputTestCommand>,
    pulses_remaining: u8,
    cooldown_ms: u16,
    mask: OutputEnableMask,
}

impl DiagTestState {
    const fn new() -> Self {
        Self {
            active_cmd: None,
            pulses_remaining: 0,
            cooldown_ms: 0,
            mask: OutputEnableMask::all_enabled(),
        }
    }
}

static DIAG_STATE: Mutex<CriticalSectionRawMutex, RefCell<DiagTestState>> =
    Mutex::new(RefCell::new(DiagTestState::new()));

static SPARK_FIRE_COUNT: AtomicU32 = AtomicU32::new(0);
static INJ_PULSE_COUNT: AtomicU32 = AtomicU32::new(0);
static TACH_PULSE_COUNT: AtomicU32 = AtomicU32::new(0);
static COP_WD_COUNT: AtomicU32 = AtomicU32::new(0);

#[inline]
fn tim1_gp16() -> TimGp16 {
    // SAFETY: `TIM1` registers match `TimGp16` for CR1, DIER, SR, EGR, CCMR1/2, CCER, CNT, PSC,
    // ARR, and CCR1..4.
    unsafe { TimGp16::from_ptr(TIM1::regs()) }
}

#[inline]
fn tim3_gp16() -> TimGp16 {
    // SAFETY: `TIM3` is a `TimGp16` peripheral instance.
    unsafe { TimGp16::from_ptr(TIM3::regs()) }
}

#[inline]
fn clear_ccif(regs: TimGp16, ch: usize) {
    // `SR` flags are `rc_w0`: writing 1 leaves other channels' pending flags intact.
    regs.sr().write_value(SrGp16(!(1u32 << (1 + ch))));
}

#[inline]
fn set_channel_ocm(regs: TimGp16, ch: usize, ocm: OcmGp) {
    regs.ccmr_output(ch / 2).modify(|w| {
        w.set_ccs(ch % 2, CcmrOutputCcs::OUTPUT);
        w.set_ocpe(ch % 2, /* val= */ false);
        w.set_ocm(ch % 2, ocm);
    });
}

fn force_channel_off(regs: TimGp16, ch: usize) {
    regs.dier().modify(|w| w.set_ccie(ch, /* val= */ false));
    set_channel_ocm(regs, ch, OcmGp::FORCE_INACTIVE);
    clear_ccif(regs, ch);
}

fn arm_channel_at_tim2_us(regs: TimGp16, ch: usize, target_tim2_us: u32, ocm: OcmGp) {
    let now_tim2 = pac::TIM2.cnt().read();
    let now_tim = regs.cnt().read().cnt();
    let delta_us = (target_tim2_us.wrapping_sub(now_tim2) as i32).clamp(4, 60_000) as u16;
    let ccr_val = now_tim.wrapping_add(delta_us);

    regs.dier().modify(|w| w.set_ccie(ch, /* val= */ false));
    // Freeze first so modifying `CCR` never causes a spurious match against the old compare value.
    set_channel_ocm(regs, ch, OcmGp::FROZEN);
    regs.ccr(ch).write(|w| w.set_ccr(ccr_val));
    clear_ccif(regs, ch);
    set_channel_ocm(regs, ch, ocm);
    regs.dier().modify(|w| w.set_ccie(ch, /* val= */ true));
}

#[inline]
fn trigger_tach_one_pulse() {
    if !pac::TIM16.cr1().read().cen() {
        pac::TIM16.cnt().write(|w| w.set_cnt(0));
        pac::TIM16.cr1().modify(|w| w.set_cen(/* val= */ true));
        TACH_PULSE_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

fn configure_1mhz_oc_timer(regs: TimGp16) {
    regs.cr1().modify(|w| {
        w.set_cen(/* val= */ false);
        w.set_arpe(/* val= */ false);
        w.set_urs(Urs::COUNTER_ONLY);
    });
    regs.psc().write_value(TIMER_1MHZ_PRESCALER);
    regs.arr().write(|w| w.set_arr(0xffff));
    for ch in 0..4 {
        set_channel_ocm(regs, ch, OcmGp::FORCE_INACTIVE);
        regs.ccer().modify(|w| {
            w.set_ccp(ch, /* val= */ false);
            w.set_cce(ch, /* val= */ true);
        });
    }
    regs.egr().write(|w| w.set_ug(/* val= */ true));
    regs.sr().write_value(SrGp16(0));
    regs.cr1().modify(|w| w.set_cen(/* val= */ true));
}

fn configure_tim16_tach_opm() {
    let t16 = pac::TIM16;
    t16.cr1().modify(|w| {
        w.set_cen(/* val= */ false);
        w.set_opm(/* val= */ true);
        w.set_arpe(/* val= */ false);
        w.set_urs(Urs::COUNTER_ONLY);
    });
    t16.psc().write_value(TIMER_1MHZ_PRESCALER);
    t16.arr().write(|w| w.set_arr(TACH_PULSE_WIDTH_US + 1));
    t16.ccmr_output(0).write(|w| {
        w.set_ccs(0, CcmrOutputCcs::OUTPUT);
        w.set_ocpe(0, /* val= */ false);
        w.set_ocm(0, OcmLimited::PWM_MODE2);
    });
    // `OC1REF = 0` while `CNT (0) < CCR1 (1)`; HIGH for `CNT = 1..=1201` (`1.200 ms`).
    t16.ccr(0).write(|w| w.set_ccr(1));
    t16.ccer().write(|w| {
        w.set_ccp(0, /* val= */ false);
        w.set_cce(0, /* val= */ true);
    });
    t16.bdtr().modify(|w| w.set_moe(/* val= */ true));
    t16.egr().write(|w| w.set_ug(/* val= */ true));
    t16.sr().write(|_| {});
}

/// Interrupt handler for `TIM1_CC` (`COP1..COP4` dwell & spark edges).
pub struct Tim1CopInterruptHandler;

impl typelevel::Handler<typelevel::TIM1_CC> for Tim1CopInterruptHandler {
    /// # Safety
    ///
    /// Invoked exclusively by the hardware `TIM1_CC` NVIC vector; synchronizes shared state via
    /// `critical_section::with`.
    unsafe fn on_interrupt() {
        let regs = tim1_gp16();
        let sr = regs.sr().read();
        let dier = regs.dier().read();

        critical_section::with(|cs| {
            let mut slots = COP_SLOTS.borrow(cs).borrow_mut();
            let now_tim2 = pac::TIM2.cnt().read();

            for (ch, slot) in slots.iter_mut().enumerate() {
                if !sr.ccif(ch) || !dier.ccie(ch) {
                    continue;
                }
                let ocm = regs.ccmr_output(ch / 2).read().ocm(ch % 2);
                match ocm {
                    OcmGp::ACTIVE_ON_MATCH => {
                        slot.dwell_high = true;
                        slot.dwell_started_tim2_us = now_tim2;
                        if let Some(spark_us) = slot.pending_spark_tim2_us.take() {
                            arm_channel_at_tim2_us(regs, ch, spark_us, OcmGp::INACTIVE_ON_MATCH);
                        } else {
                            regs.dier().modify(|w| w.set_ccie(ch, /* val= */ false));
                            set_channel_ocm(regs, ch, OcmGp::FROZEN);
                            clear_ccif(regs, ch);
                        }
                    }
                    _ => {
                        slot.dwell_high = false;
                        slot.pending_spark_tim2_us = None;
                        force_channel_off(regs, ch);
                        SPARK_FIRE_COUNT.fetch_add(1, Ordering::Relaxed);
                        trigger_tach_one_pulse();
                    }
                }
            }
        });
    }
}

/// Interrupt handler for `TIM3` (`INJ1..INJ4` opening, rollover extension, and closing).
pub struct Tim3InjInterruptHandler;

impl typelevel::Handler<typelevel::TIM3> for Tim3InjInterruptHandler {
    /// # Safety
    ///
    /// Invoked exclusively by the hardware `TIM3` NVIC vector; synchronizes shared state via
    /// `critical_section::with`.
    unsafe fn on_interrupt() {
        let regs = tim3_gp16();
        let sr = regs.sr().read();
        let dier = regs.dier().read();

        critical_section::with(|cs| {
            let mut slots = INJ_SLOTS.borrow(cs).borrow_mut();
            let now_tim2 = pac::TIM2.cnt().read();

            for (ch, slot) in slots.iter_mut().enumerate() {
                if !sr.ccif(ch) || !dier.ccie(ch) {
                    continue;
                }
                match slot.phase {
                    InjPhase::WaitOpen | InjPhase::OpenExtending => {
                        let rem_us = slot.off_target_tim2_us.wrapping_sub(now_tim2) as i32;
                        if rem_us <= 4 {
                            slot.phase = InjPhase::Closed;
                            force_channel_off(regs, ch);
                            INJ_PULSE_COUNT.fetch_add(1, Ordering::Relaxed);
                        } else if rem_us > 50_000 {
                            slot.phase = InjPhase::OpenExtending;
                            arm_channel_at_tim2_us(
                                regs,
                                ch,
                                now_tim2.wrapping_add(INJ_EXTEND_CHUNK_US as u32),
                                OcmGp::FROZEN,
                            );
                        } else {
                            slot.phase = InjPhase::OpenWaitClose;
                            arm_channel_at_tim2_us(
                                regs,
                                ch,
                                slot.off_target_tim2_us,
                                OcmGp::INACTIVE_ON_MATCH,
                            );
                        }
                    }
                    InjPhase::OpenWaitClose | InjPhase::Closed => {
                        slot.phase = InjPhase::Closed;
                        force_channel_off(regs, ch);
                        INJ_PULSE_COUNT.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        });
    }
}

type QuadPwmPins<'d, T> = (
    PwmPin<'d, T, Ch1>,
    PwmPin<'d, T, Ch2>,
    PwmPin<'d, T, Ch3>,
    PwmPin<'d, T, Ch4>,
);

/// Hardware timer outputs owned for the `'static` lifetime of the ECU firmware.
pub struct HardwareOutputs<'d> {
    _tim1: Timer<'d, TIM1>,
    _tim3: Timer<'d, TIM3>,
    _tim16: Timer<'d, TIM16>,
    _cop_pins: QuadPwmPins<'d, TIM1>,
    _inj_pins: QuadPwmPins<'d, TIM3>,
    _tach_pin: PwmPin<'d, TIM16, Ch1>,
    _tim4_pwm: SimplePwm<'d, TIM4>,
}

impl<'d> HardwareOutputs<'d> {
    /// Initializes all `1 MHz` hardware Output Compare timers and `450 Hz` `IACV` PWM in a
    /// guaranteed-OFF state so `PB15` (`ACTUATOR_OE_N`) can safely be armed afterward.
    #[allow(clippy::too_many_arguments)]
    pub fn init(
        tim1: Peri<'d, TIM1>,
        pe9_cop1: Peri<'d, PE9>,
        pe11_cop2: Peri<'d, PE11>,
        pe13_cop3: Peri<'d, PE13>,
        pe14_cop4: Peri<'d, PE14>,
        tim3: Peri<'d, TIM3>,
        pc6_inj1: Peri<'d, PC6>,
        pc7_inj2: Peri<'d, PC7>,
        pb0_inj3: Peri<'d, PB0>,
        pb1_inj4: Peri<'d, PB1>,
        tim16: Peri<'d, TIM16>,
        pb8_tach: Peri<'d, PB8>,
        tim4: Peri<'d, TIM4>,
        pd14_iacv: Peri<'d, PD14>,
        pd15_o2htc: Peri<'d, PD15>,
        _irq_tim1: impl typelevel::Binding<typelevel::TIM1_CC, Tim1CopInterruptHandler> + 'd,
        _irq_tim3: impl typelevel::Binding<typelevel::TIM3, Tim3InjInterruptHandler> + 'd,
    ) -> Self {
        let cop_pins = (
            PwmPin::new(pe9_cop1, OutputType::PushPull),
            PwmPin::new(pe11_cop2, OutputType::PushPull),
            PwmPin::new(pe13_cop3, OutputType::PushPull),
            PwmPin::new(pe14_cop4, OutputType::PushPull),
        );
        let inj_pins = (
            PwmPin::new(pc6_inj1, OutputType::PushPull),
            PwmPin::new(pc7_inj2, OutputType::PushPull),
            PwmPin::new(pb0_inj3, OutputType::PushPull),
            PwmPin::new(pb1_inj4, OutputType::PushPull),
        );
        let tach_pin = PwmPin::new(pb8_tach, OutputType::PushPull);

        let tim1_drv = Timer::new(tim1);
        configure_1mhz_oc_timer(tim1_gp16());
        tim1_drv.enable_outputs();

        let tim3_drv = Timer::new(tim3);
        configure_1mhz_oc_timer(tim3_gp16());

        let tim16_drv = Timer::new(tim16);
        configure_tim16_tach_opm();

        let mut tim4_pwm = SimplePwm::new(
            tim4,
            None,
            None,
            Some(PwmPin::new(pd14_iacv, OutputType::PushPull)),
            Some(PwmPin::new(pd15_o2htc, OutputType::PushPull)),
            Hertz(450),
            CountingMode::EdgeAlignedUp,
        );
        tim4_pwm.ch3().enable();
        tim4_pwm.ch4().enable();

        use embassy_stm32::interrupt::typelevel::Interrupt;
        typelevel::TIM1_CC::unpend();
        typelevel::TIM3::unpend();
        // SAFETY: `Tim1CopInterruptHandler` and `Tim3InjInterruptHandler` are bound via
        // `_irq_tim1`/`_irq_tim3` and only touch `TIM1`/`TIM3` registers & `COP_SLOTS`/`INJ_SLOTS`
        // inside `critical_section`.
        unsafe {
            typelevel::TIM1_CC::enable();
            typelevel::TIM3::enable();
        }

        Self {
            _tim1: tim1_drv,
            _tim3: tim3_drv,
            _tim16: tim16_drv,
            _cop_pins: cop_pins,
            _inj_pins: inj_pins,
            _tach_pin: tach_pin,
            _tim4_pwm: tim4_pwm,
        }
    }

    /// Updates `450 Hz` `PD14` (`IACV_PWM`) duty cycle (`0..=1000` per-mille).
    pub fn set_iacv_duty_per_mille(&mut self, duty_per_mille: u16) {
        let mut ch3: SimplePwmChannel<'_, TIM4> = self._tim4_pwm.ch3();
        ch3.set_duty_cycle_fraction(duty_per_mille.min(1_000) as u32, 1_000);
    }

    /// Programs hardware `TIM1` COP and `TIM3` injector Output Compare actions for one `CKP` tooth.
    pub fn schedule_tooth_outputs(
        ckp_ts_us: u32,
        cop_batch: &ToothScheduleBatch,
        inj_batch: &InjScheduleBatch,
    ) {
        critical_section::with(|cs| {
            let mask = {
                let mut diag = DIAG_STATE.borrow(cs).borrow_mut();
                diag.active_cmd = None;
                diag.pulses_remaining = 0;
                diag.cooldown_ms = 0;
                diag.mask
            };
            Self::schedule_cop_inner(cs, ckp_ts_us, cop_batch, mask);
            Self::schedule_inj_inner(cs, ckp_ts_us, inj_batch, mask);
        });
    }

    /// Updates the volatile RAM ignition & injector enable mask and immediately de-energizes any
    /// newly disabled channels.
    pub fn set_output_enable_mask(mask: OutputEnableMask) {
        critical_section::with(|cs| {
            DIAG_STATE.borrow(cs).borrow_mut().mask = mask;
            let t1 = tim1_gp16();
            let mut cop_slots = COP_SLOTS.borrow(cs).borrow_mut();
            let t3 = tim3_gp16();
            let mut inj_slots = INJ_SLOTS.borrow(cs).borrow_mut();
            let cyls = [
                Cylinder::Cyl1,
                Cylinder::Cyl2,
                Cylinder::Cyl3,
                Cylinder::Cyl4,
            ];
            for (ch, cyl) in cyls.into_iter().enumerate() {
                if !mask.is_cop_enabled(cyl) {
                    cop_slots[ch].dwell_high = false;
                    cop_slots[ch].pending_spark_tim2_us = None;
                    force_channel_off(t1, ch);
                }
                if !mask.is_inj_enabled(cyl) {
                    inj_slots[ch].phase = InjPhase::Closed;
                    force_channel_off(t3, ch);
                }
            }
        });
    }

    /// Starts or stops a Key-On Engine-Off (`RPM == 0`) diagnostic output test sequence.
    pub fn start_output_test(cmd: OutputTestCommand, engine_running: bool) -> bool {
        if matches!(cmd, OutputTestCommand::StopAllTests) {
            critical_section::with(|cs| {
                let mut diag = DIAG_STATE.borrow(cs).borrow_mut();
                diag.active_cmd = None;
                diag.pulses_remaining = 0;
                diag.cooldown_ms = 0;
                if !engine_running {
                    let t1 = tim1_gp16();
                    let mut cop_slots = COP_SLOTS.borrow(cs).borrow_mut();
                    for (ch, slot) in cop_slots.iter_mut().enumerate() {
                        slot.dwell_high = false;
                        slot.pending_spark_tim2_us = None;
                        force_channel_off(t1, ch);
                    }
                    let t3 = tim3_gp16();
                    let mut inj_slots = INJ_SLOTS.borrow(cs).borrow_mut();
                    for (ch, slot) in inj_slots.iter_mut().enumerate() {
                        slot.phase = InjPhase::Closed;
                        force_channel_off(t3, ch);
                    }
                }
            });
            return true;
        }
        if engine_running {
            return false;
        }

        critical_section::with(|cs| {
            let mut diag = DIAG_STATE.borrow(cs).borrow_mut();
            match cmd {
                OutputTestCommand::SparkTest {
                    cylinder,
                    pulse_count,
                    period_ms,
                    dwell_us,
                } => {
                    let count = pulse_count.clamp(1, 20);
                    let period = period_ms.clamp(50, 1_000);
                    let dwell = dwell_us.clamp(500, 6_000);
                    diag.active_cmd = Some(OutputTestCommand::SparkTest {
                        cylinder,
                        pulse_count: count,
                        period_ms: period,
                        dwell_us: dwell,
                    });
                    diag.pulses_remaining = count;
                    diag.cooldown_ms = 0;
                }
                OutputTestCommand::InjectorTest {
                    cylinder,
                    pulse_count,
                    period_ms,
                    pulse_width_us,
                } => {
                    let count = pulse_count.clamp(1, 20);
                    let period = period_ms.clamp(50, 1_000);
                    let pw = pulse_width_us.clamp(500, 25_000);
                    diag.active_cmd = Some(OutputTestCommand::InjectorTest {
                        cylinder,
                        pulse_count: count,
                        period_ms: period,
                        pulse_width_us: pw,
                    });
                    diag.pulses_remaining = count;
                    diag.cooldown_ms = 0;
                }
                OutputTestCommand::TachSweep {
                    pulse_count,
                    period_ms,
                } => {
                    let count = pulse_count.clamp(1, 50);
                    let period = period_ms.clamp(10, 500);
                    diag.active_cmd = Some(OutputTestCommand::TachSweep {
                        pulse_count: count,
                        period_ms: period,
                    });
                    diag.pulses_remaining = count;
                    diag.cooldown_ms = 0;
                }
                OutputTestCommand::FuelPumpPrime { .. } | OutputTestCommand::StopAllTests => {
                    diag.active_cmd = None;
                    diag.pulses_remaining = 0;
                    diag.cooldown_ms = 0;
                }
            }
        });
        true
    }

    /// Advances any active `RPM == 0` diagnostic pulse test once per `1 ms` control tick.
    pub fn tick_1khz_diag(synced: bool, rpm: u16) {
        critical_section::with(|cs| {
            let mut diag = DIAG_STATE.borrow(cs).borrow_mut();
            if synced || rpm > 0 {
                diag.active_cmd = None;
                diag.pulses_remaining = 0;
                diag.cooldown_ms = 0;
                return;
            }
            let Some(cmd) = diag.active_cmd else {
                return;
            };
            if diag.cooldown_ms > 0 {
                diag.cooldown_ms -= 1;
                return;
            }
            if diag.pulses_remaining == 0 {
                let cop_busy = COP_SLOTS
                    .borrow(cs)
                    .borrow()
                    .iter()
                    .any(|s| s.dwell_high || s.pending_spark_tim2_us.is_some());
                let inj_busy = INJ_SLOTS
                    .borrow(cs)
                    .borrow()
                    .iter()
                    .any(|s| s.phase != InjPhase::Closed);
                if !cop_busy && !inj_busy {
                    diag.active_cmd = None;
                }
                return;
            }

            let now_tim2 = pac::TIM2.cnt().read();
            match cmd {
                OutputTestCommand::SparkTest {
                    cylinder,
                    period_ms,
                    dwell_us,
                    ..
                } => {
                    let ch = cylinder.index();
                    let t1 = tim1_gp16();
                    let mut cop_slots = COP_SLOTS.borrow(cs).borrow_mut();
                    let slot = &mut cop_slots[ch];
                    if !slot.dwell_high {
                        let start_us = now_tim2.wrapping_add(10);
                        let spark_us = start_us.wrapping_add(u32::from(dwell_us));
                        slot.pending_spark_tim2_us = Some(spark_us);
                        arm_channel_at_tim2_us(t1, ch, start_us, OcmGp::ACTIVE_ON_MATCH);
                    }
                    diag.pulses_remaining -= 1;
                    diag.cooldown_ms = period_ms;
                }
                OutputTestCommand::InjectorTest {
                    cylinder,
                    period_ms,
                    pulse_width_us,
                    ..
                } => {
                    let ch = cylinder.index();
                    let t3 = tim3_gp16();
                    let mut inj_slots = INJ_SLOTS.borrow(cs).borrow_mut();
                    let slot = &mut inj_slots[ch];
                    if slot.phase == InjPhase::Closed {
                        let on_us = now_tim2.wrapping_add(10);
                        slot.off_target_tim2_us = on_us.wrapping_add(u32::from(pulse_width_us));
                        slot.phase = InjPhase::WaitOpen;
                        arm_channel_at_tim2_us(t3, ch, on_us, OcmGp::ACTIVE_ON_MATCH);
                    }
                    diag.pulses_remaining -= 1;
                    diag.cooldown_ms = period_ms;
                }
                OutputTestCommand::TachSweep { period_ms, .. } => {
                    trigger_tach_one_pulse();
                    diag.pulses_remaining -= 1;
                    diag.cooldown_ms = period_ms;
                }
                OutputTestCommand::FuelPumpPrime { .. } | OutputTestCommand::StopAllTests => {
                    diag.active_cmd = None;
                    diag.pulses_remaining = 0;
                }
            }
        });
    }

    /// Returns a snapshot of the current output enable mask, active test progress, and timer counts.
    pub fn diag_status(fuel_pump_override_ms: u16) -> DiagStatusReport {
        let (enable_mask, test_active, pulses_remaining) = critical_section::with(|cs| {
            let diag = DIAG_STATE.borrow(cs).borrow();
            (
                diag.mask,
                diag.active_cmd.is_some() || fuel_pump_override_ms > 0,
                diag.pulses_remaining,
            )
        });
        let (spark_fire_count, inj_pulse_count, tach_pulse_count, cop_watchdog_count) =
            Self::telemetry_counts();
        DiagStatusReport {
            enable_mask,
            test_active,
            pulses_remaining,
            fuel_pump_override_ms,
            spark_fire_count,
            inj_pulse_count,
            tach_pulse_count,
            cop_watchdog_count,
        }
    }

    /// Enforces coil dwell safety watchdog and forces outputs off when `Unsynced` / stalled.
    pub fn enforce_safety(synced: bool) {
        critical_section::with(|cs| {
            // Read `TIM2->CNT` *inside* the critical section so a pre-empting `TIM1_CC` ISR cannot
            // update `dwell_started_tim2_us` to a timestamp slightly newer than `now_tim2_us`.
            let now_tim2_us = pac::TIM2.cnt().read();
            let (test_active, mask) = {
                let diag = DIAG_STATE.borrow(cs).borrow();
                (diag.active_cmd.is_some(), diag.mask)
            };
            let cyls = [
                Cylinder::Cyl1,
                Cylinder::Cyl2,
                Cylinder::Cyl3,
                Cylinder::Cyl4,
            ];
            let t1 = tim1_gp16();
            let mut cop_slots = COP_SLOTS.borrow(cs).borrow_mut();
            for (ch, slot) in cop_slots.iter_mut().enumerate() {
                let elapsed_us = now_tim2_us.wrapping_sub(slot.dwell_started_tim2_us) as i32;
                let over_dwell = slot.dwell_high && elapsed_us > (MAX_COP_DWELL_US as i32);
                if over_dwell {
                    COP_WD_COUNT.fetch_add(1, Ordering::Relaxed);
                    warn!("COP WD ch={} elapsed={}us", ch, elapsed_us);
                }
                let cut_by_mask = synced && !mask.is_cop_enabled(cyls[ch]);
                if (!synced && !test_active) || over_dwell || cut_by_mask {
                    slot.dwell_high = false;
                    slot.pending_spark_tim2_us = None;
                    force_channel_off(t1, ch);
                }
            }

            let t3 = tim3_gp16();
            let mut inj_slots = INJ_SLOTS.borrow(cs).borrow_mut();
            for (ch, slot) in inj_slots.iter_mut().enumerate() {
                let cut_by_mask = synced && !mask.is_inj_enabled(cyls[ch]);
                if (!synced && !test_active) || cut_by_mask {
                    slot.phase = InjPhase::Closed;
                    force_channel_off(t3, ch);
                }
            }
        });
    }

    /// Reads accumulated `(spark_events, inj_events, tach_pulses, cop_watchdogs)`.
    pub fn telemetry_counts() -> (u32, u32, u32, u32) {
        (
            SPARK_FIRE_COUNT.load(Ordering::Relaxed),
            INJ_PULSE_COUNT.load(Ordering::Relaxed),
            TACH_PULSE_COUNT.load(Ordering::Relaxed),
            COP_WD_COUNT.load(Ordering::Relaxed),
        )
    }

    fn schedule_cop_inner(
        cs: CriticalSection<'_>,
        ckp_ts_us: u32,
        batch: &ToothScheduleBatch,
        mask: OutputEnableMask,
    ) {
        let regs = tim1_gp16();
        let mut slots = COP_SLOTS.borrow(cs).borrow_mut();

        for act in batch.actions.iter().flatten() {
            if !mask.is_cop_enabled(act.cylinder) {
                continue;
            }
            let ch = act.cylinder.index();
            let slot = &mut slots[ch];
            let target_us = ckp_ts_us.wrapping_add(act.delay_from_tooth_us as u32);

            if act.drive_dwell_high {
                if !slot.dwell_high {
                    slot.pending_spark_tim2_us = None;
                    arm_channel_at_tim2_us(regs, ch, target_us, OcmGp::ACTIVE_ON_MATCH);
                }
            } else if slot.dwell_high {
                slot.pending_spark_tim2_us = None;
                arm_channel_at_tim2_us(regs, ch, target_us, OcmGp::INACTIVE_ON_MATCH);
            } else {
                let ocm = regs.ccmr_output(ch / 2).read().ocm(ch % 2);
                if ocm == OcmGp::ACTIVE_ON_MATCH {
                    slot.pending_spark_tim2_us = Some(target_us);
                }
            }
        }
    }

    fn schedule_inj_inner(
        cs: CriticalSection<'_>,
        ckp_ts_us: u32,
        batch: &InjScheduleBatch,
        mask: OutputEnableMask,
    ) {
        let regs = tim3_gp16();
        let mut slots = INJ_SLOTS.borrow(cs).borrow_mut();

        for act in batch.actions.iter().flatten() {
            if act.pulse_width_us == 0 || !mask.is_inj_enabled(act.cylinder) {
                continue;
            }
            let ch = act.cylinder.index();
            let slot = &mut slots[ch];
            if slot.phase != InjPhase::Closed {
                continue;
            }
            let on_us = ckp_ts_us.wrapping_add(act.delay_from_tooth_us as u32);
            slot.off_target_tim2_us = on_us.wrapping_add(act.pulse_width_us);
            slot.phase = InjPhase::WaitOpen;
            arm_channel_at_tim2_us(regs, ch, on_us, OcmGp::ACTIVE_ON_MATCH);
        }
    }
}
