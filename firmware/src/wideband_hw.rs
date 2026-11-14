//! Bare-metal STM32H743 hardware driver for the onboard Bosch LSU 4.9 wideband AFE.
//!
//! Hardware allocation:
//! - `PA4` (`DAC1_OUT1`): `LSU_IP_DAC` pump current command (`0..3.3 V`, `1.65 V` = `0 mA`).
//! - `PA5` (`DAC1_OUT2`): `LSU_VM_DAC` virtual ground (`1.650 V` / code `2048`).
//! - `PA6` (`ADC2_INP3`): `LSU_UN_SENSE` Nernst voltage + `Ri` AC square-wave response.
//! - `PA7` (`ADC2_INP7`): `LSU_IP_SENSE` pump current shunt amplifier output.
//! - `PE5` (`TIM15_CH1`, AF4): `LSU_NERNST_AC` `4.000 kHz` square wave (`125 us` half-period).
//! - `PB9` (`TIM17_CH1`, AF1): `LSU_HEATER_PWM` `100 Hz` heater MOSFET gate drive.

use core::sync::atomic::{AtomicU8, AtomicU16, Ordering};
use ecu_types::RawSensorAdc;
use embassy_stm32::adc::{AdcChannel, AnyAdcChannel};
use embassy_stm32::dac::{Dac, DualValue};
use embassy_stm32::gpio::OutputType;
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::pac::adc::vals::{Adcaldif, Boost, Difsel, Exten, Pcsel, Res};
use embassy_stm32::pac::gpio::vals::Idr;
use embassy_stm32::pac::timer::regs::{Dier2chCmp, Sr2chCmp};
use embassy_stm32::pac::timer::vals::{CcmrOutputCcs, Ocm};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Ch1;
use embassy_stm32::timer::low_level::{CountingMode, Timer};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{Peri, interrupt, peripherals};

/// `DAC1` 12-bit code corresponding to `1.650 V` virtual ground (`VM = 3.30 V / 2`).
const LSU_VM_DAC_CODE: u16 = 2048;

/// `ADC2` channel index for `PA6` (`ADC2_INP3`, `LSU_UN_SENSE`).
const ADC2_CH_UN_SENSE: u8 = 3;

/// `ADC2` channel index for `PA7` (`ADC2_INP7`, `LSU_IP_SENSE`).
const ADC2_CH_IP_SENSE: u8 = 7;

/// `TIM15` prescaler for `1.000 MHz` tick from `240.0 MHz` timer kernel clock (`240 - 1`).
const TIM15_PSC: u16 = 239;

/// `TIM15` auto-reload for `125 us` half-period (`8.000 kHz` edges -> `4.000 kHz` square wave).
const TIM15_ARR: u16 = 124;

/// `ADC2` trigger offset in microseconds after each `PE5` toggle (`CCR2 = 30 us`).
/// Matches `10 kOhm + 4.7 nF` (Vpp_eff = 3.302 V) and `10 kOhm + 100 nF` within 3%.
const TIM15_SAMPLE_OFFSET_US: u16 = 30;

/// Latest `PA6` (`LSU_UN_SENSE`) 16-bit reading captured during `PE5` HIGH half-cycle.
static UN_HIGH_RAW: AtomicU16 = AtomicU16::new(0);

/// Latest `PA6` (`LSU_UN_SENSE`) 16-bit reading captured during `PE5` LOW half-cycle.
static UN_LOW_RAW: AtomicU16 = AtomicU16::new(0);

/// Latest `PA7` (`LSU_IP_SENSE`) 16-bit reading captured synchronized to `4.000 kHz` half-cycles.
static IP_SENSE_RAW: AtomicU16 = AtomicU16::new(32768);

/// Tracks whether both LOW (`bit 0`) and HIGH (`bit 1`) `PE5` half-cycles have been sampled.
static PHASE_MASK: AtomicU8 = AtomicU8::new(0);

/// Interrupt handler bound to `TIM15` in `Irqs` (NVIC Priority 2).
pub struct Tim15WidebandInterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::TIM15> for Tim15WidebandInterruptHandler {
    /// # Safety
    ///
    /// Invoked exclusively by the hardware `TIM15` NVIC vector; accesses only `TIM15`/`ADC2`
    /// registers and lock-free `AtomicU16`/`AtomicU8` statics.
    unsafe fn on_interrupt() {
        on_tim15_interrupt();
    }
}

/// Owns the STM32H743 peripherals dedicated to the onboard Bosch LSU 4.9 controller.
pub struct WidebandHw<'d> {
    dac: Dac<'d, Blocking>,
    heater_pwm: SimplePwm<'d, peripherals::TIM17>,
    _tim15: Timer<'d, peripherals::TIM15>,
    _pe5_pin: PwmPin<'d, peripherals::TIM15, Ch1>,
    _adc2: Peri<'d, peripherals::ADC2>,
    _pa6_ch: AnyAdcChannel<'d, peripherals::ADC2>,
    _pa7_ch: AnyAdcChannel<'d, peripherals::ADC2>,
}

/// Peripheral bundle transferred from `embassy_stm32::init` to [`WidebandHw::init`].
pub struct WidebandPeripherals<'d> {
    pub dac1: Peri<'d, peripherals::DAC1>,
    pub pa4_ip_dac: Peri<'d, peripherals::PA4>,
    pub pa5_vm_dac: Peri<'d, peripherals::PA5>,
    pub adc2: Peri<'d, peripherals::ADC2>,
    pub pa6_un_sense: Peri<'d, peripherals::PA6>,
    pub pa7_ip_sense: Peri<'d, peripherals::PA7>,
    pub tim15: Peri<'d, peripherals::TIM15>,
    pub pe5_nernst_ac: Peri<'d, peripherals::PE5>,
    pub tim17: Peri<'d, peripherals::TIM17>,
    pub pb9_heater_pwm: Peri<'d, peripherals::PB9>,
}

impl<'d> WidebandHw<'d> {
    /// Initializes `DAC1`, `TIM17` (`100 Hz` heater), `ADC2` (injected), and `TIM15` (`4 kHz`).
    ///
    /// # Safety / Ordering Invariant
    /// Must be called **after** `Adc1Sensors::new(...)`. `ADC1` and `ADC2` share `ADC12RST`
    /// (`AHB1RSTR`) and `ADC12_COMMON->CCR`; configuring `ADC2` directly via PAC avoids resetting
    /// `ADC1` while preserving its already-configured `PLL3_R = 48.000 MHz` kernel clock.
    pub fn init(p: WidebandPeripherals<'d>) -> Self {
        let mut dac = Dac::new_blocking(p.dac1, p.pa4_ip_dac, p.pa5_vm_dac);
        dac.set(DualValue::Bit12Right(LSU_VM_DAC_CODE, LSU_VM_DAC_CODE));

        let heater_pin = PwmPin::new(p.pb9_heater_pwm, OutputType::PushPull);
        let mut heater_pwm = SimplePwm::new(
            p.tim17,
            Some(heater_pin),
            None,
            None,
            None,
            Hertz(100),
            CountingMode::EdgeAlignedUp,
        );
        heater_pwm.ch1().set_duty_cycle_fraction(0, 1_000);
        heater_pwm.ch1().enable();

        let pa6_ch = p.pa6_un_sense.degrade_adc();
        let pa7_ch = p.pa7_ip_sense.degrade_adc();
        init_adc2_injected();

        let tim15 = Timer::new(p.tim15);
        let pe5_pin = PwmPin::new(p.pe5_nernst_ac, OutputType::PushPull);
        init_tim15_ri_exciter();

        // Wait up to 2 ms (8 half-cycles) so both HIGH and LOW `PE5` samples settle before tick 0.
        for _ in 0..20 {
            if PHASE_MASK.load(Ordering::Acquire) == 0b11 {
                break;
            }
            cortex_m::asm::delay(480 * 100);
        }

        Self {
            dac,
            heater_pwm,
            _tim15: tim15,
            _pe5_pin: pe5_pin,
            _adc2: p.adc2,
            _pa6_ch: pa6_ch,
            _pa7_ch: pa7_ch,
        }
    }

    /// Populates `lsu_un_dc_raw`, `lsu_un_ac_delta_raw`, and `lsu_ip_raw` from atomic ISR state.
    pub fn read_adc_into(&self, raw: &mut RawSensorAdc) {
        let un_high = UN_HIGH_RAW.load(Ordering::Relaxed);
        let un_low = UN_LOW_RAW.load(Ordering::Relaxed);
        let ip_raw = IP_SENSE_RAW.load(Ordering::Relaxed);

        raw.lsu_un_dc_raw = (((un_high as u32) + (un_low as u32)) / 2) as u16;
        raw.lsu_un_ac_delta_raw = un_high.abs_diff(un_low);
        raw.lsu_ip_raw = ip_raw;
    }

    /// Applies 12-bit `DAC1_OUT1` (`Ip`) pump code and `100 Hz` `TIM17_CH1` heater duty.
    pub fn apply_outputs(&mut self, ip_dac_code: u16, heater_duty_per_mille: u16) {
        let clamped_dac = ip_dac_code.min(4095);
        self.dac
            .set(DualValue::Bit12Right(clamped_dac, LSU_VM_DAC_CODE));

        let clamped_duty = heater_duty_per_mille.min(1_000) as u32;
        self.heater_pwm
            .ch1()
            .set_duty_cycle_fraction(clamped_duty, 1_000);
    }
}

/// Initializes `ADC2` directly via PAC for 16-bit software-triggered injected conversions
/// (`JSQ1 = PA6`, `JSQ2 = PA7`) with `4x` hardware oversampling (`~12.2 us` total burst).
fn init_adc2_injected() {
    let adc2 = pac::ADC2;

    adc2.cr().modify(|w| {
        w.set_deeppwd(false);
        w.set_advregen(true);
    });
    // Wait >= 10 us for the internal ADC voltage regulator startup (`~20 us` at 480 MHz CPU).
    cortex_m::asm::delay(480 * 20);

    adc2.cr().modify(|w| w.set_boost(Boost::LT50));

    for ch in 0..20 {
        adc2.difsel()
            .modify(|w| w.set_difsel(ch, Difsel::SINGLE_ENDED));
    }

    adc2.cr().modify(|w| {
        w.set_adcaldif(Adcaldif::SINGLE_ENDED);
        w.set_adcallin(true);
    });
    adc2.cr().modify(|w| w.set_adcal(true));
    while adc2.cr().read().adcal() {}
    cortex_m::asm::delay(480 * 2);

    adc2.isr().write(|w| w.set_adrdy(true));
    adc2.cr().modify(|w| w.set_aden(true));
    while !adc2.isr().read().adrdy() {}
    adc2.isr().write(|w| w.set_adrdy(true));

    adc2.cfgr().modify(|w| w.set_res(Res::BITS16));
    adc2.cfgr2().modify(|w| {
        w.set_jovse(true);
        w.set_ovsr(3); // 4x hardware oversampling.
        w.set_ovss(2); // Right-shift by 2 bits to stay in 16-bit unsigned range.
    });

    adc2.pcsel().modify(|w| {
        w.set_pcsel(ADC2_CH_UN_SENSE as usize, Pcsel::PRESELECTED);
        w.set_pcsel(ADC2_CH_IP_SENSE as usize, Pcsel::PRESELECTED);
    });

    let smp = embassy_stm32::adc::SampleTime::CYCLES64_5;
    adc2.smpr(0).modify(|w| {
        w.set_smp(ADC2_CH_UN_SENSE as usize, smp);
        w.set_smp(ADC2_CH_IP_SENSE as usize, smp);
    });

    adc2.jsqr().write(|w| {
        w.set_jl(1); // 2 injected conversions (JL = len - 1).
        w.set_jexten(Exten::DISABLED);
        w.set_jsq(0, ADC2_CH_UN_SENSE);
        w.set_jsq(1, ADC2_CH_IP_SENSE);
    });
}

/// Configures `TIM15` for `4.000 kHz` square-wave excitation on `CH1` (`PE5`, toggle at `CNT=0`)
/// and synchronized `ADC2` injected sampling at `CCR2 = 30 us` after every edge.
fn init_tim15_ri_exciter() {
    let tim15 = pac::TIM15;

    tim15.cr1().modify(|w| w.set_cen(false));
    tim15.psc().write_value(TIM15_PSC);
    tim15.arr().write(|w| w.set_arr(TIM15_ARR));
    tim15.ccr(0).write(|w| w.set_ccr(0));
    tim15.ccr(1).write(|w| w.set_ccr(TIM15_SAMPLE_OFFSET_US));

    tim15.ccmr_output().modify(|w| {
        w.set_ccs(0, CcmrOutputCcs::OUTPUT);
        w.set_ocpe(0, false);
        w.set_ocm(0, Ocm::TOGGLE);

        w.set_ccs(1, CcmrOutputCcs::OUTPUT);
        w.set_ocpe(1, false);
        w.set_ocm(1, Ocm::FROZEN);
    });

    tim15.ccer().modify(|w| {
        w.set_ccp(0, false);
        w.set_cce(0, true);
    });

    tim15.bdtr().modify(|w| w.set_moe(true));
    tim15.egr().write(|w| w.set_ug(true));
    tim15.sr().write_value(Sr2chCmp(0));

    // Dier2chCmp::set_ccie asserts `n < 1` in stm32-metapac, so write raw bitmask for UIE + CC2IE.
    tim15.dier().write_value(Dier2chCmp((1 << 0) | (1 << 2)));

    interrupt::typelevel::TIM15::set_priority(interrupt::Priority::P2);
    // SAFETY: TIM15 interrupt handler is stateless and bound via `Irqs` at Priority P2.
    unsafe {
        interrupt::typelevel::TIM15::enable();
    }

    tim15.cr1().modify(|w| w.set_cen(true));
}

/// `8.000 kHz` (`125 us`) `TIM15` interrupt service routine.
///
/// - At `CC2IF` (`t = 30 us` into each half-cycle): starts `ADC2` injected conversion (`JADSTART`).
/// - At `UIF` (`t = 0 / 125 us`, right after `PE5` toggles): reads completed `JDR1`/`JDR2` values
///   from the half-cycle that just ended and stores them atomically by polarity.
#[inline]
fn on_tim15_interrupt() {
    let tim15 = pac::TIM15;
    let sr = tim15.sr().read();

    if sr.uif() {
        tim15.sr().write_value(Sr2chCmp(!1u32));

        let adc2 = pac::ADC2;
        if adc2.isr().read().jeos() {
            let un_raw = adc2.jdr(0).read().jdata() as u16;
            let ip_raw = adc2.jdr(1).read().jdata() as u16;
            adc2.isr().write(|w| w.set_jeos(true));

            // PE5 just toggled at CNT == 0, so the completed half-cycle polarity is inverted.
            let pe5_now_high = pac::GPIOE.idr().read().idr(5) == Idr::HIGH;
            if pe5_now_high {
                UN_LOW_RAW.store(un_raw, Ordering::Relaxed);
                PHASE_MASK.fetch_or(0b01, Ordering::Release);
            } else {
                UN_HIGH_RAW.store(un_raw, Ordering::Relaxed);
                PHASE_MASK.fetch_or(0b10, Ordering::Release);
            }
            IP_SENSE_RAW.store(ip_raw, Ordering::Relaxed);
        }
    }

    if sr.ccif(1) {
        tim15.sr().write_value(Sr2chCmp(!(1u32 << 2)));
        pac::ADC2.cr().modify(|w| w.set_jadstart(true));
    }
}
