#![no_std]
#![no_main]

//! `RP2040` Honda OBD1 `24/4/1` Distributor + `VSS` + `Cyl-3` Knock + `FLEX`/`ALTF*` Simulator
//! (`_DOCS/breadboard_stage1.md`).
//!
//! Stage 1 Breadboard Wiring (`3.3V` safe direct to `STM32H743VIT6`):
//! - `GP2`  -> `STM32 PA15` (`CKP_CAPTURE` — 24 pulses per `720°` cam rev / 12 per crank rev)
//! - `GP3`  -> `STM32 PA1`  (`TDC_CAPTURE` — 4 pulses per `720°` cam rev, every 6 `CKP` teeth)
//! - `GP4`  -> `STM32 PB10` (`CYP_CAPTURE` — 1 pulse per `720°` cam rev, leads Tooth 0)
//! - `GP5`  -> `STM32 PB11` (`VSS_CAPTURE` — simulated vehicle speed proportional to RPM)
//! - `GP6`  -> `22kΩ/100kΩ -> 104 -> MCP6004 1.65V AFE -> STM32 PC2_C` (`7.5 kHz` Cyl-3 burst)
//! - `GP8`  -> `STM32 PE15` (`FLEX`: Continental `135.0 Hz` `E85`, `2.5 ms` pulse = `21.9 °C`)
//! - `GP10` -> `STM32 PB7`  (`ALTF*`: `125.0 Hz`, `35.0%` HIGH field duty)
//! - `GP24` <- Onboard `YD-RP2040` `USR` button (click toggles `Cyl-3` `7.5 kHz` knock bursts)
//! - `GP26` (`ADC0`) <- `10K` rotary potentiometer (`0V..3.3V` sweeps `150 RPM .. 9,000 RPM`!)
//! - `GND`  -> Shared `GND` rail with `STM32H743VIT6`

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use {defmt_rtt as _, panic_probe as _};

/// `125.0 MHz / (16_666 + 1) = 7,499.85 Hz` (`50%` duty = `8_333`), centered on `KnockBiquad`.
const KNOCK_7500HZ_TOP: u16 = 16_666;
const KNOCK_7500HZ_HALF_DUTY: u16 = 8_333;

/// `125.0 MHz / 250 = 500.000 kHz` (`2.000 µs` per PWM tick) for low-frequency `FLEX`/`ALTF*`.
const AUX_PWM_DIVIDER: u16 = 250;
/// `3,704 × 2 µs = 7,408 µs` (`134.99 Hz` -> `E85` / `85.0%` ethanol); `1,250 × 2 µs = 2.50 ms` (`21.9 °C`).
const FLEX_E85_TOP: u16 = 3_703;
const FLEX_TEMP_22C_COMPARE: u16 = 1_250;
/// `4,000 × 2 µs = 8,000 µs` (`125.00 Hz`); `1,400 / 4,000 = 35.0%` HIGH duty.
const ALTF_125HZ_TOP: u16 = 3_999;
const ALTF_35PCT_COMPARE: u16 = 1_400;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut ckp_out = Output::new(p.PIN_2, Level::Low);
    let mut tdc_out = Output::new(p.PIN_3, Level::Low);
    let mut cyp_out = Output::new(p.PIN_4, Level::Low);
    let mut vss_out = Output::new(p.PIN_5, Level::Low);
    let mut led = Output::new(p.PIN_25, Level::Low);
    let usr_btn = Input::new(p.PIN_24, Pull::Up);

    let mut pwm_cfg = PwmConfig::default();
    pwm_cfg.top = KNOCK_7500HZ_TOP;
    pwm_cfg.compare_a = 0;
    let mut knock_pwm = Pwm::new_output_a(p.PWM_SLICE3, p.PIN_6, pwm_cfg);

    let mut flex_cfg = PwmConfig::default();
    flex_cfg.divider = AUX_PWM_DIVIDER.to_fixed();
    flex_cfg.top = FLEX_E85_TOP;
    flex_cfg.compare_a = FLEX_TEMP_22C_COMPARE;
    let _flex_pwm = Pwm::new_output_a(p.PWM_SLICE4, p.PIN_8, flex_cfg);

    let mut altf_cfg = PwmConfig::default();
    altf_cfg.divider = AUX_PWM_DIVIDER.to_fixed();
    altf_cfg.top = ALTF_125HZ_TOP;
    altf_cfg.compare_a = ALTF_35PCT_COMPARE;
    let _altf_pwm = Pwm::new_output_a(p.PWM_SLICE5, p.PIN_10, altf_cfg);

    let mut adc = Adc::new_blocking(p.ADC, AdcConfig::default());
    let mut rpm_pot = Channel::new_pin(p.PIN_26, Pull::None);

    info!(
        "RP2040 Honda OBD1 24/4/1 Dizzy Simulator running \
         (GP2=CKP, GP3=TDC, GP4=CYP, GP6=7.5kHz Knock, GP8=FLEX E85, GP10=ALTF 35%, GP24=USR, GP26=RPM)"
    );

    let mut tooth: u8 = 0;
    let mut target_rpm: u32 = 900;
    let mut knock_burst_enabled = false;
    let mut prev_usr_low = false;

    loop {
        // Sample RPM pot and poll `USR` (`GP24`) toggle once per 720° cam revolution (`tooth == 0`).
        // `blocking_read` avoids `ADC_IRQ_FIFO` single-shot quirks and filters wiper noise.
        if tooth == 0 {
            let mut sum: u32 = 0;
            let mut ok_count: u32 = 0;
            for _ in 0..8 {
                if let Ok(sample) = adc.blocking_read(&mut rpm_pot) {
                    sum += sample as u32;
                    ok_count += 1;
                }
            }
            if let Some(avg_12b) = sum.checked_div(ok_count) {
                // Map 0..4095 -> 150..9000 RPM smoothly
                target_rpm = 150 + (avg_12b * (9_000 - 150)) / 4095;
            }
            led.toggle();
        }

        if tooth.is_multiple_of(6) {
            let usr_low = usr_btn.is_low();
            if usr_low && !prev_usr_low {
                knock_burst_enabled = !knock_burst_enabled;
                if !knock_burst_enabled {
                    let _ = knock_pwm.set_duty_cycle(0);
                }
                info!(
                    "Cyl-3 7.5 kHz knock burst (GP6): {}",
                    if knock_burst_enabled { "ON" } else { "OFF" }
                );
            }
            prev_usr_low = usr_low;
        }

        // Gate the 7.500 kHz carrier inside Cylinder 3's combustion window (`180°..240°`,
        // spanning `tooth 6..7` around `Cyl 3` 10°..50° ATDC = `190°..230°`).
        if knock_burst_enabled && tooth == 6 {
            let _ = knock_pwm.set_duty_cycle(KNOCK_7500HZ_HALF_DUTY);
            led.set_high();
        } else if tooth == 8 {
            let _ = knock_pwm.set_duty_cycle(0);
        }

        // `12` CKP teeth per `360°` crank rev => `tooth_period_us = 5_000_000 / RPM`
        let tooth_period_us = (5_000_000 / target_rpm.max(150)).max(500) as u64;
        let pulse_high_us = (tooth_period_us / 4).clamp(80, 1_500);

        // Emit leading CYP (Cyl 1 sync) & TDC (every 6 teeth = 180° crank) just before CKP edge,
        // matching `ecu-core`'s `HondaObd1Decoder` & physical distributor rotor alignment!
        if tooth == 0 {
            cyp_out.set_high();
        }
        if tooth.is_multiple_of(6) {
            tdc_out.set_high();
            vss_out.toggle();
        }

        Timer::after(Duration::from_micros(30)).await;
        ckp_out.set_high();

        Timer::after(Duration::from_micros(pulse_high_us)).await;
        ckp_out.set_low();
        tdc_out.set_low();
        cyp_out.set_low();

        let remaining_us = tooth_period_us.saturating_sub(pulse_high_us + 30).max(50);
        Timer::after(Duration::from_micros(remaining_us)).await;

        tooth = (tooth + 1) % 24;
    }
}
