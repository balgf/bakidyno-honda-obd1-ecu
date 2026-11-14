//! Direct onboard Bosch LSU 4.9 controller (`Option 8A` `TLV9064` + `DAC1` + `TIM15`/`ADC2`).
//!
//! Hardware architecture (`_DOCS/hardware_bom.md` & `_DOCS/stm32h743_pinout.md`):
//! - Virtual ground (`Pin 2 Yellow VM`): `1.650 V` (`PA5 DAC1_OUT2` buffered)
//! - Stoichiometric Nernst target (`Pin 6 Black UN`): `VM + 0.450 V = 2.100 V`
//! - Ceramic temperature target: `Ri = 300.0 Ω` (`~780 °C`), measured via `PE5 TIM15_CH1` `4 kHz`
//!   square wave through `10 kΩ + 4.7 nF` into `UN`, sampled peak-trough by `ADC2` (`EXTSEL=14`).
//! - Pump current (`Pin 1 Red IP` / `Pin 5 Green IA/Rtrim`): `61.9 Ω 0.1%` sense shunt amplified
//!   around `1.650 V` (`gain = 4.7x` via `47.0 kΩ / 10.0 kΩ` on `TLV9064`), read on `PA7`.

use crate::math::PidController;
use crate::sensors::{ADC_FULL_SCALE, VREF_VOLTS};
use ecu_types::{FaultFlags, WidebandState};

pub const LSU_VM_VOLTS: f32 = 1.650;
pub const LSU_TARGET_UN_VOLTS: f32 = LSU_VM_VOLTS + 0.450; // 2.100 V at λ = 1.000
pub const LSU_TARGET_RI_OHMS: f32 = 300.0; // Bosch LSU 4.9 nominal 780 °C impedance
pub const LSU_SHUNT_OHMS: f32 = 61.9;
pub const LSU_SHUNT_AMP_GAIN: f32 = 4.70; // 47.0 kΩ / 10.0 kΩ

// Official Bosch LSU 4.9 characteristic curve: Pump current Ip (mA) -> Lambda (λ)
const BOSCH_IP_MA_BINS: [f32; 14] = [
    -2.00, -1.60, -1.20, -0.927, -0.60, -0.352, -0.15, 0.00, 0.15, 0.34, 0.68, 1.02, 1.55, 2.22,
];
const BOSCH_LAMBDA_VALS: [f32; 14] = [
    0.650, 0.700, 0.750, 0.800, 0.850, 0.900, 0.950, 1.000, 1.075, 1.180, 1.430, 1.700, 2.420,
    5.000,
];

/// Converts Bosch LSU 4.9 pump current `Ip` (`mA`) to `λ` (`0.65 .. 5.00`).
pub fn bosch_lsu49_ip_ma_to_lambda(ip_ma: f32) -> f32 {
    if ip_ma <= BOSCH_IP_MA_BINS[0] {
        return BOSCH_LAMBDA_VALS[0];
    }
    let last = BOSCH_IP_MA_BINS.len() - 1;
    if ip_ma >= BOSCH_IP_MA_BINS[last] {
        return BOSCH_LAMBDA_VALS[last];
    }
    let mut i = 0;
    while i < last {
        let x0 = BOSCH_IP_MA_BINS[i];
        let x1 = BOSCH_IP_MA_BINS[i + 1];
        if ip_ma <= x1 {
            let t = (ip_ma - x0) / (x1 - x0);
            return BOSCH_LAMBDA_VALS[i] + (BOSCH_LAMBDA_VALS[i + 1] - BOSCH_LAMBDA_VALS[i]) * t;
        }
        i += 1;
    }
    1.000
}

pub struct Lsu49Controller {
    state: WidebandState,
    engine_run_ms: u32,
    overheat_ms: u16,
    ramp_v_eff: f32,
    heater_pid: PidController,
    pump_pi: PidController,
    ri_ohms: f32,
    lambda: f32,
    heater_duty_per_mille: u16,
    ip_dac_code: u16,
}

impl Default for Lsu49Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Lsu49Controller {
    pub const fn new() -> Self {
        Self {
            state: WidebandState::WaitForEngine,
            engine_run_ms: 0,
            overheat_ms: 0,
            ramp_v_eff: 2.0,
            // Heater PI trims around 7.5 V nominal across full [0.0 .. 12.0 V] RMS range
            // (`kd = 0` avoids 1 kHz derivative noise).
            heater_pid: PidController::new(0.045, 0.018, 0.0, -7.5, 4.5),
            // Pump PI maps (UN_actual - 2.100 V) -> DAC offset around 1.650V [-1.50 .. +1.50V]
            pump_pi: PidController::new(1.80, 45.0, 0.0, -1.55, 1.55),
            ri_ohms: 2500.0,
            lambda: 1.000,
            heater_duty_per_mille: 0,
            ip_dac_code: 2048, // Mid-scale 1.650V = 0 mA pump current
        }
    }

    #[inline]
    pub const fn state(&self) -> WidebandState {
        self.state
    }

    #[inline]
    pub const fn lambda(&self) -> f32 {
        self.lambda
    }

    #[inline]
    pub const fn ri_ohms(&self) -> f32 {
        self.ri_ohms
    }

    #[inline]
    pub const fn heater_duty_per_mille(&self) -> u16 {
        self.heater_duty_per_mille
    }

    #[inline]
    pub const fn ip_dac_code(&self) -> u16 {
        self.ip_dac_code
    }

    /// Runs the `4 kHz` (or bench `1 kHz`) Nernst pump PI loop and decodes `Ri` & `λ`.
    pub fn update_pump_loop(
        &mut self,
        lsu_un_dc_raw: u16,
        lsu_un_ac_delta_raw: u16,
        lsu_ip_raw: u16,
        dt_s: f32,
    ) {
        let un_dc_v = (lsu_un_dc_raw as f32 / ADC_FULL_SCALE) * VREF_VOLTS;
        let un_ac_v = (lsu_un_ac_delta_raw as f32 / ADC_FULL_SCALE) * VREF_VOLTS;

        // `PE5` injects 3.30Vpp square wave through 10.0 kΩ:
        // Voltage divider between 10_000 Ω and ceramic `Ri`:
        // ΔUN = 3.30V * Ri / (10_000 + Ri)  =>  Ri = 10_000 * ΔUN / (3.30 - ΔUN)
        let denom = (VREF_VOLTS - un_ac_v).max(0.05);
        self.ri_ohms = ((10_000.0 * un_ac_v) / denom).clamp(20.0, 9_999.0);

        if self.state == WidebandState::ClosedLoopReady {
            // Rich exhaust raises Nernst voltage (> 2.100V) -> pump out negative Ip;
            // Lean exhaust lowers Nernst voltage (< 2.100V) -> pump in positive Ip.
            let error_v = LSU_TARGET_UN_VOLTS - un_dc_v;
            let dac_offset_v = self.pump_pi.update(error_v, dt_s);
            let dac_v = (LSU_VM_VOLTS + dac_offset_v).clamp(0.05, VREF_VOLTS - 0.05);
            self.ip_dac_code = ((dac_v / VREF_VOLTS) * 4095.0) as u16;

            // Convert `PA7` buffered shunt voltage into `Ip (mA)`:
            let v_ip_sense = (lsu_ip_raw as f32 / ADC_FULL_SCALE) * VREF_VOLTS;
            let v_shunt = (v_ip_sense - LSU_VM_VOLTS) / LSU_SHUNT_AMP_GAIN;
            let ip_ma = (v_shunt / LSU_SHUNT_OHMS) * 1000.0;
            self.lambda = bosch_lsu49_ip_ma_to_lambda(ip_ma);
        } else {
            self.pump_pi.reset();
            self.ip_dac_code = 2048;
            self.lambda = 1.000;
        }
    }

    /// Runs Bosch's condensation-safe heater state machine (`100 Hz` / `10 ms` or `1 kHz` tick).
    ///
    /// Converts target effective RMS voltage `V_eff` into PWM duty:
    /// `Duty = (V_eff / V_batt)^2`.
    pub fn update_heater_1khz(&mut self, rpm: u16, vbatt_v: f32, faults: &mut FaultFlags) {
        if rpm < 400 || vbatt_v < 9.0 {
            self.state = WidebandState::WaitForEngine;
            self.engine_run_ms = 0;
            self.overheat_ms = 0;
            self.ramp_v_eff = 2.0;
            self.heater_pid.reset();
            self.heater_duty_per_mille = 0;
            return;
        }

        self.engine_run_ms = self.engine_run_ms.saturating_add(1);

        let v_eff_target = match self.state {
            WidebandState::WaitForEngine => {
                self.state = WidebandState::CondensationPreheat;
                2.0
            }
            WidebandState::CondensationPreheat => {
                // If sensor is already hot (warm restart, 60 <= Ri < 450 Ω), jump to ClosedLoop!
                if (60.0..450.0).contains(&self.ri_ohms) {
                    self.state = WidebandState::ClosedLoopReady;
                    7.5
                } else if self.engine_run_ms >= 15_000 {
                    self.state = WidebandState::WarmupRamp;
                    self.ramp_v_eff = 7.0; // Bosch initial warmup step after condensation phase
                    self.ramp_v_eff
                } else {
                    2.0 // Gentle 2.0V RMS evaporation preheat
                }
            }
            WidebandState::WarmupRamp => {
                // Ramp at 0.4 V/s (0.0004 V/ms) up to 11.5V until ceramic drops below 420 Ω
                self.ramp_v_eff = (self.ramp_v_eff + 0.0004).min(11.5);
                if self.ri_ohms <= 420.0 {
                    self.state = WidebandState::ClosedLoopReady;
                } else if self.engine_run_ms > 60_000 && self.ri_ohms > 3_000.0 {
                    faults.insert(FaultFlags::WIDEBAND_FAULT);
                    self.state = WidebandState::Fault;
                    return;
                }
                self.ramp_v_eff
            }
            WidebandState::ClosedLoopReady => {
                if self.ri_ohms < 60.0 {
                    self.overheat_ms = self.overheat_ms.saturating_add(1);
                    if self.overheat_ms >= 50 {
                        // Sustained ceramic overheat/short (< 60 Ω for 50 ms) -> cut heater!
                        faults.insert(FaultFlags::WIDEBAND_FAULT);
                        self.state = WidebandState::Fault;
                        self.heater_duty_per_mille = 0;
                        return;
                    }
                    0.0
                } else {
                    self.overheat_ms = 0;
                    // Positive error when sensor is too cold (`Ri > 300 Ω`) -> increase voltage.
                    let err_ohms = self.ri_ohms - LSU_TARGET_RI_OHMS;
                    (7.5 + self.heater_pid.update(err_ohms, 0.001)).clamp(0.0, 12.0)
                }
            }
            WidebandState::Fault => {
                self.heater_duty_per_mille = 0;
                return;
            }
        };

        let ratio = (v_eff_target / vbatt_v.max(9.0)).clamp(0.0, 1.0);
        let duty_frac = (ratio * ratio).clamp(0.0, 0.95);
        self.heater_duty_per_mille = (duty_frac * 1000.0) as u16;
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{FaultFlags, WidebandState};

    use crate::sensors::{ADC_FULL_SCALE, VREF_VOLTS};

    use super::{Lsu49Controller, bosch_lsu49_ip_ma_to_lambda};

    #[test]
    fn bosch_lsu49_curve_hits_stoich_rich_and_lean_benchmarks() {
        assert!((bosch_lsu49_ip_ma_to_lambda(0.0) - 1.000).abs() < 1e-4);
        assert!((bosch_lsu49_ip_ma_to_lambda(-0.927) - 0.800).abs() < 1e-3);
        assert!((bosch_lsu49_ip_ma_to_lambda(0.34) - 1.180).abs() < 1e-3);
    }

    #[test]
    fn heater_obeys_bosch_condensation_preheat_and_locks_closed_loop() {
        let mut wbo = Lsu49Controller::new();
        let mut faults = FaultFlags::default();

        // Engine off -> 0% heater duty
        wbo.update_heater_1khz(0, 14.0, &mut faults);
        assert_eq!(wbo.heater_duty_per_mille(), 0);

        // Cold engine start at 14.0V: 2.0V RMS => (2/14)^2 = 2.04% (~20 per-mille)
        wbo.update_heater_1khz(1100, 14.0, &mut faults);
        assert_eq!(wbo.state(), WidebandState::CondensationPreheat);
        assert!((wbo.heater_duty_per_mille() as i32 - 20).abs() <= 2);

        // Simulate hot restart (`Ri = 300 Ω` -> ΔUN = 3.30 * 300 / 10300 = 0.0961V)
        let un_dc_raw = ((2.100 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        let un_ac_raw = ((0.0961 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        let ip_stoich_raw = ((1.650 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        wbo.update_pump_loop(un_dc_raw, un_ac_raw, ip_stoich_raw, 0.001);
        wbo.update_heater_1khz(1100, 14.0, &mut faults);

        assert_eq!(wbo.state(), WidebandState::ClosedLoopReady);
        assert!((wbo.ri_ohms() - 300.0).abs() < 3.0);
        assert!((wbo.lambda() - 1.000).abs() < 0.005);
    }

    #[test]
    fn wideband_cold_warmup_ramp_overheat_protection_and_pump_pi_direction() {
        let mut wbo = Lsu49Controller::new();
        let mut faults = FaultFlags::default();

        // 1. Cold start (`Ri = 2,500 Ω`): 15,000 ms CondensationPreheat -> WarmupRamp
        for _ in 0..15_001 {
            wbo.update_heater_1khz(1_200, 14.0, &mut faults);
        }
        assert_eq!(wbo.state(), WidebandState::WarmupRamp);
        assert!(wbo.heater_duty_per_mille() >= 240);

        // Open-circuit sensor remaining > 3,000 Ω past 60,000 ms triggers WIDEBAND_FAULT
        let un_dc_stoich = ((2.100 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        let un_ac_open = ((1.200 / VREF_VOLTS) * ADC_FULL_SCALE) as u16; // Ri > 5,000 Ω
        let ip_zero = ((1.650 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        wbo.update_pump_loop(un_dc_stoich, un_ac_open, ip_zero, 0.001);
        for _ in 0..45_005 {
            wbo.update_heater_1khz(1_200, 14.0, &mut faults);
        }
        assert_eq!(wbo.state(), WidebandState::Fault);
        assert!(faults.contains(FaultFlags::WIDEBAND_FAULT));

        // 2. Fresh controller in ClosedLoopReady: test rich/lean pump PI & overheat cutoff
        let mut wbo2 = Lsu49Controller::new();
        let mut faults2 = FaultFlags::default();
        let un_ac_300ohm = ((0.0961 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        wbo2.update_pump_loop(un_dc_stoich, un_ac_300ohm, ip_zero, 0.001);
        wbo2.update_heater_1khz(1_200, 14.0, &mut faults2);
        wbo2.update_heater_1khz(1_200, 14.0, &mut faults2);
        assert_eq!(wbo2.state(), WidebandState::ClosedLoopReady);

        // Rich exhaust (UN = 2.50 V > 2.10 V) drives DAC below mid-scale (2048)
        let un_dc_rich = ((2.500 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        wbo2.update_pump_loop(un_dc_rich, un_ac_300ohm, ip_zero, 0.001);
        assert!(wbo2.ip_dac_code() < 2048);

        // Lean exhaust (UN = 1.60 V < 2.10 V) drives DAC above mid-scale (2048)
        let un_dc_lean = ((1.600 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        for _ in 0..10 {
            wbo2.update_pump_loop(un_dc_lean, un_ac_300ohm, ip_zero, 0.001);
        }
        assert!(wbo2.ip_dac_code() > 2048);

        // Ceramic overheat (Ri < 60 Ω for 50 ms) immediately cuts heater duty and latches Fault
        let un_ac_40ohm = ((0.0131 / VREF_VOLTS) * ADC_FULL_SCALE) as u16;
        wbo2.update_pump_loop(un_dc_stoich, un_ac_40ohm, ip_zero, 0.001);
        assert!(wbo2.ri_ohms() < 60.0);
        for _ in 0..50 {
            wbo2.update_heater_1khz(1_200, 14.0, &mut faults2);
            assert_eq!(wbo2.heater_duty_per_mille(), 0);
        }
        assert_eq!(wbo2.state(), WidebandState::Fault);
        assert!(faults2.contains(FaultFlags::WIDEBAND_FAULT));
    }
}
