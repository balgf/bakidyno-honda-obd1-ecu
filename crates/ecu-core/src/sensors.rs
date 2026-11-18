//! Ratiometric analog scaling & switch conditioning aligned with `hardware_bom.md`.
//!
//! Key hardware constants (`_DOCS/hardware_bom.md`):
//! - Precision ADC reference (`REF3033AIDBZR`): `VREF+ = 3.300 V` (`65535` 16-bit counts)
//! - `5V` sensor dividers (`12.0 kΩ` + `20.0 kΩ` `0.1%`): `0.625` ratio (`5.000V -> 3.125V`)
//! - Thermistor bias pull-ups (`D13 ECT`, `D15 IAT`): `2.49 kΩ` (`0.1%`) to `VCC1/2` (`5.00V` rail)
//! - Battery voltage divider (`PC4 IGP_VBATT`): `100.0 kΩ` top + `15.0 kΩ` bottom (`15/115` ratio)

use crate::math::{LowPassFilter, ln_f32};
use ecu_types::{FaultFlags, MapSensorCal, RawSensorAdc, SensorSnapshot, SwitchState};

pub const ADC_FULL_SCALE: f32 = 65535.0;
pub const VREF_VOLTS: f32 = 3.300;
/// `20.0 kΩ / (12.0 kΩ + 20.0 kΩ) = 0.625` (`1 / 0.625 = 1.600`).
pub const SENSOR_DIVIDER_GAIN_INV: f32 = 1.600;
/// `(100.0 kΩ + 15.0 kΩ) / 15.0 kΩ = 7.666667`.
pub const VBATT_DIVIDER_GAIN_INV: f32 = 115.0 / 15.0;
/// Precision `0.1%` thermistor bias pull-up (`2490.0 Ω`).
pub const THERMISTOR_PULLUP_OHMS: f32 = 2490.0;

// Exact 3-point Steinhart-Hart fit to Honda Civic Factory Service Manual NTC curve
// (`37870-PJ7-003` D13 ECT & D15 IAT: 2500 Ω @ 20 °C, 800 Ω @ 50 °C, 220 Ω @ 90 °C):
const HONDA_SH_A: f32 = 0.001_454_8;
const HONDA_SH_B: f32 = 0.000_232_45;
const HONDA_SH_C: f32 = 0.000_000_287_5;

/// Raw GPIO pin levels seen by the `STM32H743` *after* passing through the `3.3V`
/// `SN74HCS14QDRQ1` inverting Schmitt-trigger input stage (`hardware_bom.md` Section 6).
#[derive(Debug, Clone, Copy, Default)]
pub struct InvertedMcuSwitchPins {
    /// `PB5` (`B9 STS` `+12V` cranking -> `74HCS14` inverter -> `LOW` at MCU when cranking).
    pub pb5_sts_pin_high: bool,
    /// `PE4` (`D6 VTP*` `4.7k` pull-up: `HIGH` on harness when pressurized -> `LOW` at MCU).
    pub pe4_vtp_pin_high: bool,
    /// `PE6` (`D4 SCS` grounded when service jumper inserted -> `HIGH` at MCU).
    pub pe6_scs_pin_high: bool,
    /// `PE7` (`B5 ACS*` grounded when A/C requested -> `HIGH` at MCU).
    pub pe7_acs_pin_high: bool,
    /// `PE8` (`B8 PSP*` grounded under steering load -> `HIGH` at MCU).
    pub pe8_psp_pin_high: bool,
    /// `PE10` (`D2 BKSW*` `+12V` when braking -> `LOW` at MCU).
    pub pe10_bksw_pin_high: bool,
    /// `PE12` (`B7` clutch grounded when pressed -> `HIGH` at MCU).
    pub pe12_clutch_pin_high: bool,
}

impl InvertedMcuSwitchPins {
    /// Decodes logical Honda switch states accounting for `SN74HCS14` hardware inversion
    /// and the `4.7 kΩ` pull-up on `D6 VTP*`.
    #[inline]
    pub const fn decode(self) -> SwitchState {
        SwitchState {
            sts_cranking: !self.pb5_sts_pin_high,
            vtp_pressurized: !self.pe4_vtp_pin_high,
            scs_service_jumper: self.pe6_scs_pin_high,
            acs_ac_request: self.pe7_acs_pin_high,
            psp_steering_load: self.pe8_psp_pin_high,
            bksw_brake: !self.pe10_bksw_pin_high,
            clutch_pressed: self.pe12_clutch_pin_high,
        }
    }
}

/// Consecutive out-of-range `1 kHz` samples required before latching an analog sensor fault.
pub const RANGE_FAULT_DEBOUNCE_TICKS: u8 = 20;

/// Stateful `1 kHz` analog sensor conditioner with IIR noise filtering & range fault detection.
pub struct SensorProcessor {
    map_lpf: LowPassFilter,
    tps_lpf: LowPassFilter,
    ect_lpf: LowPassFilter,
    iat_lpf: LowPassFilter,
    vbatt_lpf: LowPassFilter,
    vcc_5v_lpf: LowPassFilter,
    map_bad_ticks: u8,
    tps_bad_ticks: u8,
    ect_bad_ticks: u8,
    iat_bad_ticks: u8,
    /// Calibration voltages (`0.0..5.0V` ratiometric) for closed/wide-open Honda `D11 TPS`.
    pub tps_closed_v: f32,
    pub tps_wot_v: f32,
}

impl Default for SensorProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorProcessor {
    pub const fn new() -> Self {
        Self {
            map_lpf: LowPassFilter::new(0.35),
            tps_lpf: LowPassFilter::new(0.40),
            ect_lpf: LowPassFilter::new(0.05),
            iat_lpf: LowPassFilter::new(0.05),
            vbatt_lpf: LowPassFilter::new(0.10),
            vcc_5v_lpf: LowPassFilter::new(0.05),
            map_bad_ticks: 0,
            tps_bad_ticks: 0,
            ect_bad_ticks: 0,
            iat_bad_ticks: 0,
            tps_closed_v: 0.45,
            tps_wot_v: 4.50,
        }
    }

    /// Resets range-fault debounce counters (used after clearing MIL codes).
    #[inline]
    pub fn reset_fault_counters(&mut self) {
        self.map_bad_ticks = 0;
        self.tps_bad_ticks = 0;
        self.ect_bad_ticks = 0;
        self.iat_bad_ticks = 0;
    }

    #[inline]
    pub fn adc_to_pin_volts(raw: u16) -> f32 {
        (raw as f32 / ADC_FULL_SCALE) * VREF_VOLTS
    }

    /// Converts an NTC thermistor ratiometric ratio `r_ratio = V_sense / V_5v` (`0.0..1.0`)
    /// into `°C` via the `2.49 kΩ` pull-up and Steinhart-Hart equation.
    pub fn honda_ntc_temp_c(v_ratio: f32) -> f32 {
        let clamped = v_ratio.clamp(0.005, 0.995);
        // Divider: R_ntc = R_pullup * V_sense / (V_5v - V_sense)
        // plus parallel loading of our 32.0 kΩ (12k + 20k) ADC divider!
        let r_effective = THERMISTOR_PULLUP_OHMS * clamped / (1.0 - clamped);
        // Compensate for parallel 32.0 kΩ (12k + 20k) divider across the thermistor:
        // 1/R_eff = 1/R_ntc + 1/32000  =>  R_ntc = (R_eff * 32000) / (32000 - R_eff)
        let denom = (32_000.0 - r_effective).max(50.0);
        let r_ntc = (r_effective * 32_000.0) / denom;

        let ln_r = ln_f32(r_ntc);
        let inv_t_kelvin = HONDA_SH_A + HONDA_SH_B * ln_r + HONDA_SH_C * ln_r * ln_r * ln_r;
        (1.0 / inv_t_kelvin) - 273.15
    }

    /// Scales a `1 kHz` raw ADC frame into calibrated physical units and updates fault flags.
    pub fn process(
        &mut self,
        raw: &RawSensorAdc,
        switches: SwitchState,
        vss_kmh: f32,
        map_cal: &MapSensorCal,
        faults: &mut FaultFlags,
    ) -> SensorSnapshot {
        // 1. Measure actual `D19/D20 VCC1/2` 5V rail via `PA0` (`12k/20k` divider)
        let vcc_5v_measured = Self::adc_to_pin_volts(raw.vcc_5v_raw) * SENSOR_DIVIDER_GAIN_INV;
        let vcc_5v = if (4.50..=5.50).contains(&vcc_5v_measured) {
            self.vcc_5v_lpf.update(vcc_5v_measured)
        } else {
            self.vcc_5v_lpf.update(5.000)
        };

        // Helper: ratiometric normalized 0..5.000V voltage (cancels 5V LDO drift!)
        let to_ratiometric_5v = |adc: u16| -> f32 {
            let v_harness = Self::adc_to_pin_volts(adc) * SENSOR_DIVIDER_GAIN_INV;
            (v_harness / vcc_5v) * 5.000
        };

        // 2. D17 MAP
        let map_v = to_ratiometric_5v(raw.map_raw);
        debounce_range_fault(
            !(0.10..=4.92).contains(&map_v),
            &mut self.map_bad_ticks,
            FaultFlags::MAP_RANGE,
            faults,
        );
        let map_raw_kpa =
            map_cal.kpa_at_0v + (map_v / 5.0) * (map_cal.kpa_at_5v - map_cal.kpa_at_0v);
        let map_kpa = self.map_lpf.update(map_raw_kpa.clamp(10.0, 450.0));

        // 3. D11 TPS
        let tps_v = to_ratiometric_5v(raw.tps_raw);
        debounce_range_fault(
            !(0.10..=4.92).contains(&tps_v),
            &mut self.tps_bad_ticks,
            FaultFlags::TPS_RANGE,
            faults,
        );
        let tps_span = (self.tps_wot_v - self.tps_closed_v).max(0.5);
        let tps_raw_pct = ((tps_v - self.tps_closed_v) / tps_span * 100.0).clamp(0.0, 100.0);
        let tps_pct = self.tps_lpf.update(tps_raw_pct);

        // 4. D13 ECT & D15 IAT (compensating 32 kΩ parallel divider load!)
        let ect_ratio = (Self::adc_to_pin_volts(raw.ect_raw) * SENSOR_DIVIDER_GAIN_INV) / vcc_5v;
        debounce_range_fault(
            !(0.01..=0.99).contains(&ect_ratio),
            &mut self.ect_bad_ticks,
            FaultFlags::ECT_RANGE,
            faults,
        );
        let ect_c = self
            .ect_lpf
            .update(Self::honda_ntc_temp_c(ect_ratio).clamp(-40.0, 150.0));

        let iat_ratio = (Self::adc_to_pin_volts(raw.iat_raw) * SENSOR_DIVIDER_GAIN_INV) / vcc_5v;
        debounce_range_fault(
            !(0.01..=0.99).contains(&iat_ratio),
            &mut self.iat_bad_ticks,
            FaultFlags::IAT_RANGE,
            faults,
        );
        let iat_c = self
            .iat_lpf
            .update(Self::honda_ntc_temp_c(iat_ratio).clamp(-40.0, 130.0));

        // 5. PC4 IGP Battery Voltage (`100k / 15k` divider)
        let vbatt_raw = Self::adc_to_pin_volts(raw.vbatt_raw) * VBATT_DIVIDER_GAIN_INV;
        let vbatt_v = self.vbatt_lpf.update(vbatt_raw.clamp(0.0, 24.0));

        // 6. PC5 D10 ELD* (Honda ELD: ~4.5V at 0A down to ~0.5V at ~65A)
        let eld_v = to_ratiometric_5v(raw.eld_raw);
        let eld_amps = ((4.60 - eld_v) * (65.0 / 4.10)).clamp(0.0, 80.0);

        SensorSnapshot {
            map_kpa,
            tps_pct,
            ect_c,
            iat_c,
            vbatt_v,
            vcc_5v_v: vcc_5v,
            eld_amps,
            vss_kmh: vss_kmh.max(0.0),
            switches,
        }
    }
}

#[inline]
fn debounce_range_fault(
    out_of_range: bool,
    counter: &mut u8,
    fault_bit: u32,
    faults: &mut FaultFlags,
) {
    if out_of_range {
        *counter = counter.saturating_add(1);
        if *counter >= RANGE_FAULT_DEBOUNCE_TICKS {
            faults.insert(fault_bit);
        }
    } else {
        *counter = 0;
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{FaultFlags, MapSensorCal, RawSensorAdc, SwitchState};

    use super::{ADC_FULL_SCALE, InvertedMcuSwitchPins, SensorProcessor, VREF_VOLTS};

    fn volts_5v_to_adc(v_harness: f32) -> u16 {
        let v_pin = v_harness * 0.625; // 12k/20k divider
        ((v_pin / VREF_VOLTS) * ADC_FULL_SCALE) as u16
    }

    #[test]
    fn sensor_scaling_is_ratiometric_and_accurate() {
        let mut proc = SensorProcessor::new();
        let mut faults = FaultFlags::default();
        let map_cal = MapSensorCal {
            kpa_at_0v: -1.5,
            kpa_at_5v: 182.5,
        };

        // Simulate 90.0 °C Honda FSM ECT (220 Ω NTC in parallel with 32k divider -> 218.50 Ω,
        // divider with 2490 Ω pullup -> V = 5.0 * 218.50 / (2490 + 218.50) = 0.4034 V)
        // and 20.0 °C Honda FSM IAT (2500 Ω NTC in parallel with 32k -> 2318.8 Ω -> 2.411 V)
        let raw = RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00),
            map_raw: volts_5v_to_adc(2.758), // ~100.0 kPa baro
            tps_raw: volts_5v_to_adc(2.475), // 50.0% TPS
            ect_raw: volts_5v_to_adc(0.4034),
            iat_raw: volts_5v_to_adc(2.411),
            vbatt_raw: (((14.0 * 15.0 / 115.0) / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
            eld_raw: volts_5v_to_adc(3.50),
            ..Default::default()
        };

        let snap = proc.process(&raw, SwitchState::default(), 60.0, &map_cal, &mut faults);
        assert!(faults.is_empty(), "Unexpected faults: {:?}", faults);
        assert!((snap.map_kpa - 100.0).abs() < 0.5, "map={}", snap.map_kpa);
        assert!((snap.tps_pct - 50.0).abs() < 0.5, "tps={}", snap.tps_pct);
        assert!((snap.ect_c - 90.0).abs() < 0.5, "ect={}", snap.ect_c);
        assert!((snap.iat_c - 20.0).abs() < 0.5, "iat={}", snap.iat_c);
        assert!((snap.vbatt_v - 14.0).abs() < 0.1, "vbatt={}", snap.vbatt_v);
    }

    #[test]
    fn sensor_processor_cancels_5v_rail_drift_and_flags_range_faults() {
        let map_cal = MapSensorCal {
            kpa_at_0v: -1.5,
            kpa_at_5v: 182.5,
        };
        let mut proc_5v0 = SensorProcessor::new();
        let mut proc_4v75 = SensorProcessor::new();
        let mut faults = FaultFlags::default();

        // Same ratiometric ratios on a 5.00V rail vs a sagging 4.75V rail
        let raw_5v0 = RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00),
            map_raw: volts_5v_to_adc(2.50),
            tps_raw: volts_5v_to_adc(2.475),
            ect_raw: volts_5v_to_adc(0.4034),
            iat_raw: volts_5v_to_adc(2.411),
            ..Default::default()
        };
        let scale = 4.75 / 5.00;
        let raw_4v75 = RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00 * scale),
            map_raw: volts_5v_to_adc(2.50 * scale),
            tps_raw: volts_5v_to_adc(2.475 * scale),
            ect_raw: volts_5v_to_adc(0.4034 * scale),
            iat_raw: volts_5v_to_adc(2.411 * scale),
            ..Default::default()
        };

        let s1 = proc_5v0.process(&raw_5v0, SwitchState::default(), 0.0, &map_cal, &mut faults);
        let s2 = proc_4v75.process(
            &raw_4v75,
            SwitchState::default(),
            0.0,
            &map_cal,
            &mut faults,
        );
        assert!(faults.is_empty());
        assert!((s1.map_kpa - s2.map_kpa).abs() < 0.05);
        assert!((s1.tps_pct - s2.tps_pct).abs() < 0.05);
        assert!((s1.ect_c - s2.ect_c).abs() < 0.05);
        assert!((s1.iat_c - s2.iat_c).abs() < 0.05);

        // Open/shorted sensor channels reject 1 ms glitches and latch after debounce window
        let shorted = RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00),
            map_raw: 0,
            tps_raw: u16::MAX,
            ect_raw: 0,
            iat_raw: u16::MAX,
            ..Default::default()
        };
        let _ = proc_5v0.process(&shorted, SwitchState::default(), 0.0, &map_cal, &mut faults);
        assert!(faults.is_empty());

        // Recovery clears debounce counter so transient spikes never accumulate
        let _ = proc_5v0.process(&raw_5v0, SwitchState::default(), 0.0, &map_cal, &mut faults);
        assert!(faults.is_empty());

        for _ in 0..super::RANGE_FAULT_DEBOUNCE_TICKS {
            let _ = proc_5v0.process(&shorted, SwitchState::default(), 0.0, &map_cal, &mut faults);
        }
        assert!(faults.contains(FaultFlags::MAP_RANGE));
        assert!(faults.contains(FaultFlags::TPS_RANGE));
        assert!(faults.contains(FaultFlags::ECT_RANGE));
        assert!(faults.contains(FaultFlags::IAT_RANGE));
    }

    #[test]
    fn switch_decoder_handles_74hcs14_inversion_and_d6_vtp_pullup() {
        let idle_pins = InvertedMcuSwitchPins {
            pb5_sts_pin_high: true,  // 0V on B9 -> HIGH out of 74HCS14 -> not cranking
            pe4_vtp_pin_high: true,  // 0V on D6 (NC switch closed to GND) -> HIGH out -> VTP off
            pe6_scs_pin_high: false, // 5V pull-up on D4 -> LOW out of 74HCS14 -> jumper open
            pe7_acs_pin_high: false,
            pe8_psp_pin_high: false,
            pe10_bksw_pin_high: true,
            pe12_clutch_pin_high: false,
        };
        let s0 = idle_pins.decode();
        assert!(!s0.sts_cranking);
        assert!(!s0.vtp_pressurized);
        assert!(!s0.scs_service_jumper);

        // VTEC oil gallery pressurizes -> USDM NC switch opens -> D6 HIGH -> 74HCS14 outputs LOW!
        let vtec_pins = InvertedMcuSwitchPins {
            pe4_vtp_pin_high: false,
            ..idle_pins
        };
        assert!(vtec_pins.decode().vtp_pressurized);
    }

    #[test]
    fn four_bar_map_reads_above_320_kpa_up_to_430_kpa() {
        let mut proc_4bar = super::SensorProcessor::new();
        let cal_4bar = MapSensorCal {
            kpa_at_0v: 0.0,
            kpa_at_5v: 430.0,
        };
        let mut faults = FaultFlags::default();
        let raw_400kpa = RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00),
            map_raw: volts_5v_to_adc(400.0 / 430.0 * 5.0),
            tps_raw: volts_5v_to_adc(2.0),
            ect_raw: volts_5v_to_adc(0.45),
            iat_raw: volts_5v_to_adc(2.41),
            vbatt_raw: (((14.0 * 15.0 / 115.0) / super::VREF_VOLTS) * super::ADC_FULL_SCALE) as u16,
            eld_raw: volts_5v_to_adc(3.5),
            ..Default::default()
        };
        let snap = proc_4bar.process(
            &raw_400kpa,
            SwitchState::default(),
            0.0,
            &cal_4bar,
            &mut faults,
        );
        assert!(
            snap.map_kpa > 395.0 && snap.map_kpa < 405.0,
            "got {}",
            snap.map_kpa
        );
        assert!(faults.is_empty());
    }
}
