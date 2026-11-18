//! Vehicle auxiliary I/O control and frequency/duty input decoders for Honda OBD1:
//! - `PD8` (`A16 ALTC*`): Honda dual-mode Denso alternator load control.
//! - `PD9` (`A17 IAB*`): P72 dual-runner intake air bypass butterfly solenoid.
//! - `PD10` (`A20 EVAP*`): Evaporative emission canister purge solenoid (`10 Hz` software PWM).
//! - `PE0` (`A11 EGR* / A18`) & `PE1` (`A19 VAR*`): Configurable spare/warning outputs.
//! - `PB7` (`D9 ALTF*`, `EXTI7`): Alternator `FR` field duty-cycle & frequency decoder.
//! - `PE15` (`Aux Header FLEX`, `EXTI15`): Continental GM `50–150 Hz` Flex Fuel ethanol &
//!   fuel temperature decoder.

use ecu_types::SensorSnapshot;
pub use ecu_types::{
    AltfState, AuxIoConfig, AuxIoTelemetry, AuxOutputs, AuxSpareMode, FlexFuelState,
    FlexSensorStatus,
};

const NOISE_GLITCH_MIN_US: u32 = 50;
const EVAP_PWM_PERIOD_MS: u16 = 100;
const FLEX_MIN_HZ: f32 = 45.0;
const FLEX_MAX_HZ: f32 = 155.0;
const FLEX_PULSE_MIN_US: f32 = 1_000.0;
const FLEX_PULSE_MAX_US: f32 = 5_000.0;
const FLEX_TEMP_MIN_C: f32 = -40.0;
const FLEX_TEMP_SPAN_C: f32 = 165.0;
const ALTF_MIN_HZ: f32 = 30.0;
const ALTF_MAX_HZ: f32 = 500.0;

/// Clamps [`AuxIoConfig`] thresholds to safe bounds while preserving valid hysteresis margins.
pub fn sanitize_aux_config(mut cfg: AuxIoConfig) -> AuxIoConfig {
    let finite_or = |v: f32, fallback: f32| if v.is_finite() { v } else { fallback };
    let def = AuxIoConfig::default_honda_obd1();

    cfg.altc_eco_vbatt_on_v =
        finite_or(cfg.altc_eco_vbatt_on_v, def.altc_eco_vbatt_on_v).clamp(11.5, 16.0);
    cfg.altc_eco_vbatt_off_v = finite_or(cfg.altc_eco_vbatt_off_v, def.altc_eco_vbatt_off_v)
        .clamp(10.5, cfg.altc_eco_vbatt_on_v - 0.1);
    cfg.altc_max_eld_amps =
        finite_or(cfg.altc_max_eld_amps, def.altc_max_eld_amps).clamp(1.0, 80.0);
    cfg.altc_eld_hyst_amps = finite_or(cfg.altc_eld_hyst_amps, def.altc_eld_hyst_amps)
        .clamp(cfg.altc_max_eld_amps, 85.0);
    cfg.altc_wot_unload_tps_pct =
        finite_or(cfg.altc_wot_unload_tps_pct, def.altc_wot_unload_tps_pct).clamp(20.0, 100.0);

    cfg.iab_engage_rpm = cfg.iab_engage_rpm.clamp(1_500, 9_000);
    cfg.iab_disengage_rpm = cfg.iab_disengage_rpm.clamp(1_000, cfg.iab_engage_rpm);

    cfg.evap_min_ect_c = finite_or(cfg.evap_min_ect_c, def.evap_min_ect_c).clamp(-20.0, 110.0);
    cfg.evap_min_rpm = cfg.evap_min_rpm.clamp(400, 5_000);
    cfg.flex_timeout_ms = cfg.flex_timeout_ms.clamp(50, 5_000);
    cfg.altf_timeout_ms = cfg.altf_timeout_ms.clamp(50, 5_000);
    cfg
}

/// Edge-timestamped period + pulse-width tracker using a `1.000 MHz` wrapping `u32` timer.
#[derive(Debug, Clone, Copy, Default)]
pub struct EdgePulseCapture {
    has_rise: bool,
    has_fall: bool,
    has_edge: bool,
    last_rise_us: u32,
    last_fall_us: u32,
    last_edge_us: u32,
    last_high_us: u32,
    last_low_us: u32,
}

impl EdgePulseCapture {
    /// Creates an empty capture state ready for the first rising/falling edge.
    pub const fn new() -> Self {
        Self {
            has_rise: false,
            has_fall: false,
            has_edge: false,
            last_rise_us: 0,
            last_fall_us: 0,
            last_edge_us: 0,
            last_high_us: 0,
            last_low_us: 0,
        }
    }

    /// Records a single pin transition at `now_us` (`1 MHz`) and returns `(period_us, pulse_us)`
    /// once a full period completes.
    pub fn on_edge(
        &mut self,
        is_rising: bool,
        now_us: u32,
        active_low_pulse: bool,
    ) -> Option<(u32, u32)> {
        if self.has_edge && now_us.wrapping_sub(self.last_edge_us) < NOISE_GLITCH_MIN_US {
            return None;
        }
        self.has_edge = true;
        self.last_edge_us = now_us;

        if is_rising {
            if self.has_fall {
                self.last_low_us = now_us.wrapping_sub(self.last_fall_us);
            }
            let out = if self.has_rise {
                let period_us = now_us.wrapping_sub(self.last_rise_us);
                let pulse_us = if active_low_pulse {
                    self.last_low_us
                } else {
                    self.last_high_us
                };
                (period_us > 0).then_some((period_us, pulse_us))
            } else {
                None
            };
            self.last_rise_us = now_us;
            self.has_rise = true;
            out
        } else {
            if self.has_rise {
                self.last_high_us = now_us.wrapping_sub(self.last_rise_us);
            }
            self.last_fall_us = now_us;
            self.has_fall = true;
            None
        }
    }
}

#[derive(Debug, Clone)]
pub struct FlexFuelDecoder {
    state: FlexFuelState,
    ever_valid: bool,
}

impl Default for FlexFuelDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FlexFuelDecoder {
    pub const fn new() -> Self {
        Self {
            state: FlexFuelState::new(),
            ever_valid: false,
        }
    }

    /// Advances the Flex Fuel decoder on each `1 kHz` control tick.
    pub fn update_1khz(&mut self, sample: Option<(u32, u32)>, cfg: &AuxIoConfig) -> FlexFuelState {
        if let Some((period_us, pulse_us)) = sample {
            let freq_hz = 1_000_000.0 / period_us.max(1) as f32;
            if (FLEX_MIN_HZ..=FLEX_MAX_HZ).contains(&freq_hz) {
                let raw_eth = (freq_hz - 50.0).clamp(0.0, 100.0);
                let raw_temp = (FLEX_TEMP_MIN_C
                    + ((pulse_us as f32 - FLEX_PULSE_MIN_US)
                        / (FLEX_PULSE_MAX_US - FLEX_PULSE_MIN_US))
                        * FLEX_TEMP_SPAN_C)
                    .clamp(FLEX_TEMP_MIN_C, 125.0);

                if !self.ever_valid {
                    self.state.ethanol_pct = raw_eth;
                    self.state.fuel_temp_c = raw_temp;
                    self.ever_valid = true;
                } else {
                    self.state.ethanol_pct = 0.75 * self.state.ethanol_pct + 0.25 * raw_eth;
                    self.state.fuel_temp_c = 0.75 * self.state.fuel_temp_c + 0.25 * raw_temp;
                }
                self.state.freq_hz = freq_hz;
                self.state.status = FlexSensorStatus::Valid;
                self.state.age_ms = 0;
            } else {
                self.state.freq_hz = freq_hz;
                self.state.ethanol_pct = 0.0;
                self.state.fuel_temp_c = 20.0;
                self.state.status = FlexSensorStatus::OutOfRangeFault;
                self.state.age_ms = 0;
                self.ever_valid = false;
            }
        } else if self.state.status != FlexSensorStatus::NotDetected {
            self.state.age_ms = self.state.age_ms.saturating_add(1);
            if self.state.age_ms >= cfg.flex_timeout_ms {
                self.state = FlexFuelState::new();
                self.ever_valid = false;
            }
        }

        self.state
    }
}

#[derive(Debug, Clone)]
pub struct AltfDecoder {
    state: AltfState,
    ever_valid: bool,
}

impl Default for AltfDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl AltfDecoder {
    pub const fn new() -> Self {
        Self {
            state: AltfState::new(),
            ever_valid: false,
        }
    }

    /// Advances the `ALTF*` field load decoder on each `1 kHz` control tick.
    pub fn update_1khz(&mut self, sample: Option<(u32, u32)>, cfg: &AuxIoConfig) -> AltfState {
        if let Some((period_us, pulse_us)) = sample {
            let freq_hz = 1_000_000.0 / period_us.max(1) as f32;
            if (ALTF_MIN_HZ..=ALTF_MAX_HZ).contains(&freq_hz) {
                let raw_duty = ((pulse_us as f32 * 100.0) / period_us as f32).clamp(0.0, 100.0);
                if !self.ever_valid {
                    self.state.field_duty_pct = raw_duty;
                    self.ever_valid = true;
                } else {
                    self.state.field_duty_pct = 0.75 * self.state.field_duty_pct + 0.25 * raw_duty;
                }
                self.state.freq_hz = freq_hz;
                self.state.valid = true;
                self.state.age_ms = 0;
            }
        } else if self.state.valid {
            self.state.age_ms = self.state.age_ms.saturating_add(1);
            if self.state.age_ms >= cfg.altf_timeout_ms {
                self.state = AltfState::new();
                self.ever_valid = false;
            }
        }

        self.state
    }
}

/// Deterministic `1 kHz` controller for Honda OBD1 auxiliary outputs and frequency inputs.
#[derive(Debug, Clone)]
pub struct AuxIoController {
    altc_eco_latched: bool,
    iab_high_latched: bool,
    pe0_latched: bool,
    pe1_latched: bool,
    evap_phase_ms: u16,
    flex_decoder: FlexFuelDecoder,
    altf_decoder: AltfDecoder,
    latest: AuxIoTelemetry,
}

impl Default for AuxIoController {
    fn default() -> Self {
        Self::new()
    }
}

impl AuxIoController {
    pub const fn new() -> Self {
        Self {
            altc_eco_latched: false,
            iab_high_latched: false,
            pe0_latched: false,
            pe1_latched: false,
            evap_phase_ms: 0,
            flex_decoder: FlexFuelDecoder::new(),
            altf_decoder: AltfDecoder::new(),
            latest: AuxIoTelemetry {
                outputs: AuxOutputs {
                    altc_on: false,
                    iab_on: false,
                    evap_on: false,
                    evap_duty_per_mille: 0,
                    pe0_on: false,
                    pe1_on: false,
                },
                flex: FlexFuelState::new(),
                altf: AltfState::new(),
            },
        }
    }

    #[inline]
    pub const fn snapshot(&self) -> AuxIoTelemetry {
        self.latest
    }

    /// Evaluates `ALTC*`, `IAB*`, `EVAP*`, `PE0`/`PE1`, `ALTF*`, and `FLEX` at `1 kHz`.
    #[allow(clippy::too_many_arguments)]
    pub fn update_1khz(
        &mut self,
        rpm: u16,
        sensors: &SensorSnapshot,
        wideband_closed_loop: bool,
        flex_sample: Option<(u32, u32)>,
        altf_sample: Option<(u32, u32)>,
        cfg: &AuxIoConfig,
    ) -> AuxIoTelemetry {
        let flex = self.flex_decoder.update_1khz(flex_sample, cfg);
        let altf = self.altf_decoder.update_1khz(altf_sample, cfg);

        let altc_on = self.update_altc(rpm, sensors, cfg);
        let iab_on = self.update_iab(rpm, cfg);
        let (evap_duty_per_mille, evap_on) =
            self.update_evap(rpm, sensors, wideband_closed_loop, cfg);
        let pe0_on = eval_spare_mode(cfg.pe0_mode, rpm, sensors, &mut self.pe0_latched);
        let pe1_on = eval_spare_mode(cfg.pe1_mode, rpm, sensors, &mut self.pe1_latched);

        let telemetry = AuxIoTelemetry {
            outputs: AuxOutputs {
                altc_on,
                iab_on,
                evap_on,
                evap_duty_per_mille,
                pe0_on,
                pe1_on,
            },
            flex,
            altf,
        };
        self.latest = telemetry;
        telemetry
    }

    fn update_altc(&mut self, rpm: u16, sensors: &SensorSnapshot, cfg: &AuxIoConfig) -> bool {
        if !cfg.altc_enabled || rpm < 500 || sensors.ect_c < 40.0 {
            self.altc_eco_latched = false;
            return false;
        }

        // Unload alternator cleanly at heavy WOT as long as battery isn't critically depleted.
        if sensors.tps_pct >= cfg.altc_wot_unload_tps_pct && sensors.vbatt_v >= 12.2 {
            self.altc_eco_latched = true;
            return true;
        }

        if self.altc_eco_latched {
            if sensors.vbatt_v < cfg.altc_eco_vbatt_off_v
                || sensors.eld_amps > cfg.altc_eld_hyst_amps
            {
                self.altc_eco_latched = false;
            }
        } else if sensors.vbatt_v >= cfg.altc_eco_vbatt_on_v
            && sensors.eld_amps <= cfg.altc_max_eld_amps
        {
            self.altc_eco_latched = true;
        }

        self.altc_eco_latched
    }

    fn update_iab(&mut self, rpm: u16, cfg: &AuxIoConfig) -> bool {
        if !cfg.iab_enabled || rpm < 400 {
            self.iab_high_latched = false;
            return false;
        }

        if self.iab_high_latched {
            if rpm < cfg.iab_disengage_rpm {
                self.iab_high_latched = false;
            }
        } else if rpm >= cfg.iab_engage_rpm {
            self.iab_high_latched = true;
        }

        if cfg.iab_invert {
            !self.iab_high_latched
        } else {
            self.iab_high_latched
        }
    }

    fn update_evap(
        &mut self,
        rpm: u16,
        sensors: &SensorSnapshot,
        wideband_closed_loop: bool,
        cfg: &AuxIoConfig,
    ) -> (u16, bool) {
        self.evap_phase_ms = (self.evap_phase_ms + 1) % EVAP_PWM_PERIOD_MS;

        let purge_allowed = cfg.evap_enabled
            && rpm >= cfg.evap_min_rpm
            && sensors.ect_c >= cfg.evap_min_ect_c
            && sensors.tps_pct >= 2.0
            && sensors.tps_pct <= 70.0
            && sensors.map_kpa >= 30.0
            && sensors.map_kpa <= 90.0
            && wideband_closed_loop;

        if !purge_allowed {
            return (0, false);
        }

        // Smooth load/RPM-scaled purge duty cycle (100..=500‰ across moderate cruise window).
        let load_frac = ((sensors.map_kpa - 30.0) / 50.0).clamp(0.0, 1.0);
        let duty_per_mille = (100.0 + load_frac * 400.0) as u16;
        let on_ms = (duty_per_mille * EVAP_PWM_PERIOD_MS) / 1_000;
        (duty_per_mille, self.evap_phase_ms < on_ms)
    }
}

fn eval_spare_mode(
    mode: AuxSpareMode,
    rpm: u16,
    sensors: &SensorSnapshot,
    latched: &mut bool,
) -> bool {
    match mode {
        AuxSpareMode::Disabled => {
            *latched = false;
            false
        }
        AuxSpareMode::ShiftLight {
            engage_rpm,
            disengage_rpm,
        } => {
            if *latched {
                if rpm < disengage_rpm {
                    *latched = false;
                }
            } else if rpm >= engage_rpm {
                *latched = true;
            }
            *latched
        }
        AuxSpareMode::WotTrigger {
            min_tps_pct,
            min_rpm,
        } => {
            let active = rpm >= min_rpm && sensors.tps_pct >= min_tps_pct;
            *latched = active;
            active
        }
        AuxSpareMode::CoolantFanBackup { on_c, off_c } => {
            if *latched {
                if sensors.ect_c <= off_c {
                    *latched = false;
                }
            } else if sensors.ect_c >= on_c {
                *latched = true;
            }
            *latched
        }
        AuxSpareMode::OverTempWarning { warn_c, clear_c } => {
            if *latched {
                if sensors.ect_c <= clear_c {
                    *latched = false;
                }
            } else if sensors.ect_c >= warn_c {
                *latched = true;
            }
            *latched
        }
        AuxSpareMode::BoostTrigger { on_kpa, off_kpa } => {
            let low_kpa = off_kpa.min(on_kpa);
            if *latched {
                if sensors.map_kpa <= low_kpa {
                    *latched = false;
                }
            } else if sensors.map_kpa >= on_kpa {
                *latched = true;
            }
            *latched
        }
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::SensorSnapshot;

    use super::{
        AltfDecoder, AuxIoConfig, AuxIoController, AuxSpareMode, EdgePulseCapture, FlexFuelDecoder,
        FlexSensorStatus,
    };

    #[test]
    fn flex_fuel_decoder_tracks_continental_50_to_150hz_and_timeout() {
        let cfg = AuxIoConfig::default_honda_obd1();
        let mut dec = FlexFuelDecoder::new();

        // E85 = 135.0 Hz (`7407 µs` period), ~60 °C (`3424 µs` pulse)
        let period_us = (1_000_000.0 / 135.0) as u32;
        let pulse_us = (1_000.0 + ((60.0 + 40.0) / 165.0) * 4_000.0) as u32;
        let st = dec.update_1khz(Some((period_us, pulse_us)), &cfg);
        assert_eq!(st.status, FlexSensorStatus::Valid);
        assert!((st.ethanol_pct - 85.0).abs() < 0.2);
        assert!((st.fuel_temp_c - 60.0).abs() < 0.5);
        assert!((st.fuel_volume_multiplier() - 1.41).abs() < 0.03);
        assert!((st.spark_advance_offset_deg() - 4.0).abs() < 0.05);

        // Out-of-range contamination frequency (`180 Hz`) raises fault
        let fault = dec.update_1khz(Some((5_555, 2_000)), &cfg);
        assert_eq!(fault.status, FlexSensorStatus::OutOfRangeFault);
        assert!((fault.fuel_volume_multiplier() - 1.0).abs() < 0.001);

        // Timeout without edges restores safe NotDetected E0 baseline
        let mut after = fault;
        for _ in 0..cfg.flex_timeout_ms {
            after = dec.update_1khz(None, &cfg);
        }
        assert_eq!(after.status, FlexSensorStatus::NotDetected);
        assert!((after.ethanol_pct - 0.0).abs() < 0.001);
    }

    #[test]
    fn edge_pulse_capture_handles_u32_wraparound_and_glitch_filter() {
        let mut cap = EdgePulseCapture::new();
        let t0 = u32::MAX - 2_000;
        assert_eq!(cap.on_edge(true, t0, false), None);
        // Glitch (`20 µs`) ignored
        assert_eq!(cap.on_edge(false, t0.wrapping_add(20), false), None);
        // Valid falling edge `2500 µs` after rising (`3000 µs` -> across wraparound!)
        assert_eq!(cap.on_edge(false, t0.wrapping_add(2_500), false), None);
        let sample = cap
            .on_edge(true, t0.wrapping_add(10_000), false)
            .expect("period completes on second rise");
        assert_eq!(sample, (10_000, 2_500));
    }

    #[test]
    fn altc_iab_evap_and_spare_outputs_follow_hysteresis_and_safety_gates() {
        let mut cfg = AuxIoConfig::default_honda_obd1();
        cfg.pe0_mode = AuxSpareMode::ShiftLight {
            engage_rpm: 7_000,
            disengage_rpm: 6_600,
        };
        cfg.pe1_mode = AuxSpareMode::OverTempWarning {
            warn_c: 105.0,
            clear_c: 98.0,
        };
        let mut ctrl = AuxIoController::new();

        // Key-ON, engine OFF -> every output strictly OFF (`0`)
        let warm_idle = SensorSnapshot {
            ect_c: 85.0,
            vbatt_v: 14.1,
            eld_amps: 8.0,
            tps_pct: 15.0,
            map_kpa: 55.0,
            ..Default::default()
        };
        let ko = ctrl.update_1khz(0, &warm_idle, true, None, None, &cfg);
        assert_eq!(ko.outputs.bitmask(), 0);

        // Cruise 3000 RPM warm with low ELD -> ALTC Eco active, EVAP pulsing, IAB off
        let cruise = ctrl.update_1khz(3_000, &warm_idle, true, None, None, &cfg);
        assert!(cruise.outputs.altc_on);
        assert!(!cruise.outputs.iab_on);
        assert!(cruise.outputs.evap_duty_per_mille >= 100);
        assert!(!cruise.outputs.pe0_on);

        // High ELD (>22 A) drops ALTC back to 14.4V charging (`altc_on = false`)
        let heavy_eld = SensorSnapshot {
            eld_amps: 26.0,
            ..warm_idle
        };
        let high_load = ctrl.update_1khz(3_000, &heavy_eld, true, None, None, &cfg);
        assert!(!high_load.outputs.altc_on);

        // High RPM (`7100 RPM`) + hot ECT (`106 °C`) triggers IAB, ShiftLight (PE0), OverTemp (PE1)
        let hot = SensorSnapshot {
            ect_c: 106.0,
            tps_pct: 90.0,
            map_kpa: 98.0,
            ..warm_idle
        };
        let high_rpm = ctrl.update_1khz(7_100, &hot, true, None, None, &cfg);
        assert!(high_rpm.outputs.iab_on);
        assert!(high_rpm.outputs.altc_on); // WOT unload!
        assert!(high_rpm.outputs.pe0_on);
        assert!(high_rpm.outputs.pe1_on);
        assert!(!high_rpm.outputs.evap_on); // No EVAP under WOT!

        // Drop RPM to 5600 (inside IAB hysteresis) & ECT to 100 (inside PE1 hysteresis)
        let mid_rpm = ctrl.update_1khz(
            5_600,
            &SensorSnapshot {
                ect_c: 100.0,
                ..warm_idle
            },
            true,
            None,
            None,
            &cfg,
        );
        assert!(mid_rpm.outputs.iab_on);
        assert!(!mid_rpm.outputs.pe0_on);
        assert!(mid_rpm.outputs.pe1_on);
    }

    #[test]
    fn altf_decoder_computes_field_duty_and_handles_staleness() {
        let cfg = AuxIoConfig::default_honda_obd1();
        let mut altf = AltfDecoder::new();
        // 200 Hz (`5000 µs` period), `3500 µs` high = 70% alternator field duty
        let st = altf.update_1khz(Some((5_000, 3_500)), &cfg);
        assert!(st.valid);
        assert!((st.freq_hz - 200.0).abs() < 0.2);
        assert!((st.field_duty_pct - 70.0).abs() < 0.2);

        let mut stale = st;
        for _ in 0..cfg.altf_timeout_ms {
            stale = altf.update_1khz(None, &cfg);
        }
        assert!(!stale.valid);
        assert!((stale.field_duty_pct - 0.0).abs() < 0.001);
    }

    #[test]
    fn wot_trigger_coolant_backup_iab_invert_and_active_low_pulse_work() {
        let mut cap = EdgePulseCapture::new();
        assert_eq!(cap.on_edge(true, 1_000, true), None);
        assert_eq!(cap.on_edge(false, 4_000, true), None);
        // Rising at 11,000 µs -> period = 10,000 µs, low pulse = 11,000 - 4,000 = 7,000 µs
        assert_eq!(cap.on_edge(true, 11_000, true), Some((10_000, 7_000)));

        let mut cfg = AuxIoConfig::default_honda_obd1();
        cfg.iab_invert = true;
        cfg.pe0_mode = AuxSpareMode::WotTrigger {
            min_tps_pct: 80.0,
            min_rpm: 4_000,
        };
        cfg.pe1_mode = AuxSpareMode::CoolantFanBackup {
            on_c: 96.0,
            off_c: 91.0,
        };

        let mut ctrl = AuxIoController::new();
        let sns = SensorSnapshot {
            ect_c: 97.0,
            tps_pct: 85.0,
            map_kpa: 99.0,
            vbatt_v: 14.0,
            ..Default::default()
        };

        // Below IAB engage (4,500 < 5,750), iab_invert=true drives IAB solenoid ON to hold
        // butterflies closed
        let t1 = ctrl.update_1khz(4_500, &sns, true, None, None, &cfg);
        assert!(t1.outputs.iab_on);
        assert!(t1.outputs.pe0_on);
        assert!(t1.outputs.pe1_on);

        // Above IAB engage (6,000 >= 5,750), iab_invert=true releases solenoid (`iab_on = false`)
        // Coolant at 93 °C stays latched in CoolantFanBackup hysteresis (91..96 °C)
        let t2 = ctrl.update_1khz(
            6_000,
            &SensorSnapshot {
                ect_c: 93.0,
                tps_pct: 50.0,
                ..sns
            },
            true,
            None,
            None,
            &cfg,
        );
        assert!(!t2.outputs.iab_on);
        assert!(!t2.outputs.pe0_on);
        assert!(t2.outputs.pe1_on);

        // BoostTrigger hysteresis on PE0 (on >= 140 kPa, stays latched at 130 kPa, off <= 125 kPa)
        cfg.pe0_mode = AuxSpareMode::BoostTrigger {
            on_kpa: 140.0,
            off_kpa: 125.0,
        };
        let b1 = ctrl.update_1khz(
            4_500,
            &SensorSnapshot {
                map_kpa: 145.0,
                ..sns
            },
            true,
            None,
            None,
            &cfg,
        );
        assert!(b1.outputs.pe0_on);
        let b2 = ctrl.update_1khz(
            4_500,
            &SensorSnapshot {
                map_kpa: 130.0,
                ..sns
            },
            true,
            None,
            None,
            &cfg,
        );
        assert!(b2.outputs.pe0_on);
        let b3 = ctrl.update_1khz(
            4_500,
            &SensorSnapshot {
                map_kpa: 120.0,
                ..sns
            },
            true,
            None,
            None,
            &cfg,
        );
        assert!(!b3.outputs.pe0_on);
    }
}
