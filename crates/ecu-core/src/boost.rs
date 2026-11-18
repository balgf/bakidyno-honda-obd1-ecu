//! Electronic boost control (`PE1` / `A19 BOOST`), overboost & lean-boost safety cuts,
//! Flex Fuel boost blending, 2-step launch control, and flat-foot shifting (`PE12` `B7` +
//! `PB11` `B10`).

use crate::math::{PidController, interp_1d_u16};
use ecu_types::{
    BoostConfig, BoostMode, BoostTelemetry, FaultFlags, FlexFuelState, HONDA_FIRING_ORDER,
    SensorSnapshot, SyncState, WidebandState,
};

/// Clamps [`BoostConfig`] parameters to physically safe bounds and preserves monotonic axes
/// and valid hysteresis margins.
pub fn sanitize_boost_config(mut cfg: BoostConfig) -> BoostConfig {
    let finite_or = |v: f32, fallback: f32| if v.is_finite() { v } else { fallback };
    let def = BoostConfig::default_honda_obd1();

    cfg.solenoid_freq_hz = cfg.solenoid_freq_hz.clamp(10, 60);
    cfg.min_duty_per_mille = cfg.min_duty_per_mille.min(950);
    cfg.max_duty_per_mille = cfg.max_duty_per_mille.clamp(cfg.min_duty_per_mille, 1_000);
    cfg.spool_duty_per_mille = cfg.spool_duty_per_mille.min(1_000);
    cfg.spool_margin_kpa = finite_or(cfg.spool_margin_kpa, def.spool_margin_kpa).clamp(5.0, 120.0);

    cfg.min_tps_pct = finite_or(cfg.min_tps_pct, def.min_tps_pct).clamp(0.0, 95.0);
    cfg.full_tps_pct =
        finite_or(cfg.full_tps_pct, def.full_tps_pct).clamp(cfg.min_tps_pct + 1.0, 100.0);
    cfg.min_rpm = cfg.min_rpm.clamp(600, 7_000);
    cfg.min_ect_c = finite_or(cfg.min_ect_c, def.min_ect_c).clamp(-20.0, 110.0);

    let mut prev_rpm = 0u16;
    for bin in &mut cfg.rpm_bins {
        let min_next = prev_rpm.saturating_add(100);
        *bin = (*bin).clamp(min_next.max(500), 11_000);
        prev_rpm = *bin;
    }
    for d in &mut cfg.base_duty_per_mille {
        *d = (*d).min(1_000);
    }
    for p in &mut cfg.target_map_kpa {
        *p = (*p).clamp(100, 440);
    }

    cfg.pid_kp_x100 = cfg.pid_kp_x100.min(2_000);
    cfg.pid_ki_x100 = cfg.pid_ki_x100.min(2_000);
    cfg.pid_kd_x100 = cfg.pid_kd_x100.min(1_000);
    cfg.pid_authority_per_mille = cfg.pid_authority_per_mille.min(600);

    cfg.flex_boost_adder_kpa =
        finite_or(cfg.flex_boost_adder_kpa, def.flex_boost_adder_kpa).clamp(0.0, 150.0);

    cfg.traction_low_vss_kmh =
        finite_or(cfg.traction_low_vss_kmh, def.traction_low_vss_kmh).clamp(0.0, 200.0);
    cfg.traction_high_vss_kmh = finite_or(cfg.traction_high_vss_kmh, def.traction_high_vss_kmh)
        .clamp(cfg.traction_low_vss_kmh + 1.0, 260.0);
    cfg.traction_max_map_kpa =
        finite_or(cfg.traction_max_map_kpa, def.traction_max_map_kpa).clamp(100.0, 440.0);

    cfg.overboost_cut_kpa =
        finite_or(cfg.overboost_cut_kpa, def.overboost_cut_kpa).clamp(115.0, 445.0);
    cfg.overboost_hyst_kpa = finite_or(cfg.overboost_hyst_kpa, def.overboost_hyst_kpa)
        .clamp(2.0, (cfg.overboost_cut_kpa - 100.0).max(2.0));
    cfg.cold_overboost_cut_kpa = finite_or(cfg.cold_overboost_cut_kpa, def.cold_overboost_cut_kpa)
        .clamp(110.0, cfg.overboost_cut_kpa);

    cfg.knock_drop_retard_deg_x10 = cfg.knock_drop_retard_deg_x10.clamp(5, 120);
    cfg.lean_min_map_kpa =
        finite_or(cfg.lean_min_map_kpa, def.lean_min_map_kpa).clamp(105.0, 400.0);
    cfg.lean_max_lambda_err =
        finite_or(cfg.lean_max_lambda_err, def.lean_max_lambda_err).clamp(0.02, 0.30);
    cfg.lean_delay_ms = cfg.lean_delay_ms.clamp(20, 2_000);
    cfg.stft_max_map_kpa = finite_or(cfg.stft_max_map_kpa, def.stft_max_map_kpa).clamp(80.0, 450.0);

    cfg.launch_rpm = cfg.launch_rpm.clamp(1_800, 8_000);
    cfg.launch_window_rpm = cfg.launch_window_rpm.clamp(50, 1_500);
    cfg.launch_max_vss_kmh =
        finite_or(cfg.launch_max_vss_kmh, def.launch_max_vss_kmh).clamp(1.0, 40.0);
    cfg.launch_min_tps_pct =
        finite_or(cfg.launch_min_tps_pct, def.launch_min_tps_pct).clamp(10.0, 95.0);
    cfg.launch_retard_deg_x10 = cfg.launch_retard_deg_x10.clamp(-150, 250);
    cfg.launch_fuel_enrich_pct = cfg.launch_fuel_enrich_pct.min(50);
    cfg.launch_boost_duty_per_mille = cfg.launch_boost_duty_per_mille.min(1_000);

    cfg.ffs_min_rpm = cfg.ffs_min_rpm.clamp(2_500, 9_000);
    cfg.ffs_rpm_drop = cfg.ffs_rpm_drop.clamp(100, 2_000);
    cfg.ffs_min_tps_pct = finite_or(cfg.ffs_min_tps_pct, def.ffs_min_tps_pct).clamp(25.0, 98.0);
    cfg.ffs_retard_deg_x10 = cfg.ffs_retard_deg_x10.clamp(-100, 300);
    cfg.ffs_max_duration_ms = cfg.ffs_max_duration_ms.clamp(100, 3_000);

    cfg
}

/// Combined outputs produced on each `1 kHz` control tick by [`BoostController`].
#[derive(Debug, Clone, Copy)]
pub struct BoostControlOutput {
    pub solenoid_duty_per_mille: u16,
    pub solenoid_on: bool,
    /// Immediately cuts both fuel & spark across all 4 cylinders on overboost / lean-boost fault.
    pub hard_cut_all: bool,
    /// Optional blended spark target override + blend fraction (`0.0..=1.0`) for launch / FFS.
    pub spark_blend_target: Option<(i16, f32)>,
    /// Per-cylinder spark enables (`[Cyl1..Cyl4]`) used by rotating 2-step & FFS spark cuts.
    pub cyl_spark_enabled: [bool; 4],
    /// Volumetric fuel multiplier combining Flex Fuel compensation + 2-step launch enrichment.
    pub fuel_multiplier: f32,
    /// Ignition advance adder (`0.1°`) from Flex Fuel octane trim.
    pub flex_spark_add_deg_x10: i16,
    /// `false` above `stft_max_map_kpa` so wideband never pulls `-25%` fuel under boost.
    pub allow_negative_stft: bool,
    pub telemetry: BoostTelemetry,
}

/// Deterministic `1 kHz` boost control, turbo safety cut, Flex blend, 2-step & FFS state machine.
pub struct BoostController {
    pid: PidController,
    prev_map_kpa: f32,
    d_map_lpf: f32,
    baro_kpa: f32,
    baro_captured: bool,
    baro_settle_ticks: u8,
    peak_boost_kpa: f32,

    overboost_cut_latched: bool,
    overboost_clear_ms: u16,
    lean_fault_ms: u16,
    lean_cut_latched: bool,

    pwm_tick_in_period: u16,
    pwm_on_ticks_this_period: u16,
    pwm_dither_err: f32,

    prev_clutch_pressed: bool,
    ffs_latched: bool,
    ffs_target_rpm: u16,
    ffs_timer_ms: u16,

    cut_phase: usize,
    cut_phase_div: u8,
    latest: BoostTelemetry,
}

impl Default for BoostController {
    fn default() -> Self {
        Self::new()
    }
}

impl BoostController {
    pub const fn new() -> Self {
        Self {
            pid: PidController::new(2.5, 1.8, 0.2, -300.0, 300.0),
            prev_map_kpa: 101.3,
            d_map_lpf: 0.0,
            baro_kpa: 101.3,
            baro_captured: false,
            baro_settle_ticks: 0,
            peak_boost_kpa: 0.0,
            overboost_cut_latched: false,
            overboost_clear_ms: 0,
            lean_fault_ms: 0,
            lean_cut_latched: false,
            pwm_tick_in_period: 0,
            pwm_on_ticks_this_period: 0,
            pwm_dither_err: 0.0,
            prev_clutch_pressed: false,
            ffs_latched: false,
            ffs_target_rpm: 5_000,
            ffs_timer_ms: 0,
            cut_phase: 0,
            cut_phase_div: 0,
            latest: BoostTelemetry::new(),
        }
    }

    #[inline]
    pub const fn telemetry(&self) -> BoostTelemetry {
        self.latest
    }

    /// Resets peak boost hold counter (called on MIL fault clear).
    #[inline]
    pub fn reset_peak_hold(&mut self) {
        self.peak_boost_kpa = 0.0;
    }

    /// Executes one `1 kHz` (`1 ms`) boost, safety-cut, 2-step launch, and FFS evaluation.
    #[allow(clippy::too_many_arguments)]
    pub fn update_1khz(
        &mut self,
        sync_state: SyncState,
        rpm: u16,
        sensors: &SensorSnapshot,
        flex: &FlexFuelState,
        max_knock_retard_deg_x10: i16,
        wbo_state: WidebandState,
        measured_lambda: f32,
        target_lambda: f32,
        faults: &mut FaultFlags,
        cfg: &BoostConfig,
    ) -> BoostControlOutput {
        // 1. Key-on barometric pressure capture before cranking begins.
        let map_fault = faults.contains(FaultFlags::MAP_RANGE);
        if !self.baro_captured
            && rpm == 0
            && !sensors.switches.sts_cranking
            && !map_fault
            && (70.0..=112.0).contains(&sensors.map_kpa)
        {
            self.baro_settle_ticks = self.baro_settle_ticks.saturating_add(1);
            if self.baro_settle_ticks >= 5 {
                self.baro_kpa = sensors.map_kpa;
                self.baro_captured = true;
            }
        }

        let boost_kpa = sensors.map_kpa - self.baro_kpa;
        if rpm > 0 && !map_fault && boost_kpa > self.peak_boost_kpa {
            self.peak_boost_kpa = boost_kpa;
        }

        // 2. Flex Fuel compensation multipliers & octane timing trim.
        let (flex_fuel_mult, flex_spark_add_deg) = if cfg.flex_comp_enabled && flex.is_valid() {
            (
                flex.fuel_volume_multiplier(),
                flex.spark_advance_offset_deg(),
            )
        } else {
            (1.0, 0.0)
        };
        let flex_spark_add_deg_x10 = (flex_spark_add_deg * 10.0) as i16;

        // 3. Overboost and Cold-ECT Boost Safety Cut.
        let cold_limit_active = sensors.ect_c < cfg.min_ect_c;
        let cut_thresh_kpa = if cold_limit_active {
            cfg.cold_overboost_cut_kpa
        } else {
            cfg.overboost_cut_kpa
        };
        if cfg.overboost_cut_enabled && rpm > 0 && !map_fault {
            if sensors.map_kpa >= cut_thresh_kpa {
                self.overboost_cut_latched = true;
                self.overboost_clear_ms = 0;
                faults.insert(FaultFlags::OVERBOOST_CUT);
            } else if self.overboost_cut_latched {
                if sensors.map_kpa <= cut_thresh_kpa - cfg.overboost_hyst_kpa {
                    self.overboost_clear_ms = self.overboost_clear_ms.saturating_add(1);
                    if self.overboost_clear_ms >= 150 {
                        self.overboost_cut_latched = false;
                        self.overboost_clear_ms = 0;
                    }
                } else {
                    self.overboost_clear_ms = 0;
                }
            }
        } else {
            self.overboost_cut_latched = false;
            self.overboost_clear_ms = 0;
        }

        // 4. Lean-Under-Boost Safety Cut (`WidebandState::ClosedLoopReady` required).
        if cfg.lean_prot_enabled
            && wbo_state == WidebandState::ClosedLoopReady
            && rpm >= cfg.min_rpm
            && !map_fault
            && sensors.map_kpa >= cfg.lean_min_map_kpa
        {
            if measured_lambda > target_lambda + cfg.lean_max_lambda_err {
                self.lean_fault_ms = self.lean_fault_ms.saturating_add(1);
                if self.lean_fault_ms >= cfg.lean_delay_ms {
                    self.lean_cut_latched = true;
                    faults.insert(FaultFlags::LEAN_BOOST_CUT);
                }
            } else {
                self.lean_fault_ms = 0;
            }
        } else {
            self.lean_fault_ms = 0;
        }
        if self.lean_cut_latched && (rpm == 0 || sensors.tps_pct < 15.0) {
            self.lean_cut_latched = false;
        }

        let hard_cut_all = self.overboost_cut_latched || self.lean_cut_latched;
        let knock_drop_active =
            cfg.knock_drop_enabled && max_knock_retard_deg_x10 >= cfg.knock_drop_retard_deg_x10;

        // 5. 2-Step Launch Control & Flat-Foot Shift (`PE12 B7 CLUTCH` + `PB11 B10 VSS`).
        let engine_running =
            sync_state == SyncState::FullSynced720 && rpm >= 400 && !sensors.switches.sts_cranking;
        let clutch_now = sensors.switches.clutch_pressed;
        let clutch_rising = clutch_now && !self.prev_clutch_pressed;
        self.prev_clutch_pressed = clutch_now;

        let launch_armed = cfg.launch_enabled
            && engine_running
            && !hard_cut_all
            && clutch_now
            && sensors.vss_kmh <= cfg.launch_max_vss_kmh
            && sensors.tps_pct >= cfg.launch_min_tps_pct
            && rpm >= 1_500;

        if !clutch_now || sensors.tps_pct < cfg.ffs_min_tps_pct || !engine_running || hard_cut_all {
            self.ffs_latched = false;
            self.ffs_timer_ms = 0;
        } else if cfg.ffs_enabled
            && clutch_rising
            && sensors.vss_kmh > cfg.launch_max_vss_kmh
            && rpm >= cfg.ffs_min_rpm
        {
            self.ffs_latched = true;
            self.ffs_target_rpm = rpm.saturating_sub(cfg.ffs_rpm_drop).max(cfg.ffs_min_rpm);
            self.ffs_timer_ms = 0;
        } else if self.ffs_latched {
            self.ffs_timer_ms = self.ffs_timer_ms.saturating_add(1);
            if self.ffs_timer_ms > cfg.ffs_max_duration_ms {
                self.ffs_latched = false;
            }
        }

        let mut spark_blend_target = None;
        let mut cyl_spark_enabled = [true; 4];
        let mut launch_fuel_mult = 1.0f32;
        let launch_active;
        let ffs_active;

        if launch_armed {
            let win = cfg.launch_window_rpm.max(50) as f32;
            let start_rpm = cfg.launch_rpm.saturating_sub(cfg.launch_window_rpm) as f32;
            let ramp = ((rpm as f32 - start_rpm) / win).clamp(0.0, 1.0);
            launch_active = ramp > 0.0;
            ffs_active = false;
            if ramp > 0.0 {
                spark_blend_target = Some((cfg.launch_retard_deg_x10, ramp));
                launch_fuel_mult = 1.0 + (cfg.launch_fuel_enrich_pct as f32 * 0.01 * ramp);
            }
            if rpm >= cfg.launch_rpm {
                let over = (rpm - cfg.launch_rpm) as f32;
                let cut_frac = (0.5 + (over / 150.0) * 0.5).clamp(0.5, 1.0);
                cyl_spark_enabled = self.next_rotating_spark_mask(cut_frac);
            }
        } else if self.ffs_latched {
            launch_active = false;
            ffs_active = true;
            spark_blend_target = Some((cfg.ffs_retard_deg_x10, 1.0));
            if rpm >= self.ffs_target_rpm {
                let over = (rpm - self.ffs_target_rpm) as f32;
                let cut_frac = (0.5 + (over / 200.0) * 0.5).clamp(0.5, 1.0);
                cyl_spark_enabled = self.next_rotating_spark_mask(cut_frac);
            }
        } else {
            launch_active = false;
            ffs_active = false;
        }

        // 6. Electronic Boost Solenoid Target & PID/Feedforward Duty (`PE1` / `A19 BOOST`).
        let mut target_kpa = interp_1d_u16(&cfg.rpm_bins, &cfg.target_map_kpa, rpm as f32);
        if cfg.flex_comp_enabled && flex.is_valid() {
            let eth_frac = ((flex.ethanol_pct - 10.0) / 75.0).clamp(0.0, 1.0);
            target_kpa += cfg.flex_boost_adder_kpa * eth_frac;
        }
        if cfg.traction_boost_enabled {
            let vss_span = (cfg.traction_high_vss_kmh - cfg.traction_low_vss_kmh).max(1.0);
            let t = ((sensors.vss_kmh - cfg.traction_low_vss_kmh) / vss_span).clamp(0.0, 1.0);
            let cap_kpa = cfg.traction_max_map_kpa + (440.0 - cfg.traction_max_map_kpa) * t;
            target_kpa = target_kpa.min(cap_kpa);
        }

        let tps_span = (cfg.full_tps_pct - cfg.min_tps_pct).max(1.0);
        let tps_scale = ((sensors.tps_pct - cfg.min_tps_pct) / tps_span).clamp(0.0, 1.0);

        let boost_allowed = engine_running
            && cfg.mode != BoostMode::Off
            && !hard_cut_all
            && !map_fault
            && !knock_drop_active
            && !cold_limit_active
            && rpm >= cfg.min_rpm
            && sensors.tps_pct >= cfg.min_tps_pct;

        let solenoid_duty_per_mille = if !boost_allowed {
            self.pid.reset();
            self.prev_map_kpa = sensors.map_kpa;
            self.d_map_lpf = 0.0;
            if engine_running
                && cfg.mode != BoostMode::Off
                && launch_active
                && !hard_cut_all
                && !map_fault
                && !cold_limit_active
            {
                cfg.launch_boost_duty_per_mille
                    .clamp(cfg.min_duty_per_mille, cfg.max_duty_per_mille)
            } else {
                0
            }
        } else {
            let base_duty =
                interp_1d_u16(&cfg.rpm_bins, &cfg.base_duty_per_mille, rpm as f32) * tps_scale;
            let raw_duty = match cfg.mode {
                BoostMode::Off => 0.0,
                BoostMode::OpenLoop => base_duty,
                BoostMode::ClosedLoop => {
                    let eff_target = self.baro_kpa + (target_kpa - self.baro_kpa) * tps_scale;
                    let error = eff_target - sensors.map_kpa;
                    if error > cfg.spool_margin_kpa {
                        self.pid.reset();
                        self.prev_map_kpa = sensors.map_kpa;
                        self.d_map_lpf = 0.0;
                        cfg.spool_duty_per_mille as f32
                    } else {
                        let auth = cfg.pid_authority_per_mille as f32;
                        self.pid.kp = cfg.pid_kp_x100 as f32 * 0.01;
                        self.pid.ki = cfg.pid_ki_x100 as f32 * 0.01;
                        self.pid.kd = 0.0;
                        self.pid.out_min = -auth;
                        self.pid.out_max = auth;
                        let pi_term = self.pid.update(error, 0.001);
                        // Filtered derivative on measurement (~20 ms tau) prevents 1 kHz ADC
                        // jitter from chattering the boost solenoid.
                        let raw_d_map = (sensors.map_kpa - self.prev_map_kpa) / 0.001;
                        self.d_map_lpf += 0.05 * (raw_d_map - self.d_map_lpf);
                        let kd = cfg.pid_kd_x100 as f32 * 0.01;
                        self.prev_map_kpa = sensors.map_kpa;
                        let pid_corr = (pi_term - kd * self.d_map_lpf).clamp(-auth, auth);
                        base_duty + pid_corr
                    }
                }
            };
            let clamped =
                raw_duty.clamp(cfg.min_duty_per_mille as f32, cfg.max_duty_per_mille as f32) as u16;
            if launch_active {
                clamped.max(
                    cfg.launch_boost_duty_per_mille
                        .clamp(cfg.min_duty_per_mille, cfg.max_duty_per_mille),
                )
            } else {
                clamped
            }
        };

        let solenoid_on = self.advance_solenoid_pwm(solenoid_duty_per_mille, cfg);
        let fuel_multiplier = (flex_fuel_mult * launch_fuel_mult).clamp(0.80, 2.20);
        let allow_negative_stft = sensors.map_kpa <= cfg.stft_max_map_kpa;

        let telemetry = BoostTelemetry {
            mode: cfg.mode,
            baro_kpa: self.baro_kpa,
            boost_kpa,
            target_kpa,
            solenoid_duty_per_mille,
            solenoid_on,
            overboost_cut_active: self.overboost_cut_latched,
            lean_cut_active: self.lean_cut_latched,
            knock_drop_active,
            cold_limit_active,
            launch_active,
            ffs_active,
            flex_fuel_mult,
            flex_spark_add_deg,
            peak_boost_kpa: self.peak_boost_kpa,
        };
        self.latest = telemetry;

        BoostControlOutput {
            solenoid_duty_per_mille,
            solenoid_on,
            hard_cut_all,
            spark_blend_target,
            cyl_spark_enabled,
            fuel_multiplier,
            flex_spark_add_deg_x10,
            allow_negative_stft,
            telemetry,
        }
    }

    fn next_rotating_spark_mask(&mut self, cut_frac: f32) -> [bool; 4] {
        if cut_frac <= 0.0 {
            return [true; 4];
        }
        if cut_frac >= 0.99 {
            return [false; 4];
        }
        let cut_slots = if cut_frac >= 0.72 {
            3
        } else if cut_frac >= 0.45 {
            2
        } else {
            1
        };
        self.cut_phase_div = self.cut_phase_div.wrapping_add(1);
        if self.cut_phase_div >= 4 {
            self.cut_phase_div = 0;
            self.cut_phase = (self.cut_phase + 1) % 4;
        }
        let mut enabled = [true; 4];
        for i in 0..cut_slots {
            let cyl = HONDA_FIRING_ORDER[(self.cut_phase + i) % 4];
            enabled[cyl.index()] = false;
        }
        enabled
    }

    /// Cycle-coherent low-frequency software PWM (`10..=60 Hz`) with fractional dithering carry
    /// so `PE1` achieves `< 0.3%` average duty precision without mechanical valve chatter.
    fn advance_solenoid_pwm(&mut self, duty_per_mille: u16, cfg: &BoostConfig) -> bool {
        let freq = cfg.solenoid_freq_hz.clamp(10, 60);
        let period_ms = (1_000 / freq).max(16);

        if self.pwm_tick_in_period == 0 {
            let eff_duty = if duty_per_mille == 0 {
                0
            } else if duty_per_mille >= 1_000 {
                1_000
            } else {
                duty_per_mille
            };
            let ideal_on = (eff_duty as f32 * period_ms as f32) * 0.001 + self.pwm_dither_err;
            let on_ticks = (ideal_on as u16).min(period_ms);
            self.pwm_dither_err = (ideal_on - on_ticks as f32).clamp(-1.0, 1.0);
            self.pwm_on_ticks_this_period = on_ticks;
        }

        let raw_on = self.pwm_tick_in_period < self.pwm_on_ticks_this_period;
        self.pwm_tick_in_period = (self.pwm_tick_in_period + 1) % period_ms;
        if cfg.invert_solenoid { !raw_on } else { raw_on }
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{
        BoostConfig, BoostMode, FaultFlags, FlexFuelState, FlexSensorStatus, SensorSnapshot,
        SwitchState, SyncState, WidebandState,
    };

    use super::{BoostController, sanitize_boost_config};

    fn warm_boost_sensors(
        map_kpa: f32,
        tps_pct: f32,
        vss_kmh: f32,
        clutch: bool,
    ) -> SensorSnapshot {
        SensorSnapshot {
            map_kpa,
            tps_pct,
            ect_c: 85.0,
            iat_c: 32.0,
            vbatt_v: 14.0,
            vcc_5v_v: 5.0,
            eld_amps: 10.0,
            vss_kmh,
            switches: SwitchState {
                clutch_pressed: clutch,
                ..Default::default()
            },
        }
    }

    #[test]
    fn closed_loop_spool_assist_flex_blend_traction_cap_and_dithered_pwm() {
        let mut ctrl = BoostController::new();
        let mut cfg = BoostConfig::default_honda_obd1();
        cfg.mode = BoostMode::ClosedLoop;
        cfg.flex_boost_adder_kpa = 40.0;
        cfg.traction_boost_enabled = true;
        cfg.traction_low_vss_kmh = 30.0;
        cfg.traction_high_vss_kmh = 80.0;
        cfg.traction_max_map_kpa = 140.0;
        let cfg = sanitize_boost_config(cfg);

        // Capture key-on baro = 98.5 kPa while stopped
        let ko_sns = warm_boost_sensors(98.5, 0.0, 0.0, false);
        let mut faults = FaultFlags::default();
        let e0 = FlexFuelState::new();
        for _ in 0..10 {
            ctrl.update_1khz(
                SyncState::Unsynced,
                0,
                &ko_sns,
                &e0,
                0,
                WidebandState::WaitForEngine,
                1.0,
                1.0,
                &mut faults,
                &cfg,
            );
        }
        assert!((ctrl.telemetry().baro_kpa - 98.5).abs() < 0.01);

        // Low speed (20 km/h) caps target to traction_max_map_kpa (140 kPa) even with E85
        let e85 = FlexFuelState {
            status: FlexSensorStatus::Valid,
            freq_hz: 135.0,
            ethanol_pct: 85.0,
            fuel_temp_c: 25.0,
            age_ms: 0,
        };
        let low_spd = warm_boost_sensors(105.0, 95.0, 20.0, false);
        let out_low = ctrl.update_1khz(
            SyncState::FullSynced720,
            4_800,
            &low_spd,
            &e85,
            0,
            WidebandState::ClosedLoopReady,
            0.82,
            0.82,
            &mut faults,
            &cfg,
        );
        assert!((out_low.telemetry.target_kpa - 140.0).abs() < 0.5);
        // 140 - 105 = 35 kPa > 25 kPa spool margin -> fast-spool 900‰ duty!
        assert_eq!(out_low.solenoid_duty_per_mille, 900);
        assert!((out_low.fuel_multiplier - 1.41).abs() < 0.03);
        assert_eq!(out_low.flex_spark_add_deg_x10, 40);

        // At 100 km/h with E85 after settling onto 208 kPa, traction cap lifts -> target = 210 kPa!
        let hi_spd = warm_boost_sensors(208.0, 95.0, 100.0, false);
        let mut out_hi = out_low;
        for _ in 0..80 {
            out_hi = ctrl.update_1khz(
                SyncState::FullSynced720,
                4_800,
                &hi_spd,
                &e85,
                0,
                WidebandState::ClosedLoopReady,
                0.80,
                0.80,
                &mut faults,
                &cfg,
            );
        }
        assert!((out_hi.telemetry.target_kpa - 210.0).abs() < 0.5);
        assert!(out_hi.solenoid_duty_per_mille > 400);
        assert!(!out_hi.allow_negative_stft);
        assert!(out_hi.telemetry.peak_boost_kpa > 105.0);
    }

    #[test]
    fn overboost_lean_boost_and_knock_drop_protect_engine() {
        let mut ctrl = BoostController::new();
        let mut cfg = BoostConfig::default_honda_obd1();
        cfg.mode = BoostMode::ClosedLoop;
        cfg.overboost_cut_kpa = 240.0;
        cfg.overboost_hyst_kpa = 20.0;
        cfg.lean_prot_enabled = true;
        cfg.lean_min_map_kpa = 140.0;
        cfg.lean_max_lambda_err = 0.08;
        cfg.lean_delay_ms = 50;
        let mut faults = FaultFlags::default();
        let e0 = FlexFuelState::new();

        // 1. Overboost at 245 kPa latches OVERBOOST_CUT, drops solenoid to 0%, cuts fuel+spark
        let over = warm_boost_sensors(245.0, 95.0, 90.0, false);
        let out = ctrl.update_1khz(
            SyncState::FullSynced720,
            5_500,
            &over,
            &e0,
            0,
            WidebandState::ClosedLoopReady,
            0.80,
            0.80,
            &mut faults,
            &cfg,
        );
        assert!(out.hard_cut_all);
        assert_eq!(out.solenoid_duty_per_mille, 0);
        assert!(faults.contains(FaultFlags::OVERBOOST_CUT));

        // Drops below 220 kPa (240 - 20) for 150 ms -> recovers combustion cleanly
        let safe = warm_boost_sensors(215.0, 95.0, 90.0, false);
        let mut rec = out;
        for _ in 0..160 {
            rec = ctrl.update_1khz(
                SyncState::FullSynced720,
                5_500,
                &safe,
                &e0,
                0,
                WidebandState::ClosedLoopReady,
                0.80,
                0.80,
                &mut faults,
                &cfg,
            );
        }
        assert!(!rec.hard_cut_all);

        // 2. Sustained knock retard >= 3.5 deg drops solenoid to 0% wastegate spring
        let boost_ok = warm_boost_sensors(175.0, 95.0, 90.0, false);
        let knk = ctrl.update_1khz(
            SyncState::FullSynced720,
            5_500,
            &boost_ok,
            &e0,
            40,
            WidebandState::ClosedLoopReady,
            0.80,
            0.80,
            &mut faults,
            &cfg,
        );
        assert!(knk.telemetry.knock_drop_active);
        assert_eq!(knk.solenoid_duty_per_mille, 0);

        // 3. Lean under boost (0.95 vs 0.80 target) for 50 ms latches LEAN_BOOST_CUT
        let mut lean = knk;
        for _ in 0..55 {
            lean = ctrl.update_1khz(
                SyncState::FullSynced720,
                5_500,
                &boost_ok,
                &e0,
                0,
                WidebandState::ClosedLoopReady,
                0.95,
                0.80,
                &mut faults,
                &cfg,
            );
        }
        assert!(lean.hard_cut_all);
        assert!(faults.contains(FaultFlags::LEAN_BOOST_CUT));
    }

    #[test]
    fn two_step_launch_and_flat_foot_shift_manage_spark_retard_and_rotating_cuts() {
        let mut ctrl = BoostController::new();
        let mut cfg = BoostConfig::default_honda_obd1();
        cfg.mode = BoostMode::OpenLoop;
        cfg.launch_enabled = true;
        cfg.launch_rpm = 4_500;
        cfg.launch_window_rpm = 300;
        cfg.launch_retard_deg_x10 = -40;
        cfg.launch_fuel_enrich_pct = 12;
        cfg.ffs_enabled = true;
        cfg.ffs_min_rpm = 5_000;
        cfg.ffs_rpm_drop = 400;
        cfg.ffs_max_duration_ms = 500;
        let mut faults = FaultFlags::default();
        let e0 = FlexFuelState::new();

        // 1. Stationary clutch pressed + WOT @ 4,600 RPM -> 2-step active, rotating spark cut
        let launch_sns = warm_boost_sensors(115.0, 85.0, 0.0, true);
        let l1 = ctrl.update_1khz(
            SyncState::FullSynced720,
            4_600,
            &launch_sns,
            &e0,
            0,
            WidebandState::ClosedLoopReady,
            0.85,
            0.85,
            &mut faults,
            &cfg,
        );
        assert!(l1.telemetry.launch_active);
        assert!(!l1.telemetry.ffs_active);
        assert_eq!(l1.spark_blend_target, Some((-40, 1.0)));
        assert!((l1.fuel_multiplier - 1.12).abs() < 0.01);
        assert!(l1.cyl_spark_enabled.iter().any(|&on| !on));
        assert!(l1.solenoid_duty_per_mille >= cfg.launch_boost_duty_per_mille);

        // 2. Release clutch, roll at 95 km/h, then clutch in @ 7,200 RPM -> FFS holds ~6,800 RPM
        let roll_off = warm_boost_sensors(165.0, 95.0, 95.0, false);
        let _ = ctrl.update_1khz(
            SyncState::FullSynced720,
            7_200,
            &roll_off,
            &e0,
            0,
            WidebandState::ClosedLoopReady,
            0.82,
            0.82,
            &mut faults,
            &cfg,
        );
        let ffs_sns = warm_boost_sensors(160.0, 95.0, 95.0, true);
        let ffs = ctrl.update_1khz(
            SyncState::FullSynced720,
            7_200,
            &ffs_sns,
            &e0,
            0,
            WidebandState::ClosedLoopReady,
            0.82,
            0.82,
            &mut faults,
            &cfg,
        );
        assert!(ffs.telemetry.ffs_active);
        assert!(!ffs.telemetry.launch_active);
        assert_eq!(ffs.spark_blend_target, Some((50, 1.0)));
        assert!(ffs.cyl_spark_enabled.iter().any(|&on| !on));
    }
}
