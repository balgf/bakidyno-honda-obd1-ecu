//! Pure `no_std` crank-windowed Knock DSP for the Honda Civic OBD1 `STM32H743` ECU.
//!
//! Hardware signal chain (`_DOCS/hardware_bom.md` & `_DOCS/stm32h743_pinout.md`):
//! - Sensor input: OEM Honda resonant piezo knock sensor (`B11`/`D3`) -> analog front-end ->
//!   `PC2_C` (`ADC3_INP0`, `SYSCFG_PMCR.PC2SO = 1`).
//! - Sampling: `200.000 kS/s` `16-bit` (`ADC3` paced by `TIM6_TRGO`, transferred via `BDMA_CH0` into
//!   `D3` `SRAM4`) inside each cylinder's crank-angle combustion window (`1-3-4-2`).
//! - Filtering: `RBJ` constant-0dB-peak 2nd-order biquad bandpass tuned to `~7.5 kHz` (`Q = 4.0`).
//! - Adaptation & Action: 8-bin RPM noise floor tracker, per-cylinder attack retard + timed decay
//!   feeding [`crate::ignition::IgnitionModel`], and Honda Code 23 (`FaultFlags::KNOCK_SENSOR`).

use ecu_types::{
    CYLINDER_TDC_DEG_X10, Cylinder, FaultFlags, HONDA_FIRING_ORDER, KnockConfig, KnockTelemetry,
    SyncState,
};

use crate::math::{sin_cos_f32, sqrt_f32};
use crate::trigger::{
    CkpToothEvent, DEG_X10_PER_CKP_TOOTH, angle_delta_to_delay_us, angle_offset_in_tooth_window,
};

/// Exact `ADC3` hardware conversion rate (`240 MHz / 1200 = 200.000 kHz`).
pub const KNOCK_SAMPLE_RATE_HZ: u32 = 200_000;
/// Minimum valid burst sample count for meaningful bandpass RMS estimation.
pub const MIN_KNOCK_WINDOW_SAMPLES: usize = 16;
/// Maximum burst capture length (`1024` samples @ `200 kS/s` = `5.12 ms`).
pub const MAX_KNOCK_WINDOW_SAMPLES: usize = 1_024;
/// RPM breakpoints for the adaptive background noise floor curve.
pub const KNOCK_RPM_BINS: [u16; 8] = [1_000, 2_000, 3_000, 4_000, 5_000, 6_000, 7_000, 8_000];

/// Knock capture window scheduled from a `30.0°` `CKP` tooth edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KnockWindowPlan {
    pub cylinder: Cylinder,
    /// Hardware `TIM2->CNT` (`1 MHz`) timestamp of the parent `CKP` edge.
    pub tooth_timestamp_us: u32,
    /// Delay (`µs`) from `tooth_timestamp_us` until window opens (`TDC + window_start`).
    pub delay_from_tooth_us: u16,
    /// Window duration (`µs`).
    pub duration_us: u32,
    /// Exact number of `200 kS/s` ADC samples to capture via `BDMA_CH0`.
    pub sample_count: u16,
    pub rpm: u16,
}

/// Constant-0dB-peak RBJ 2nd-order IIR bandpass filter (`b1 = 0`, `b2 = -b0`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KnockBiquad {
    pub b0: f32,
    pub a1: f32,
    pub a2: f32,
}

impl KnockBiquad {
    /// Precomputed RBJ constant-peak-gain bandpass coefficients for `f0 = 7500 Hz`, `Q = 4.0`,
    /// `fs = 200000 Hz`.
    pub const DEFAULT_7500HZ: Self = Self {
        b0: 0.028_353_302,
        a1: -1.889_600_3,
        a2: 0.943_293_4,
    };

    /// Computes normalized 2nd-order RBJ constant-0dB-peak bandpass coefficients.
    pub fn design(center_freq_hz: u16, q_x10: u16, sample_rate_hz: u32) -> Self {
        let fs = (sample_rate_hz.max(10_000)) as f32;
        let f0 = (center_freq_hz as f32).clamp(500.0, fs * 0.40);
        let q = ((q_x10 as f32) / 10.0).clamp(0.5, 20.0);

        let omega0 = core::f32::consts::TAU * (f0 / fs);
        let (sin_w, cos_w) = sin_cos_f32(omega0);
        let alpha = sin_w / (2.0 * q);
        let a0 = 1.0 + alpha;

        Self {
            b0: alpha / a0,
            a1: (-2.0 * cos_w) / a0,
            a2: (1.0 - alpha) / a0,
        }
    }

    /// Removes window DC offset and returns `(dc_mean_counts, bandpass_rms_counts)`.
    pub fn measure_bandpass_rms(&self, samples: &[u16]) -> (u16, f32) {
        if samples.is_empty() {
            return (0, 0.0);
        }

        let mut sum: u64 = 0;
        for &s in samples {
            sum += s as u64;
        }
        let n = samples.len();
        let mean = (sum as f32) / (n as f32);
        let dc_mean_counts = (sum / (n as u64)).min(u16::MAX as u64) as u16;

        // Skip transient startup settling on longer captures so DC step/phase discontinuities at
        // sample 0 do not inflate out-of-band energy.
        let settle = if n > 48 { (n / 6).min(32) } else { 0 };

        let mut x1 = 0.0_f32;
        let mut x2 = 0.0_f32;
        let mut y1 = 0.0_f32;
        let mut y2 = 0.0_f32;
        let mut sum_sq = 0.0_f32;
        let mut counted = 0usize;

        for (idx, &raw) in samples.iter().enumerate() {
            let x = (raw as f32) - mean;
            let y = self.b0 * (x - x2) - self.a1 * y1 - self.a2 * y2;
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;

            if idx >= settle {
                sum_sq += y * y;
                counted += 1;
            }
        }

        let rms = if counted > 0 {
            sqrt_f32(sum_sq / (counted as f32))
        } else {
            0.0
        };
        (dc_mean_counts, rms)
    }
}

/// Sanitized configuration clamp preventing divide-by-zero or physically invalid windows.
pub fn sanitize_knock_config(mut cfg: KnockConfig) -> KnockConfig {
    cfg.center_freq_hz = cfg.center_freq_hz.clamp(2_000, 20_000);
    cfg.q_x10 = cfg.q_x10.clamp(10, 120);
    cfg.window_start_deg_x10 = cfg.window_start_deg_x10.clamp(0, 600);
    let min_end = cfg.window_start_deg_x10.saturating_add(50);
    cfg.window_end_deg_x10 = cfg.window_end_deg_x10.clamp(min_end, 900);
    cfg.min_rpm = cfg.min_rpm.clamp(400, 6_000);
    cfg.threshold_ratio_x100 = cfg.threshold_ratio_x100.clamp(120, 1_000);
    cfg.min_abs_rms_counts = cfg.min_abs_rms_counts.clamp(10, 20_000);
    cfg.step_retard_deg_x10 = cfg.step_retard_deg_x10.clamp(5, 80);
    cfg.max_retard_deg_x10 = cfg.max_retard_deg_x10.clamp(cfg.step_retard_deg_x10, 150);
    cfg.decay_ms_per_0_1deg = cfg.decay_ms_per_0_1deg.clamp(10, 5_000);
    if cfg.diag_min_dc_counts >= cfg.diag_max_dc_counts {
        cfg.diag_min_dc_counts = 3_000;
        cfg.diag_max_dc_counts = 62_000;
    }
    cfg
}

/// Per-cylinder closed-loop Knock Window DSP & retard controller.
#[derive(Debug, Clone)]
pub struct KnockDetector {
    config: KnockConfig,
    filter: KnockBiquad,
    rpm_noise_bins: [f32; 8],
    per_cyl_rms_counts: [u16; 4],
    per_cyl_noise_floor_counts: [u16; 4],
    per_cyl_knock_events: [u32; 4],
    per_cyl_retard_deg_x10: [i16; 4],
    hold_ms: [u16; 4],
    decay_acc_ms: [u16; 4],
    last_cylinder: u8,
    last_rpm: u16,
    last_sample_count: u16,
    last_dc_mean_counts: u16,
    window_count: u32,
    sensor_fault: bool,
}

impl Default for KnockDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl KnockDetector {
    pub const fn new() -> Self {
        let cfg = KnockConfig::default_honda_obd1();
        let seed_floor = cfg.min_abs_rms_counts as f32;
        Self {
            config: cfg,
            filter: KnockBiquad::DEFAULT_7500HZ,
            rpm_noise_bins: [seed_floor; 8],
            per_cyl_rms_counts: [0; 4],
            per_cyl_noise_floor_counts: [cfg.min_abs_rms_counts; 4],
            per_cyl_knock_events: [0; 4],
            per_cyl_retard_deg_x10: [0; 4],
            hold_ms: [0; 4],
            decay_acc_ms: [0; 4],
            last_cylinder: 0,
            last_rpm: 0,
            last_sample_count: 0,
            last_dc_mean_counts: 0,
            window_count: 0,
            sensor_fault: false,
        }
    }

    #[inline]
    pub const fn config(&self) -> KnockConfig {
        self.config
    }

    #[inline]
    pub const fn per_cyl_retard_deg_x10(&self) -> [i16; 4] {
        self.per_cyl_retard_deg_x10
    }

    /// Updates runtime configuration and recalculates bandpass coefficients immediately.
    pub fn set_config(&mut self, cfg: KnockConfig) {
        let clean = sanitize_knock_config(cfg);
        self.config = clean;
        self.filter = KnockBiquad::design(clean.center_freq_hz, clean.q_x10, KNOCK_SAMPLE_RATE_HZ);
        if !clean.sensor_diag_enabled {
            self.sensor_fault = false;
        }
        if !clean.retard_enabled || !clean.enabled {
            self.per_cyl_retard_deg_x10 = [0; 4];
            self.hold_ms = [0; 4];
            self.decay_acc_ms = [0; 4];
        }
        for retard in &mut self.per_cyl_retard_deg_x10 {
            *retard = (*retard).min(clean.max_retard_deg_x10);
        }
    }
}

/// Pure function computing whether a knock combustion capture window starts inside the upcoming
/// `30.0°` `CKP` tooth window, decoupled from `KnockDetector` mutable state for ISR use.
pub fn plan_knock_window_on_ckp_tooth(
    cfg: &KnockConfig,
    ev: &CkpToothEvent,
) -> Option<KnockWindowPlan> {
    if !cfg.enabled
        || ev.sync_state != SyncState::FullSynced720
        || ev.rpm < cfg.min_rpm
        || ev.tooth_period_us == 0
    {
        return None;
    }

    for &cyl in &HONDA_FIRING_ORDER {
        let win_start_abs_x10 =
            (CYLINDER_TDC_DEG_X10[cyl.index()] + cfg.window_start_deg_x10) % 7200;
        if let Some(delta_x10) =
            angle_offset_in_tooth_window(ev.crank_angle_deg_x10, win_start_abs_x10)
        {
            let delay_from_tooth_us = angle_delta_to_delay_us(delta_x10, ev.tooth_period_us);
            let span_x10 = cfg
                .window_end_deg_x10
                .saturating_sub(cfg.window_start_deg_x10)
                .max(50);
            let duration_us = ((span_x10 as u64 * ev.tooth_period_us as u64)
                / (DEG_X10_PER_CKP_TOOTH as u64))
                .clamp(100, 10_000) as u32;
            let sample_count = ((duration_us as u64 * KNOCK_SAMPLE_RATE_HZ as u64) / 1_000_000)
                .clamp(
                    MIN_KNOCK_WINDOW_SAMPLES as u64,
                    MAX_KNOCK_WINDOW_SAMPLES as u64,
                ) as u16;
            return Some(KnockWindowPlan {
                cylinder: cyl,
                tooth_timestamp_us: ev.timestamp_us,
                delay_from_tooth_us,
                duration_us,
                sample_count,
                rpm: ev.rpm,
            });
        }
    }
    None
}

impl KnockDetector {
    /// Schedules a combustion knock window when its start angle falls inside the next `30.0°`
    /// `CKP` tooth segment.
    pub fn plan_on_ckp_tooth(&self, ev: &CkpToothEvent) -> Option<KnockWindowPlan> {
        plan_knock_window_on_ckp_tooth(&self.config, ev)
    }

    /// Evaluates one completed combustion window buffer for cylinder `cyl` at engine speed `rpm`.
    ///
    /// Returns `true` if a knock event was detected on this window.
    pub fn process_window(&mut self, cyl: Cylinder, rpm: u16, samples: &[u16]) -> bool {
        if !self.config.enabled || samples.len() < MIN_KNOCK_WINDOW_SAMPLES {
            return false;
        }

        let (dc_mean, rms_counts) = self.filter.measure_bandpass_rms(samples);
        let cidx = cyl.index();
        self.last_cylinder = cidx as u8;
        self.last_rpm = rpm;
        self.last_sample_count = samples.len().min(u16::MAX as usize) as u16;
        self.last_dc_mean_counts = dc_mean;
        self.window_count = self.window_count.saturating_add(1);
        self.per_cyl_rms_counts[cidx] = rms_counts.clamp(0.0, u16::MAX as f32) as u16;

        if self.config.sensor_diag_enabled {
            self.sensor_fault = dc_mean < self.config.diag_min_dc_counts
                || dc_mean > self.config.diag_max_dc_counts;
        } else {
            self.sensor_fault = false;
        }

        if self.sensor_fault || rpm < self.config.min_rpm {
            return false;
        }

        let floor_counts = self.interpolated_rpm_floor(rpm);
        let effective_floor = floor_counts.max(self.config.min_abs_rms_counts as f32);
        self.per_cyl_noise_floor_counts[cidx] = effective_floor.clamp(0.0, u16::MAX as f32) as u16;

        let threshold = effective_floor * ((self.config.threshold_ratio_x100 as f32) / 100.0);
        if rms_counts > threshold {
            self.per_cyl_knock_events[cidx] = self.per_cyl_knock_events[cidx].saturating_add(1);
            if self.config.retard_enabled {
                let next = self.per_cyl_retard_deg_x10[cidx]
                    .saturating_add(self.config.step_retard_deg_x10)
                    .min(self.config.max_retard_deg_x10);
                self.per_cyl_retard_deg_x10[cidx] = next;
                self.hold_ms[cidx] = self.config.recovery_hold_ms;
                self.decay_acc_ms[cidx] = 0;
            }
            true
        } else {
            self.update_rpm_noise_floor(rpm, rms_counts);
            false
        }
    }

    /// Runs `1 kHz` retard recovery decay and maintains [`FaultFlags::KNOCK_SENSOR`].
    pub fn tick_1khz(&mut self, engine_unsynced: bool, faults: &mut FaultFlags) {
        if self.sensor_fault && self.config.sensor_diag_enabled {
            faults.insert(FaultFlags::KNOCK_SENSOR);
        } else {
            faults.remove(FaultFlags::KNOCK_SENSOR);
        }

        if engine_unsynced || !self.config.enabled || !self.config.retard_enabled {
            if engine_unsynced {
                self.per_cyl_retard_deg_x10 = [0; 4];
                self.hold_ms = [0; 4];
                self.decay_acc_ms = [0; 4];
            }
            return;
        }

        for c in 0..4 {
            if self.per_cyl_retard_deg_x10[c] <= 0 {
                self.per_cyl_retard_deg_x10[c] = 0;
                self.hold_ms[c] = 0;
                self.decay_acc_ms[c] = 0;
                continue;
            }
            if self.hold_ms[c] > 0 {
                self.hold_ms[c] -= 1;
                continue;
            }
            self.decay_acc_ms[c] = self.decay_acc_ms[c].saturating_add(1);
            if self.decay_acc_ms[c] >= self.config.decay_ms_per_0_1deg {
                self.decay_acc_ms[c] = 0;
                self.per_cyl_retard_deg_x10[c] -= 1;
            }
        }
    }

    /// Exports structured diagnostics snapshot combined with hardware capture overruns.
    pub fn telemetry(&self, capture_overruns: u32) -> KnockTelemetry {
        let mut bins_u16 = [0u16; 8];
        for (dst, &src) in bins_u16.iter_mut().zip(self.rpm_noise_bins.iter()) {
            *dst = src.clamp(0.0, u16::MAX as f32) as u16;
        }
        KnockTelemetry {
            enabled: self.config.enabled,
            retard_enabled: self.config.retard_enabled,
            sensor_fault: self.sensor_fault,
            last_cylinder: self.last_cylinder,
            center_freq_hz: self.config.center_freq_hz,
            last_rpm: self.last_rpm,
            last_sample_count: self.last_sample_count,
            last_dc_mean_counts: self.last_dc_mean_counts,
            per_cyl_rms_counts: self.per_cyl_rms_counts,
            per_cyl_noise_floor_counts: self.per_cyl_noise_floor_counts,
            per_cyl_knock_events: self.per_cyl_knock_events,
            per_cyl_retard_deg_x10: self.per_cyl_retard_deg_x10,
            rpm_noise_bins_counts: bins_u16,
            window_count: self.window_count,
            capture_overruns,
        }
    }

    fn interpolated_rpm_floor(&self, rpm: u16) -> f32 {
        let (i0, i1, frac) = rpm_bin_weights(rpm);
        self.rpm_noise_bins[i0] * (1.0 - frac) + self.rpm_noise_bins[i1] * frac
    }

    fn update_rpm_noise_floor(&mut self, rpm: u16, quiet_rms: f32) {
        const ALPHA: f32 = 0.08;
        let clamped_rms = quiet_rms.clamp(50.0, 20_000.0);
        let (i0, i1, frac) = rpm_bin_weights(rpm);
        if i0 == i1 {
            self.rpm_noise_bins[i0] += ALPHA * (clamped_rms - self.rpm_noise_bins[i0]);
        } else {
            let w0 = ALPHA * (1.0 - frac);
            let w1 = ALPHA * frac;
            self.rpm_noise_bins[i0] += w0 * (clamped_rms - self.rpm_noise_bins[i0]);
            self.rpm_noise_bins[i1] += w1 * (clamped_rms - self.rpm_noise_bins[i1]);
        }
    }
}

#[inline]
fn rpm_bin_weights(rpm: u16) -> (usize, usize, f32) {
    if rpm <= KNOCK_RPM_BINS[0] {
        return (0, 0, 0.0);
    }
    let last = KNOCK_RPM_BINS.len() - 1;
    if rpm >= KNOCK_RPM_BINS[last] {
        return (last, last, 0.0);
    }
    let mut i = 0;
    while i < last {
        let r0 = KNOCK_RPM_BINS[i] as f32;
        let r1 = KNOCK_RPM_BINS[i + 1] as f32;
        if (rpm as f32) <= r1 {
            let frac = (((rpm as f32) - r0) / (r1 - r0)).clamp(0.0, 1.0);
            return (i, i + 1, frac);
        }
        i += 1;
    }
    (last, last, 0.0)
}

#[cfg(test)]
mod tests {
    use ecu_types::{Cylinder, FaultFlags, HONDA_FIRING_ORDER, KnockConfig, SyncState};

    use crate::ignition::IgnitionModel;
    use crate::math::sin_cos_f32;
    use crate::trigger::CkpToothEvent;

    use super::{KNOCK_SAMPLE_RATE_HZ, KnockBiquad, KnockDetector};

    fn make_sine_buffer(freq_hz: f32, amp_counts: f32, dc_counts: f32, len: usize) -> [u16; 512] {
        let mut buf = [0u16; 512];
        for (i, dst) in buf.iter_mut().take(len).enumerate() {
            let phase =
                core::f32::consts::TAU * freq_hz * (i as f32) / (KNOCK_SAMPLE_RATE_HZ as f32);
            let (s, _) = sin_cos_f32(phase);
            *dst = (dc_counts + amp_counts * s).clamp(0.0, 65_535.0) as u16;
        }
        buf
    }

    #[test]
    fn default_const_biquad_matches_dynamic_design() {
        let dyn_bq = KnockBiquad::design(7_500, 40, KNOCK_SAMPLE_RATE_HZ);
        let def_bq = KnockBiquad::DEFAULT_7500HZ;
        assert!((dyn_bq.b0 - def_bq.b0).abs() < 1e-5);
        assert!((dyn_bq.a1 - def_bq.a1).abs() < 1e-4);
        assert!((dyn_bq.a2 - def_bq.a2).abs() < 1e-4);
    }

    #[test]
    fn bandpass_selects_7_5khz_and_rejects_out_of_band_signals() {
        let bq = KnockBiquad::DEFAULT_7500HZ;
        // 400 samples = 2.0 ms @ 200 kS/s (15 full cycles of 7.5 kHz) with 1.65V mid-rail DC bias
        let in_band = make_sine_buffer(7_500.0, 4_000.0, 32_768.0, 400);
        let low_band = make_sine_buffer(1_200.0, 4_000.0, 32_768.0, 400);
        let high_band = make_sine_buffer(35_000.0, 4_000.0, 32_768.0, 400);

        let (dc_in, rms_in) = bq.measure_bandpass_rms(&in_band[..400]);
        let (_dc_lo, rms_lo) = bq.measure_bandpass_rms(&low_band[..400]);
        let (_dc_hi, rms_hi) = bq.measure_bandpass_rms(&high_band[..400]);

        assert!((dc_in as i32 - 32_768).abs() < 80);
        assert!(
            rms_in > 2_300.0,
            "7.5 kHz in-band RMS should be ~2,828 counts (got {rms_in})"
        );
        assert!(
            rms_lo < rms_in * 0.12,
            "1.2 kHz mechanical fundamental must be attenuated >18 dB (got {rms_lo} vs {rms_in})"
        );
        assert!(
            rms_hi < rms_in * 0.12,
            "35 kHz switching harmonic must be attenuated >18 dB (got {rms_hi} vs {rms_in})"
        );
    }

    #[test]
    fn schedules_knock_windows_for_1_3_4_2_and_isolates_per_cylinder_retard() {
        let mut kd = KnockDetector::new();
        let mut firing_windows = [Cylinder::Cyl1; 4];
        let mut planned_count = 0usize;

        // 4,500 RPM -> 1,111 us per 30° CKP tooth
        for tooth_idx in 0..24u8 {
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: (tooth_idx as u16) * 300,
                tooth_period_us: 1_111,
                rpm: 4_500,
                timestamp_us: (tooth_idx as u32) * 1_111,
            };
            if let Some(plan) = kd.plan_on_ckp_tooth(&ev) {
                // Default 10°..50° ATDC -> delay = 10° (370 us), span = 40° (1481 us -> ~296 samples)
                assert!((365..=375).contains(&plan.delay_from_tooth_us));
                assert!((290..=305).contains(&plan.sample_count));
                firing_windows[planned_count] = plan.cylinder;
                planned_count += 1;
            }
        }
        assert_eq!(planned_count, 4);
        assert_eq!(
            firing_windows,
            [
                Cylinder::Cyl1,
                Cylinder::Cyl3,
                Cylinder::Cyl4,
                Cylinder::Cyl2
            ]
        );

        // Adapt baseline on quiet engine noise across all cylinders, then knock ONLY on Cylinder 3
        let quiet = make_sine_buffer(7_500.0, 120.0, 32_768.0, 300);
        let knock = make_sine_buffer(7_500.0, 3_500.0, 32_768.0, 300);
        for _ in 0..5 {
            for &c in &HONDA_FIRING_ORDER {
                assert!(!kd.process_window(c, 4_500, &quiet[..300]));
            }
        }
        assert!(kd.process_window(Cylinder::Cyl3, 4_500, &knock[..300]));
        assert!(!kd.process_window(Cylinder::Cyl1, 4_500, &quiet[..300]));

        let retard = kd.per_cyl_retard_deg_x10();
        assert_eq!(retard[Cylinder::Cyl3.index()], 15);
        assert_eq!(retard[Cylinder::Cyl1.index()], 0);
        assert_eq!(retard[Cylinder::Cyl4.index()], 0);
        assert_eq!(retard[Cylinder::Cyl2.index()], 0);

        // Verify IgnitionModel retards ONLY Cyl3 spark schedule and leaves Cyl1 untouched!
        let mut ign = IgnitionModel::new();
        ign.set_per_cyl_knock_retard_deg_x10(retard);
        let ev_c3 = CkpToothEvent {
            sync_state: SyncState::FullSynced720,
            tooth_index: 5,
            crank_angle_deg_x10: 1_500, // 150°..180° (Cyl3 TDC = 180°)
            tooth_period_us: 1_111,
            rpm: 4_500,
            timestamp_us: 5_000,
        };
        let mut ign_no_knk = IgnitionModel::new();
        ign_no_knk.set_per_cyl_knock_retard_deg_x10([0; 4]);
        let batch_knk = ign.schedule_on_ckp_tooth(&ev_c3, 250, 2_500);
        let batch_clean = ign_no_knk.schedule_on_ckp_tooth(&ev_c3, 250, 2_500);
        let spark_knk = batch_knk
            .actions
            .iter()
            .flatten()
            .find(|a| a.cylinder == Cylinder::Cyl3 && !a.drive_dwell_high)
            .expect("Cyl3 spark with knock");
        let spark_clean = batch_clean
            .actions
            .iter()
            .flatten()
            .find(|a| a.cylinder == Cylinder::Cyl3 && !a.drive_dwell_high)
            .expect("Cyl3 spark without knock");
        assert!(
            spark_knk.delay_from_tooth_us > spark_clean.delay_from_tooth_us,
            "Knock retard must delay spark later toward TDC"
        );

        // Hold for recovery_hold_ms (400 ms), then decay 0.1° per 150 ms back to 0!
        let mut faults = FaultFlags::default();
        for _ in 0..400 {
            kd.tick_1khz(false, &mut faults);
        }
        assert_eq!(kd.per_cyl_retard_deg_x10()[Cylinder::Cyl3.index()], 15);
        for _ in 0..(150 * 15) {
            kd.tick_1khz(false, &mut faults);
        }
        assert_eq!(kd.per_cyl_retard_deg_x10()[Cylinder::Cyl3.index()], 0);
    }

    #[test]
    fn sensor_diag_flags_open_or_short_circuit_when_enabled() {
        let mut kd = KnockDetector::new();
        let cfg = KnockConfig {
            sensor_diag_enabled: true,
            ..KnockConfig::default_honda_obd1()
        };
        kd.set_config(cfg);

        // Shorted to ground (DC = 50 counts < 3000)
        let shorted = make_sine_buffer(7_500.0, 10.0, 50.0, 200);
        assert!(!kd.process_window(Cylinder::Cyl1, 3_000, &shorted[..200]));
        let mut faults = FaultFlags::default();
        kd.tick_1khz(false, &mut faults);
        assert!(faults.contains(FaultFlags::KNOCK_SENSOR));

        // Healthy biased mid-rail clears Code 23
        let healthy = make_sine_buffer(7_500.0, 100.0, 32_768.0, 200);
        assert!(!kd.process_window(Cylinder::Cyl1, 3_000, &healthy[..200]));
        kd.tick_1khz(false, &mut faults);
        assert!(!faults.contains(FaultFlags::KNOCK_SENSOR));
    }

    #[test]
    fn engine_coordinator_closes_knock_retard_loop_and_respects_scs_override() {
        use crate::EngineCoordinator;
        use ecu_types::{CalibrationBlob, RawSensorAdc, SwitchState};

        let cal = CalibrationBlob::default_honda_obd1();
        let mut ecu = EngineCoordinator::new();
        let raw = RawSensorAdc::default();
        let dt_us = 1_250u32; // 4,000 RPM (`1,250 us` per 30° CKP tooth)
        let mut ts = 10_000u32;

        // Spin up 720° sync across 48 CKP teeth (~60 ms)
        let _ = ecu.on_ckp_capture(ts, &cal);
        for _cycle in 0..2u8 {
            for tooth in 0..24u8 {
                if tooth == 0 {
                    ecu.on_cyp_capture(ts + 40);
                }
                if tooth.is_multiple_of(6) {
                    ecu.on_tdc_capture(ts + 80);
                }
                ts += dt_us;
                let _ = ecu.on_ckp_capture(ts, &cal);
            }
        }
        let _ = ecu.tick_1khz(ts, &raw, SwitchState::default(), 0.0, false, &cal);
        assert_eq!(ecu.trigger.rpm(), 4_000);

        // Inject 7.5 kHz knock on Cylinder 4 during combustion window
        let knock = make_sine_buffer(7_500.0, 3_800.0, 32_768.0, 220);
        assert!(ecu.process_knock_window(Cylinder::Cyl4, 4_000, &knock[..220]));

        // After 1 kHz tick, IgnitionModel receives Cyl 4 retard (1.5°) while Cyl 1 remains 0°
        let _ = ecu.tick_1khz(ts + 1_000, &raw, SwitchState::default(), 0.0, false, &cal);
        let ret = ecu.ignition.per_cyl_knock_retard_deg_x10();
        assert_eq!(ret[Cylinder::Cyl4.index()], 15);
        assert_eq!(ret[Cylinder::Cyl1.index()], 0);

        // Service Check Connector (SCS) base-timing lock must zero ignition knock retard temporarily
        let scs = SwitchState {
            scs_service_jumper: true,
            ..Default::default()
        };
        let _ = ecu.tick_1khz(ts + 2_000, &raw, scs, 0.0, false, &cal);
        assert_eq!(ecu.ignition.per_cyl_knock_retard_deg_x10(), [0; 4]);
    }
}
