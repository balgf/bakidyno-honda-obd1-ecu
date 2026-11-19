//! Sequential Smart COP ignition model & `< 30°` final-tooth `TIM1` scheduler (`1-3-4-2`).

use crate::math::{interp_1d_u16, interp_2d_i16};
use crate::trigger::{
    CkpToothEvent, DEG_X10_PER_CKP_TOOTH, ToothBatch, angle_delta_to_delay_us,
    angle_offset_in_tooth_window,
};
use ecu_types::{
    CYLINDER_TDC_DEG_X10, CalibrationBlob, Cylinder, HONDA_FIRING_ORDER, OutputEnableMask,
    SensorSnapshot, SyncState,
};

/// Hardware `TIM1` Output Compare action to arm during the upcoming `30.0°` `CKP` tooth window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CopCompareAction {
    pub cylinder: Cylinder,
    /// Offset in `µs` after `CkpToothEvent::timestamp_us` (`0 .. < 65_535` fits 16-bit `TIM1`!).
    pub delay_from_tooth_us: u16,
    /// `true` = start coil dwell (`5V` HIGH via `TC4427`); `false` = fire spark (`0V` LOW + fire
    /// `TIM16` `TACH_OUT`).
    pub drive_dwell_high: bool,
}

/// Up to 4 coil edges can fall inside a single `30.0°` tooth window at high RPM / dwell overlap.
pub type ToothScheduleBatch = ToothBatch<CopCompareAction>;

/// Cycle-latch reset point (`120.0°` ATDC), safely past latest spark and ~300° before earliest dwell.
const IGN_RESET_OFFSET_FROM_TDC_X10: i32 = 1_200;

/// Stateful per-cycle tooth scheduler for Smart COP dwell start and spark release (`TIM1`).
///
/// Tracks per-cylinder latches across each `720°` cam cycle so minor RPM/tooth-period jitter or
/// advance updates across `30.0°` `CKP` boundaries never skip a dwell/spark event or double-fire.
#[derive(Debug, Clone, Copy, Default)]
pub struct IgnitionToothScheduler {
    dwell_scheduled_in_cycle: [bool; 4],
    spark_scheduled_in_cycle: [bool; 4],
}

impl IgnitionToothScheduler {
    pub const fn new() -> Self {
        Self {
            dwell_scheduled_in_cycle: [false; 4],
            spark_scheduled_in_cycle: [false; 4],
        }
    }

    #[inline]
    pub fn reset_cycle(&mut self) {
        self.dwell_scheduled_in_cycle = [false; 4];
        self.spark_scheduled_in_cycle = [false; 4];
    }

    /// Evaluates which Smart COP dwell-start (`HIGH`) and spark (`LOW`) compare events fall within
    /// the next `30.0°` tooth segment (or just crossed behind the tooth edge by `<= 30.0°`).
    pub fn schedule_on_ckp_tooth(
        &mut self,
        tooth: &CkpToothEvent,
        spark_advance_deg_x10: i16,
        per_cyl_knock_retard_deg_x10: &[i16; 4],
        dwell_us: u32,
    ) -> ToothScheduleBatch {
        self.schedule_on_ckp_tooth_masked(
            tooth,
            spark_advance_deg_x10,
            per_cyl_knock_retard_deg_x10,
            dwell_us,
            OutputEnableMask::all_enabled(),
        )
    }

    /// Like [`Self::schedule_on_ckp_tooth`], suppressing any cylinder disabled in `mask`.
    pub fn schedule_on_ckp_tooth_masked(
        &mut self,
        tooth: &CkpToothEvent,
        spark_advance_deg_x10: i16,
        per_cyl_knock_retard_deg_x10: &[i16; 4],
        dwell_us: u32,
        mask: OutputEnableMask,
    ) -> ToothScheduleBatch {
        let mut batch = ToothScheduleBatch::default();
        if tooth.sync_state != SyncState::FullSynced720 || tooth.rpm == 0 {
            self.reset_cycle();
            return batch;
        }

        // Convert `dwell_us` into `0.1°` crank degrees at current instantaneous tooth speed:
        // `300` (`30.0°`) ticks take `tooth_period_us` µs =>
        // `dwell_deg_x10 = dwell_us * 300 / tooth_period_us`
        let period_us = tooth.tooth_period_us.max(1);
        let dwell_deg_x10 = ((dwell_us as u64 * DEG_X10_PER_CKP_TOOTH as u64) / (period_us as u64))
            .clamp(10, 2400) as i32;

        for &cyl in &HONDA_FIRING_ORDER {
            let idx = cyl.index();
            let cyl_enabled = mask.is_cop_enabled(cyl);
            let tdc_x10 = CYLINDER_TDC_DEG_X10[idx] as i32;
            let reset_x10 = (tdc_x10 + IGN_RESET_OFFSET_FROM_TDC_X10).rem_euclid(7200) as u16;
            if angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, reset_x10).is_some() {
                self.dwell_scheduled_in_cycle[idx] = false;
                self.spark_scheduled_in_cycle[idx] = false;
                continue;
            }
            if !cyl_enabled && !self.dwell_scheduled_in_cycle[idx] {
                continue;
            }

            let cyl_adv_x10 = (spark_advance_deg_x10 - per_cyl_knock_retard_deg_x10[idx]) as i32;
            let spark_angle_x10 = (tdc_x10 - cyl_adv_x10).rem_euclid(7200) as u16;
            let dwell_start_x10 = (spark_angle_x10 as i32 - dwell_deg_x10).rem_euclid(7200) as u16;

            if cyl_enabled && !self.dwell_scheduled_in_cycle[idx] {
                if let Some(d_deg) =
                    angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, dwell_start_x10)
                {
                    self.dwell_scheduled_in_cycle[idx] = true;
                    batch.push(CopCompareAction {
                        cylinder: cyl,
                        delay_from_tooth_us: angle_delta_to_delay_us(d_deg, period_us),
                        drive_dwell_high: true,
                    });
                } else {
                    let dwell_behind_x10 = (tooth.crank_angle_deg_x10 as i32
                        - dwell_start_x10 as i32)
                        .rem_euclid(7200);
                    let forward_to_spark_x10 = (spark_angle_x10 as i32
                        - tooth.crank_angle_deg_x10 as i32)
                        .rem_euclid(7200);
                    // Only catch up a late dwell edge if spark is still strictly ahead in this
                    // cycle so we never initiate a coil charge after spark angle already passed.
                    if (1..=DEG_X10_PER_CKP_TOOTH as i32).contains(&dwell_behind_x10)
                        && (1..=2400).contains(&forward_to_spark_x10)
                    {
                        self.dwell_scheduled_in_cycle[idx] = true;
                        batch.push(CopCompareAction {
                            cylinder: cyl,
                            delay_from_tooth_us: 0,
                            drive_dwell_high: true,
                        });
                    }
                }
            }

            // Always release any coil whose dwell already began before a mid-cycle spark cut so
            // it never over-dwells into the 9.5 ms hardware watchdog.
            if !self.spark_scheduled_in_cycle[idx]
                && (cyl_enabled || self.dwell_scheduled_in_cycle[idx])
            {
                if let Some(s_deg) =
                    angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, spark_angle_x10)
                {
                    self.spark_scheduled_in_cycle[idx] = true;
                    batch.push(CopCompareAction {
                        cylinder: cyl,
                        delay_from_tooth_us: angle_delta_to_delay_us(s_deg, period_us),
                        drive_dwell_high: false,
                    });
                } else if self.dwell_scheduled_in_cycle[idx] {
                    let spark_behind_x10 = (tooth.crank_angle_deg_x10 as i32
                        - spark_angle_x10 as i32)
                        .rem_euclid(7200);
                    if (1..=DEG_X10_PER_CKP_TOOTH as i32).contains(&spark_behind_x10) {
                        self.spark_scheduled_in_cycle[idx] = true;
                        batch.push(CopCompareAction {
                            cylinder: cyl,
                            delay_from_tooth_us: 0,
                            drive_dwell_high: false,
                        });
                    }
                }
            }
        }

        batch
    }
}

pub struct IgnitionModel {
    per_cyl_knock_retard_deg_x10: [i16; 4],
    tooth_scheduler: IgnitionToothScheduler,
}

impl Default for IgnitionModel {
    fn default() -> Self {
        Self::new()
    }
}

impl IgnitionModel {
    pub const fn new() -> Self {
        Self {
            per_cyl_knock_retard_deg_x10: [0; 4],
            tooth_scheduler: IgnitionToothScheduler::new(),
        }
    }

    #[inline]
    pub fn reset_cycle(&mut self) {
        self.tooth_scheduler.reset_cycle();
    }

    #[inline]
    pub const fn per_cyl_knock_retard_deg_x10(&self) -> [i16; 4] {
        self.per_cyl_knock_retard_deg_x10
    }

    #[inline]
    pub fn set_per_cyl_knock_retard_deg_x10(&mut self, retard_deg_x10: [i16; 4]) {
        self.per_cyl_knock_retard_deg_x10 = retard_deg_x10;
    }

    /// Computes base spark advance (`0.1° BTDC`) and Smart COP dwell (`µs`) at `1 kHz`.
    pub fn compute_1khz(
        &self,
        rpm: u16,
        use_vtec_map: bool,
        sensors: &SensorSnapshot,
        timing_override_deg_x10: Option<i16>,
        cal: &CalibrationBlob,
    ) -> (i16, u32) {
        self.compute_1khz_with_trims(
            rpm,
            use_vtec_map,
            sensors,
            timing_override_deg_x10,
            0,
            None,
            cal,
        )
    }

    /// Computes spark advance (`0.1° BTDC`) including Flex Fuel octane advance and optional
    /// blended 2-step launch / flat-foot shift retard target.
    #[allow(clippy::too_many_arguments)]
    pub fn compute_1khz_with_trims(
        &self,
        rpm: u16,
        use_vtec_map: bool,
        sensors: &SensorSnapshot,
        timing_override_deg_x10: Option<i16>,
        flex_add_deg_x10: i16,
        spark_blend_target: Option<(i16, f32)>,
        cal: &CalibrationBlob,
    ) -> (i16, u32) {
        let dwell_us = interp_1d_u16(
            &cal.vbatt_bins_x10,
            &cal.cop_dwell_us,
            sensors.vbatt_v * 10.0,
        )
        .clamp(1_200.0, 5_500.0) as u32;

        if let Some(lock_deg) = timing_override_deg_x10 {
            return (lock_deg, dwell_us);
        }

        // Honda D4 `SCS` Service Check jumper at idle locks spark to `16.0° BTDC` for timing
        // light alignment!
        if sensors.switches.scs_service_jumper && rpm > 0 && rpm < 1_350 {
            return (cal.scs_base_timing_deg_x10, dwell_us);
        }

        // Cranking timing (`10.0° BTDC` easy hot/cold start without starter kickback)
        if sensors.switches.sts_cranking || rpm < 380 {
            return (100, dwell_us);
        }

        let table = if use_vtec_map {
            &cal.spark_vtec_cam_deg_x10
        } else {
            &cal.spark_low_cam_deg_x10
        };
        let mut adv_x10 = interp_2d_i16(
            &cal.rpm_bins,
            &cal.map_kpa_bins,
            table,
            rpm as f32,
            sensors.map_kpa,
        ) as i16;

        // Ethanol octane advance trim
        adv_x10 = adv_x10.saturating_add(flex_add_deg_x10);

        // High intake-air / coolant safety retard (> 50 °C IAT or > 102 °C ECT)
        if sensors.iat_c > 50.0 {
            adv_x10 -= ((sensors.iat_c - 50.0) * 1.5) as i16;
        }
        if sensors.ect_c > 102.0 {
            adv_x10 -= ((sensors.ect_c - 102.0) * 3.0) as i16;
        }

        if let Some((target_deg_x10, blend)) = spark_blend_target {
            let t = blend.clamp(0.0, 1.0);
            let blended = adv_x10 as f32 + ((target_deg_x10 - adv_x10) as f32 * t);
            return ((blended as i16).clamp(-150, 450), dwell_us);
        }

        (adv_x10.clamp(-50, 450), dwell_us)
    }

    /// Angle-domain scheduler invoked on every `CKP` tooth edge (`TIM2` ISR).
    ///
    /// Arms 16-bit `TIM1` Output Compare channels **only within the final `< 30.0°` tooth window**,
    /// with per-cycle latches and boundary catch-up so minor tooth jitter or timing shifts across
    /// a `30.0°` boundary never drop coil dwell or spark.
    pub fn schedule_on_ckp_tooth(
        &mut self,
        tooth: &CkpToothEvent,
        spark_advance_deg_x10: i16,
        dwell_us: u32,
    ) -> ToothScheduleBatch {
        self.schedule_on_ckp_tooth_masked(
            tooth,
            spark_advance_deg_x10,
            dwell_us,
            OutputEnableMask::all_enabled(),
        )
    }

    /// Like [`Self::schedule_on_ckp_tooth`], suppressing any cylinder disabled in `mask`.
    pub fn schedule_on_ckp_tooth_masked(
        &mut self,
        tooth: &CkpToothEvent,
        spark_advance_deg_x10: i16,
        dwell_us: u32,
        mask: OutputEnableMask,
    ) -> ToothScheduleBatch {
        self.tooth_scheduler.schedule_on_ckp_tooth_masked(
            tooth,
            spark_advance_deg_x10,
            &self.per_cyl_knock_retard_deg_x10,
            dwell_us,
            mask,
        )
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{CalibrationBlob, Cylinder, SensorSnapshot, SyncState};

    use crate::trigger::CkpToothEvent;

    use super::IgnitionModel;

    #[test]
    fn schedules_all_four_cops_in_exact_1_3_4_2_sequence_over_720_deg() {
        let mut ign = IgnitionModel::new();
        let mut spark_order = [Cylinder::Cyl1; 4];
        let mut spark_count = 0usize;
        let mut dwell_count = 0usize;

        // 6,000 RPM (`833 µs` per `30°` tooth), `25.0° BTDC` (`250`), `2.8 ms` dwell
        for tooth_idx in 0..24u8 {
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: tooth_idx as u16 * 300,
                tooth_period_us: 833,
                rpm: 6000,
                timestamp_us: tooth_idx as u32 * 833,
            };
            let batch = ign.schedule_on_ckp_tooth(&ev, 250, 2_800);
            for act in batch.actions.iter().flatten() {
                assert!(act.delay_from_tooth_us <= 833);
                if act.drive_dwell_high {
                    dwell_count += 1;
                } else {
                    spark_order[spark_count] = act.cylinder;
                    spark_count += 1;
                }
            }
        }

        assert_eq!(
            dwell_count, 4,
            "Each cylinder must start dwell exactly once per 720°"
        );
        assert_eq!(
            spark_count, 4,
            "Each cylinder must fire spark exactly once per 720°"
        );
        // Cyl1 fires at 720-25 = 695° (tooth 23), while Cyl3 fires at 180-25 = 155° (tooth 5),
        // Cyl4 at 335° (tooth 11), Cyl2 at 515° (tooth 17) -> circular order 3, 4, 2, 1 == 1-3-4-2!
        assert_eq!(
            spark_order,
            [
                Cylinder::Cyl3,
                Cylinder::Cyl4,
                Cylinder::Cyl2,
                Cylinder::Cyl1
            ]
        );
    }

    #[test]
    fn scs_service_jumper_locks_idle_spark_to_16_deg_btdc() {
        let cal = CalibrationBlob::default_honda_obd1();
        let ign = IgnitionModel::new();
        let mut sns = SensorSnapshot {
            map_kpa: 32.0,
            vbatt_v: 14.0,
            ..Default::default()
        };
        sns.switches.scs_service_jumper = true;

        let (adv, _dwell) = ign.compute_1khz(800, /* high_cam= */ false, &sns, None, &cal);
        assert_eq!(adv, 160);
    }

    #[test]
    fn cranking_timing_override_dwell_and_thermal_retard_behave_accurately() {
        let cal = CalibrationBlob::default_honda_obd1();
        let ign = IgnitionModel::new();
        let base_sns = SensorSnapshot {
            map_kpa: 90.0,
            ect_c: 85.0,
            iat_c: 30.0,
            vbatt_v: 10.0,
            ..Default::default()
        };

        // 1. Cranking (< 380 RPM) locks spark to 10.0° BTDC (`100`) and scales dwell for 10.0V
        // (`3,900 µs`)
        let (crank_adv, dwell_10v) = ign.compute_1khz(250, false, &base_sns, None, &cal);
        assert_eq!(crank_adv, 100);
        assert_eq!(dwell_10v, 3_900);

        // 2. Higher battery voltage (15.0V) reduces coil dwell (`2,500 µs`)
        let high_v_sns = SensorSnapshot {
            vbatt_v: 15.0,
            ..base_sns
        };
        let (base_adv, dwell_15v) = ign.compute_1khz(3_500, false, &high_v_sns, None, &cal);
        assert_eq!(dwell_15v, 2_500);

        // 3. Explicit timing override locks advance regardless of RPM/load
        let (override_adv, _) = ign.compute_1khz(3_500, false, &high_v_sns, Some(160), &cal);
        assert_eq!(override_adv, 160);

        // 4. High IAT (70 °C -> -3.0° = -30) + High ECT (112 °C -> -3.0° = -30) retards spark by
        // 6.0° (`60`)
        let hot_sns = SensorSnapshot {
            iat_c: 70.0,
            ect_c: 112.0,
            ..high_v_sns
        };
        let (hot_adv, _) = ign.compute_1khz(3_500, false, &hot_sns, None, &cal);
        assert_eq!(hot_adv, base_adv - 60);
    }

    #[test]
    fn boundary_crossing_advance_and_jitter_never_drop_or_double_fire_dwell_or_spark() {
        let mut ign = IgnitionModel::new();
        let mut dwell_events = [0u8; 4];
        let mut spark_events = [0u8; 4];

        // Simulate aggressive timing step right across a 30.0° tooth boundary (`690.0°` Cyl1):
        // Teeth 0..22 run at `29.0° BTDC` (`spark = 691.0°`, past `660..690`), then at tooth 23
        // (`690.0°`) advance steps to `31.0° BTDC` (`spark = 689.0°`, behind `690.0°`) AND
        // tooth_period_us tightens from 2,500 -> 2,300 µs (shifting `dwell_start` backward too!).
        for tooth_idx in 0..24u8 {
            let (adv, period_us, rpm) = if tooth_idx >= 23 {
                (310, 2_300, 2_173)
            } else {
                (290, 2_500, 2_000)
            };
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: tooth_idx as u16 * 300,
                tooth_period_us: period_us,
                rpm,
                timestamp_us: tooth_idx as u32 * 2_500,
            };
            let batch = ign.schedule_on_ckp_tooth(&ev, adv, 2_800);
            for act in batch.actions.iter().flatten() {
                if act.drive_dwell_high {
                    dwell_events[act.cylinder.index()] += 1;
                } else {
                    spark_events[act.cylinder.index()] += 1;
                }
            }
        }
        assert_eq!(dwell_events, [1; 4], "Every cylinder must dwell once");
        assert_eq!(spark_events, [1; 4], "Every cylinder must fire spark once");

        // Second cycle: step advance BACKWARD (`31.0° -> 27.0°`) right after Cyl3 sparked at 149°,
        // placing the recomputed spark (`153°`) inside tooth 5 (`150..180`). Latch must block a
        // duplicate second spark in the same 720° cycle!
        let mut cycle2_sparks = [0u8; 4];
        for tooth_idx in 0..24u8 {
            let adv = if tooth_idx >= 5 { 270 } else { 310 };
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: tooth_idx as u16 * 300,
                tooth_period_us: 2_500,
                rpm: 2_000,
                timestamp_us: 60_000 + tooth_idx as u32 * 2_500,
            };
            let batch = ign.schedule_on_ckp_tooth(&ev, adv, 2_800);
            for act in batch.actions.iter().flatten() {
                if !act.drive_dwell_high {
                    cycle2_sparks[act.cylinder.index()] += 1;
                }
            }
        }
        assert_eq!(
            cycle2_sparks, [1; 4],
            "Cycle latch must prevent double-firing when advance decreases across a tooth edge"
        );
    }

    #[test]
    fn catchup_never_initiates_dwell_after_spark_angle_already_passed() {
        let mut ign = IgnitionModel::new();
        // Sync first appears at tooth 0 (`0.0°`), which for Cyl1 (`TDC = 0.0°`, `20.0° BTDC`) is
        // AFTER Cyl1 spark (`700.0°`) and AFTER Cyl1 dwell start (`664.0°`). Must NOT dwell late!
        let ev = CkpToothEvent {
            sync_state: SyncState::FullSynced720,
            tooth_index: 0,
            crank_angle_deg_x10: 0,
            tooth_period_us: 2_500,
            rpm: 2_000,
            timestamp_us: 1_000,
        };
        let batch = ign.schedule_on_ckp_tooth(&ev, 200, 3_000);
        assert!(
            batch
                .actions
                .iter()
                .flatten()
                .all(|a| a.cylinder != Cylinder::Cyl1),
            "Cyl1 must not start dwell retroactively after passing spark angle"
        );
    }

    #[test]
    fn mid_cycle_spark_cut_releases_already_started_dwell_and_blocks_next_cycle() {
        let mut ign = IgnitionModel::new();
        let all_on = ecu_types::OutputEnableMask::all_enabled();
        let mut cut_all = ecu_types::OutputEnableMask::all_enabled();
        cut_all.ignition_enabled = false;

        // 6,000 RPM (`833 µs` per `30°`), `25.0° BTDC`, `2.8 ms` (`100.8°`) dwell:
        // Cyl 1 starts dwell at `594.2°` (tooth 19, `570..600°`) and sparks at `695.0°` (tooth 23).
        let mut cyl1_dwell_started = false;
        let mut cyl1_spark_released = false;
        for tooth_idx in 0..24u8 {
            let mask = if cyl1_dwell_started { cut_all } else { all_on };
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: tooth_idx as u16 * 300,
                tooth_period_us: 833,
                rpm: 6_000,
                timestamp_us: tooth_idx as u32 * 833,
            };
            let batch = ign.schedule_on_ckp_tooth_masked(&ev, 250, 2_800, mask);
            for act in batch.actions.iter().flatten() {
                if act.cylinder == Cylinder::Cyl1 {
                    if act.drive_dwell_high {
                        cyl1_dwell_started = true;
                    } else {
                        cyl1_spark_released = true;
                    }
                }
            }
        }
        assert!(cyl1_dwell_started, "Cyl1 should start dwell before the cut");
        assert!(
            cyl1_spark_released,
            "Mid-cycle cut must still release Cyl1 dwell at spark angle instead of over-dwelling"
        );

        // Next full 720° cycle with cut still active must schedule zero dwell and zero spark!
        for tooth_idx in 0..24u8 {
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: tooth_idx as u16 * 300,
                tooth_period_us: 833,
                rpm: 6_000,
                timestamp_us: 20_000 + tooth_idx as u32 * 833,
            };
            let batch = ign.schedule_on_ckp_tooth_masked(&ev, 250, 2_800, cut_all);
            assert_eq!(
                batch.len, 0,
                "Sustained spark cut must not start any new dwell on tooth {tooth_idx}"
            );
        }
    }
}
