//! Sequential Smart COP ignition model & `< 30°` final-tooth `TIM1` scheduler (`1-3-4-2`).

use crate::math::{interp_1d_u16, interp_2d_i16};
use crate::trigger::{
    CkpToothEvent, DEG_X10_PER_CKP_TOOTH, ToothBatch, angle_delta_to_delay_us,
    angle_offset_in_tooth_window,
};
use ecu_types::{
    CYLINDER_TDC_DEG_X10, CalibrationBlob, Cylinder, HONDA_FIRING_ORDER, SensorSnapshot, SyncState,
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

pub struct IgnitionModel {
    per_cyl_knock_retard_deg_x10: [i16; 4],
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
        }
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

        // High intake-air / coolant safety retard (> 50 °C IAT or > 102 °C ECT)
        if sensors.iat_c > 50.0 {
            adv_x10 -= ((sensors.iat_c - 50.0) * 1.5) as i16;
        }
        if sensors.ect_c > 102.0 {
            adv_x10 -= ((sensors.ect_c - 102.0) * 3.0) as i16;
        }

        (adv_x10.clamp(-50, 450), dwell_us)
    }

    /// Angle-domain scheduler invoked on every `CKP` tooth edge (`TIM2` ISR, Priority 0 `ITCM`).
    ///
    /// Arms 16-bit `TIM1` Output Compare channels **only within the final `< 30.0°` tooth window**,
    /// guaranteeing `delay_from_tooth_us < 65_535` down to `150 RPM` while preserving `< 0.05°`
    /// accuracy at `9,000 RPM`!
    pub fn schedule_on_ckp_tooth(
        &self,
        tooth: &CkpToothEvent,
        spark_advance_deg_x10: i16,
        dwell_us: u32,
    ) -> ToothScheduleBatch {
        let mut batch = ToothScheduleBatch::default();
        if tooth.sync_state != SyncState::FullSynced720 || tooth.rpm == 0 {
            return batch;
        }

        // Convert `dwell_us` into `0.1°` crank degrees at current instantaneous tooth speed:
        // `300` (`30.0°`) ticks take `tooth_period_us` µs =>
        // `dwell_deg_x10 = dwell_us * 300 / tooth_period_us`
        let dwell_deg_x10 = ((dwell_us as u64 * DEG_X10_PER_CKP_TOOTH as u64)
            / (tooth.tooth_period_us.max(1) as u64))
            .clamp(10, 2400) as i32;

        for &cyl in &HONDA_FIRING_ORDER {
            let tdc_x10 = CYLINDER_TDC_DEG_X10[cyl.index()] as i32;
            let cyl_adv_x10 =
                (spark_advance_deg_x10 - self.per_cyl_knock_retard_deg_x10[cyl.index()]) as i32;

            let spark_angle_x10 = (tdc_x10 - cyl_adv_x10).rem_euclid(7200) as u16;
            let dwell_start_x10 = (spark_angle_x10 as i32 - dwell_deg_x10).rem_euclid(7200) as u16;

            if let Some(d_deg) =
                angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, dwell_start_x10)
            {
                batch.push(CopCompareAction {
                    cylinder: cyl,
                    delay_from_tooth_us: angle_delta_to_delay_us(d_deg, tooth.tooth_period_us),
                    drive_dwell_high: true,
                });
            }

            if let Some(s_deg) =
                angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, spark_angle_x10)
            {
                batch.push(CopCompareAction {
                    cylinder: cyl,
                    delay_from_tooth_us: angle_delta_to_delay_us(s_deg, tooth.tooth_period_us),
                    drive_dwell_high: false,
                });
            }
        }

        batch
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{CalibrationBlob, Cylinder, SensorSnapshot, SyncState};

    use crate::trigger::CkpToothEvent;

    use super::IgnitionModel;

    #[test]
    fn schedules_all_four_cops_in_exact_1_3_4_2_sequence_over_720_deg() {
        let ign = IgnitionModel::new();
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
}
