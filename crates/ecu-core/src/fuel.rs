//! Speed-Density sequential fuel model (`A1 INJ1`, `A3 INJ2`, `A5 INJ3`, `A2 INJ4`).

use crate::math::{
    PidController, interp_1d_i16_u16, interp_1d_i16_u32, interp_1d_u16, interp_2d_u16,
};
use crate::trigger::{
    CkpToothEvent, DEG_X10_PER_CKP_TOOTH, ToothBatch, angle_delta_to_delay_us,
    angle_offset_in_tooth_window,
};
use ecu_types::{
    CYLINDER_TDC_DEG_X10, CalibrationBlob, Cylinder, HONDA_FIRING_ORDER, SensorSnapshot, SyncState,
    WidebandState,
};

/// Dry air specific gas constant `R_specific = 287.05 J/(kg·K)`.
const R_AIR_J_PER_KG_K: f32 = 287.05;
/// Typical pump gasoline density `~0.745 g/cm³`.
const GASOLINE_DENSITY_G_PER_CC: f32 = 0.745;
/// Target End-of-Injection angle ATDC compression (`450.0°` = `90°` into the intake stroke).
pub const DEFAULT_INJ_EOI_ATDC_DEG_X10: i32 = 4500;
/// Cycle-latch reset point placed `120°` after EOI (`570.0°` ATDC), disjoint from `SOI` up to 75%.
const INJ_RESET_OFFSET_FROM_EOI_X10: i32 = 1200;

/// Hardware `TIM3` Output Compare action to arm sequential fuel injection during a `30.0°` tooth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InjCompareAction {
    pub cylinder: Cylinder,
    /// Offset in `µs` after `CkpToothEvent::timestamp_us` (`0 .. < 65_000`) to open the injector.
    pub delay_from_tooth_us: u16,
    /// Exact microsecond duration (`1 ..= 90_000`) to hold the injector open.
    pub pulse_width_us: u32,
}

/// Up to 4 injector open events scheduled within a single `30.0°` `CKP` tooth window.
pub type InjScheduleBatch = ToothBatch<InjCompareAction>;

#[derive(Debug, Clone, Copy)]
pub struct FuelOutput {
    pub ve_pct: f32,
    pub target_lambda: f32,
    pub stft_pct: f32,
    pub final_pw_us: [u32; 4],
}

/// Stateful per-cycle tooth scheduler converting computed pulse widths (`µs`) into Start-of-Injection
/// `TIM3` compare actions anchored to `DEFAULT_INJ_EOI_ATDC_DEG_X10`.
#[derive(Debug, Clone, Copy, Default)]
pub struct FuelToothScheduler {
    inj_scheduled_in_cycle: [bool; 4],
}

impl FuelToothScheduler {
    pub const fn new() -> Self {
        Self {
            inj_scheduled_in_cycle: [false; 4],
        }
    }

    #[inline]
    pub fn reset_cycle(&mut self) {
        self.inj_scheduled_in_cycle = [false; 4];
    }

    /// Computes exact injector-opening compare events inside the upcoming `30.0°` `CKP` segment.
    pub fn schedule_on_ckp_tooth(
        &mut self,
        tooth: &CkpToothEvent,
        inj_pw_us: &[u32; 4],
    ) -> InjScheduleBatch {
        let mut batch = InjScheduleBatch::default();
        if tooth.sync_state != SyncState::FullSynced720 || tooth.rpm == 0 {
            self.reset_cycle();
            return batch;
        }

        let period_us = tooth.tooth_period_us.max(1) as u64;
        for &cyl in &HONDA_FIRING_ORDER {
            let idx = cyl.index();
            let tdc_x10 = CYLINDER_TDC_DEG_X10[idx] as i32;
            let eoi_x10 = tdc_x10 + DEFAULT_INJ_EOI_ATDC_DEG_X10;
            let reset_x10 = (eoi_x10 + INJ_RESET_OFFSET_FROM_EOI_X10).rem_euclid(7200) as u16;

            // Reset per-cycle latch 120° after EOI so dynamic PW shifts never double-fire.
            if angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, reset_x10).is_some() {
                self.inj_scheduled_in_cycle[idx] = false;
                continue;
            }

            let pw_us = inj_pw_us[idx];
            if pw_us == 0 || self.inj_scheduled_in_cycle[idx] {
                continue;
            }

            let pw_deg_x10 =
                ((pw_us as u64 * DEG_X10_PER_CKP_TOOTH as u64) / period_us).clamp(1, 5_400) as i32;
            let soi_x10 = (eoi_x10 - pw_deg_x10).rem_euclid(7200) as u16;

            if let Some(delta_deg) =
                angle_offset_in_tooth_window(tooth.crank_angle_deg_x10, soi_x10)
            {
                self.inj_scheduled_in_cycle[idx] = true;
                batch.push(InjCompareAction {
                    cylinder: cyl,
                    delay_from_tooth_us: angle_delta_to_delay_us(delta_deg, tooth.tooth_period_us),
                    pulse_width_us: pw_us,
                });
            } else {
                let behind_x10 =
                    (tooth.crank_angle_deg_x10 as i32 - soi_x10 as i32).rem_euclid(7200);
                if (1..=300).contains(&behind_x10) {
                    self.inj_scheduled_in_cycle[idx] = true;
                    batch.push(InjCompareAction {
                        cylinder: cyl,
                        delay_from_tooth_us: 0,
                        pulse_width_us: pw_us,
                    });
                }
            }
        }

        batch
    }
}

pub struct FuelModel {
    stft_pi: PidController,
    prev_tps_pct: f32,
    accel_enrich_us: f32,
    tooth_scheduler: FuelToothScheduler,
}

impl Default for FuelModel {
    fn default() -> Self {
        Self::new()
    }
}

impl FuelModel {
    pub const fn new() -> Self {
        Self {
            // Closed-loop Short-Term Fuel Trim authority bounded to [-25%, +25%]
            stft_pi: PidController::new(12.0, 28.0, 0.0, -25.0, 25.0),
            prev_tps_pct: 0.0,
            accel_enrich_us: 0.0,
            tooth_scheduler: FuelToothScheduler::new(),
        }
    }

    #[inline]
    pub fn reset_cycle(&mut self) {
        self.tooth_scheduler.reset_cycle();
    }

    /// Looks up the target lambda (`0.65..=1.25`) from the RPM × MAP target table.
    pub fn lookup_target_lambda(&self, rpm: u16, map_kpa: f32, cal: &CalibrationBlob) -> f32 {
        (interp_2d_u16(
            &cal.rpm_bins,
            &cal.map_kpa_bins,
            &cal.target_lambda_x1000,
            rpm as f32,
            map_kpa,
        ) * 0.001)
            .clamp(0.65, 1.25)
    }

    pub fn compute_1khz(
        &mut self,
        rpm: u16,
        use_vtec_map: bool,
        sensors: &SensorSnapshot,
        wbo_state: WidebandState,
        measured_lambda: f32,
        cal: &CalibrationBlob,
    ) -> FuelOutput {
        self.compute_1khz_with_trims(
            rpm,
            use_vtec_map,
            sensors,
            wbo_state,
            measured_lambda,
            cal,
            1.0,
            true,
            false,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub fn compute_1khz_with_trims(
        &mut self,
        rpm: u16,
        use_vtec_map: bool,
        sensors: &SensorSnapshot,
        wbo_state: WidebandState,
        measured_lambda: f32,
        cal: &CalibrationBlob,
        fuel_multiplier: f32,
        allow_negative_stft: bool,
        hard_cut_all: bool,
    ) -> FuelOutput {
        // 1. Hard rev-limiter or turbo safety cut (overboost / lean-boost) cuts all injectors!
        if hard_cut_all || rpm >= cal.rev_limit_hard_rpm {
            self.stft_pi.reset();
            return FuelOutput {
                ve_pct: 0.0,
                target_lambda: 1.0,
                stft_pct: 0.0,
                final_pw_us: [0; 4],
            };
        }

        let deadtime_us = interp_1d_u16(
            &cal.vbatt_bins_x10,
            &cal.inj_deadtime_us,
            sensors.vbatt_v * 10.0,
        );

        // 2. Engine stopped (`RPM == 0`): injectors strictly OFF
        if rpm == 0 {
            self.stft_pi.reset();
            self.prev_tps_pct = sensors.tps_pct;
            self.accel_enrich_us = 0.0;
            return FuelOutput {
                ve_pct: 0.0,
                target_lambda: 1.0,
                stft_pct: 0.0,
                final_pw_us: [0; 4],
            };
        }

        // 3. Cranking mode (`STS` starter active or `RPM < 380`)
        if sensors.switches.sts_cranking || rpm < 380 {
            self.stft_pi.reset();
            let crank_us = interp_1d_i16_u32(&cal.ect_bins_c, &cal.cranking_pw_us, sensors.ect_c)
                + deadtime_us;
            let pw = crank_us.clamp(0.0, 90_000.0) as u32;
            return FuelOutput {
                ve_pct: 60.0,
                target_lambda: 0.85,
                stft_pct: 0.0,
                final_pw_us: [pw; 4],
            };
        }

        // 4. Speed-Density Ideal Gas Law (`m_air = (P * V_cyl * VE) / (R * T_iat)`)
        let ve_table = if use_vtec_map {
            &cal.ve_vtec_cam_x10
        } else {
            &cal.ve_low_cam_x10
        };
        let ve_pct = interp_2d_u16(
            &cal.rpm_bins,
            &cal.map_kpa_bins,
            ve_table,
            rpm as f32,
            sensors.map_kpa,
        ) * 0.1;

        let target_lambda = self.lookup_target_lambda(rpm, sensors.map_kpa, cal);

        let v_cyl_m3 = (cal.displacement_cc as f32 / 4.0) * 1e-6;
        let p_pa = sensors.map_kpa * 1_000.0;
        let t_kelvin = (sensors.iat_c + 273.15).max(220.0);
        // Air mass in grams per intake event
        let air_mass_g =
            ((p_pa * v_cyl_m3) / (R_AIR_J_PER_KG_K * t_kelvin)) * (ve_pct * 0.01) * 1_000.0;

        let stoich_afr = (cal.stoich_afr_x100 as f32) * 0.01;
        let desired_afr = (stoich_afr * target_lambda).max(8.0);
        let base_fuel_g = air_mass_g / desired_afr;

        // Injector mass flow rate in g / µs:
        // (cc/min * 0.745 g/cc) / 60_000_000 µs/min
        let inj_g_per_us =
            ((cal.injector_flow_cc_min.max(50) as f32) * GASOLINE_DENSITY_G_PER_CC) / 60_000_000.0;
        let base_pw_us = base_fuel_g / inj_g_per_us;

        // 5. Warmup Enrichment (`ECT`)
        let warmup_mult =
            interp_1d_i16_u16(&cal.ect_bins_c, &cal.warmup_enrichment_pct, sensors.ect_c) * 0.01;

        // 6. Transient TPS Wall-Wetting Acceleration Enrichment (`1 kHz` decay)
        let dtps_per_ms = sensors.tps_pct - self.prev_tps_pct;
        self.prev_tps_pct = sensors.tps_pct;
        if dtps_per_ms > 0.35 {
            self.accel_enrich_us = (self.accel_enrich_us + dtps_per_ms * 420.0).min(4_500.0);
        } else {
            self.accel_enrich_us *= 0.94; // Smooth ~35 ms film evaporation decay
            if self.accel_enrich_us < 10.0 {
                self.accel_enrich_us = 0.0;
            }
        }

        // 7. Closed-Loop Bosch LSU 4.9 Short-Term Fuel Trim (`ECT >= 60 °C` & not in fuel cut)
        //    When boosted above `stft_max_map_kpa`, disallow negative STFT fuel pulls (`< 0%`)
        //    so exhaust/wideband noise can never lean out high boost.
        self.stft_pi.out_min = if allow_negative_stft { -25.0 } else { 0.0 };
        let stft_pct = if wbo_state == WidebandState::ClosedLoopReady && sensors.ect_c >= 60.0 {
            // Positive error when measured exhaust is leaner (`λ_meas > λ_target`) -> add fuel!
            let lambda_err = measured_lambda - target_lambda;
            self.stft_pi.update(lambda_err, 0.001)
        } else {
            self.stft_pi.reset();
            0.0
        };
        let stft_mult = 1.0 + (stft_pct * 0.01);
        let safe_mult = fuel_multiplier.clamp(0.80, 2.20);

        let effective_pw_us =
            (base_pw_us * warmup_mult * safe_mult * stft_mult) + self.accel_enrich_us + deadtime_us;
        let clamped_pw = effective_pw_us.clamp(0.0, 60_000.0) as u32;

        // Soft rev-limit cuts 2 alternating cylinders (`Cyl2` & `Cyl3`) before hard cut
        let mut final_pw_us = [clamped_pw; 4];
        if rpm >= cal.rev_limit_soft_rpm {
            final_pw_us[1] = 0;
            final_pw_us[2] = 0;
        }

        FuelOutput {
            ve_pct,
            target_lambda,
            stft_pct,
            final_pw_us,
        }
    }

    /// Angle-domain scheduler invoked on every `CKP` tooth edge (`TIM2` capture).
    ///
    /// Phases sequential port-fuel injection (`1-3-4-2`) by anchoring End-of-Injection at
    /// `DEFAULT_INJ_EOI_ATDC_DEG_X10` and opening each injector inside the `< 30.0°` `CKP` window
    /// containing Start-of-Injection, guaranteeing at most one open event per cylinder per `720°`.
    pub fn schedule_on_ckp_tooth(
        &mut self,
        tooth: &CkpToothEvent,
        inj_pw_us: &[u32; 4],
    ) -> InjScheduleBatch {
        self.tooth_scheduler.schedule_on_ckp_tooth(tooth, inj_pw_us)
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{CalibrationBlob, Cylinder, SensorSnapshot, SyncState, WidebandState};

    use crate::trigger::CkpToothEvent;

    use super::FuelModel;

    #[test]
    fn speed_density_produces_realistic_honda_240cc_pulsewidths() {
        let cal = CalibrationBlob::default_honda_obd1();
        let mut fuel = FuelModel::new();
        let sns = SensorSnapshot {
            map_kpa: 32.0, // Warm idle manifold vacuum
            tps_pct: 0.0,
            ect_c: 88.0,
            iat_c: 32.0,
            vbatt_v: 14.1,
            ..Default::default()
        };

        let idle = fuel.compute_1khz(
            850,
            /* high_cam= */ false,
            &sns,
            WidebandState::ClosedLoopReady,
            1.00,
            &cal,
        );
        // Stock 1.6L Honda on 240cc injectors idles right around 2.1 - 3.2 ms total PW
        // (incl ~0.87ms deadtime)!
        assert!(
            (1_900..=3_400).contains(&idle.final_pw_us[0]),
            "Unexpected warm idle PW: {} us",
            idle.final_pw_us[0]
        );

        // Instant 0% -> 100% WOT stomp triggers transient wall-wetting enrichment (+4.5ms),
        // then decays smoothly to steady-state WOT VTEC (~11 - 15.5 ms PW on 240cc injectors)
        let wot_sns = SensorSnapshot {
            map_kpa: 100.0,
            tps_pct: 100.0,
            ..sns
        };
        let stomp = fuel.compute_1khz(
            6800,
            true,
            &wot_sns,
            WidebandState::ClosedLoopReady,
            0.865,
            &cal,
        );
        let mut steady_wot = stomp;
        for _ in 0..120 {
            steady_wot = fuel.compute_1khz(
                6800,
                true,
                &wot_sns,
                WidebandState::ClosedLoopReady,
                0.865,
                &cal,
            );
        }
        assert!(
            stomp.final_pw_us[0] > steady_wot.final_pw_us[0] + 3_000,
            "Throttle stomp must add transient wall-wetting enrichment"
        );
        assert!(
            (10_500..=16_000).contains(&steady_wot.final_pw_us[0]),
            "Unexpected steady WOT VTEC PW: {} us",
            steady_wot.final_pw_us[0]
        );
    }

    #[test]
    fn schedules_all_four_injectors_in_exact_1_3_4_2_sequence_over_720_deg() {
        let mut fuel = FuelModel::new();

        // 1. Steady 3,000 RPM (`1,666 µs` per `30°` tooth), `5.0 ms` (`5,000 µs` = `90°`) PW:
        //    SOI = EOI(450°) - 90° = 360° ATDC -> fires starting at tooth 0 in [4, 2, 1, 3].
        for _cycle in 0..2 {
            let mut order = [Cylinder::Cyl1; 4];
            let mut count = 0usize;
            for tooth_idx in 0..24u8 {
                let ev = CkpToothEvent {
                    sync_state: SyncState::FullSynced720,
                    tooth_index: tooth_idx,
                    crank_angle_deg_x10: tooth_idx as u16 * 300,
                    tooth_period_us: 1_666,
                    rpm: 3000,
                    timestamp_us: tooth_idx as u32 * 1_666,
                };
                let batch = fuel.schedule_on_ckp_tooth(&ev, &[5_000; 4]);
                for act in batch.actions.iter().flatten() {
                    assert!(act.delay_from_tooth_us <= 1_666);
                    order[count] = act.cylinder;
                    count += 1;
                }
            }
            assert_eq!(count, 4, "Each cylinder must inject once per 720° cam rev");
            assert_eq!(
                order,
                [
                    Cylinder::Cyl4,
                    Cylinder::Cyl2,
                    Cylinder::Cyl1,
                    Cylinder::Cyl3
                ]
            );
        }

        // 2. Dynamic PW transient (`5.0 ms` -> `9.0 ms` -> `3.0 ms`) across 4 cam revolutions:
        //    no cylinder ever double-fires closer than 540° crank degrees!
        let mut last_fire_tooth: [Option<u32>; 4] = [None; 4];
        for abs_tooth in 0..(24 * 4u32) {
            let tooth_idx = (abs_tooth % 24) as u8;
            let pw = if abs_tooth < 30 {
                5_000
            } else if abs_tooth < 65 {
                9_000
            } else {
                3_000
            };
            let ev = CkpToothEvent {
                sync_state: SyncState::FullSynced720,
                tooth_index: tooth_idx,
                crank_angle_deg_x10: tooth_idx as u16 * 300,
                tooth_period_us: 1_666,
                rpm: 3000,
                timestamp_us: abs_tooth * 1_666,
            };
            let batch = fuel.schedule_on_ckp_tooth(&ev, &[pw; 4]);
            for act in batch.actions.iter().flatten() {
                if let Some(prev) = last_fire_tooth[act.cylinder.index()] {
                    assert!(
                        abs_tooth - prev >= 18,
                        "Cylinder {:?} double-fired too close ({} teeth)",
                        act.cylinder,
                        abs_tooth - prev
                    );
                }
                last_fire_tooth[act.cylinder.index()] = Some(abs_tooth);
            }
        }
        assert!(last_fire_tooth.iter().all(Option::is_some));
    }

    #[test]
    fn cranking_closed_loop_stft_and_rev_limiters_enforce_safety_bounds() {
        let cal = CalibrationBlob::default_honda_obd1();
        let mut fuel = FuelModel::new();
        let warm_sns = SensorSnapshot {
            map_kpa: 45.0,
            tps_pct: 12.0,
            ect_c: 85.0,
            iat_c: 30.0,
            vbatt_v: 14.0,
            ..Default::default()
        };

        // 1. Engine stopped (0 RPM) -> strictly 0 µs pulsewidth
        let stopped = fuel.compute_1khz(
            0,
            false,
            &warm_sns,
            WidebandState::ClosedLoopReady,
            1.0,
            &cal,
        );
        assert_eq!(stopped.final_pw_us, [0; 4]);

        // 2. Cold cranking (0 °C, 220 RPM) delivers rich prime pulsewidth (> 14,000 µs)
        let cold_crank_sns = SensorSnapshot {
            ect_c: 0.0,
            ..warm_sns
        };
        let crank = fuel.compute_1khz(
            220,
            false,
            &cold_crank_sns,
            WidebandState::WaitForEngine,
            1.0,
            &cal,
        );
        assert!(crank.final_pw_us[0] > 14_000);
        assert!((crank.target_lambda - 0.85).abs() < 1e-4);

        // 3. Closed-loop STFT adds fuel when lean (λ = 1.15 vs target 1.00) up to +25%,
        //    and resets to 0% if coolant is below 60 °C
        let mut stft = 0.0;
        for _ in 0..6_000 {
            let out = fuel.compute_1khz(
                2_500,
                false,
                &warm_sns,
                WidebandState::ClosedLoopReady,
                1.15,
                &cal,
            );
            stft = out.stft_pct;
        }
        assert!(
            (stft - 25.0).abs() < 1e-3,
            "STFT must clamp at +25%: {stft}"
        );

        let cold_cruise = fuel.compute_1khz(
            2_500,
            false,
            &SensorSnapshot {
                ect_c: 55.0,
                ..warm_sns
            },
            WidebandState::ClosedLoopReady,
            1.15,
            &cal,
        );
        assert_eq!(cold_cruise.stft_pct, 0.0);

        // 4. Soft rev limit (7,350 RPM) cuts Cyl2 & Cyl3; Hard rev limit (7,500 RPM) cuts all 4
        let soft = fuel.compute_1khz(
            7_350,
            true,
            &warm_sns,
            WidebandState::ClosedLoopReady,
            1.0,
            &cal,
        );
        assert!(soft.final_pw_us[0] > 0);
        assert_eq!(soft.final_pw_us[1], 0);
        assert_eq!(soft.final_pw_us[2], 0);
        assert!(soft.final_pw_us[3] > 0);

        let hard = fuel.compute_1khz(
            7_500,
            true,
            &warm_sns,
            WidebandState::ClosedLoopReady,
            1.0,
            &cal,
        );
        assert_eq!(hard.final_pw_us, [0; 4]);
    }
}
