//! Top-level ECU coordinator orchestrating `1 kHz` control ticks and hard real-time tooth edges.

use crate::boost::{BoostController, sanitize_boost_config};
use crate::fuel::{FuelModel, FuelToothScheduler, InjScheduleBatch};
use crate::ignition::{IgnitionModel, IgnitionToothScheduler, ToothScheduleBatch};
use crate::knock::{KnockDetector, KnockWindowPlan, plan_knock_window_on_ckp_tooth};
use crate::math::interp_1d_i16_u16;
use crate::sensors::SensorProcessor;
use crate::trigger::{CkpToothEvent, HondaObd1Decoder};
use crate::vtec::VtecController;
use crate::wideband::Lsu49Controller;
use ecu_types::{
    ActuatorOutputs, BoostConfig, BoostTelemetry, CalibrationBlob, Cylinder, FaultFlags,
    FlexFuelState, KnockConfig, KnockTelemetry, OutputEnableMask, RawSensorAdc, SensorSnapshot,
    SwitchState, SyncState, TelemetrySnapshot,
};

/// Combined ignition & injector compare actions emitted on a single `CKP` tooth edge.
#[derive(Debug, Clone, Default)]
pub struct ToothScheduleOutputs {
    pub event: Option<CkpToothEvent>,
    pub cop: ToothScheduleBatch,
    pub inj: InjScheduleBatch,
}

/// Control-loop outputs pushed from `1 kHz` `EngineCoordinator` into the `TIM2` ISR domain.
#[derive(Debug, Clone, Copy)]
pub struct RtControlTargets {
    pub cal_ready: bool,
    pub trigger_offset_deg_x10: i16,
    pub spark_advance_deg_x10: i16,
    pub cop_dwell_us: u32,
    pub per_cyl_knock_retard_deg_x10: [i16; 4],
    pub inj_pw_us: [u32; 4],
    pub knock_config: KnockConfig,
    pub output_mask: OutputEnableMask,
}

impl Default for RtControlTargets {
    fn default() -> Self {
        Self::new()
    }
}

impl RtControlTargets {
    pub const fn new() -> Self {
        Self {
            cal_ready: false,
            trigger_offset_deg_x10: 0,
            spark_advance_deg_x10: 160,
            cop_dwell_us: 3_000,
            per_cyl_knock_retard_deg_x10: [0; 4],
            inj_pw_us: [0; 4],
            knock_config: KnockConfig::default_honda_obd1(),
            output_mask: OutputEnableMask::all_enabled(),
        }
    }
}

/// Real-time trigger snapshot pulled atomically from the `TIM2` ISR domain on each `1 kHz` tick.
#[derive(Debug, Clone, Copy)]
pub struct RtTriggerStatus {
    pub trigger: HondaObd1Decoder,
    pub last_angle_deg_x10: u16,
    pub trigger_faults: FaultFlags,
}

/// Interrupt-domain real-time trigger & compare scheduler (`TIM2` ISR -> `TIM1`/`TIM3`/`TIM7`).
///
/// Decoupled cleanly from slow `1 kHz` table lookups, `USB`, `knock DSP`, and `SDMMC` logging so
/// no Thread-Mode execution or `ECU_STATE` lock can drop a `CKP` edge or delay spark/fuel compare
/// arming.
pub struct RtTriggerEngine {
    trigger: HondaObd1Decoder,
    ignition_tooth: IgnitionToothScheduler,
    fuel_tooth: FuelToothScheduler,
    last_angle_deg_x10: u16,
    trigger_faults: FaultFlags,
    targets: RtControlTargets,
}

impl Default for RtTriggerEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl RtTriggerEngine {
    pub const fn new() -> Self {
        Self {
            trigger: HondaObd1Decoder::new(),
            ignition_tooth: IgnitionToothScheduler::new(),
            fuel_tooth: FuelToothScheduler::new(),
            last_angle_deg_x10: 0,
            trigger_faults: FaultFlags(0),
            targets: RtControlTargets::new(),
        }
    }

    #[inline]
    pub fn is_full_synced(&self) -> bool {
        self.trigger.sync_state() == SyncState::FullSynced720
    }

    #[inline]
    pub fn on_cyp_capture(&mut self, timestamp_us: u32) {
        self.trigger
            .on_cyp_capture(timestamp_us, &mut self.trigger_faults);
    }

    #[inline]
    pub fn on_tdc_capture(&mut self, timestamp_us: u32) {
        self.trigger
            .on_tdc_capture(timestamp_us, &mut self.trigger_faults);
    }

    #[inline]
    pub fn on_ckp_capture(
        &mut self,
        timestamp_us: u32,
    ) -> (ToothScheduleOutputs, Option<KnockWindowPlan>) {
        if !self.targets.cal_ready {
            return (ToothScheduleOutputs::default(), None);
        }
        if let Some(ev) = self.trigger.on_ckp_capture(
            timestamp_us,
            self.targets.trigger_offset_deg_x10,
            &mut self.trigger_faults,
        ) {
            self.last_angle_deg_x10 = ev.crank_angle_deg_x10;
            let cop = self.ignition_tooth.schedule_on_ckp_tooth_masked(
                &ev,
                self.targets.spark_advance_deg_x10,
                &self.targets.per_cyl_knock_retard_deg_x10,
                self.targets.cop_dwell_us,
                self.targets.output_mask,
            );
            let masked_inj_pw = apply_inj_mask(self.targets.inj_pw_us, self.targets.output_mask);
            let inj = self.fuel_tooth.schedule_on_ckp_tooth(&ev, &masked_inj_pw);
            let knock = plan_knock_window_on_ckp_tooth(&self.targets.knock_config, &ev);
            (
                ToothScheduleOutputs {
                    event: Some(ev),
                    cop,
                    inj,
                },
                knock,
            )
        } else {
            self.ignition_tooth.reset_cycle();
            self.fuel_tooth.reset_cycle();
            (ToothScheduleOutputs::default(), None)
        }
    }

    /// Runs the `400 ms` stall watchdog and returns an atomic trigger status snapshot for `1 kHz`.
    #[inline]
    pub fn sync_1khz(&mut self, now_us: u32) -> RtTriggerStatus {
        self.trigger.check_stall_timeout(now_us);
        if self.trigger.sync_state() == SyncState::Unsynced {
            self.ignition_tooth.reset_cycle();
            self.fuel_tooth.reset_cycle();
        }
        let faults = self.trigger_faults;
        self.trigger_faults = FaultFlags(0);
        RtTriggerStatus {
            trigger: self.trigger,
            last_angle_deg_x10: self.last_angle_deg_x10,
            trigger_faults: faults,
        }
    }

    #[inline]
    pub fn update_targets(&mut self, targets: RtControlTargets) {
        self.targets = targets;
    }
}

pub struct EngineCoordinator {
    pub trigger: HondaObd1Decoder,
    pub sensors_proc: SensorProcessor,
    pub vtec: VtecController,
    pub wideband: Lsu49Controller,
    pub fuel: FuelModel,
    pub ignition: IgnitionModel,
    pub knock: KnockDetector,
    pub boost: BoostController,

    uptime_ms: u32,
    seq: u32,
    last_angle_deg_x10: u16,
    fan_on_latched: bool,
    timing_override_deg_x10: Option<i16>,
    output_mask: OutputEnableMask,
    runtime_cut_mask: OutputEnableMask,
    fuel_pump_override_ms: u16,
    boost_cfg: BoostConfig,
    flex_state: FlexFuelState,

    latest_sensors: SensorSnapshot,
    latest_actuators: ActuatorOutputs,
    last_computed_inj_pw_us: [u32; 4],
    latest_faults: FaultFlags,
    latest_ve_pct: f32,
    latest_target_lambda: f32,
    latest_stft_pct: f32,
}

impl Default for EngineCoordinator {
    fn default() -> Self {
        Self::new()
    }
}

impl EngineCoordinator {
    pub const fn new() -> Self {
        Self {
            trigger: HondaObd1Decoder::new(),
            sensors_proc: SensorProcessor::new(),
            vtec: VtecController::new(),
            wideband: Lsu49Controller::new(),
            fuel: FuelModel::new(),
            ignition: IgnitionModel::new(),
            knock: KnockDetector::new(),
            boost: BoostController::new(),
            uptime_ms: 0,
            seq: 0,
            last_angle_deg_x10: 0,
            fan_on_latched: false,
            timing_override_deg_x10: None,
            output_mask: OutputEnableMask::all_enabled(),
            runtime_cut_mask: OutputEnableMask::all_enabled(),
            fuel_pump_override_ms: 0,
            boost_cfg: BoostConfig::default_honda_obd1(),
            flex_state: FlexFuelState::new(),
            latest_sensors: SensorSnapshot {
                map_kpa: 100.0,
                tps_pct: 0.0,
                ect_c: 25.0,
                iat_c: 25.0,
                vbatt_v: 12.6,
                vcc_5v_v: 5.0,
                eld_amps: 0.0,
                vss_kmh: 0.0,
                switches: SwitchState {
                    sts_cranking: false,
                    vtp_pressurized: false,
                    scs_service_jumper: false,
                    acs_ac_request: false,
                    psp_steering_load: false,
                    bksw_brake: false,
                    clutch_pressed: false,
                },
            },
            latest_actuators: ActuatorOutputs {
                inj_pw_us: [0; 4],
                spark_advance_deg_x10: 160,
                cop_dwell_us: 3000,
                vts_high_side_on: false,
                fuel_pump_relay_on: false,
                fan_relay_on: false,
                ac_clutch_relay_on: false,
                mil_lamp_on: false,
                iacv_duty_per_mille: 350,
                lsu_heater_duty_per_mille: 0,
                lsu_ip_dac_code: 2048,
            },
            last_computed_inj_pw_us: [0; 4],
            latest_faults: FaultFlags(0),
            latest_ve_pct: 0.0,
            latest_target_lambda: 1.0,
            latest_stft_pct: 0.0,
        }
    }

    #[inline]
    pub fn set_timing_override_deg_x10(&mut self, override_deg_x10: Option<i16>) {
        self.timing_override_deg_x10 = override_deg_x10;
    }

    #[inline]
    pub const fn output_enable_mask(&self) -> OutputEnableMask {
        self.output_mask
    }

    #[inline]
    pub const fn effective_output_mask(&self) -> OutputEnableMask {
        combine_masks(self.output_mask, self.runtime_cut_mask)
    }

    #[inline]
    pub fn set_output_enable_mask(&mut self, mask: OutputEnableMask) {
        self.output_mask = mask;
        let eff = self.effective_output_mask();
        self.latest_actuators.inj_pw_us = apply_inj_mask(self.latest_actuators.inj_pw_us, eff);
        self.last_computed_inj_pw_us = apply_inj_mask(self.last_computed_inj_pw_us, eff);
    }

    #[inline]
    pub fn set_boost_config(&mut self, cfg: BoostConfig) {
        self.boost_cfg = sanitize_boost_config(cfg);
    }

    #[inline]
    pub const fn boost_config(&self) -> BoostConfig {
        self.boost_cfg
    }

    #[inline]
    pub const fn boost_telemetry(&self) -> BoostTelemetry {
        self.boost.telemetry()
    }

    #[inline]
    pub fn set_flex_state(&mut self, flex: FlexFuelState) {
        self.flex_state = flex;
    }

    #[inline]
    pub const fn fuel_pump_override_ms(&self) -> u16 {
        self.fuel_pump_override_ms
    }

    #[inline]
    pub fn set_fuel_pump_override_ms(&mut self, duration_ms: u16) {
        self.fuel_pump_override_ms = duration_ms;
    }

    /// Clears latched RAM MIL flags without bypassing active safety lockouts.
    #[inline]
    pub fn clear_faults(&mut self) {
        self.latest_faults = FaultFlags(0);
        self.sensors_proc.reset_fault_counters();
        self.boost.reset_peak_hold();
    }

    #[inline]
    pub fn process_knock_window(&mut self, cyl: Cylinder, rpm: u16, samples: &[u16]) -> bool {
        self.knock.process_window(cyl, rpm, samples)
    }

    #[inline]
    pub fn set_knock_config(&mut self, cfg: KnockConfig) {
        self.knock.set_config(cfg);
    }

    #[inline]
    pub const fn knock_config(&self) -> KnockConfig {
        self.knock.config()
    }

    #[inline]
    pub fn knock_telemetry(&self, capture_overruns: u32) -> KnockTelemetry {
        self.knock.telemetry(capture_overruns)
    }

    #[inline]
    pub fn on_cyp_capture(&mut self, timestamp_us: u32) {
        self.trigger
            .on_cyp_capture(timestamp_us, &mut self.latest_faults);
    }

    #[inline]
    pub fn on_tdc_capture(&mut self, timestamp_us: u32) {
        self.trigger
            .on_tdc_capture(timestamp_us, &mut self.latest_faults);
    }

    /// Invoked on every `CKP` (`PA15 TIM2_CH1`) edge; updates crank phase & returns `TIM1` COP +
    /// `TIM3` INJ actions.
    pub fn on_ckp_capture(
        &mut self,
        timestamp_us: u32,
        cal: &CalibrationBlob,
    ) -> ToothScheduleOutputs {
        if let Some(ev) = self.trigger.on_ckp_capture(
            timestamp_us,
            cal.trigger_offset_deg_x10,
            &mut self.latest_faults,
        ) {
            self.last_angle_deg_x10 = ev.crank_angle_deg_x10;
            let eff_mask = self.effective_output_mask();
            let cop = self.ignition.schedule_on_ckp_tooth_masked(
                &ev,
                self.latest_actuators.spark_advance_deg_x10,
                self.latest_actuators.cop_dwell_us,
                eff_mask,
            );
            let masked_inj_pw = apply_inj_mask(self.latest_actuators.inj_pw_us, eff_mask);
            let inj = self.fuel.schedule_on_ckp_tooth(&ev, &masked_inj_pw);
            ToothScheduleOutputs {
                event: Some(ev),
                cop,
                inj,
            }
        } else {
            self.ignition.reset_cycle();
            self.fuel.reset_cycle();
            ToothScheduleOutputs::default()
        }
    }

    /// Synchronizes slow `1 kHz` control state with the latest interrupt-domain `TIM2` trigger snapshot.
    #[inline]
    pub fn apply_rt_status(&mut self, rt: RtTriggerStatus) {
        self.trigger = rt.trigger;
        self.last_angle_deg_x10 = rt.last_angle_deg_x10;
        self.latest_faults.insert(rt.trigger_faults.0);
        if self.trigger.sync_state() == SyncState::Unsynced {
            self.ignition.reset_cycle();
            self.fuel.reset_cycle();
        }
    }

    /// Exports the latest `1 kHz` schedule parameters into the `TIM2` ISR domain.
    #[inline]
    pub fn export_rt_targets(&self, cal: &CalibrationBlob) -> RtControlTargets {
        RtControlTargets {
            cal_ready: true,
            trigger_offset_deg_x10: cal.trigger_offset_deg_x10,
            spark_advance_deg_x10: self.latest_actuators.spark_advance_deg_x10,
            cop_dwell_us: self.latest_actuators.cop_dwell_us,
            per_cyl_knock_retard_deg_x10: self.ignition.per_cyl_knock_retard_deg_x10(),
            inj_pw_us: self.last_computed_inj_pw_us,
            knock_config: self.knock.config(),
            output_mask: self.effective_output_mask(),
        }
    }

    /// Executes the deterministic `1 kHz` (`1 ms`) control loop.
    pub fn tick_1khz(
        &mut self,
        now_us: u32,
        raw_adc: &RawSensorAdc,
        switches: SwitchState,
        vss_kmh: f32,
        pb14_vts_fault_low: bool,
        cal: &CalibrationBlob,
    ) -> ActuatorOutputs {
        self.uptime_ms = self.uptime_ms.saturating_add(1);
        self.seq = self.seq.saturating_add(1);
        self.trigger.check_stall_timeout(now_us);
        if self.trigger.sync_state() == SyncState::Unsynced {
            self.ignition.reset_cycle();
            self.fuel.reset_cycle();
        }

        let rpm = self.trigger.rpm();

        // 1. Scale ratiometric sensors & switches
        let sensors = self.sensors_proc.process(
            raw_adc,
            switches,
            vss_kmh,
            &cal.map_cal,
            &mut self.latest_faults,
        );

        // 2. Direct Bosch LSU 4.9 Wideband pump + condensation-safe heater loops
        self.wideband.update_pump_loop(
            raw_adc.lsu_un_dc_raw,
            raw_adc.lsu_un_ac_delta_raw,
            raw_adc.lsu_ip_raw,
            0.001,
        );
        self.wideband
            .update_heater_1khz(rpm, sensors.vbatt_v, &mut self.latest_faults);

        // 3. Honda VTEC state machine (`A4 VTS*` + `D6 VTP*`)
        let (vts_high_side_on, use_vtec_maps) = self.vtec.update_1khz(
            rpm,
            &sensors,
            &cal.vtec,
            pb14_vts_fault_low,
            &mut self.latest_faults,
        );

        // 4. Knock DSP 1 kHz Decay/Fault Update (evaluated before Boost knock-drop safety)
        let engine_unsynced = self.trigger.sync_state() == SyncState::Unsynced;
        self.knock
            .tick_1khz(engine_unsynced, &mut self.latest_faults);
        let max_knock_retard = self
            .knock
            .per_cyl_retard_deg_x10()
            .into_iter()
            .max()
            .unwrap_or(0);

        // 5. Turbo Boost Control, Overboost/Lean-Boost Safety, Flex Trims, 2-Step & FFS
        let table_target_lambda = self.fuel.lookup_target_lambda(rpm, sensors.map_kpa, cal);
        let boost_out = self.boost.update_1khz(
            self.trigger.sync_state(),
            rpm,
            &sensors,
            &self.flex_state,
            max_knock_retard,
            self.wideband.state(),
            self.wideband.lambda(),
            table_target_lambda,
            &mut self.latest_faults,
            &self.boost_cfg,
        );
        self.runtime_cut_mask = if boost_out.hard_cut_all {
            OutputEnableMask {
                ignition_enabled: false,
                cop_cyl_enabled: [false; 4],
                injectors_enabled: false,
                inj_cyl_enabled: [false; 4],
            }
        } else {
            OutputEnableMask {
                ignition_enabled: true,
                cop_cyl_enabled: boost_out.cyl_spark_enabled,
                injectors_enabled: true,
                inj_cyl_enabled: [true; 4],
            }
        };

        // 6. Speed-Density Fuel + Flex/Launch Multipliers + Wideband STFT Guard
        let fuel_out = self.fuel.compute_1khz_with_trims(
            rpm,
            use_vtec_maps,
            &sensors,
            self.wideband.state(),
            self.wideband.lambda(),
            cal,
            boost_out.fuel_multiplier,
            boost_out.allow_negative_stft,
            boost_out.hard_cut_all,
        );

        // 7. Ignition Advance (Flex Octane + Launch/FFS Blended Retard + Thermal/Knock)
        let suppress_knock_retard =
            switches.scs_service_jumper || self.timing_override_deg_x10.is_some();
        self.ignition
            .set_per_cyl_knock_retard_deg_x10(if suppress_knock_retard {
                [0; 4]
            } else {
                self.knock.per_cyl_retard_deg_x10()
            });
        let (spark_advance_deg_x10, cop_dwell_us) = self.ignition.compute_1khz_with_trims(
            rpm,
            use_vtec_maps,
            &sensors,
            self.timing_override_deg_x10,
            boost_out.flex_spark_add_deg_x10,
            boost_out.spark_blend_target,
            cal,
        );

        // 6. Auxiliary Honda OBD1 Connector A Relays & IACV
        // `A7/A8 FLR`: Prime fuel pump for 2.0 s at key-on, then hold ON whenever engine is
        // turning or cranking (or during a KOEO fuel-pump test prime)!
        if rpm > 0 {
            self.fuel_pump_override_ms = 0;
        } else if self.fuel_pump_override_ms > 0 {
            self.fuel_pump_override_ms = self.fuel_pump_override_ms.saturating_sub(1);
        }
        let fuel_pump_relay_on = self.uptime_ms <= 2_000
            || rpm > 0
            || sensors.switches.sts_cranking
            || self.fuel_pump_override_ms > 0;

        // `A12 FANC*`: Radiator fan hysteresis
        if sensors.ect_c >= cal.fan_on_ect_c as f32 {
            self.fan_on_latched = true;
        } else if sensors.ect_c <= cal.fan_off_ect_c as f32 {
            self.fan_on_latched = false;
        }

        // `A15 ACC*`: Engage A/C clutch when requested unless WOT (> 82% TPS), high RPM (> 6,200),
        // or stalling (< 600)
        let ac_clutch_relay_on = sensors.switches.acs_ac_request
            && (600..=6_200).contains(&rpm)
            && sensors.tps_pct < 82.0;

        // `A9 IACV` (`450 Hz` PWM): Warmup base curve + idle-up adders for A/C (`+8%`) & power
        // steering (`+4%`)
        let mut iacv_duty = interp_1d_i16_u16(
            &cal.ect_bins_c,
            &cal.iacv_base_duty_per_mille,
            sensors.ect_c,
        );
        if sensors.switches.acs_ac_request {
            iacv_duty += 80.0;
        }
        if sensors.switches.psp_steering_load {
            iacv_duty += 40.0;
        }

        // `A13 MIL`: Key-on 2.0 s bulb check, solid ON if fault active (or blink when SCS jumped)
        let mil_lamp_on = if self.uptime_ms <= 2_000 {
            true
        } else if !self.latest_faults.is_empty() {
            if sensors.switches.scs_service_jumper {
                (self.uptime_ms / 400).is_multiple_of(2)
            } else {
                true
            }
        } else {
            false
        };

        let eff_mask = self.effective_output_mask();
        let masked_pw_us = apply_inj_mask(fuel_out.final_pw_us, eff_mask);
        let actuators = ActuatorOutputs {
            inj_pw_us: if self.trigger.sync_state() == SyncState::Unsynced {
                [0; 4]
            } else {
                masked_pw_us
            },
            spark_advance_deg_x10,
            cop_dwell_us,
            vts_high_side_on,
            fuel_pump_relay_on,
            fan_relay_on: self.fan_on_latched,
            ac_clutch_relay_on,
            mil_lamp_on,
            iacv_duty_per_mille: iacv_duty.clamp(0.0, 950.0) as u16,
            lsu_heater_duty_per_mille: self.wideband.heater_duty_per_mille(),
            lsu_ip_dac_code: self.wideband.ip_dac_code(),
        };

        self.latest_sensors = sensors;
        self.latest_actuators = actuators;
        self.last_computed_inj_pw_us = masked_pw_us;
        self.latest_ve_pct = fuel_out.ve_pct;
        self.latest_target_lambda = fuel_out.target_lambda;
        self.latest_stft_pct = fuel_out.stft_pct;

        actuators
    }

    /// Exports an atomic telemetry snapshot for USB-C (`proto-native` / `TunerStudio`) & SD log.
    pub fn snapshot(&self) -> TelemetrySnapshot {
        TelemetrySnapshot {
            seq: self.seq,
            uptime_ms: self.uptime_ms,
            sync_state: self.trigger.sync_state(),
            rpm: self.trigger.rpm(),
            crank_angle_deg_x10: self.last_angle_deg_x10,
            sync_loss_count: self.trigger.sync_loss_count(),
            sensors: self.latest_sensors,
            vtec_state: self.vtec.state(),
            wideband_state: self.wideband.state(),
            lambda: self.wideband.lambda(),
            lsu_ri_ohms: self.wideband.ri_ohms(),
            ve_pct: self.latest_ve_pct,
            target_lambda: self.latest_target_lambda,
            stft_pct: self.latest_stft_pct,
            actuators: self.latest_actuators,
            faults: self.latest_faults,
        }
    }
}

#[inline]
const fn combine_masks(
    user_mask: OutputEnableMask,
    runtime_mask: OutputEnableMask,
) -> OutputEnableMask {
    OutputEnableMask {
        ignition_enabled: user_mask.ignition_enabled && runtime_mask.ignition_enabled,
        cop_cyl_enabled: [
            user_mask.cop_cyl_enabled[0] && runtime_mask.cop_cyl_enabled[0],
            user_mask.cop_cyl_enabled[1] && runtime_mask.cop_cyl_enabled[1],
            user_mask.cop_cyl_enabled[2] && runtime_mask.cop_cyl_enabled[2],
            user_mask.cop_cyl_enabled[3] && runtime_mask.cop_cyl_enabled[3],
        ],
        injectors_enabled: user_mask.injectors_enabled && runtime_mask.injectors_enabled,
        inj_cyl_enabled: [
            user_mask.inj_cyl_enabled[0] && runtime_mask.inj_cyl_enabled[0],
            user_mask.inj_cyl_enabled[1] && runtime_mask.inj_cyl_enabled[1],
            user_mask.inj_cyl_enabled[2] && runtime_mask.inj_cyl_enabled[2],
            user_mask.inj_cyl_enabled[3] && runtime_mask.inj_cyl_enabled[3],
        ],
    }
}

#[inline]
const fn apply_inj_mask(mut pw_us: [u32; 4], mask: OutputEnableMask) -> [u32; 4] {
    if !mask.injectors_enabled {
        return [0; 4];
    }
    let mut i = 0;
    while i < 4 {
        if !mask.inj_cyl_enabled[i] {
            pw_us[i] = 0;
        }
        i += 1;
    }
    pw_us
}

#[cfg(test)]
mod tests {
    use ecu_types::{CalibrationBlob, FaultFlags, RawSensorAdc, SwitchState, SyncState};

    use crate::sensors::{ADC_FULL_SCALE, VREF_VOLTS};

    use super::EngineCoordinator;

    fn volts_5v_to_adc(v_harness: f32) -> u16 {
        let v_pin = v_harness * 0.625;
        ((v_pin / VREF_VOLTS) * ADC_FULL_SCALE) as u16
    }

    fn nominal_raw_adc(ect_v: f32, tps_v: f32) -> RawSensorAdc {
        RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00),
            map_raw: volts_5v_to_adc(1.80), // ~65 kPa
            tps_raw: volts_5v_to_adc(tps_v),
            ect_raw: volts_5v_to_adc(ect_v),
            iat_raw: volts_5v_to_adc(2.411), // ~20 °C
            vbatt_raw: (((14.0 * 15.0 / 115.0) / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
            eld_raw: volts_5v_to_adc(3.80),
            lsu_un_dc_raw: ((2.100 / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
            lsu_un_ac_delta_raw: ((0.0961 / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
            lsu_ip_raw: ((1.650 / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
        }
    }

    #[test]
    fn fuel_pump_prime_mil_bulb_check_fan_hysteresis_and_iacv_idle_up() {
        let cal = CalibrationBlob::default_honda_obd1();
        let mut ecu = EngineCoordinator::new();
        // ~85 °C warm ECT (0.45V), closed throttle (0.45V)
        let warm_adc = nominal_raw_adc(0.45, 0.45);

        // 1. Key-ON 2.0 s (2,000 ticks) primes fuel pump & illuminates MIL bulb check, with 0 µs
        // INJ PW
        let mut act = ecu.tick_1khz(1_000, &warm_adc, SwitchState::default(), 0.0, false, &cal);
        assert!(act.fuel_pump_relay_on);
        assert!(act.mil_lamp_on);
        assert_eq!(act.inj_pw_us, [0; 4]);

        for tick in 2..=2_000u32 {
            act = ecu.tick_1khz(
                tick * 1_000,
                &warm_adc,
                SwitchState::default(),
                0.0,
                false,
                &cal,
            );
        }
        assert!(act.fuel_pump_relay_on);
        assert!(act.mil_lamp_on);

        // 2. Tick 2,001 with engine still stopped: fuel pump relay & MIL both turn OFF!
        act = ecu.tick_1khz(
            2_001_000,
            &warm_adc,
            SwitchState::default(),
            0.0,
            false,
            &cal,
        );
        assert!(!act.fuel_pump_relay_on);
        assert!(!act.mil_lamp_on);
        let base_iacv = act.iacv_duty_per_mille;

        // 3. Starter cranking (`sts_cranking`) + A/C request + power steering load
        let sw_load = SwitchState {
            sts_cranking: true,
            acs_ac_request: true,
            psp_steering_load: true,
            ..Default::default()
        };
        act = ecu.tick_1khz(2_002_000, &warm_adc, sw_load, 0.0, false, &cal);
        assert!(act.fuel_pump_relay_on);
        assert_eq!(act.iacv_duty_per_mille, base_iacv + 120);

        // 4. Radiator fan hysteresis: heat ECT > 93 °C (0.32V -> ~98 °C) -> latches ON,
        //    holds ON at ~91 °C (0.39V), and turns OFF below 89 °C (0.50V -> ~82 °C)
        let hot_adc = nominal_raw_adc(0.32, 0.45);
        for i in 0..80u32 {
            act = ecu.tick_1khz(
                (2_010 + i) * 1_000,
                &hot_adc,
                SwitchState::default(),
                0.0,
                false,
                &cal,
            );
        }
        assert!(act.fan_relay_on);

        let mid_adc = nominal_raw_adc(0.39, 0.45);
        for i in 0..80u32 {
            act = ecu.tick_1khz(
                (2_100 + i) * 1_000,
                &mid_adc,
                SwitchState::default(),
                0.0,
                false,
                &cal,
            );
        }
        assert!(act.fan_relay_on);

        let cool_adc = nominal_raw_adc(0.55, 0.45);
        for i in 0..80u32 {
            act = ecu.tick_1khz(
                (2_200 + i) * 1_000,
                &cool_adc,
                SwitchState::default(),
                0.0,
                false,
                &cal,
            );
        }
        assert!(!act.fan_relay_on);

        // 5. Fault activates solid MIL normally, and blinks at 400 ms period when SCS jumper is on
        act = ecu.tick_1khz(
            2_400_000,
            &cool_adc,
            SwitchState::default(),
            0.0,
            true,
            &cal,
        );
        assert!(ecu.snapshot().faults.contains(FaultFlags::VTS_SOLENOID_HW));
        assert!(act.mil_lamp_on);

        let scs_sw = SwitchState {
            scs_service_jumper: true,
            ..Default::default()
        };
        let mut saw_on = false;
        let mut saw_off = false;
        for i in 0..900u32 {
            let a = ecu.tick_1khz((2_500 + i) * 1_000, &cool_adc, scs_sw, 0.0, false, &cal);
            if a.mil_lamp_on {
                saw_on = true;
            } else {
                saw_off = true;
            }
        }
        assert!(saw_on && saw_off, "SCS jumper must blink MIL on fault");
    }

    #[test]
    fn full_720_tooth_scheduling_ac_wot_cutout_and_stall_safety_recovery() {
        let cal = CalibrationBlob::default_honda_obd1();
        let mut ecu = EngineCoordinator::new();
        let cruise_adc = nominal_raw_adc(0.42, 1.26); // ~88 °C, ~20% TPS
        let dt_us: u32 = 2_000; // 2,500 RPM (`2,000 µs` per 30° tooth)
        let mut ts_us: u32 = 10_000;

        // Prime first CKP edge
        let _ = ecu.on_ckp_capture(ts_us, &cal);

        let ac_sw = SwitchState {
            acs_ac_request: true,
            ..Default::default()
        };

        let mut spark_fires = 0usize;
        let mut inj_fires = 0usize;

        // Run 2 full 720° cam cycles (48 CKP teeth) with 1 kHz control ticks interleaved
        for cycle in 0..2u8 {
            for tooth in 0..24u8 {
                if tooth == 0 {
                    ecu.on_cyp_capture(ts_us + 50);
                }
                if tooth.is_multiple_of(6) {
                    ecu.on_tdc_capture(ts_us + 100);
                }
                ts_us += dt_us;
                let out = ecu.on_ckp_capture(ts_us, &cal);
                if cycle == 1 {
                    spark_fires += out
                        .cop
                        .actions
                        .iter()
                        .flatten()
                        .filter(|a| !a.drive_dwell_high)
                        .count();
                    inj_fires += out.inj.actions.iter().flatten().count();
                }
                let _ = ecu.tick_1khz(ts_us, &cruise_adc, ac_sw, 60.0, false, &cal);
            }
        }

        let snap = ecu.snapshot();
        assert_eq!(snap.sync_state, SyncState::FullSynced720);
        assert_eq!(snap.rpm, 2_500);
        assert!(snap.actuators.inj_pw_us[0] > 1_500);
        assert!(snap.actuators.ac_clutch_relay_on);
        assert_eq!(spark_fires, 4);
        assert_eq!(inj_fires, 4);

        // WOT (> 82% TPS -> 4.30V = 95% TPS) cuts out A/C clutch; timing override locks spark
        ecu.set_timing_override_deg_x10(Some(160));
        let wot_adc = nominal_raw_adc(0.42, 4.30);
        let mut act = snap.actuators;
        for i in 0..20u32 {
            ts_us += 1_000;
            if i.is_multiple_of(2) {
                let _ = ecu.on_ckp_capture(ts_us, &cal);
            }
            act = ecu.tick_1khz(ts_us, &wot_adc, ac_sw, 60.0, false, &cal);
        }
        assert!(!act.ac_clutch_relay_on);
        assert_eq!(act.spark_advance_deg_x10, 160);

        // Advance past the 2,000 ms key-on prime window, then simulate stall (> 400 ms no CKP)
        for _ in 0..2_100 {
            ts_us += 1_000;
            act = ecu.tick_1khz(ts_us, &cruise_adc, ac_sw, 0.0, false, &cal);
        }
        let stalled = ecu.snapshot();
        assert_eq!(stalled.sync_state, SyncState::Unsynced);
        assert_eq!(stalled.rpm, 0);
        assert_eq!(act.inj_pw_us, [0; 4]);
        assert!(!act.fuel_pump_relay_on);
        assert!(!act.ac_clutch_relay_on);
    }

    #[test]
    fn rt_trigger_engine_and_1khz_coordinator_split_stays_coherent() {
        use super::RtTriggerEngine;

        let cal = CalibrationBlob::default_honda_obd1();
        let mut ecu = EngineCoordinator::new();
        let mut rt = RtTriggerEngine::new();
        let cruise_adc = nominal_raw_adc(0.42, 1.26);
        let dt_us: u32 = 1_667; // ~3,000 RPM
        let mut ts_us: u32 = 10_000;

        rt.update_targets(ecu.export_rt_targets(&cal));
        let _ = rt.on_ckp_capture(ts_us);

        let mut cycle1_sparks = [0u8; 4];
        let mut cycle1_inj = [0u8; 4];
        for cycle in 0..2u8 {
            for tooth in 0..24u8 {
                if tooth == 0 {
                    rt.on_cyp_capture(ts_us + 40);
                }
                if tooth.is_multiple_of(6) {
                    rt.on_tdc_capture(ts_us + 80);
                }
                ts_us += dt_us;
                let (out, _knock) = rt.on_ckp_capture(ts_us);
                if cycle == 1 {
                    for act in out.cop.actions.iter().flatten() {
                        if !act.drive_dwell_high {
                            cycle1_sparks[act.cylinder.index()] += 1;
                        }
                    }
                    for act in out.inj.actions.iter().flatten() {
                        cycle1_inj[act.cylinder.index()] += 1;
                    }
                }
                let st = rt.sync_1khz(ts_us);
                ecu.apply_rt_status(st);
                let _ = ecu.tick_1khz(
                    ts_us,
                    &cruise_adc,
                    SwitchState::default(),
                    50.0,
                    false,
                    &cal,
                );
                rt.update_targets(ecu.export_rt_targets(&cal));
            }
        }

        let snap = ecu.snapshot();
        assert_eq!(snap.sync_state, SyncState::FullSynced720);
        assert_eq!(snap.rpm, 2_999);
        assert_eq!(snap.sync_loss_count, 0);
        assert!(snap.faults.is_empty());
        assert_eq!(cycle1_sparks, [1; 4]);
        assert_eq!(cycle1_inj, [1; 4]);
    }

    #[test]
    fn clear_faults_resets_latched_mil_without_defeating_vtec_lockout() {
        let mut cal = CalibrationBlob::default_honda_obd1();
        cal.vtec.require_vtp_pressure_switch = true;
        cal.vtec.vtp_confirm_timeout_ms = 5;
        let mut ecu = EngineCoordinator::new();

        // Sync up to 5,200 RPM with VTP missing so VTEC enters FaultLockout + latches Code 22
        let mut ts = 1_000_000u32;
        let tooth_us = 961u32;
        let mut wot_adc = nominal_raw_adc(0.45, 4.30);
        wot_adc.map_raw = volts_5v_to_adc(3.10);
        for cycle in 0..3 {
            for tooth in 0..24 {
                if tooth == 0 {
                    ecu.on_cyp_capture(ts.wrapping_sub(50));
                }
                if tooth % 6 == 0 {
                    ecu.on_tdc_capture(ts.wrapping_sub(25));
                }
                let _ = ecu.on_ckp_capture(ts, &cal);
                ts += tooth_us;
                if cycle == 2 {
                    let _ = ecu.tick_1khz(ts, &wot_adc, SwitchState::default(), 75.0, false, &cal);
                }
            }
        }
        assert!(ecu.snapshot().faults.contains(FaultFlags::VTP_OIL_PRESSURE));
        assert_eq!(
            ecu.snapshot().vtec_state,
            ecu_types::VtecState::FaultLockout
        );

        // Clearing MIL codes resets the fault mask cleanly while leaving VTEC lockout protected
        ecu.clear_faults();
        assert!(ecu.snapshot().faults.is_empty());
        assert_eq!(
            ecu.snapshot().vtec_state,
            ecu_types::VtecState::FaultLockout
        );
    }

    #[test]
    fn output_enable_mask_cuts_individual_and_global_spark_and_fuel() {
        let cal = CalibrationBlob::default_honda_obd1();
        let mut ecu = EngineCoordinator::new();
        let mut mask = ecu_types::OutputEnableMask::all_enabled();
        mask.cop_cyl_enabled[1] = false; // Cut Cyl 2 spark
        mask.inj_cyl_enabled[2] = false; // Cut Cyl 3 injector
        ecu.set_output_enable_mask(mask);

        let cruise_adc = nominal_raw_adc(0.55, 1.20);
        let mut ts = 100_000u32;
        let dt_us = 1_667u32;
        let _ = ecu.on_ckp_capture(ts, &cal);
        let mut sparks = [0u32; 4];
        let mut injs = [0u32; 4];

        for cycle in 0..2 {
            for tooth in 0..24u8 {
                if tooth == 0 {
                    ecu.on_cyp_capture(ts + 40);
                }
                if tooth.is_multiple_of(6) {
                    ecu.on_tdc_capture(ts + 80);
                }
                ts += dt_us;
                let out = ecu.on_ckp_capture(ts, &cal);
                if cycle == 1 {
                    for act in out.cop.actions.iter().flatten() {
                        if !act.drive_dwell_high {
                            sparks[act.cylinder.index()] += 1;
                        }
                    }
                    for act in out.inj.actions.iter().flatten() {
                        injs[act.cylinder.index()] += 1;
                    }
                }
                let _ = ecu.tick_1khz(ts, &cruise_adc, SwitchState::default(), 50.0, false, &cal);
            }
        }

        assert_eq!(sparks, [1, 0, 1, 1]);
        assert_eq!(injs, [1, 1, 0, 1]);
        assert_eq!(ecu.snapshot().actuators.inj_pw_us[2], 0);
        assert!(ecu.snapshot().actuators.inj_pw_us[0] > 0);
    }

    #[test]
    fn turbo_boost_flex_two_step_and_overboost_cut_work_end_to_end() {
        use ecu_types::{BoostConfig, BoostMode, FlexFuelState, FlexSensorStatus, MapSensorCal};

        let mut cal = CalibrationBlob::default_honda_obd1();
        cal.map_cal = MapSensorCal {
            kpa_at_0v: 0.0,
            kpa_at_5v: 400.0,
        };
        let mut boost_cfg = BoostConfig::default_honda_obd1();
        boost_cfg.mode = BoostMode::ClosedLoop;
        boost_cfg.overboost_cut_kpa = 230.0;
        boost_cfg.overboost_hyst_kpa = 20.0;
        boost_cfg.launch_enabled = true;
        boost_cfg.launch_rpm = 4_500;
        boost_cfg.launch_window_rpm = 300;
        boost_cfg.launch_retard_deg_x10 = -30;

        let mut ecu = EngineCoordinator::new();
        ecu.set_boost_config(boost_cfg);

        // Helper for 4-bar MAP raw ADC
        let make_4bar_adc = |kpa: f32, tps_v: f32| RawSensorAdc {
            map_raw: volts_5v_to_adc((kpa / 400.0) * 5.0),
            ..nominal_raw_adc(0.45, tps_v)
        };

        // Sync up at ~4,600 RPM (dt_us = 1087 us per 30° tooth)
        let mut ts = 100_000u32;
        let dt_us = 1_087u32;
        let boost_adc = make_4bar_adc(140.0, 4.20);
        let _ = ecu.on_ckp_capture(ts, &cal);
        for cycle in 0..2 {
            for tooth in 0..24u8 {
                if tooth == 0 {
                    ecu.on_cyp_capture(ts + 40);
                }
                if tooth.is_multiple_of(6) {
                    ecu.on_tdc_capture(ts + 80);
                }
                ts += dt_us;
                let _ = ecu.on_ckp_capture(ts, &cal);
                let _ = ecu.tick_1khz(ts, &boost_adc, SwitchState::default(), 60.0, false, &cal);
            }
            if cycle == 0 {
                assert_eq!(ecu.snapshot().sync_state, SyncState::FullSynced720);
            }
        }

        // 1. E85 Flex Fuel increases pulsewidth (~1.41x) and advances spark (+4.0 deg)
        let e0_act = ecu.tick_1khz(
            ts + 1_000,
            &boost_adc,
            SwitchState::default(),
            60.0,
            false,
            &cal,
        );
        ecu.set_flex_state(FlexFuelState {
            status: FlexSensorStatus::Valid,
            freq_hz: 135.0,
            ethanol_pct: 85.0,
            fuel_temp_c: 25.0,
            age_ms: 0,
        });
        let e85_act = ecu.tick_1khz(
            ts + 2_000,
            &boost_adc,
            SwitchState::default(),
            60.0,
            false,
            &cal,
        );
        assert!(e85_act.inj_pw_us[0] > e0_act.inj_pw_us[0] + 1_500);
        assert!(e85_act.spark_advance_deg_x10 >= e0_act.spark_advance_deg_x10 + 35);

        // 2. Stationary + clutch pressed enables 2-step launch retard & rotating spark cut
        let clutch_sw = SwitchState {
            clutch_pressed: true,
            ..Default::default()
        };
        let launch_act = ecu.tick_1khz(ts + 3_000, &boost_adc, clutch_sw, 0.0, false, &cal);
        assert!(ecu.boost_telemetry().launch_active);
        assert_eq!(launch_act.spark_advance_deg_x10, -30);
        assert!(
            ecu.effective_output_mask()
                .cop_cyl_enabled
                .iter()
                .any(|&c| !c)
        );

        // 3. Overboost > 230 kPa cuts all fuel + spark and sets FaultFlags::OVERBOOST_CUT
        let over_adc = make_4bar_adc(245.0, 4.20);
        let mut cut_act = launch_act;
        for i in 0..10u32 {
            cut_act = ecu.tick_1khz(
                ts + (4 + i) * 1_000,
                &over_adc,
                SwitchState::default(),
                80.0,
                false,
                &cal,
            );
        }
        assert_eq!(cut_act.inj_pw_us, [0; 4]);
        assert!(!ecu.effective_output_mask().ignition_enabled);
        assert!(!ecu.effective_output_mask().injectors_enabled);
        assert!(ecu.snapshot().faults.contains(FaultFlags::OVERBOOST_CUT));
        assert_eq!(ecu.boost_telemetry().solenoid_duty_per_mille, 0);
    }
}
