//! Top-level ECU coordinator orchestrating `1 kHz` control ticks and hard real-time tooth edges.

use crate::fuel::{FuelModel, InjScheduleBatch};
use crate::ignition::{IgnitionModel, ToothScheduleBatch};
use crate::knock::KnockDetector;
use crate::math::interp_1d_i16_u16;
use crate::sensors::SensorProcessor;
use crate::trigger::{CkpToothEvent, HondaObd1Decoder};
use crate::vtec::VtecController;
use crate::wideband::Lsu49Controller;
use ecu_types::{
    ActuatorOutputs, CalibrationBlob, Cylinder, FaultFlags, KnockConfig, KnockTelemetry,
    RawSensorAdc, SensorSnapshot, SwitchState, SyncState, TelemetrySnapshot,
};

/// Combined ignition & injector compare actions emitted on a single `CKP` tooth edge.
#[derive(Debug, Clone, Default)]
pub struct ToothScheduleOutputs {
    pub event: Option<CkpToothEvent>,
    pub cop: ToothScheduleBatch,
    pub inj: InjScheduleBatch,
}

pub struct EngineCoordinator {
    pub trigger: HondaObd1Decoder,
    pub sensors_proc: SensorProcessor,
    pub vtec: VtecController,
    pub wideband: Lsu49Controller,
    pub fuel: FuelModel,
    pub ignition: IgnitionModel,
    pub knock: KnockDetector,

    uptime_ms: u32,
    seq: u32,
    last_angle_deg_x10: u16,
    fan_on_latched: bool,
    timing_override_deg_x10: Option<i16>,

    latest_sensors: SensorSnapshot,
    latest_actuators: ActuatorOutputs,
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
            uptime_ms: 0,
            seq: 0,
            last_angle_deg_x10: 0,
            fan_on_latched: false,
            timing_override_deg_x10: None,
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
            let cop = self.ignition.schedule_on_ckp_tooth(
                &ev,
                self.latest_actuators.spark_advance_deg_x10,
                self.latest_actuators.cop_dwell_us,
            );
            let inj = self
                .fuel
                .schedule_on_ckp_tooth(&ev, &self.latest_actuators.inj_pw_us);
            ToothScheduleOutputs {
                event: Some(ev),
                cop,
                inj,
            }
        } else {
            self.fuel.reset_cycle();
            ToothScheduleOutputs::default()
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

        // 4. Speed-Density Fuel + Closed-Loop Wideband STFT
        let fuel_out = self.fuel.compute_1khz(
            rpm,
            use_vtec_maps,
            &sensors,
            self.wideband.state(),
            self.wideband.lambda(),
            cal,
        );

        // 5. Knock DSP 1 kHz Decay/Fault Update + Ignition Advance + Smart COP Dwell
        let engine_unsynced = self.trigger.sync_state() == SyncState::Unsynced;
        self.knock
            .tick_1khz(engine_unsynced, &mut self.latest_faults);
        let suppress_knock_retard =
            switches.scs_service_jumper || self.timing_override_deg_x10.is_some();
        self.ignition
            .set_per_cyl_knock_retard_deg_x10(if suppress_knock_retard {
                [0; 4]
            } else {
                self.knock.per_cyl_retard_deg_x10()
            });
        let (spark_advance_deg_x10, cop_dwell_us) = self.ignition.compute_1khz(
            rpm,
            use_vtec_maps,
            &sensors,
            self.timing_override_deg_x10,
            cal,
        );

        // 6. Auxiliary Honda OBD1 Connector A Relays & IACV
        // `A7/A8 FLR`: Prime fuel pump for 2.0 s at key-on, then hold ON whenever engine is
        // turning or cranking!
        let fuel_pump_relay_on =
            self.uptime_ms <= 2_000 || rpm > 0 || sensors.switches.sts_cranking;

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

        let actuators = ActuatorOutputs {
            inj_pw_us: if self.trigger.sync_state() == SyncState::Unsynced {
                [0; 4]
            } else {
                fuel_out.final_pw_us
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
}
