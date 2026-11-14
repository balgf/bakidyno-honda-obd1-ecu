//! Honda VTEC controller (`A4 VTS*` `+12V` high-side switch + `D6 VTP*` oil pressure check).

use ecu_types::{FaultFlags, SensorSnapshot, VtecConfig, VtecState};

#[derive(Debug, Clone)]
pub struct VtecController {
    state: VtecState,
    engage_wait_ms: u16,
}

impl Default for VtecController {
    fn default() -> Self {
        Self::new()
    }
}

impl VtecController {
    pub const fn new() -> Self {
        Self {
            state: VtecState::LowCam,
            engage_wait_ms: 0,
        }
    }

    #[inline]
    pub const fn state(&self) -> VtecState {
        self.state
    }

    /// Returns `(vts_solenoid_on, use_vtec_maps)` for a `1 ms` (`1 kHz`) control tick.
    ///
    /// - `pb14_vts_fault_low`: active-low fault flag from `TPS1H100A-Q1` (`true` = fault).
    pub fn update_1khz(
        &mut self,
        rpm: u16,
        sensors: &SensorSnapshot,
        cfg: &VtecConfig,
        pb14_vts_fault_low: bool,
        faults: &mut FaultFlags,
    ) -> (bool, bool) {
        if pb14_vts_fault_low {
            faults.insert(FaultFlags::VTS_SOLENOID_HW);
            self.state = VtecState::FaultLockout;
            return (false, false);
        }

        if self.state == VtecState::FaultLockout {
            // Reset fault lockout only once engine drops back to idle (< 1200 RPM)
            if rpm < 1200 {
                self.state = VtecState::LowCam;
                self.engage_wait_ms = 0;
            } else {
                return (false, false);
            }
        }

        let entry_ok = rpm >= cfg.engage_rpm
            && sensors.map_kpa >= cfg.min_map_kpa as f32
            && sensors.tps_pct >= cfg.min_tps_pct as f32
            && sensors.vss_kmh >= cfg.min_vss_kmh as f32
            && sensors.ect_c >= cfg.min_ect_c as f32;

        // Hysteresis margins prevent VTEC chatter right at crossover
        let hold_ok = rpm >= cfg.disengage_rpm
            && sensors.map_kpa >= (cfg.min_map_kpa.saturating_sub(8)) as f32
            && sensors.tps_pct >= (cfg.min_tps_pct.saturating_sub(10)) as f32
            && sensors.ect_c >= (cfg.min_ect_c - 3) as f32;

        match self.state {
            VtecState::LowCam => {
                if entry_ok {
                    if cfg.require_vtp_pressure_switch {
                        self.state = VtecState::EngagingWaitPressure;
                        self.engage_wait_ms = 0;
                        (true, false) // Energize A4 VTS*, switch maps once oil pressure rises!
                    } else {
                        self.state = VtecState::HighCamActive;
                        (true, true)
                    }
                } else {
                    (false, false)
                }
            }
            VtecState::EngagingWaitPressure => {
                if !hold_ok {
                    self.state = VtecState::LowCam;
                    self.engage_wait_ms = 0;
                    return (false, false);
                }
                if sensors.switches.vtp_pressurized {
                    self.state = VtecState::HighCamActive;
                    self.engage_wait_ms = 0;
                    (true, true)
                } else {
                    self.engage_wait_ms = self.engage_wait_ms.saturating_add(1);
                    if self.engage_wait_ms >= cfg.vtp_confirm_timeout_ms {
                        faults.insert(FaultFlags::VTP_OIL_PRESSURE);
                        self.state = VtecState::FaultLockout;
                        (false, false)
                    } else {
                        (true, false)
                    }
                }
            }
            VtecState::HighCamActive => {
                if !hold_ok {
                    self.state = VtecState::LowCam;
                    self.engage_wait_ms = 0;
                    (false, false)
                } else if cfg.require_vtp_pressure_switch && !sensors.switches.vtp_pressurized {
                    // Sudden loss of VTEC oil pressure while engaged -> protect valvetrain!
                    faults.insert(FaultFlags::VTP_OIL_PRESSURE);
                    self.state = VtecState::FaultLockout;
                    (false, false)
                } else {
                    (true, true)
                }
            }
            VtecState::FaultLockout => (false, false),
        }
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{CalibrationBlob, FaultFlags, SensorSnapshot, VtecState};

    use super::VtecController;

    #[test]
    fn vtec_engages_after_oil_pressure_and_faults_on_timeout() {
        let cal = CalibrationBlob::default_honda_obd1();
        let mut vtec = VtecController::new();
        let mut faults = FaultFlags::default();

        let mut sns = SensorSnapshot {
            map_kpa: 98.0,
            tps_pct: 95.0,
            ect_c: 85.0,
            vss_kmh: 75.0,
            ..Default::default()
        };

        // Trigger VTEC at 5,200 RPM: solenoid turns ON immediately, maps wait for D6 VTP
        let (sol, maps) = vtec.update_1khz(
            5200,
            &sns,
            &cal.vtec,
            /* vts_feedback_fault= */ false,
            &mut faults,
        );
        assert!(sol && !maps);
        assert_eq!(vtec.state(), VtecState::EngagingWaitPressure);

        // 40 ms later, oil gallery pressurizes -> HighCamActive!
        for _ in 0..40 {
            let _ = vtec.update_1khz(
                5200,
                &sns,
                &cal.vtec,
                /* vts_feedback_fault= */ false,
                &mut faults,
            );
        }
        sns.switches.vtp_pressurized = true;
        let (sol, maps) = vtec.update_1khz(
            5200,
            &sns,
            &cal.vtec,
            /* vts_feedback_fault= */ false,
            &mut faults,
        );
        assert!(sol && maps);
        assert_eq!(vtec.state(), VtecState::HighCamActive);
        assert!(faults.is_empty());

        // Drop RPM to 4700 (inside hysteresis 4600..4800) -> stays engaged!
        let (sol, maps) = vtec.update_1khz(
            4700,
            &sns,
            &cal.vtec,
            /* vts_feedback_fault= */ false,
            &mut faults,
        );
        assert!(sol && maps);

        // Drop RPM to 4500 (< 4600) -> cleanly disengages
        let (sol, maps) = vtec.update_1khz(
            4500,
            &sns,
            &cal.vtec,
            /* vts_feedback_fault= */ false,
            &mut faults,
        );
        assert!(!sol && !maps);
        assert_eq!(vtec.state(), VtecState::LowCam);
    }

    #[test]
    fn vtec_faults_on_vtp_timeout_pressure_drop_or_solenoid_short_and_recovers_at_idle() {
        let mut cal = CalibrationBlob::default_honda_obd1();
        let mut vtec = VtecController::new();
        let mut faults = FaultFlags::default();
        let mut sns = SensorSnapshot {
            map_kpa: 98.0,
            tps_pct: 95.0,
            ect_c: 85.0,
            vss_kmh: 75.0,
            ..Default::default()
        };

        // 1. Enter EngagingWaitPressure, then hold 250 ms without VTP pressure ->
        // VTP_OIL_PRESSURE fault!
        let (sol, maps) = vtec.update_1khz(5_200, &sns, &cal.vtec, false, &mut faults);
        assert!(sol && !maps);
        for _ in 0..cal.vtec.vtp_confirm_timeout_ms {
            let _ = vtec.update_1khz(5_200, &sns, &cal.vtec, false, &mut faults);
        }
        assert_eq!(vtec.state(), VtecState::FaultLockout);
        assert!(faults.contains(FaultFlags::VTP_OIL_PRESSURE));

        // Stays locked out at 3,000 RPM, then resets to LowCam when engine idles (< 1,200 RPM)
        let (sol, _) = vtec.update_1khz(3_000, &sns, &cal.vtec, false, &mut faults);
        assert!(!sol);
        assert_eq!(vtec.state(), VtecState::FaultLockout);
        let _ = vtec.update_1khz(900, &sns, &cal.vtec, false, &mut faults);
        assert_eq!(vtec.state(), VtecState::LowCam);

        // 2. Engage HighCamActive with VTP pressurized, then simulate oil-pressure drop mid-pull
        faults = FaultFlags::default();
        sns.switches.vtp_pressurized = true;
        let _ = vtec.update_1khz(5_200, &sns, &cal.vtec, false, &mut faults);
        let (sol, maps) = vtec.update_1khz(5_200, &sns, &cal.vtec, false, &mut faults);
        assert!(sol && maps);
        assert_eq!(vtec.state(), VtecState::HighCamActive);

        sns.switches.vtp_pressurized = false;
        let (sol, maps) = vtec.update_1khz(5_200, &sns, &cal.vtec, false, &mut faults);
        assert!(!sol && !maps);
        assert_eq!(vtec.state(), VtecState::FaultLockout);
        assert!(faults.contains(FaultFlags::VTP_OIL_PRESSURE));

        // 3. Hardware TPS1H100 fault on PB14 immediately locks out and sets VTS_SOLENOID_HW
        let _ = vtec.update_1khz(900, &sns, &cal.vtec, false, &mut faults);
        let (sol, maps) = vtec.update_1khz(5_200, &sns, &cal.vtec, true, &mut faults);
        assert!(!sol && !maps);
        assert_eq!(vtec.state(), VtecState::FaultLockout);
        assert!(faults.contains(FaultFlags::VTS_SOLENOID_HW));

        // 4. JDM bypass (`require_vtp_pressure_switch = false`) enters HighCamActive immediately
        cal.vtec.require_vtp_pressure_switch = false;
        let _ = vtec.update_1khz(900, &sns, &cal.vtec, false, &mut faults);
        let (sol, maps) = vtec.update_1khz(5_200, &sns, &cal.vtec, false, &mut faults);
        assert!(sol && maps);
        assert_eq!(vtec.state(), VtecState::HighCamActive);
    }
}
