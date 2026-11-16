//! Hard real-time Honda OBD1 `24/4/1` (`CKP`, `TDC`, `CYP`) distributor trigger decoder.
//!
//! Hardware mapping (`_DOCS/stm32h743_pinout.md`):
//! - `TIM2` 32-bit free-running counter at `1.000 MHz` (`1 tick = 1 µs`, wraps every `71.58 min`)
//! - `PA15` (`TIM2_CH1`): `CKP` (`B15+/B16-`) — **24 teeth per `720°` cam rev** (`30.0°` crank)
//! - `PA1`  (`TIM2_CH2`): `TDC` (`B13+/B14-`) — **4 teeth per `720°` cam rev** (`180.0°` crank)
//! - `PB10` (`TIM2_CH3`): `CYP` (`B11+/B12-`) — **1 tooth per `720°` cam rev** (`Cyl 1` index)

use ecu_types::{FaultFlags, SyncState};

/// Teeth on the `CKP` wheel over a full `720.0°` 4-stroke engine cycle.
pub const CKP_TEETH_PER_720: u8 = 24;
/// Crank degrees × 10 per `CKP` tooth (`30.0°` = `300`).
pub const DEG_X10_PER_CKP_TOOTH: u16 = 300;
/// `12` `CKP` teeth per `360°` crankshaft revolution:
/// `RPM = 60_000_000 / (12 * tooth_period_us) = 5_000_000 / tooth_period_us`.
const RPM_NUMERATOR_1MHZ: u32 = 5_000_000;

/// Fixed-capacity `no_std` batch of compare actions falling inside one `30.0°` `CKP` tooth window.
#[derive(Debug, Clone)]
pub struct ToothBatch<T: Copy> {
    pub actions: [Option<T>; 4],
    pub len: usize,
}

impl<T: Copy> Default for ToothBatch<T> {
    fn default() -> Self {
        Self {
            actions: [None; 4],
            len: 0,
        }
    }
}

impl<T: Copy> ToothBatch<T> {
    #[inline]
    pub fn push(&mut self, action: T) {
        if self.len < self.actions.len() {
            self.actions[self.len] = Some(action);
            self.len += 1;
        }
    }
}

/// Checks whether `target_angle_x10` (`0..7200`) falls within `[window_start, window_start + 300)`
/// (`30.0°` `CKP` tooth window) and returns the angular offset (`0..300` in `0.1°` units).
#[inline]
pub fn angle_offset_in_tooth_window(window_start_x10: u16, target_angle_x10: u16) -> Option<u16> {
    let delta = (target_angle_x10 as i32 - window_start_x10 as i32).rem_euclid(7200) as u16;
    if delta < DEG_X10_PER_CKP_TOOTH {
        Some(delta)
    } else {
        None
    }
}

/// Converts an angular offset inside a `30.0°` `CKP` tooth window (`0.1°` units) into `µs`.
#[inline]
pub fn angle_delta_to_delay_us(delta_deg_x10: u16, tooth_period_us: u32) -> u16 {
    ((delta_deg_x10 as u64 * tooth_period_us as u64) / DEG_X10_PER_CKP_TOOTH as u64).min(65_000)
        as u16
}

/// Snapshot emitted on every valid `CKP` tooth edge (`30.0°` crank resolution) to feed
/// the angle-to-time `TIM1` (COP) and `TIM3` (Sequential Fuel) Output Compare schedulers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CkpToothEvent {
    pub sync_state: SyncState,
    /// `0..24` (`0` = first `CKP` edge immediately following the `CYP` Cyl 1 pulse).
    pub tooth_index: u8,
    /// Crank angle (`0..7200` in `0.1°` units) at this `CKP` edge after `trigger_offset_deg_x10`.
    pub crank_angle_deg_x10: u16,
    /// Instantaneous `30.0°` tooth duration (`µs`).
    pub tooth_period_us: u32,
    /// Instantaneous engine speed (`RPM`).
    pub rpm: u16,
    /// Hardware `TIM2->CNT` (`1 MHz`) timestamp of this `CKP` capture.
    pub timestamp_us: u32,
}

/// Deterministic `no_std` state machine decoding Honda's 3-sensor distributor (`24/4/1`).
#[derive(Debug, Clone, Copy)]
pub struct HondaObd1Decoder {
    sync_state: SyncState,
    tooth_index: u8,
    cyp_armed: bool,
    tdc_edges_since_cyp: u8,
    ckp_teeth_since_tdc: u8,
    last_ckp_ts_us: Option<u32>,
    prev_period_us: u32,
    rpm: u16,
    sync_loss_count: u32,
}

impl Default for HondaObd1Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl HondaObd1Decoder {
    pub const fn new() -> Self {
        Self {
            sync_state: SyncState::Unsynced,
            tooth_index: 0,
            cyp_armed: false,
            tdc_edges_since_cyp: 0,
            ckp_teeth_since_tdc: 0,
            last_ckp_ts_us: None,
            prev_period_us: 0,
            rpm: 0,
            sync_loss_count: 0,
        }
    }

    #[inline]
    pub const fn sync_state(&self) -> SyncState {
        self.sync_state
    }

    #[inline]
    pub const fn rpm(&self) -> u16 {
        self.rpm
    }

    #[inline]
    pub const fn sync_loss_count(&self) -> u32 {
        self.sync_loss_count
    }

    #[inline]
    pub const fn tooth_period_us(&self) -> u32 {
        self.prev_period_us
    }

    /// Resets sync to `Unsynced` if the engine stalls (`now_us - last_ckp > 400_000 µs`).
    pub fn check_stall_timeout(&mut self, now_us: u32) {
        if let Some(last) = self.last_ckp_ts_us
            && now_us.wrapping_sub(last) > 400_000
        {
            self.sync_state = SyncState::Unsynced;
            self.cyp_armed = false;
            self.last_ckp_ts_us = None;
            self.prev_period_us = 0;
            self.rpm = 0;
        }
    }

    /// Called from `TIM2_CH3` (`PB10 CYP`) ISR — 1 pulse per `720°` cam revolution.
    #[inline]
    pub fn on_cyp_capture(&mut self, _timestamp_us: u32, faults: &mut FaultFlags) {
        if self.sync_state == SyncState::FullSynced720 {
            // Verify that exactly 24 CKP teeth elapsed since the previous CYP pulse!
            // Because CYP arrives just before tooth 0, `tooth_index` should be `23` (last tooth).
            if self.tooth_index != 23 {
                self.sync_loss_count = self.sync_loss_count.saturating_add(1);
                faults.insert(FaultFlags::CKP_SYNC_LOSS);
            }
        }
        self.cyp_armed = true;
        self.tdc_edges_since_cyp = 0;
    }

    /// Called from `TIM2_CH2` (`PA1 TDC`) ISR — 4 pulses per `720°` (`180°` / `6` `CKP` teeth).
    #[inline]
    pub fn on_tdc_capture(&mut self, _timestamp_us: u32, faults: &mut FaultFlags) {
        if self.sync_state == SyncState::FullSynced720 && self.tdc_edges_since_cyp > 0 {
            // Between consecutive TDC pulses we must see exactly 6 CKP teeth (`6 * 30° = 180°`).
            if self.ckp_teeth_since_tdc != 6 {
                faults.insert(FaultFlags::TDC_SENSOR);
            }
        }
        self.ckp_teeth_since_tdc = 0;
        self.tdc_edges_since_cyp = self.tdc_edges_since_cyp.saturating_add(1);
        if self.sync_state == SyncState::Unsynced && self.last_ckp_ts_us.is_some() {
            self.sync_state = SyncState::HalfSynced360;
        }
    }

    /// Called from `TIM2_CH1` (`PA15 CKP`) ISR — 24 pulses per `720°` (`30.0°` crank per tooth).
    pub fn on_ckp_capture(
        &mut self,
        timestamp_us: u32,
        trigger_offset_deg_x10: i16,
        faults: &mut FaultFlags,
    ) -> Option<CkpToothEvent> {
        let Some(prev_ts) = self.last_ckp_ts_us else {
            self.last_ckp_ts_us = Some(timestamp_us);
            return None;
        };

        let dt_us = timestamp_us.wrapping_sub(prev_ts);
        // Reject physically impossible glitch pulses (< 380 µs => > 13,150 RPM)
        if dt_us < 380 {
            return None;
        }

        // If engine paused > 350 ms between teeth, treat as a fresh crank start
        if dt_us > 350_000 {
            self.sync_state = SyncState::Unsynced;
            self.prev_period_us = 0;
            self.last_ckp_ts_us = Some(timestamp_us);
            return None;
        }

        // Tooth-to-tooth acceleration/noise sanity check once running (> 800 RPM => dt < 6250 µs):
        // Even aggressive compression-stroke cranking stays well inside [0.30x .. 3.20x].
        if self.prev_period_us > 0 && self.prev_period_us < 6_250 {
            let min_ok = self.prev_period_us / 3;
            let max_ok = self.prev_period_us.saturating_mul(3);
            if dt_us < min_ok || dt_us > max_ok {
                self.sync_loss_count = self.sync_loss_count.saturating_add(1);
                faults.insert(FaultFlags::CKP_SYNC_LOSS);
                self.sync_state = SyncState::Unsynced;
                self.prev_period_us = dt_us;
                self.last_ckp_ts_us = Some(timestamp_us);
                return None;
            }
        }

        self.last_ckp_ts_us = Some(timestamp_us);
        self.prev_period_us = dt_us;
        self.ckp_teeth_since_tdc = self.ckp_teeth_since_tdc.saturating_add(1);

        if self.cyp_armed {
            self.tooth_index = 0;
            self.cyp_armed = false;
            self.sync_state = SyncState::FullSynced720;
        } else {
            self.tooth_index = (self.tooth_index + 1) % CKP_TEETH_PER_720;
            if self.tooth_index == 0 && self.sync_state == SyncState::FullSynced720 {
                // Wrapped all 24 teeth without seeing CYP!
                faults.insert(FaultFlags::CYP_SENSOR);
            }
        }

        let rpm = (RPM_NUMERATOR_1MHZ / dt_us).min(15_000) as u16;
        self.rpm = rpm;

        let raw_angle = (self.tooth_index as i32) * (DEG_X10_PER_CKP_TOOTH as i32)
            + (trigger_offset_deg_x10 as i32);
        let crank_angle_deg_x10 = raw_angle.rem_euclid(7200) as u16;

        Some(CkpToothEvent {
            sync_state: self.sync_state,
            tooth_index: self.tooth_index,
            crank_angle_deg_x10,
            tooth_period_us: dt_us,
            rpm,
            timestamp_us,
        })
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{FaultFlags, SyncState};

    use super::{
        HondaObd1Decoder, ToothBatch, angle_delta_to_delay_us, angle_offset_in_tooth_window,
    };

    #[test]
    fn decodes_full_720_honda_24_4_1_cycle_and_wraps_32bit_timer() {
        let mut dec = HondaObd1Decoder::new();
        let mut faults = FaultFlags::default();

        // Simulate 6,000 RPM: 1 crank rev = 10,000 µs -> 12 CKP teeth/rev -> 833 µs/tooth!
        // Start right near u32::MAX to prove 32-bit TIM2 rollover (`wrapping_sub`) is glitch-free!
        let mut ts: u32 = u32::MAX - 5_000;
        let dt: u32 = 833;

        // Prime first CKP edge before CYP
        assert!(dec.on_ckp_capture(ts, 0, &mut faults).is_none());

        // Run 3 complete 720° cam cycles (72 CKP teeth)
        for cycle in 0..3 {
            for tooth in 0..24u8 {
                if tooth == 0 {
                    dec.on_cyp_capture(ts.wrapping_add(100), &mut faults);
                }
                if tooth % 6 == 0 {
                    dec.on_tdc_capture(ts.wrapping_add(200), &mut faults);
                }
                ts = ts.wrapping_add(dt);
                let ev = dec
                    .on_ckp_capture(ts, 0, &mut faults)
                    .expect("expected valid tooth event");

                assert_eq!(ev.sync_state, SyncState::FullSynced720);
                assert_eq!(ev.tooth_index, tooth);
                assert_eq!(ev.crank_angle_deg_x10, (tooth as u16) * 300);
                assert!(
                    (ev.rpm as i32 - 6002).abs() <= 5,
                    "cycle={cycle} rpm={}",
                    ev.rpm
                );
            }
        }

        assert!(
            faults.is_empty(),
            "No trigger faults expected: {:?}",
            faults
        );
        assert_eq!(dec.sync_loss_count(), 0);
    }

    #[test]
    fn rejects_glitches_detects_missing_cyp_tdc_and_handles_stall_timeout() {
        let mut dec = HondaObd1Decoder::new();
        let mut faults = FaultFlags::default();
        let dt: u32 = 2_000; // 2,500 RPM (`2,000 µs` per 30° tooth)
        let mut ts: u32 = 10_000;

        // 1. Prime first CKP edge, then TDC edge -> transitions Unsynced -> HalfSynced360
        assert!(dec.on_ckp_capture(ts, 0, &mut faults).is_none());
        dec.on_tdc_capture(ts + 100, &mut faults);
        assert_eq!(dec.sync_state(), SyncState::HalfSynced360);

        // 2. Glitch pulse (< 380 µs) is ignored without advancing last_ckp_ts_us
        assert!(dec.on_ckp_capture(ts + 150, 0, &mut faults).is_none());

        // 3. Achieve FullSynced720 and run one clean 24-tooth cycle with negative offset (-5.0°)
        for tooth in 0..24u8 {
            if tooth == 0 {
                dec.on_cyp_capture(ts + 50, &mut faults);
            }
            if tooth.is_multiple_of(6) {
                dec.on_tdc_capture(ts + 100, &mut faults);
            }
            ts += dt;
            let ev = dec
                .on_ckp_capture(ts, -50, &mut faults)
                .expect("valid tooth");
            assert_eq!(ev.tooth_index, tooth);
            let expected_angle = ((tooth as i32) * 300 - 50).rem_euclid(7200) as u16;
            assert_eq!(ev.crank_angle_deg_x10, expected_angle);
        }
        assert!(faults.is_empty());

        // 4. Wrap from tooth 23 -> tooth 0 WITHOUT a CYP pulse -> sets CYP_SENSOR fault!
        ts += dt;
        let ev_no_cyp = dec.on_ckp_capture(ts, 0, &mut faults).expect("tooth 0");
        assert_eq!(ev_no_cyp.tooth_index, 0);
        assert!(faults.contains(FaultFlags::CYP_SENSOR));

        // 5. Fire TDC after only 1 CKP tooth instead of 6 -> sets TDC_SENSOR fault!
        dec.on_tdc_capture(ts + 50, &mut faults);
        assert!(faults.contains(FaultFlags::TDC_SENSOR));

        // 6. Fire CYP at tooth 0 instead of tooth 23 -> sets CKP_SYNC_LOSS and increments counter!
        dec.on_cyp_capture(ts + 100, &mut faults);
        assert!(faults.contains(FaultFlags::CKP_SYNC_LOSS));
        assert_eq!(dec.sync_loss_count(), 1);

        // 7. Re-sync tooth 0, then inject a >3x missing-tooth gap (8,000 vs 2,000 µs) -> sync loss!
        ts += dt;
        let _ = dec.on_ckp_capture(ts, 0, &mut faults);
        assert_eq!(dec.sync_state(), SyncState::FullSynced720);
        ts += 8_000;
        assert!(dec.on_ckp_capture(ts, 0, &mut faults).is_none());
        assert_eq!(dec.sync_state(), SyncState::Unsynced);
        assert_eq!(dec.sync_loss_count(), 2);

        // 8. Stall timeout (> 400,000 µs without edges) clears RPM and period
        dec.check_stall_timeout(ts + 450_000);
        assert_eq!(dec.sync_state(), SyncState::Unsynced);
        assert_eq!(dec.rpm(), 0);
        assert_eq!(dec.tooth_period_us(), 0);
    }

    #[test]
    fn tooth_batch_and_angle_window_helpers_handle_boundaries() {
        assert_eq!(angle_offset_in_tooth_window(6_900, 7_199), Some(299));
        assert_eq!(angle_offset_in_tooth_window(6_900, 0), None);
        assert_eq!(angle_offset_in_tooth_window(7_050, 100), Some(250));
        assert_eq!(angle_delta_to_delay_us(150, 2_000), 1_000);

        let mut batch = ToothBatch::<u8>::default();
        for i in 0..6u8 {
            batch.push(i);
        }
        assert_eq!(batch.len, 4);
        assert_eq!(batch.actions, [Some(0), Some(1), Some(2), Some(3)]);
    }
}
