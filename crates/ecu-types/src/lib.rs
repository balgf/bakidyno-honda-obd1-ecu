#![no_std]

//! Canonical `no_std` data structures shared across:
//! - `ecu-core` (deterministic engine control math & state machines)
//! - `firmware` (`STM32H743VIT6` real-time target)
//! - `proto-native` (`COBS` + `CRC32` + `postcard` RPC for `ecu-cli` and future custom `tuner-gui`)
//! - `proto-tunerstudio` (optional TunerStudio page/realtime adapter)

use serde::{Deserialize, Serialize};

/// `"HON1"` magic header identifying a valid Honda OBD1 calibration image.
pub const CALIBRATION_MAGIC: u32 = 0x484f_4e31;
/// Schema version incremented whenever `CalibrationBlob` layout changes.
pub const CALIBRATION_VERSION: u16 = 1;

/// Physical cylinder identifier (1 = timing-belt end, 4 = distributor/trans end).
///
/// Maps 1:1 to hardware channels:
/// - `COP1..COP4` (`PE9`, `PE11`, `PE13`, `PE14` on `TIM1_CH1..CH4`)
/// - `INJ1..INJ4` (`PC6/A1`, `PC7/A3`, `PB0/A5`, `PB1/A2` on `TIM3_CH1..CH4`)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Cylinder {
    Cyl1 = 0,
    Cyl2 = 1,
    Cyl3 = 2,
    Cyl4 = 3,
}

impl Cylinder {
    #[inline]
    pub const fn index(self) -> usize {
        self as usize
    }

    #[inline]
    pub const fn number(self) -> u8 {
        (self as u8) + 1
    }
}

/// Standard inline-4 Honda firing sequence (`1 – 3 – 4 – 2`), spaced `180.0°` crank apart.
pub const HONDA_FIRING_ORDER: [Cylinder; 4] = [
    Cylinder::Cyl1,
    Cylinder::Cyl3,
    Cylinder::Cyl4,
    Cylinder::Cyl2,
];

/// Compression TDC angles in `0.1°` crank units (`0..7200`) for `[Cyl1, Cyl2, Cyl3, Cyl4]`.
///
/// - `Cyl1`: `0.0°`
/// - `Cyl3`: `180.0°` (`1800`)
/// - `Cyl4`: `360.0°` (`3600`)
/// - `Cyl2`: `540.0°` (`5400`)
pub const CYLINDER_TDC_DEG_X10: [u16; 4] = [0, 5400, 1800, 3600];

/// Distributor `24/4/1` synchronization status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum SyncState {
    /// Waiting for valid distributor pulses. All fuel/COP outputs remain disarmed.
    Unsynced = 0,
    /// `CKP` (`24T`) + `TDC` (`4T`) seen, awaiting `CYP` (`1T`) for full `720°` cam phase.
    HalfSynced360 = 1,
    /// Full `720°` sequential phase locked via `CYP` + `TDC` + `CKP`.
    FullSynced720 = 2,
}

///Bosch LSU 4.9 onboard wideband controller operating state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum WidebandState {
    /// Engine off (`RPM == 0`); heater off to protect zirconia element from thermal shock.
    WaitForEngine = 0,
    /// Bosch `2.0V` RMS condensation-evaporation preheat phase (`~15 s`).
    CondensationPreheat = 1,
    /// Controlled `0.4 V/s` effective heater ramp up toward `780 °C`.
    WarmupRamp = 2,
    /// Closed-loop `Ri = 300 Ω` (`780 °C`) locked; `λ` measurement valid for closed-loop fuel.
    ClosedLoopReady = 3,
    /// Sensor open-circuit or overheat fault detected.
    Fault = 4,
}

/// VTEC spool solenoid (`A4 VTS*`) & oil-pressure switch (`D6 VTP*`) state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum VtecState {
    LowCam = 0,
    /// `A4 VTS*` driven `+12V`; waiting up to `250 ms` for `D6 VTP*` oil pressure confirmation.
    EngagingWaitPressure = 1,
    /// VTEC oil pressure confirmed (or software bypass active); High-Cam VE/Spark maps active.
    HighCamActive = 2,
    /// Oil pressure failed (`CEL Code 22`) or `TPS1H100` hardware fault (`CEL Code 21`).
    FaultLockout = 3,
}

/// Bitmask of active ECU diagnostic fault codes (aligned with classic Honda OBD1 MIL codes).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FaultFlags(pub u32);

impl FaultFlags {
    pub const NONE: u32 = 0;
    pub const MAP_RANGE: u32 = 1 << 0; // Honda Code 3/5
    pub const ECT_RANGE: u32 = 1 << 1; // Honda Code 6
    pub const TPS_RANGE: u32 = 1 << 2; // Honda Code 7
    pub const TDC_SENSOR: u32 = 1 << 3; // Honda Code 8
    pub const CYP_SENSOR: u32 = 1 << 4; // Honda Code 9
    pub const IAT_RANGE: u32 = 1 << 5; // Honda Code 10
    pub const CKP_SYNC_LOSS: u32 = 1 << 6; // Honda Code 4
    pub const VTS_SOLENOID_HW: u32 = 1 << 7; // Honda Code 21 (`PB14` TPS1H100 fault)
    pub const VTP_OIL_PRESSURE: u32 = 1 << 8; // Honda Code 22 (`D6` pressure timeout)
    pub const KNOCK_SENSOR: u32 = 1 << 9; // Honda Code 23
    pub const WIDEBAND_FAULT: u32 = 1 << 10; // Onboard LSU 4.9 fault
    pub const OVERBOOST_CUT: u32 = 1 << 11; // Manifold pressure exceeded overboost cut limit
    pub const LEAN_BOOST_CUT: u32 = 1 << 12; // Wideband reported lean AFR under boost

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    #[inline]
    pub fn insert(&mut self, bit: u32) {
        self.0 |= bit;
    }

    #[inline]
    pub fn remove(&mut self, bit: u32) {
        self.0 &= !bit;
    }

    #[inline]
    pub const fn contains(self, bit: u32) -> bool {
        (self.0 & bit) != 0
    }
}

/// Debounced logical state of Honda OBD1 digital switch inputs (post-`SN74HCS14` inversion fixup).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SwitchState {
    /// `B9 STS`: Starter cranking `+12V` active.
    pub sts_cranking: bool,
    /// `D6 VTP*`: VTEC gallery pressurized (`NC` ground switch opened or `A4->D6` `+12V` high).
    pub vtp_pressurized: bool,
    /// `D4 SCS`: Service Check Connector grounded (locks spark to `16.0° BTDC` at idle & MIL).
    pub scs_service_jumper: bool,
    /// `B5 ACS*`: Climate control A/C request active.
    pub acs_ac_request: bool,
    /// `B8 PSP*`: Power-steering high hydraulic pressure switch active.
    pub psp_steering_load: bool,
    /// `D2 BKSW*`: Brake pedal pressed (`+12V`).
    pub bksw_brake: bool,
    /// `B7`: Clutch switch pressed (for 2-step launch / flat-foot shift).
    pub clutch_pressed: bool,
}

/// Raw 16-bit ADC counts (`0..65535` against `3.300V` `REF3033` `VREF+`) and hardware inputs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSensorAdc {
    /// `PC0` (`ADC1_INP10`) — `D17 MAP` (`12k/20k` `0.625` divider).
    pub map_raw: u16,
    /// `PC1` (`ADC1_INP11`) — `D11 TPS` (`12k/20k` `0.625` divider).
    pub tps_raw: u16,
    /// `PA2` (`ADC1_INP14`) — `D13 ECT` (`2.49 kΩ` pull-up to `5V` + `12k/20k` divider).
    pub ect_raw: u16,
    /// `PA3` (`ADC1_INP15`) — `D15 IAT` (`2.49 kΩ` pull-up to `5V` + `12k/20k` divider).
    pub iat_raw: u16,
    /// `PC4` (`ADC1_INP4`) — `IGP` switched battery (`100k/15k` divider).
    pub vbatt_raw: u16,
    /// `PA0` (`ADC1_INP16`) — `D19/D20 VCC1/2` `5.00V` sensor rail (`12k/20k` divider).
    pub vcc_5v_raw: u16,
    /// `PC5` (`ADC1_INP8`) — `D10 ELD*` Electrical Load Detector (`12k/20k` divider).
    pub eld_raw: u16,
    /// `PA6` (`ADC2_INP3`) — LSU 4.9 Nernst cell DC average (`UN`).
    pub lsu_un_dc_raw: u16,
    /// `PA6` (`ADC2_INP3`) — LSU 4.9 synchronous `4 kHz` peak-to-trough delta (`ΔUN` for `Ri`).
    pub lsu_un_ac_delta_raw: u16,
    /// `PA7` (`ADC2_INP7`) — LSU 4.9 buffered pump shunt voltage (`Ip` across `61.9 Ω`).
    pub lsu_ip_raw: u16,
}

/// Calibrated physical sensor values updated at `1 kHz`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorSnapshot {
    pub map_kpa: f32,
    pub tps_pct: f32,
    pub ect_c: f32,
    pub iat_c: f32,
    pub vbatt_v: f32,
    pub vcc_5v_v: f32,
    pub eld_amps: f32,
    pub vss_kmh: f32,
    pub switches: SwitchState,
}

/// Commanded actuator outputs computed by `ecu-core`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ActuatorOutputs {
    /// Commanded injector pulse width (`µs`) including deadtime (`INJ1..INJ4`).
    pub inj_pw_us: [u32; 4],
    /// Commanded ignition spark advance in `0.1° BTDC` (e.g., `160` = `16.0° BTDC`).
    pub spark_advance_deg_x10: i16,
    /// Commanded Smart COP coil dwell duration (`µs`).
    pub cop_dwell_us: u32,
    /// `A4 VTS*` `+12V` high-side VTEC spool solenoid enable (`PA8`).
    pub vts_high_side_on: bool,
    /// `A7/A8 FLR` Fuel Pump Main Relay low-side (`PD3`).
    pub fuel_pump_relay_on: bool,
    /// `A12 FANC*` Radiator fan relay low-side (`PB12`).
    pub fan_relay_on: bool,
    /// `A15 ACC*` A/C compressor clutch relay low-side (`PB13`).
    pub ac_clutch_relay_on: bool,
    /// `A13 MIL` Check Engine Light low-side (`PD5`).
    pub mil_lamp_on: bool,
    /// `A9 IACV` `450 Hz` PWM duty (`0..=1000` = `0.0%..=100.0%`, `PD14`).
    pub iacv_duty_per_mille: u16,
    /// `PB9` Bosch LSU 4.9 `100 Hz` heater PWM duty (`0..=1000` = `0.0%..=100.0%`).
    pub lsu_heater_duty_per_mille: u16,
    /// `PA4 DAC1_OUT1` 12-bit code (`0..=4095`) driving LSU 4.9 pump current (`Ip`).
    pub lsu_ip_dac_code: u16,
}

/// Complete live telemetry frame streamed over USB-C (`proto-native`) & logged to MicroSD.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetrySnapshot {
    pub seq: u32,
    pub uptime_ms: u32,
    pub sync_state: SyncState,
    pub rpm: u16,
    pub crank_angle_deg_x10: u16,
    pub sync_loss_count: u32,
    pub sensors: SensorSnapshot,
    pub vtec_state: VtecState,
    pub wideband_state: WidebandState,
    pub lambda: f32,
    pub lsu_ri_ohms: f32,
    pub ve_pct: f32,
    pub target_lambda: f32,
    pub stft_pct: f32,
    pub actuators: ActuatorOutputs,
    pub faults: FaultFlags,
}

/// VTEC engagement & safety parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VtecConfig {
    pub engage_rpm: u16,
    pub disengage_rpm: u16,
    pub min_map_kpa: u16,
    pub min_tps_pct: u8,
    pub min_vss_kmh: u8,
    pub min_ect_c: i16,
    /// Set `true` for USDM `D6 VTP` switch or `A4->D6` jumper; `false` for un-wired JDM heads.
    pub require_vtp_pressure_switch: bool,
    pub vtp_confirm_timeout_ms: u16,
}

/// MAP sensor linear calibration (`kPa = offset_kpa + (v_norm_5v / 5.0) * span_kpa`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MapSensorCal {
    /// Pressure at `0.00V` ratiometric (`-1.5 kPa` for OEM Honda Denso `1.78 bar` MAP).
    pub kpa_at_0v: f32,
    /// Pressure at `5.00V` ratiometric (`182.5 kPa` for OEM Honda Denso `1.78 bar` MAP).
    pub kpa_at_5v: f32,
}

/// Canonical non-volatile ECU calibration blob (`~2.95 KB` — dual slots in `32 KB` `SPI1` FRAM).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CalibrationBlob {
    pub magic: u32,
    pub version: u16,
    pub displacement_cc: u16,
    pub injector_flow_cc_min: u16,
    /// Stoichiometric AFR × 100 (`1470` = `14.70:1` E0 gasoline, `1410` = `14.10:1` E10).
    pub stoich_afr_x100: u16,
    pub rev_limit_soft_rpm: u16,
    pub rev_limit_hard_rpm: u16,
    /// Base distributor alignment lock when `D4 SCS` is grounded (`160` = `16.0° BTDC`).
    pub scs_base_timing_deg_x10: i16,
    /// Distributor mechanical trigger offset relative to Cyl 1 TDC in `0.1°` crank units.
    pub trigger_offset_deg_x10: i16,
    pub fan_on_ect_c: i16,
    pub fan_off_ect_c: i16,
    pub map_cal: MapSensorCal,
    pub vtec: VtecConfig,

    /// Shared `16`-column RPM axis (`500..8500 RPM`).
    pub rpm_bins: [u16; 16],
    /// Shared `16`-row MAP axis (`20..110 kPa` NA, extendable to `250+ kPa` boosted).
    pub map_kpa_bins: [u16; 16],

    /// Low-Cam Volumetric Efficiency `[map_idx][rpm_idx]` in `0.1%` units (`850` = `85.0%`).
    pub ve_low_cam_x10: [[u16; 16]; 16],
    /// VTEC High-Cam Volumetric Efficiency `[map_idx][rpm_idx]` in `0.1%` units.
    pub ve_vtec_cam_x10: [[u16; 16]; 16],
    /// Low-Cam Ignition Advance `[map_idx][rpm_idx]` in `0.1° BTDC` (`280` = `28.0° BTDC`).
    pub spark_low_cam_deg_x10: [[i16; 16]; 16],
    /// VTEC High-Cam Ignition Advance `[map_idx][rpm_idx]` in `0.1° BTDC`.
    pub spark_vtec_cam_deg_x10: [[i16; 16]; 16],
    /// Target Lambda `[map_idx][rpm_idx]` in `0.001 λ` (`1000` = `1.000 λ`, `860` = `0.860 λ`).
    pub target_lambda_x1000: [[u16; 16]; 16],

    /// Battery voltage axis in `0.1V` (`80..=160` -> `8.0V..=16.0V`).
    pub vbatt_bins_x10: [u16; 8],
    /// Injector opening deadtime (`µs`) vs `vbatt_bins_x10`.
    pub inj_deadtime_us: [u16; 8],
    /// Smart COP (`K20` / `R8`) coil dwell (`µs`) vs `vbatt_bins_x10`.
    pub cop_dwell_us: [u16; 8],

    /// Coolant temperature axis (`°C`) for warmup & cranking enrichment.
    pub ect_bins_c: [i16; 8],
    /// Post-start / warmup fuel multiplier (`100` = `1.00x`, `145` = `+45%` cold enrichment).
    pub warmup_enrichment_pct: [u16; 8],
    /// Cranking prime pulse width (`µs`) vs `ect_bins_c`.
    pub cranking_pw_us: [u32; 8],
    /// Base open-loop `IACV` duty (`0..1000` per-mille) vs `ect_bins_c`.
    pub iacv_base_duty_per_mille: [u16; 8],
}

impl CalibrationBlob {
    /// Conservative, physically grounded base calibration for a stock-ish `1.6L` Honda OBD1 VTEC
    /// (`D16Z6` / `B16A` class, `240 cc/min` High-Z injectors, K20/R8 Smart COPs, OEM MAP).
    pub fn default_honda_obd1() -> Self {
        let rpm_bins: [u16; 16] = [
            600, 900, 1200, 1600, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6800, 7600,
            8400,
        ];
        let map_kpa_bins: [u16; 16] = [
            20, 28, 35, 42, 50, 58, 65, 72, 78, 84, 90, 95, 100, 105, 115, 130,
        ];

        let mut ve_low_cam_x10 = [[0u16; 16]; 16];
        let mut ve_vtec_cam_x10 = [[0u16; 16]; 16];
        let mut spark_low_cam_deg_x10 = [[0i16; 16]; 16];
        let mut spark_vtec_cam_deg_x10 = [[0i16; 16]; 16];
        let mut target_lambda_x1000 = [[1000u16; 16]; 16];

        let mut r = 0;
        while r < 16 {
            let map = map_kpa_bins[r] as i32;
            let mut c = 0;
            while c < 16 {
                let rpm = rpm_bins[c] as i32;

                // Smooth, realistic NA Honda Low-Cam VE (55% idle vacuum -> ~94% torque peak)
                let load_factor = (map - 20) * 280 / 85; // 0..~360 (0.0%..36.0%)
                let rpm_bell = 120 - ((rpm - 4800).abs() * 120 / 4800);
                let low_ve = (560 + load_factor + rpm_bell).clamp(480, 960) as u16;
                ve_low_cam_x10[r][c] = low_ve;

                // VTEC High-Cam breathes ~8–12% stronger from 4800–8400 RPM
                let vtec_boost = if rpm >= 4500 {
                    40 + ((rpm - 4500) * 70 / 3900)
                } else {
                    15
                };
                ve_vtec_cam_x10[r][c] = (low_ve + vtec_boost as u16).clamp(500, 1060);

                // Safe conservative ignition advance:
                // ~16° at idle, ~34° light-load cruise, ~26–28° WOT (100 kPa)
                let base_rpm_adv = ((rpm - 600) * 180 / 4400).clamp(0, 180);
                let vacuum_adv = ((95 - map).max(0)) * 90 / 75;
                let low_spk = (160 + base_rpm_adv + vacuum_adv - ((map - 95).max(0) * 4))
                    .clamp(100, 380) as i16;
                spark_low_cam_deg_x10[r][c] = low_spk;
                spark_vtec_cam_deg_x10[r][c] = (low_spk + 15).clamp(100, 390);

                // Target Lambda: 1.000 stoich in cruise, tapering to 0.865 (~12.7:1 AFR) at WOT
                target_lambda_x1000[r][c] = if map >= 92 {
                    865
                } else if map >= 80 {
                    930
                } else {
                    1000
                };

                c += 1;
            }
            r += 1;
        }

        Self {
            magic: CALIBRATION_MAGIC,
            version: CALIBRATION_VERSION,
            displacement_cc: 1590,
            injector_flow_cc_min: 240,
            stoich_afr_x100: 1470,
            rev_limit_soft_rpm: 7300,
            rev_limit_hard_rpm: 7500,
            scs_base_timing_deg_x10: 160, // Honda factory 16.0° BTDC timing mark
            trigger_offset_deg_x10: 0,
            fan_on_ect_c: 93,
            fan_off_ect_c: 89,
            map_cal: MapSensorCal {
                kpa_at_0v: -1.5,
                kpa_at_5v: 182.5,
            },
            vtec: VtecConfig {
                engage_rpm: 4800,
                disengage_rpm: 4600,
                min_map_kpa: 65,
                min_tps_pct: 45,
                min_vss_kmh: 25,
                min_ect_c: 65,
                require_vtp_pressure_switch: true,
                vtp_confirm_timeout_ms: 250,
            },
            rpm_bins,
            map_kpa_bins,
            ve_low_cam_x10,
            ve_vtec_cam_x10,
            spark_low_cam_deg_x10,
            spark_vtec_cam_deg_x10,
            target_lambda_x1000,
            vbatt_bins_x10: [80, 100, 115, 125, 135, 142, 150, 160],
            inj_deadtime_us: [2150, 1550, 1250, 1080, 940, 860, 790, 730],
            cop_dwell_us: [4600, 3900, 3450, 3150, 2850, 2650, 2500, 2350],
            ect_bins_c: [-20, 0, 20, 40, 60, 80, 95, 110],
            warmup_enrichment_pct: [165, 145, 128, 114, 104, 100, 100, 103],
            cranking_pw_us: [18_000, 13_500, 9_500, 6_800, 5_200, 4_400, 4_200, 4_200],
            iacv_base_duty_per_mille: [680, 600, 520, 450, 390, 340, 340, 380],
        }
    }

    #[inline]
    pub const fn validate_header(&self) -> bool {
        self.magic == CALIBRATION_MAGIC && self.version == CALIBRATION_VERSION
    }
}

/// Runtime-tunable Knock Window DSP configuration (kept independent of `CalibrationBlob` so
/// TunerStudio page layout and NVM calibration size remain ABI-stable).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KnockConfig {
    /// Enables `ADC3` (`PC2_C`) combustion-window capture and `~7.5 kHz` DSP analysis.
    pub enabled: bool,
    /// Applies per-cylinder knock retard feedback to `IgnitionModel` (`false` = monitor only).
    pub retard_enabled: bool,
    /// Checks biased sensor front-end DC plausibility for `FaultFlags::KNOCK_SENSOR` (Code 23).
    pub sensor_diag_enabled: bool,
    /// Bandpass center frequency in `Hz` (default `7_500 Hz` for OEM Honda resonant piezo).
    pub center_freq_hz: u16,
    /// Biquad quality factor `Q * 10` (default `40` -> `Q = 4.0`, `-3 dB` BW `~1.9 kHz`).
    pub q_x10: u16,
    /// Combustion window start angle ATDC in `0.1°` crank units (default `100` = `10.0° ATDC`).
    pub window_start_deg_x10: u16,
    /// Combustion window end angle ATDC in `0.1°` crank units (default `500` = `50.0° ATDC`).
    pub window_end_deg_x10: u16,
    /// Minimum engine speed (`RPM`) above which knock windows are captured and evaluated.
    pub min_rpm: u16,
    /// Knock detection ratio `threshold * 100` above the RPM adaptive noise floor (`250` = `2.50x`).
    pub threshold_ratio_x100: u16,
    /// Minimum absolute bandpass RMS (`16-bit ADC counts`) before knock can trigger (`250` ~ `13 mV`).
    pub min_abs_rms_counts: u16,
    /// Per-event retard step applied to the knocking cylinder (`0.1°`, e.g. `15` = `1.5°`).
    pub step_retard_deg_x10: i16,
    /// Maximum allowed knock retard per cylinder (`0.1°`, e.g. `80` = `8.0°`).
    pub max_retard_deg_x10: i16,
    /// Hold time (`ms`) after the last knock event before retard starts decaying.
    pub recovery_hold_ms: u16,
    /// Milliseconds required to recover `0.1°` of retard after `recovery_hold_ms` elapses.
    pub decay_ms_per_0_1deg: u16,
    /// Lower DC bias bound (`16-bit counts`) when `sensor_diag_enabled == true`.
    pub diag_min_dc_counts: u16,
    /// Upper DC bias bound (`16-bit counts`) when `sensor_diag_enabled == true`.
    pub diag_max_dc_counts: u16,
}

impl Default for KnockConfig {
    fn default() -> Self {
        Self::default_honda_obd1()
    }
}

impl KnockConfig {
    pub const fn default_honda_obd1() -> Self {
        Self {
            enabled: true,
            retard_enabled: true,
            sensor_diag_enabled: false,
            center_freq_hz: 7_500,
            q_x10: 40,
            window_start_deg_x10: 100,
            window_end_deg_x10: 500,
            min_rpm: 1_200,
            threshold_ratio_x100: 250,
            min_abs_rms_counts: 250,
            step_retard_deg_x10: 15,
            max_retard_deg_x10: 80,
            recovery_hold_ms: 400,
            decay_ms_per_0_1deg: 150,
            diag_min_dc_counts: 3_000,
            diag_max_dc_counts: 62_000,
        }
    }
}

/// Live Knock Window DSP diagnostics & per-cylinder retard telemetry.
///
/// Array fields (`per_cyl_*`) are indexed by [`Cylinder::index()`] (`0=Cyl1, 1=Cyl2, 2=Cyl3, 3=Cyl4`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KnockTelemetry {
    pub enabled: bool,
    pub retard_enabled: bool,
    pub sensor_fault: bool,
    /// Most recently processed cylinder index (`0=Cyl1 .. 3=Cyl4`).
    pub last_cylinder: u8,
    pub center_freq_hz: u16,
    pub last_rpm: u16,
    pub last_sample_count: u16,
    pub last_dc_mean_counts: u16,
    pub per_cyl_rms_counts: [u16; 4],
    pub per_cyl_noise_floor_counts: [u16; 4],
    pub per_cyl_knock_events: [u32; 4],
    pub per_cyl_retard_deg_x10: [i16; 4],
    pub rpm_noise_bins_counts: [u16; 8],
    pub window_count: u32,
    pub capture_overruns: u32,
}

/// Magic header identifying a valid persisted `RuntimeConfigBlob` (`"CIKA"`).
pub const RUNTIME_CONFIG_MAGIC: u32 = 0x4349_4b41;
/// Schema version for persisted [`RuntimeConfigBlob`].
pub const RUNTIME_CONFIG_VERSION: u16 = 2;

/// Operating mode for configurable spare low-side driver outputs (`PE0`, `PE1`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AuxSpareMode {
    /// Output always remains OFF (safe default when chassis pin assignment varies).
    Disabled,
    /// Activates above `engage_rpm` with hysteresis down to `disengage_rpm`.
    ShiftLight { engage_rpm: u16, disengage_rpm: u16 },
    /// Activates under wide-open throttle when `tps_pct >= min_tps_pct` and `rpm >= min_rpm`.
    WotTrigger { min_tps_pct: f32, min_rpm: u16 },
    /// Secondary coolant fan / radiator trigger at `on_c` with turn-off at `off_c`.
    CoolantFanBackup { on_c: f32, off_c: f32 },
    /// Over-temperature warning lamp/solenoid latching above `warn_c` until `< clear_c`.
    OverTempWarning { warn_c: f32, clear_c: f32 },
    /// Boost/pressure trigger (e.g., water-meth pump or dash indicator) latching above `on_kpa`.
    BoostTrigger { on_kpa: f32, off_kpa: f32 },
}

/// Configuration parameters for Phase D.4 auxiliary outputs and frequency inputs.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AuxIoConfig {
    /// Enables Honda dual-mode `ALTC*` (`PD8`, `A16`) Eco/Charge alternator control.
    pub altc_enabled: bool,
    /// Battery voltage threshold above which `ALTC*` may drop to ~12.6V Eco setpoint.
    pub altc_eco_vbatt_on_v: f32,
    /// Battery voltage threshold below which `ALTC*` forces full ~14.4V charging.
    pub altc_eco_vbatt_off_v: f32,
    /// Maximum Electrical Load Detector current (`ELD`) eligible for Eco charging.
    pub altc_max_eld_amps: f32,
    /// `ELD` hysteresis release threshold forcing full charging above heavy electrical draw.
    pub altc_eld_hyst_amps: f32,
    /// Throttle threshold (`%`) above which alternator unloads in Eco mode for WOT power.
    pub altc_wot_unload_tps_pct: f32,
    /// Enables Honda P72 `IAB*` (`PD9`, `A17`) dual-runner butterfly control.
    pub iab_enabled: bool,
    /// Inverts `IAB*` output polarity for OEM normally-closed vacuum solenoid setups.
    pub iab_invert: bool,
    /// Engine speed threshold (`RPM`) where `IAB*` transitions above crossover.
    pub iab_engage_rpm: u16,
    /// Engine speed hysteresis release threshold (`RPM`) for `IAB*`.
    pub iab_disengage_rpm: u16,
    /// Enables closed-loop `EVAP*` (`PD10`, `A20`) canister purge modulation.
    pub evap_enabled: bool,
    /// Minimum coolant temperature (`°C`) required before canister purge begins.
    pub evap_min_ect_c: f32,
    /// Minimum engine speed (`RPM`) for off-idle EVAP purge.
    pub evap_min_rpm: u16,
    /// Stale timeout (`ms`) before `FLEX` sensor reverts to `NotDetected`.
    pub flex_timeout_ms: u32,
    /// Set `true` if Flex temperature pulse is active-low instead of active-high.
    pub flex_active_low_pulse: bool,
    /// Stale timeout (`ms`) before `ALTF*` alternator field capture marks invalid.
    pub altf_timeout_ms: u32,
    /// Set `true` if `ALTF*` duty encodes field load on the low half-cycle.
    pub altf_active_low_duty: bool,
    /// Behavior selector for `PE0` (`A11 EGR* / A18`).
    pub pe0_mode: AuxSpareMode,
    /// Behavior selector for `PE1` (`A19 VAR*`).
    pub pe1_mode: AuxSpareMode,
}

impl Default for AuxIoConfig {
    fn default() -> Self {
        Self::default_honda_obd1()
    }
}

impl AuxIoConfig {
    /// Conservative Honda OBD1 bench & street baseline matching P28/P72 pin conventions.
    pub const fn default_honda_obd1() -> Self {
        Self {
            altc_enabled: true,
            altc_eco_vbatt_on_v: 13.8,
            altc_eco_vbatt_off_v: 13.2,
            altc_max_eld_amps: 15.0,
            altc_eld_hyst_amps: 22.0,
            altc_wot_unload_tps_pct: 75.0,
            iab_enabled: true,
            iab_invert: false,
            iab_engage_rpm: 5_750,
            iab_disengage_rpm: 5_500,
            evap_enabled: true,
            evap_min_ect_c: 65.0,
            evap_min_rpm: 1_200,
            flex_timeout_ms: 250,
            flex_active_low_pulse: false,
            altf_timeout_ms: 150,
            altf_active_low_duty: false,
            pe0_mode: AuxSpareMode::Disabled,
            pe1_mode: AuxSpareMode::Disabled,
        }
    }
}

/// Detection status for the Continental `50–150 Hz` Flex Fuel sensor (`PE15`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlexSensorStatus {
    /// No recent valid edges detected (sensor absent or unplugged).
    NotDetected,
    /// Active signal within nominal `45–155 Hz` Continental ethanol window.
    Valid,
    /// Edges observed outside valid `45–155 Hz` range (contamination / wiring fault).
    OutOfRangeFault,
}

/// Decoded ethanol concentration, fuel temperature, and derived fuel/timing trims.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FlexFuelState {
    pub status: FlexSensorStatus,
    pub freq_hz: f32,
    pub ethanol_pct: f32,
    pub fuel_temp_c: f32,
    pub age_ms: u32,
}

impl Default for FlexFuelState {
    fn default() -> Self {
        Self::new()
    }
}

impl FlexFuelState {
    /// Safe gasoline-baseline state prior to receiving Flex Fuel edges.
    pub const fn new() -> Self {
        Self {
            status: FlexSensorStatus::NotDetected,
            freq_hz: 0.0,
            ethanol_pct: 0.0,
            fuel_temp_c: 20.0,
            age_ms: 0,
        }
    }

    /// Returns `true` when the Continental sensor is actively streaming valid pulses.
    #[inline]
    pub const fn is_valid(&self) -> bool {
        matches!(self.status, FlexSensorStatus::Valid)
    }

    /// Blended stoichiometric Air-Fuel Ratio (`14.7` at `E0`, `9.0` at `E100`).
    pub fn stoich_afr(&self) -> f32 {
        let e = if self.is_valid() {
            (self.ethanol_pct * 0.01).clamp(0.0, 1.0)
        } else {
            0.0
        };
        14.7 * (1.0 - e) + 9.0 * e
    }

    /// Volumetric injector flow multiplier relative to pump gasoline
    /// (`1.000` at `E0`, `~1.412` at `E85`).
    pub fn fuel_volume_multiplier(&self) -> f32 {
        if !self.is_valid() {
            return 1.0;
        }
        let e = (self.ethanol_pct * 0.01).clamp(0.0, 1.0);
        let density_ratio = 1.0 + 0.0662 * e;
        (14.7 / self.stoich_afr()) / density_ratio
    }

    /// Conservative octane timing trim (`0.0°` up to `E10`, ramping linearly to `+4.0°` at `E85+`).
    pub fn spark_advance_offset_deg(&self) -> f32 {
        if !self.is_valid() {
            return 0.0;
        }
        ((self.ethanol_pct - 10.0) / 75.0).clamp(0.0, 1.0) * 4.0
    }
}

/// Decoded `PB7` (`D9 ALTF*`) alternator field duty cycle state.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AltfState {
    pub valid: bool,
    pub freq_hz: f32,
    pub field_duty_pct: f32,
    pub age_ms: u32,
}

impl Default for AltfState {
    fn default() -> Self {
        Self::new()
    }
}

impl AltfState {
    pub const fn new() -> Self {
        Self {
            valid: false,
            freq_hz: 0.0,
            field_duty_pct: 0.0,
            age_ms: 0,
        }
    }
}

/// Commanded discrete gate states for Phase D.4 auxiliary low-side drivers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct AuxOutputs {
    /// `PD8` (`A16 ALTC*`): `true` pulls `A16` low -> Eco/WOT unload (~12.6V), `false` -> ~14.5V.
    pub altc_on: bool,
    /// `PD9` (`A17 IAB*`): dual-runner intake manifold butterfly solenoid gate drive.
    pub iab_on: bool,
    /// `PD10` (`A20 EVAP*`): modulated purge valve gate state for the current `1 ms` tick.
    pub evap_on: bool,
    /// Commanded EVAP purge duty cycle (`0..=1000` per-mille at `10 Hz`).
    pub evap_duty_per_mille: u16,
    /// `PE0` (`A11 EGR* / A18`) low-side gate drive.
    pub pe0_on: bool,
    /// `PE1` (`A19 VAR*`) low-side gate drive.
    pub pe1_on: bool,
}

impl AuxOutputs {
    /// Packs the 5 discrete pin states into a compact `u8` bitmask (`bit0=ALTC..bit4=PE1`).
    pub const fn bitmask(&self) -> u8 {
        (self.altc_on as u8)
            | ((self.iab_on as u8) << 1)
            | ((self.evap_on as u8) << 2)
            | ((self.pe0_on as u8) << 3)
            | ((self.pe1_on as u8) << 4)
    }
}

/// Combined auxiliary telemetry and output snapshot produced every `1 ms`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct AuxIoTelemetry {
    pub outputs: AuxOutputs,
    pub flex: FlexFuelState,
    pub altf: AltfState,
}

/// Operating mode for the `PE1` (`A19 BOOST`) electronic boost control solenoid driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BoostMode {
    /// Wastegate solenoid always off (`0%` duty -> mechanical wastegate spring pressure).
    #[default]
    Off,
    /// Open-loop duty interpolated from `rpm_bins` / `base_duty_per_mille` and tapered by TPS.
    OpenLoop,
    /// Feedforward base duty + fast-spool assist + bounded PID targeting `MAP kPa`.
    ClosedLoop,
}

/// Turbo boost control, overboost/lean-boost safety, Flex Fuel blend, 2-step launch, and
/// flat-foot shift configuration (persisted in `RuntimeConfigBlob`).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoostConfig {
    /// Selects `PE1` (`A19 BOOST`) solenoid operation (`Off`, `OpenLoop`, or `ClosedLoop`).
    pub mode: BoostMode,
    /// Software PWM carrier frequency (`10..=60 Hz`; `30 Hz` nominal for MAC 3-port valves).
    pub solenoid_freq_hz: u16,
    /// Inverts `PE1` solenoid duty for normally-open pneumatic setups.
    pub invert_solenoid: bool,
    /// Minimum duty clamp (`0..=1000` per-mille).
    pub min_duty_per_mille: u16,
    /// Maximum duty clamp (`0..=1000` per-mille, default `900` = `90.0%`).
    pub max_duty_per_mille: u16,
    /// Fast-spool wastegate duty commanded when `MAP` lags target by > `spool_margin_kpa`.
    pub spool_duty_per_mille: u16,
    /// Target minus actual `MAP kPa` error above which fast-spool duty holds wastegate closed.
    pub spool_margin_kpa: f32,
    /// Minimum throttle (`%`) required for boost control above `0%` duty.
    pub min_tps_pct: f32,
    /// Throttle (`%`) above which 100% of mapped boost target / open-loop duty applies.
    pub full_tps_pct: f32,
    /// Minimum engine speed (`RPM`) for active boost control.
    pub min_rpm: u16,
    /// Minimum coolant temperature (`°C`) before electronic boost rises above spring pressure.
    pub min_ect_c: f32,
    /// Shared `8`-point RPM axis for `base_duty_per_mille` and `target_map_kpa`.
    pub rpm_bins: [u16; 8],
    /// Open-loop / closed-loop feedforward duty (`0..=1000` per-mille) across `rpm_bins`.
    pub base_duty_per_mille: [u16; 8],
    /// Closed-loop target absolute manifold pressure (`kPa`) across `rpm_bins`.
    pub target_map_kpa: [u16; 8],
    /// Closed-loop proportional gain × 100 (`per-mille / kPa`, `250` = `2.5%/kPa`).
    pub pid_kp_x100: u16,
    /// Closed-loop integral gain × 100 (`per-mille / (kPa·s)`, `180` = `1.8%/kPa·s`).
    pub pid_ki_x100: u16,
    /// Closed-loop derivative gain × 100 (`per-mille · s / kPa`).
    pub pid_kd_x100: u16,
    /// Symmetrical closed-loop PID correction authority around `base_duty` (`0..=600` per-mille).
    pub pid_authority_per_mille: u16,

    /// Applies `PE15` Continental Flex Fuel volume and timing compensation when sensor is `Valid`.
    pub flex_comp_enabled: bool,
    /// Additional target `MAP kPa` blended linearly from `E10 (0%)` to `E85+ (100%)`.
    pub flex_boost_adder_kpa: f32,

    /// Enables low-speed boost-by-speed ceiling using `B10 VSS` (`TIM2_CH4`) for FWD traction.
    pub traction_boost_enabled: bool,
    /// Vehicle speed (`km/h`) at or below which `traction_max_map_kpa` caps target boost.
    pub traction_low_vss_kmh: f32,
    /// Vehicle speed (`km/h`) above which full target boost unlocks linearly.
    pub traction_high_vss_kmh: f32,
    /// Maximum target `MAP kPa` allowed at `<= traction_low_vss_kmh`.
    pub traction_max_map_kpa: f32,

    /// Enables fuel + spark overboost protection cut (`FaultFlags::OVERBOOST_CUT`).
    pub overboost_cut_enabled: bool,
    /// Warm overboost cut threshold (`kPa` absolute, e.g., `250.0` = `~21.6 psi` gauge).
    pub overboost_cut_kpa: f32,
    /// Recovery hysteresis (`kPa`) below active cut threshold before restoring combustion.
    pub overboost_hyst_kpa: f32,
    /// Stricter overboost cut threshold (`kPa` absolute) when `ECT < min_ect_c`.
    pub cold_overboost_cut_kpa: f32,

    /// Drops boost solenoid to `0%` (spring pressure) when max knock retard >= threshold.
    pub knock_drop_enabled: bool,
    /// Maximum per-cylinder knock retard (`0.1°`, e.g., `35` = `3.5°`) triggering boost drop.
    pub knock_drop_retard_deg_x10: i16,

    /// Triggers safety cut and drops solenoid to `0%` if boosted AFR goes leaner than target.
    pub lean_prot_enabled: bool,
    /// Minimum `MAP kPa` above which lean-boost protection monitors wideband lambda.
    pub lean_min_map_kpa: f32,
    /// Maximum allowed positive lambda error (`λ_meas - λ_target`) under boost.
    pub lean_max_lambda_err: f32,
    /// Sustained lean duration (`ms`) required before latching `FaultFlags::LEAN_BOOST_CUT`.
    pub lean_delay_ms: u16,
    /// Maximum `MAP kPa` above which negative closed-loop STFT pull (`< 0%`) is inhibited.
    pub stft_max_map_kpa: f32,

    /// Enables 2-step launch control on `PE12` (`B7` clutch) below `launch_max_vss_kmh`.
    pub launch_enabled: bool,
    /// Target 2-step launch engine speed (`RPM`).
    pub launch_rpm: u16,
    /// Lead-in retard/cut ramp window below `launch_rpm`.
    pub launch_window_rpm: u16,
    /// Maximum stationary speed (`km/h`) eligible for 2-step launch (`> max` enters FFS logic).
    pub launch_max_vss_kmh: f32,
    /// Minimum throttle (`%`) required to arm 2-step launch.
    pub launch_min_tps_pct: f32,
    /// Spark advance commanded at `launch_rpm` (`0.1° BTDC`, negative = ATDC for anti-lag spool).
    pub launch_retard_deg_x10: i16,
    /// Additional fuel enrichment (`%`) applied while 2-step launch is active.
    pub launch_fuel_enrich_pct: u8,
    /// Minimum boost solenoid duty (`0..=1000` per-mille) held during active 2-step launch.
    pub launch_boost_duty_per_mille: u16,

    /// Enables WOT clutch flat-foot shift limiter above `launch_max_vss_kmh`.
    pub ffs_enabled: bool,
    /// Minimum RPM required to engage flat-foot shift on clutch press.
    pub ffs_min_rpm: u16,
    /// RPM subtracted from clutch-entry RPM to set the gear-change hold target.
    pub ffs_rpm_drop: u16,
    /// Minimum throttle (`%`) required for flat-foot shift activation.
    pub ffs_min_tps_pct: f32,
    /// Spark timing held during flat-foot shift (`0.1° BTDC`).
    pub ffs_retard_deg_x10: i16,
    /// Maximum continuous flat-foot shift duration (`ms`) per clutch press.
    pub ffs_max_duration_ms: u16,
}

impl Default for BoostConfig {
    fn default() -> Self {
        Self::default_honda_obd1()
    }
}

impl BoostConfig {
    /// Conservative turbo baseline with overboost protection armed and wastegate solenoid off
    /// until explicitly enabled by the tuner.
    pub const fn default_honda_obd1() -> Self {
        Self {
            mode: BoostMode::Off,
            solenoid_freq_hz: 30,
            invert_solenoid: false,
            min_duty_per_mille: 0,
            max_duty_per_mille: 900,
            spool_duty_per_mille: 900,
            spool_margin_kpa: 25.0,
            min_tps_pct: 25.0,
            full_tps_pct: 80.0,
            min_rpm: 1_500,
            min_ect_c: 60.0,
            rpm_bins: [2_500, 3_200, 4_000, 4_800, 5_600, 6_400, 7_200, 8_200],
            base_duty_per_mille: [350, 380, 420, 450, 460, 470, 480, 480],
            target_map_kpa: [155, 165, 170, 170, 170, 170, 165, 160],
            pid_kp_x100: 250,
            pid_ki_x100: 180,
            pid_kd_x100: 20,
            pid_authority_per_mille: 300,
            flex_comp_enabled: true,
            flex_boost_adder_kpa: 30.0,
            traction_boost_enabled: false,
            traction_low_vss_kmh: 35.0,
            traction_high_vss_kmh: 75.0,
            traction_max_map_kpa: 145.0,
            overboost_cut_enabled: true,
            overboost_cut_kpa: 250.0,
            overboost_hyst_kpa: 15.0,
            cold_overboost_cut_kpa: 165.0,
            knock_drop_enabled: true,
            knock_drop_retard_deg_x10: 35,
            lean_prot_enabled: false,
            lean_min_map_kpa: 145.0,
            lean_max_lambda_err: 0.08,
            lean_delay_ms: 150,
            stft_max_map_kpa: 115.0,
            launch_enabled: false,
            launch_rpm: 4_500,
            launch_window_rpm: 250,
            launch_max_vss_kmh: 8.0,
            launch_min_tps_pct: 50.0,
            launch_retard_deg_x10: 0,
            launch_fuel_enrich_pct: 10,
            launch_boost_duty_per_mille: 500,
            ffs_enabled: false,
            ffs_min_rpm: 4_500,
            ffs_rpm_drop: 400,
            ffs_min_tps_pct: 80.0,
            ffs_retard_deg_x10: 50,
            ffs_max_duration_ms: 900,
        }
    }
}

/// Live turbo boost controller, safety cut, Flex trim, 2-step launch, and FFS telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BoostTelemetry {
    pub mode: BoostMode,
    pub baro_kpa: f32,
    pub boost_kpa: f32,
    pub target_kpa: f32,
    pub solenoid_duty_per_mille: u16,
    pub solenoid_on: bool,
    pub overboost_cut_active: bool,
    pub lean_cut_active: bool,
    pub knock_drop_active: bool,
    pub cold_limit_active: bool,
    pub launch_active: bool,
    pub ffs_active: bool,
    pub flex_fuel_mult: f32,
    pub flex_spark_add_deg: f32,
    pub peak_boost_kpa: f32,
}

impl Default for BoostTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

impl BoostTelemetry {
    pub const fn new() -> Self {
        Self {
            mode: BoostMode::Off,
            baro_kpa: 101.3,
            boost_kpa: 0.0,
            target_kpa: 101.3,
            solenoid_duty_per_mille: 0,
            solenoid_on: false,
            overboost_cut_active: false,
            lean_cut_active: false,
            knock_drop_active: false,
            cold_limit_active: false,
            launch_active: false,
            ffs_active: false,
            flex_fuel_mult: 1.0,
            flex_spark_add_deg: 0.0,
            peak_boost_kpa: 0.0,
        }
    }
}

/// Non-volatile runtime feature settings (`KnockConfig` + `AuxIoConfig` + `BoostConfig`) stored
/// independently of the TunerStudio `2,816`-byte calibration page.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RuntimeConfigBlob {
    pub magic: u32,
    pub version: u16,
    pub knock: KnockConfig,
    pub aux: AuxIoConfig,
    pub boost: BoostConfig,
}

impl Default for RuntimeConfigBlob {
    fn default() -> Self {
        Self::default_honda_obd1()
    }
}

impl RuntimeConfigBlob {
    pub const fn default_honda_obd1() -> Self {
        Self {
            magic: RUNTIME_CONFIG_MAGIC,
            version: RUNTIME_CONFIG_VERSION,
            knock: KnockConfig::default_honda_obd1(),
            aux: AuxIoConfig::default_honda_obd1(),
            boost: BoostConfig::default_honda_obd1(),
        }
    }

    #[inline]
    pub const fn validate_header(&self) -> bool {
        self.magic == RUNTIME_CONFIG_MAGIC && self.version == RUNTIME_CONFIG_VERSION
    }
}

/// Physical non-volatile storage medium detected on `SPI1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NvmMedium {
    /// No `SPI1` NVM device responded to JEDEC `RDID (0x9F)` (running RAM-only).
    #[default]
    None,
    /// Standard JEDEC `SPI` NOR Flash (`PY25Q64HA`, `W25Q64`, etc.; 24-bit addr + 4 KB erase).
    SpiNorFlash,
    /// Ferroelectric RAM (`FM25V02A`, `MB85RS256B`, etc.; instant byte writes, no erase).
    SpiFram,
}

/// Live `SPI1` non-volatile memory hardware & ping-pong slot summary reported over USB RPC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NvmStatusReport {
    pub detected: bool,
    pub medium: NvmMedium,
    /// Raw `RDID (0x9F)` identification bytes (`3 B` NOR Flash, `4 B` Fujitsu FRAM, `9 B` Cypress).
    pub rdid: [u8; 9],
    pub rdid_len: u8,
    /// Number of SPI address bytes transmitted after `READ`/`WRITE` opcodes (`2` or `3`).
    pub addr_bytes: u8,
    /// Decoded total chip capacity in bytes (`0` if capacity could not be decoded).
    pub capacity_bytes: u32,
    pub cal_active_slot: Option<u8>,
    pub cal_active_seq: u32,
    pub cfg_active_slot: Option<u8>,
    pub cfg_active_seq: u32,
}

/// Runtime enable/disable mask for sequential ignition (`COP1..4`) and fuel injectors (`INJ1..4`).
///
/// Stored strictly in volatile RAM (never written to NVM) so an ECU reset or power-cycle always
/// restores all 4 cylinders enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutputEnableMask {
    /// Master ignition enable (`false` = cuts all 4 Smart COP channels `PE9/PE11/PE13/PE14`).
    pub ignition_enabled: bool,
    /// Per-cylinder ignition enable (`[Cyl1, Cyl2, Cyl3, Cyl4]`).
    pub cop_cyl_enabled: [bool; 4],
    /// Master fuel injector enable (`false` = cuts all 4 injector channels `PC6/PC7/PB0/PB1`).
    pub injectors_enabled: bool,
    /// Per-cylinder fuel injector enable (`[Cyl1, Cyl2, Cyl3, Cyl4]`).
    pub inj_cyl_enabled: [bool; 4],
}

impl Default for OutputEnableMask {
    fn default() -> Self {
        Self::all_enabled()
    }
}

impl OutputEnableMask {
    /// Returns a mask with master and per-cylinder ignition and injectors all enabled.
    pub const fn all_enabled() -> Self {
        Self {
            ignition_enabled: true,
            cop_cyl_enabled: [true; 4],
            injectors_enabled: true,
            inj_cyl_enabled: [true; 4],
        }
    }

    /// Returns `true` if Smart COP ignition is enabled for `cyl`.
    #[inline]
    pub const fn is_cop_enabled(self, cyl: Cylinder) -> bool {
        self.ignition_enabled && self.cop_cyl_enabled[cyl.index()]
    }

    /// Returns `true` if fuel injection is enabled for `cyl`.
    #[inline]
    pub const fn is_inj_enabled(self, cyl: Cylinder) -> bool {
        self.injectors_enabled && self.inj_cyl_enabled[cyl.index()]
    }

    /// Returns `true` if any master or per-cylinder ignition/injector cut is currently active.
    #[inline]
    pub const fn any_cut_active(self) -> bool {
        !self.ignition_enabled
            || !self.injectors_enabled
            || !self.cop_cyl_enabled[0]
            || !self.cop_cyl_enabled[1]
            || !self.cop_cyl_enabled[2]
            || !self.cop_cyl_enabled[3]
            || !self.inj_cyl_enabled[0]
            || !self.inj_cyl_enabled[1]
            || !self.inj_cyl_enabled[2]
            || !self.inj_cyl_enabled[3]
    }
}

/// Key-On Engine-Off (`RPM == 0`) diagnostic output test command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputTestCommand {
    /// Immediately stops any active diagnostic output test sequence.
    StopAllTests,
    /// Pulses a single Smart COP channel (`PE9/PE11/PE13/PE14`) + `PB8 TACH_OUT` at `RPM == 0`.
    SparkTest {
        cylinder: Cylinder,
        pulse_count: u8,
        period_ms: u16,
        dwell_us: u16,
    },
    /// Pulses a single sequential fuel injector (`PC6/PC7/PB0/PB1`) at `RPM == 0`.
    InjectorTest {
        cylinder: Cylinder,
        pulse_count: u8,
        period_ms: u16,
        pulse_width_us: u16,
    },
    /// Pulses `PB8 TACH_OUT` (`TIM16` `1.2 ms` one-pulse) at `RPM == 0` to verify cluster tach.
    TachSweep { pulse_count: u8, period_ms: u16 },
    /// Energizes `PD3` (`A7/A8 FLR` Main Relay fuel pump) for `duration_ms` while `RPM == 0`.
    FuelPumpPrime { duration_ms: u16 },
}

/// Live ignition/injector enable mask, active test progress, and hardware timer counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiagStatusReport {
    pub enable_mask: OutputEnableMask,
    pub test_active: bool,
    pub pulses_remaining: u8,
    pub fuel_pump_override_ms: u16,
    pub spark_fire_count: u32,
    pub inj_pulse_count: u32,
    pub tach_pulse_count: u32,
    pub cop_watchdog_count: u32,
}

/// Target automotive platform / harness family reported by ECU firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum EcuPlatform {
    HondaObd1 = 0,
    Toyota4aFe = 1,
    Universal4Cyl = 2,
}

/// Crank/cam trigger decoder topology implemented by ECU firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum EcuTriggerKind {
    Honda24_4_1 = 0,
    Toyota24_2NeG = 1,
    MissingTooth36_2 = 2,
}

/// Ignition driver topology supported by ECU hardware/firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum EcuIgnitionKind {
    SmartCop4Ch = 0,
    DistributorSingleCoil = 1,
    WastedSpark2Ch = 2,
}

/// Bitmask of hardware and firmware features supported by the connected ECU.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EcuFeatureFlags(pub u32);

impl EcuFeatureFlags {
    pub const NONE: u32 = 0;
    pub const VTEC: u32 = 1 << 0;
    pub const ELD: u32 = 1 << 1;
    pub const SMART_ALTERNATOR: u32 = 1 << 2;
    pub const IAB: u32 = 1 << 3;
    pub const KNOCK_DSP: u32 = 1 << 4;
    pub const FLEX_FUEL: u32 = 1 << 5;
    pub const EBC_BOOST: u32 = 1 << 6;
    pub const LAUNCH_2STEP: u32 = 1 << 7;
    pub const FLAT_FOOT_SHIFT: u32 = 1 << 8;
    pub const WIDEBAND_LSU49: u32 = 1 << 9;
    pub const SD_DATALOGGER: u32 = 1 << 10;

    #[inline]
    pub const fn contains(self, bit: u32) -> bool {
        (self.0 & bit) != 0
    }
}

/// Hardware and firmware capabilities reported via `Request::GetCapabilities`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EcuCapabilitiesReport {
    pub platform: EcuPlatform,
    pub trigger: EcuTriggerKind,
    pub ignition: EcuIgnitionKind,
    pub cylinders: u8,
    pub cal_version: u16,
    pub features: EcuFeatureFlags,
    pub max_map_kpa: u16,
    pub board_rev_major: u8,
    pub board_rev_minor: u8,
}

impl EcuCapabilitiesReport {
    /// Canonical capability descriptor for the STM32H743 Stage 2 Honda OBD1 ECU firmware.
    pub const fn honda_obd1_stage2() -> Self {
        Self {
            platform: EcuPlatform::HondaObd1,
            trigger: EcuTriggerKind::Honda24_4_1,
            ignition: EcuIgnitionKind::SmartCop4Ch,
            cylinders: 4,
            cal_version: CALIBRATION_VERSION,
            features: EcuFeatureFlags(
                EcuFeatureFlags::VTEC
                    | EcuFeatureFlags::ELD
                    | EcuFeatureFlags::SMART_ALTERNATOR
                    | EcuFeatureFlags::IAB
                    | EcuFeatureFlags::KNOCK_DSP
                    | EcuFeatureFlags::FLEX_FUEL
                    | EcuFeatureFlags::EBC_BOOST
                    | EcuFeatureFlags::LAUNCH_2STEP
                    | EcuFeatureFlags::FLAT_FOOT_SHIFT
                    | EcuFeatureFlags::WIDEBAND_LSU49
                    | EcuFeatureFlags::SD_DATALOGGER,
            ),
            max_map_kpa: 450,
            board_rev_major: 2,
            board_rev_minor: 0,
        }
    }

    #[inline]
    pub const fn has_feature(self, bit: u32) -> bool {
        self.features.contains(bit)
    }
}

/// Computes IEEE 802.3 CRC-32 over an arbitrary byte slice.
#[inline]
pub fn crc32(bytes: &[u8]) -> u32 {
    crc32fast::hash(bytes)
}

#[cfg(test)]
mod tests {
    use super::{
        CALIBRATION_MAGIC, CALIBRATION_VERSION, CYLINDER_TDC_DEG_X10, CalibrationBlob, Cylinder,
        FaultFlags, HONDA_FIRING_ORDER,
    };

    #[test]
    fn default_honda_calibration_has_valid_header_and_monotonic_axes() {
        let mut cal = CalibrationBlob::default_honda_obd1();
        assert!(cal.validate_header());
        for w in cal.rpm_bins.windows(2) {
            assert!(w[0] < w[1], "RPM bins must strictly increase");
        }
        for w in cal.map_kpa_bins.windows(2) {
            assert!(w[0] < w[1], "MAP bins must strictly increase");
        }
        assert_eq!(cal.scs_base_timing_deg_x10, 160);
        assert!(cal.vtec.disengage_rpm < cal.vtec.engage_rpm);

        cal.magic = 0;
        assert!(!cal.validate_header());
        cal.magic = CALIBRATION_MAGIC;
        cal.version = CALIBRATION_VERSION + 1;
        assert!(!cal.validate_header());
    }

    #[test]
    fn firing_order_matches_honda_1_3_4_2_and_tdc_angles() {
        assert_eq!(
            HONDA_FIRING_ORDER,
            [
                Cylinder::Cyl1,
                Cylinder::Cyl3,
                Cylinder::Cyl4,
                Cylinder::Cyl2
            ]
        );
        assert_eq!(Cylinder::Cyl1.number(), 1);
        assert_eq!(Cylinder::Cyl4.number(), 4);
        assert_eq!(CYLINDER_TDC_DEG_X10[Cylinder::Cyl1.index()], 0);
        assert_eq!(CYLINDER_TDC_DEG_X10[Cylinder::Cyl3.index()], 1800);
        assert_eq!(CYLINDER_TDC_DEG_X10[Cylinder::Cyl4.index()], 3600);
        assert_eq!(CYLINDER_TDC_DEG_X10[Cylinder::Cyl2.index()], 5400);

        let mut faults = FaultFlags::default();
        assert!(faults.is_empty());
        faults.insert(FaultFlags::MAP_RANGE);
        faults.insert(FaultFlags::WIDEBAND_FAULT);
        assert!(!faults.is_empty());
        assert!(faults.contains(FaultFlags::MAP_RANGE));
        assert!(faults.contains(FaultFlags::WIDEBAND_FAULT));
        assert!(!faults.contains(FaultFlags::ECT_RANGE));

        let caps = super::EcuCapabilitiesReport::honda_obd1_stage2();
        assert_eq!(caps.platform, super::EcuPlatform::HondaObd1);
        assert_eq!(caps.trigger, super::EcuTriggerKind::Honda24_4_1);
        assert!(caps.has_feature(super::EcuFeatureFlags::VTEC));
        assert!(caps.has_feature(super::EcuFeatureFlags::EBC_BOOST));
    }
}
