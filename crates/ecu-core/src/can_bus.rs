//! Standard `500 kbps` 11-bit classic CAN telemetry frame packing (`FDCAN1`, `PD0`/`PD1`).
//!
//! Uses 8-byte classic CAN payloads on standard identifiers `0x640..=0x642` so both classic
//! (`SN65HVD230`) and CAN-FD (`TCAN1042`) receivers, dashes, and bus loggers interoperate cleanly.

use ecu_types::{SyncState, TelemetrySnapshot, VtecState, WidebandState};

use crate::aux_io::AuxIoTelemetry;

/// Nominal CAN 2.0A / FDCAN arbitration bitrate (`500 kbps`).
pub const CAN_NOMINAL_BITRATE_BPS: u32 = 500_000;
/// Broadcast ID for engine core channels (`RPM`, `MAP`, `TPS`, `Inj PW`).
pub const CAN_ID_CORE: u16 = 0x640;
/// Broadcast ID for temperatures, battery, lambda, spark advance, and engine state bits.
pub const CAN_ID_ENV_SPARK: u16 = 0x641;
/// Broadcast ID for Flex Fuel, `ALTF*`, auxiliary outputs, and canonical fault bitflags.
pub const CAN_ID_AUX_FAULTS: u16 = 0x642;
/// Internal silicon self-test loopback frame ID.
pub const CAN_ID_SELF_TEST: u16 = 0x64f;
/// Signature payload transmitted during startup internal-loopback verification.
pub const CAN_SELF_TEST_PAYLOAD: [u8; 8] = *b"OBD1CAN\x01";

/// Sentinel byte meaning "frequency sensor absent or invalid" in CAN frame `0x642`.
pub const CAN_SENTINEL_UNAVAILABLE: u8 = 0xff;

/// Single classic 8-byte CAN frame ready to queue into `FDCAN1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrameData {
    pub id: u16,
    pub dlc: u8,
    pub data: [u8; 8],
}

/// Decoded contents of CAN frame `0x640` (`CAN_ID_CORE`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedCoreFrame {
    pub rpm: u16,
    pub map_kpa: f32,
    pub tps_pct: f32,
    pub inj_pw_us: u16,
}

/// Decoded contents of CAN frame `0x641` (`CAN_ID_ENV_SPARK`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodedEnvSparkFrame {
    pub vbatt_v: f32,
    pub ect_c: i16,
    pub iat_c: i16,
    pub lambda: f32,
    pub spark_advance_deg: f32,
    pub flags: u8,
}

/// Decoded contents of CAN frame `0x642` (`CAN_ID_AUX_FAULTS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodedAuxFaultsFrame {
    pub ethanol_pct: Option<u8>,
    pub fuel_temp_c: Option<i16>,
    pub altf_duty_pct: Option<u8>,
    pub aux_bitmask: u8,
    pub fault_bits: u32,
}

/// Packs live ECU telemetry and auxiliary I/O into the 3 standard `0x640..=0x642` CAN frames.
pub fn encode_telemetry_frames(
    snap: &TelemetrySnapshot,
    aux: &AuxIoTelemetry,
) -> [CanFrameData; 3] {
    [
        encode_core_frame(snap),
        encode_env_spark_frame(snap, aux),
        encode_aux_faults_frame(snap, aux),
    ]
}

/// Encodes `0x640` (`RPM`, `MAP kPa x10`, `TPS % x10`, `Inj1 PW µs`).
pub fn encode_core_frame(snap: &TelemetrySnapshot) -> CanFrameData {
    let rpm_be = snap.rpm.to_be_bytes();
    let map_x10 = (snap.sensors.map_kpa * 10.0).clamp(0.0, 65_535.0) as u16;
    let map_be = map_x10.to_be_bytes();
    let tps_x10 = (snap.sensors.tps_pct * 10.0).clamp(0.0, 1_000.0) as u16;
    let tps_be = tps_x10.to_be_bytes();
    let inj_pw = snap.actuators.inj_pw_us[0].min(u16::MAX as u32) as u16;
    let inj_be = inj_pw.to_be_bytes();

    CanFrameData {
        id: CAN_ID_CORE,
        dlc: 8,
        data: [
            rpm_be[0], rpm_be[1], map_be[0], map_be[1], tps_be[0], tps_be[1], inj_be[0], inj_be[1],
        ],
    }
}

/// Encodes `0x641` (`VBatt mV`, `ECT +40`, `IAT +40`, `Lambda x1000`, `Spark 0.5°`, status flags).
pub fn encode_env_spark_frame(snap: &TelemetrySnapshot, aux: &AuxIoTelemetry) -> CanFrameData {
    let vbatt_mv = (snap.sensors.vbatt_v * 1_000.0).clamp(0.0, 65_535.0) as u16;
    let vbatt_be = vbatt_mv.to_be_bytes();
    let ect_u8 = (snap.sensors.ect_c + 40.0).clamp(0.0, 254.0) as u8;
    let iat_u8 = (snap.sensors.iat_c + 40.0).clamp(0.0, 254.0) as u8;
    let lam_x1000 = (snap.lambda * 1_000.0).clamp(0.0, 65_535.0) as u16;
    let lam_be = lam_x1000.to_be_bytes();
    let spark_u8 = ((snap.actuators.spark_advance_deg_x10 + 200) / 5).clamp(0, 255) as u8;

    let flags = ((snap.vtec_state == VtecState::HighCamActive) as u8)
        | (((snap.sync_state == SyncState::FullSynced720) as u8) << 1)
        | (((snap.wideband_state == WidebandState::ClosedLoopReady) as u8) << 2)
        | ((snap.actuators.fuel_pump_relay_on as u8) << 3)
        | ((snap.actuators.fan_relay_on as u8) << 4)
        | ((snap.actuators.ac_clutch_relay_on as u8) << 5)
        | ((snap.actuators.mil_lamp_on as u8) << 6)
        | ((aux.flex.is_valid() as u8) << 7);

    CanFrameData {
        id: CAN_ID_ENV_SPARK,
        dlc: 8,
        data: [
            vbatt_be[0],
            vbatt_be[1],
            ect_u8,
            iat_u8,
            lam_be[0],
            lam_be[1],
            spark_u8,
            flags,
        ],
    }
}

/// Encodes `0x642` (`Ethanol %`, `FuelTemp +40`, `ALTF %`, `Aux mask`, `FaultFlags`).
pub fn encode_aux_faults_frame(snap: &TelemetrySnapshot, aux: &AuxIoTelemetry) -> CanFrameData {
    let (eth_u8, ftemp_u8) = if aux.flex.is_valid() {
        (
            aux.flex.ethanol_pct.clamp(0.0, 100.0) as u8,
            (aux.flex.fuel_temp_c + 40.0).clamp(0.0, 165.0) as u8,
        )
    } else {
        (CAN_SENTINEL_UNAVAILABLE, CAN_SENTINEL_UNAVAILABLE)
    };

    let altf_u8 = if aux.altf.valid {
        aux.altf.field_duty_pct.clamp(0.0, 100.0) as u8
    } else {
        CAN_SENTINEL_UNAVAILABLE
    };

    let faults_be = snap.faults.0.to_be_bytes();

    CanFrameData {
        id: CAN_ID_AUX_FAULTS,
        dlc: 8,
        data: [
            eth_u8,
            ftemp_u8,
            altf_u8,
            aux.outputs.bitmask(),
            faults_be[0],
            faults_be[1],
            faults_be[2],
            faults_be[3],
        ],
    }
}

/// Decodes CAN frame `0x640` payload bytes back into physical units.
pub fn decode_core_frame(data: &[u8; 8]) -> DecodedCoreFrame {
    DecodedCoreFrame {
        rpm: u16::from_be_bytes([data[0], data[1]]),
        map_kpa: u16::from_be_bytes([data[2], data[3]]) as f32 * 0.1,
        tps_pct: u16::from_be_bytes([data[4], data[5]]) as f32 * 0.1,
        inj_pw_us: u16::from_be_bytes([data[6], data[7]]),
    }
}

/// Decodes CAN frame `0x641` payload bytes back into physical units.
pub fn decode_env_spark_frame(data: &[u8; 8]) -> DecodedEnvSparkFrame {
    DecodedEnvSparkFrame {
        vbatt_v: u16::from_be_bytes([data[0], data[1]]) as f32 * 0.001,
        ect_c: data[2] as i16 - 40,
        iat_c: data[3] as i16 - 40,
        lambda: u16::from_be_bytes([data[4], data[5]]) as f32 * 0.001,
        spark_advance_deg: (data[6] as i16 * 5 - 200) as f32 * 0.1,
        flags: data[7],
    }
}

/// Decodes CAN frame `0x642` payload bytes back into physical units.
pub fn decode_aux_faults_frame(data: &[u8; 8]) -> DecodedAuxFaultsFrame {
    DecodedAuxFaultsFrame {
        ethanol_pct: (data[0] != CAN_SENTINEL_UNAVAILABLE).then_some(data[0]),
        fuel_temp_c: (data[1] != CAN_SENTINEL_UNAVAILABLE).then_some(data[1] as i16 - 40),
        altf_duty_pct: (data[2] != CAN_SENTINEL_UNAVAILABLE).then_some(data[2]),
        aux_bitmask: data[3],
        fault_bits: u32::from_be_bytes([data[4], data[5], data[6], data[7]]),
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{
        ActuatorOutputs, FaultFlags, SensorSnapshot, SyncState, TelemetrySnapshot, VtecState,
        WidebandState,
    };

    use crate::aux_io::{AltfState, AuxIoTelemetry, AuxOutputs, FlexFuelState, FlexSensorStatus};

    use super::{
        CAN_ID_AUX_FAULTS, CAN_ID_CORE, CAN_ID_ENV_SPARK, decode_aux_faults_frame,
        decode_core_frame, decode_env_spark_frame, encode_telemetry_frames,
    };

    #[test]
    fn can_telemetry_frames_roundtrip_all_channels_accurately() {
        let mut faults = FaultFlags::default();
        faults.insert(FaultFlags::MAP_RANGE);
        faults.insert(FaultFlags::VTP_OIL_PRESSURE);

        let snap = TelemetrySnapshot {
            seq: 42,
            uptime_ms: 12_345,
            sync_state: SyncState::FullSynced720,
            rpm: 6_450,
            crank_angle_deg_x10: 1_800,
            sync_loss_count: 1,
            sensors: SensorSnapshot {
                map_kpa: 99.4,
                tps_pct: 92.5,
                ect_c: 89.0,
                iat_c: 34.0,
                vbatt_v: 13.85,
                vcc_5v_v: 5.0,
                eld_amps: 11.2,
                vss_kmh: 112.0,
                ..Default::default()
            },
            vtec_state: VtecState::HighCamActive,
            wideband_state: WidebandState::ClosedLoopReady,
            lambda: 0.872,
            lsu_ri_ohms: 300.0,
            ve_pct: 96.0,
            target_lambda: 0.87,
            stft_pct: 1.5,
            actuators: ActuatorOutputs {
                inj_pw_us: [11_420; 4],
                spark_advance_deg_x10: 235,
                cop_dwell_us: 2_800,
                vts_high_side_on: true,
                fuel_pump_relay_on: true,
                fan_relay_on: false,
                ac_clutch_relay_on: false,
                mil_lamp_on: true,
                iacv_duty_per_mille: 300,
                lsu_heater_duty_per_mille: 650,
                lsu_ip_dac_code: 2_400,
            },
            faults,
        };

        let aux = AuxIoTelemetry {
            outputs: AuxOutputs {
                altc_on: true,
                iab_on: true,
                evap_on: false,
                evap_duty_per_mille: 0,
                pe0_on: false,
                pe1_on: true,
            },
            flex: FlexFuelState {
                status: FlexSensorStatus::Valid,
                freq_hz: 135.0,
                ethanol_pct: 85.0,
                fuel_temp_c: 48.0,
                age_ms: 0,
            },
            altf: AltfState {
                valid: true,
                freq_hz: 180.0,
                field_duty_pct: 62.0,
                age_ms: 0,
            },
        };

        let frames = encode_telemetry_frames(&snap, &aux);
        assert_eq!(frames[0].id, CAN_ID_CORE);
        assert_eq!(frames[1].id, CAN_ID_ENV_SPARK);
        assert_eq!(frames[2].id, CAN_ID_AUX_FAULTS);

        let f0 = decode_core_frame(&frames[0].data);
        assert_eq!(f0.rpm, 6_450);
        assert!((f0.map_kpa - 99.4).abs() < 0.15);
        assert!((f0.tps_pct - 92.5).abs() < 0.15);
        assert_eq!(f0.inj_pw_us, 11_420);

        let f1 = decode_env_spark_frame(&frames[1].data);
        assert!((f1.vbatt_v - 13.85).abs() < 0.005);
        assert_eq!(f1.ect_c, 89);
        assert_eq!(f1.iat_c, 34);
        assert!((f1.lambda - 0.872).abs() < 0.002);
        assert!((f1.spark_advance_deg - 23.5).abs() < 0.3);
        assert_ne!(f1.flags & 0x07, 0);
        assert_eq!(f1.flags & 0x80, 0x80);

        let f2 = decode_aux_faults_frame(&frames[2].data);
        assert_eq!(f2.ethanol_pct, Some(85));
        assert_eq!(f2.fuel_temp_c, Some(48));
        assert_eq!(f2.altf_duty_pct, Some(62));
        assert_eq!(f2.aux_bitmask, aux.outputs.bitmask());
        assert_eq!(f2.fault_bits, faults.0);
    }

    #[test]
    fn can_frames_encode_unavailable_sentinels_and_subzero_extremes() {
        let snap = TelemetrySnapshot {
            seq: 1,
            uptime_ms: 500,
            sync_state: SyncState::Unsynced,
            rpm: 0,
            crank_angle_deg_x10: 0,
            sync_loss_count: 0,
            sensors: SensorSnapshot {
                map_kpa: 100.0,
                tps_pct: 0.0,
                ect_c: -30.0,
                iat_c: -40.0,
                vbatt_v: 12.15,
                ..Default::default()
            },
            vtec_state: VtecState::LowCam,
            wideband_state: WidebandState::WaitForEngine,
            lambda: 1.000,
            lsu_ri_ohms: 2_500.0,
            ve_pct: 0.0,
            target_lambda: 1.0,
            stft_pct: 0.0,
            actuators: ActuatorOutputs {
                spark_advance_deg_x10: -50, // -5.0° ATDC retarded spark
                ..Default::default()
            },
            faults: FaultFlags::default(),
        };
        let aux = AuxIoTelemetry::default();

        let frames = encode_telemetry_frames(&snap, &aux);
        let f1 = decode_env_spark_frame(&frames[1].data);
        assert_eq!(f1.ect_c, -30);
        assert_eq!(f1.iat_c, -40);
        assert!((f1.spark_advance_deg - (-5.0)).abs() < 0.1);
        assert_eq!(f1.flags, 0);

        let f2 = decode_aux_faults_frame(&frames[2].data);
        assert_eq!(f2.ethanol_pct, None);
        assert_eq!(f2.fuel_temp_c, None);
        assert_eq!(f2.altf_duty_pct, None);
        assert_eq!(f2.aux_bitmask, 0);
        assert_eq!(f2.fault_bits, 0);
    }
}
