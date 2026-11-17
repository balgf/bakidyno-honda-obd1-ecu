#![no_std]

//! Native framed RPC protocol (`postcard` + `CRC32` + `COBS` `0x00` packet delimiter)
//! shared by `firmware`, `ecu-cli`, and our future custom Rust `tuner-gui`.

pub use ecu_types::{
    AltfState, AuxIoConfig, AuxIoTelemetry, AuxOutputs, AuxSpareMode, FlexFuelState,
    FlexSensorStatus, KnockConfig, KnockTelemetry, NvmMedium, NvmStatusReport, RuntimeConfigBlob,
};
use ecu_types::{TelemetrySnapshot, crc32};
use heapless::Vec;
use serde::{Deserialize, Serialize};

pub const PROTO_VERSION: u16 = 1;
pub const MAX_CHUNK_BYTES: usize = 256;
pub const MAX_FRAME_BYTES: usize = 512;
/// Magic sentinel (`"FORM"`) required in [`Request::FormatSdFat32`] to prevent accidental wipes.
pub const SD_FORMAT_CONFIRM_MAGIC: u32 = 0x464f_524d;

/// Filesystem/partition table layout observed read-only on the MicroSD slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SdDiskKind {
    Unknown,
    Blank,
    MbrFat32,
    MbrExFat,
    MbrNtfs,
    MbrLinux,
    MbrOther(u8),
    Gpt,
    SuperfloppyFat32,
    SuperfloppyExFat,
}

/// Compact root directory file summary returned by [`Response::SdFileList`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SdDirEntry {
    pub name: [u8; 12],
    pub name_len: u8,
    pub size_bytes: u32,
}

/// Live `SDMMC1` + FAT32 + `RTC` + `PC13` mark status snapshot returned by [`Request::GetSdStatus`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SdStatusReport {
    pub pd4_pin_high: bool,
    pub pc13_pin_high: bool,
    pub pc13_active_high: bool,
    pub card_initialized: bool,
    pub disk_kind: SdDiskKind,
    pub fat32_mounted: bool,
    pub logging_active: bool,
    pub format_in_progress: bool,
    pub last_error_code: u8,
    pub card_prod_name: [u8; 5],
    pub capacity_mb: u32,
    pub active_file: [u8; 12],
    pub active_file_len: u8,
    pub bytes_written: u32,
    pub records_logged: u32,
    pub dropped_records: u32,
    pub write_errors: u32,
    pub mark_count: u32,
    pub rtc_year: u16,
    pub rtc_month: u8,
    pub rtc_day: u8,
    pub rtc_hour: u8,
    pub rtc_minute: u8,
    pub rtc_second: u8,
}

/// Host-to-ECU RPC request (`no_std` stack-allocated; `Box` intentionally avoided).
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Request {
    Ping {
        nonce: u32,
    },
    GetTelemetry,
    ReadCalibrationChunk {
        offset: u16,
        len: u16,
    },
    WriteCalibrationChunk {
        offset: u16,
        data: Vec<u8, MAX_CHUNK_BYTES>,
    },
    /// Atomically commits active RAM calibration into `SPI1` FRAM / `W25Q64` with updated `CRC32`.
    CommitCalibrationToNvm,
    /// Locks spark timing (e.g., `Some(160)` = `16.0° BTDC`) for base-timing verification.
    SetTimingOverride {
        advance_deg_x10: Option<i16>,
    },
    /// Reads live MicroSD card, FAT32 logger, `PD4`/`PC13` pin, and `RTC` state.
    GetSdStatus,
    /// Lists up to 8 FAT32 root directory files starting from entry `offset`.
    ListSdFiles {
        offset: u16,
    },
    /// Formats the inserted MicroSD card as MBR + FAT32 (`confirm_magic == SD_FORMAT_CONFIRM_MAGIC`).
    FormatSdFat32 {
        confirm_magic: u32,
    },
    /// Downloads up to [`MAX_CHUNK_BYTES`] from a root-directory FAT32 file (e.g., `LOG00001.BDL`).
    ReadSdFileChunk {
        name: [u8; 12],
        name_len: u8,
        offset: u32,
        len: u16,
    },
    /// Updates the STM32H743 `LSE` `RTC` calendar & wall-clock time (`weekday`: `0` = Sun .. `6` = Sat).
    SetRtcDateTime {
        year: u16,
        month: u8,
        day: u8,
        weekday: u8,
        hour: u8,
        minute: u8,
        second: u8,
    },
    /// Triggers a manual datalog mark event over USB RPC (equivalent to `PC13` `KEY` button press).
    TriggerLogMark,
    /// Reads live `ADC3` Knock Window DSP configuration, per-cylinder RMS/floor/retard, and stats.
    GetKnockStatus,
    /// Updates runtime `ADC3` Knock Window DSP configuration (takes effect immediately in RAM).
    SetKnockConfig {
        config: KnockConfig,
    },
    /// Clears latched RAM MIL fault flags while leaving sensor plausibility & lockouts active.
    ClearFaults,
    /// Reads live Phase D.4 auxiliary I/O, Flex Fuel, and `ALTF*` configuration & telemetry.
    GetAuxStatus,
    /// Updates runtime Phase D.4 auxiliary I/O & frequency capture configuration in RAM.
    SetAuxConfig {
        config: AuxIoConfig,
    },
    /// Persists active RAM `KnockConfig` + `AuxIoConfig` into dedicated `SPI1` NVM slots.
    CommitRuntimeConfigToNvm,
    /// Reads detected `SPI1` NVM medium (`SpiNorFlash` vs `SpiFram`), RDID, capacity & slot info.
    GetNvmStatus,
    /// Reads live `FDCAN1` (`PD0`/`PD1`) and Honda `D7 DLC` (`PA9`/`PA10`) bus diagnostics.
    GetBusStatus,
}

/// Live Knock Window DSP config & telemetry snapshot returned by [`Request::GetKnockStatus`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KnockStatusReport {
    pub config: KnockConfig,
    pub telemetry: KnockTelemetry,
}

/// Live Phase D.4 Auxiliary I/O, Flex Fuel & `ALTF*` report returned by [`Request::GetAuxStatus`].
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct AuxStatusReport {
    pub config: AuxIoConfig,
    pub telemetry: AuxIoTelemetry,
}

/// Live `FDCAN1` (`PD0`/`PD1`) and Honda `D7 DLC` (`PA9`/`PA10`) status returned by
/// [`Request::GetBusStatus`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BusStatusReport {
    pub can_self_test_passed: bool,
    pub can_tx_ok: u32,
    pub can_tx_drop: u32,
    pub can_rx_count: u32,
    pub can_tec: u8,
    pub dlc_answered_frames: u32,
}

/// ECU-to-Host RPC response (`no_std` stack-allocated; `Box` intentionally avoided).
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Response {
    Pong {
        proto_version: u16,
        nonce: u32,
        uptime_ms: u32,
    },
    Telemetry(TelemetrySnapshot),
    CalibrationChunk {
        offset: u16,
        total_len: u16,
        data: Vec<u8, MAX_CHUNK_BYTES>,
    },
    Ack,
    Error(ProtoError),
    SdStatus(SdStatusReport),
    SdFileList {
        total_files: u16,
        entries: Vec<SdDirEntry, 8>,
    },
    SdFileChunk {
        offset: u32,
        total_size: u32,
        data: Vec<u8, MAX_CHUNK_BYTES>,
    },
    KnockStatus(KnockStatusReport),
    AuxStatus(AuxStatusReport),
    NvmStatus(NvmStatusReport),
    BusStatus(BusStatusReport),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtoError {
    BufferTooSmall,
    SerializeFailed,
    BadCobsEncoding,
    CrcMismatch,
    DeserializeFailed,
    OutOfBounds,
    NvmWriteFailed,
    SdNotReady,
    SdBusy,
    SdFormatRejectedEngineRunning,
    SdFileNotFound,
    SdIoFailed,
}

/// Pure `no_std` Consistent Overhead Byte Stuffing (`COBS`) encoder appending a `0x00` delimiter.
pub fn cobs_encode_with_delimiter(input: &[u8], out: &mut [u8]) -> Result<usize, ProtoError> {
    let max_needed = input.len() + (input.len() / 254) + 2;
    if out.len() < max_needed {
        return Err(ProtoError::BufferTooSmall);
    }

    let mut code_idx = 0usize;
    let mut write_idx = 1usize;
    let mut code = 1u8;

    for &byte in input {
        if byte == 0 {
            out[code_idx] = code;
            code_idx = write_idx;
            write_idx += 1;
            code = 1;
        } else {
            out[write_idx] = byte;
            write_idx += 1;
            code += 1;
            if code == 0xff {
                out[code_idx] = code;
                code_idx = write_idx;
                write_idx += 1;
                code = 1;
            }
        }
    }

    out[code_idx] = code;
    out[write_idx] = 0x00; // Packet delimiter
    Ok(write_idx + 1)
}

/// Pure `no_std` `COBS` decoder (accepts slice with or without trailing `0x00` delimiter).
pub fn cobs_decode(input: &[u8], out: &mut [u8]) -> Result<usize, ProtoError> {
    let trimmed = match input.iter().position(|&b| b == 0) {
        Some(pos) => &input[..pos],
        None => input,
    };
    if trimmed.is_empty() {
        return Ok(0);
    }

    let mut read_idx = 0usize;
    let mut write_idx = 0usize;

    while read_idx < trimmed.len() {
        let code = trimmed[read_idx];
        if code == 0 {
            return Err(ProtoError::BadCobsEncoding);
        }
        read_idx += 1;
        let copy_len = (code as usize) - 1;
        if read_idx + copy_len > trimmed.len() || write_idx + copy_len > out.len() {
            return Err(ProtoError::BadCobsEncoding);
        }
        out[write_idx..write_idx + copy_len]
            .copy_from_slice(&trimmed[read_idx..read_idx + copy_len]);
        read_idx += copy_len;
        write_idx += copy_len;

        if code < 0xff && read_idx < trimmed.len() {
            if write_idx >= out.len() {
                return Err(ProtoError::BufferTooSmall);
            }
            out[write_idx] = 0;
            write_idx += 1;
        }
    }

    Ok(write_idx)
}

/// Encodes a `Serialize` message (`Request`/`Response`) as `COBS([payload | crc32_le]) + 0x00`.
pub fn encode_frame<T: Serialize>(msg: &T, out_frame: &mut [u8]) -> Result<usize, ProtoError> {
    let mut raw = [0u8; MAX_FRAME_BYTES];
    let used = postcard::to_slice(msg, &mut raw[..MAX_FRAME_BYTES - 4])
        .map_err(|_| ProtoError::SerializeFailed)?
        .len();
    let checksum = crc32(&raw[..used]);
    raw[used..used + 4].copy_from_slice(&checksum.to_le_bytes());
    cobs_encode_with_delimiter(&raw[..used + 4], out_frame)
}

/// Decodes and `CRC32`-verifies a `COBS`-framed message (`Request` or `Response`).
pub fn decode_frame<T: for<'de> Deserialize<'de>>(frame: &[u8]) -> Result<T, ProtoError> {
    let mut raw = [0u8; MAX_FRAME_BYTES];
    let n = cobs_decode(frame, &mut raw)?;
    if n < 4 {
        return Err(ProtoError::CrcMismatch);
    }
    let payload_len = n - 4;
    let expected_crc = u32::from_le_bytes([
        raw[payload_len],
        raw[payload_len + 1],
        raw[payload_len + 2],
        raw[payload_len + 3],
    ]);
    let actual_crc = crc32(&raw[..payload_len]);
    if expected_crc != actual_crc {
        return Err(ProtoError::CrcMismatch);
    }
    postcard::from_bytes(&raw[..payload_len]).map_err(|_| ProtoError::DeserializeFailed)
}

/// Maximum serialized size of [`RuntimeConfigBlob`] persisted in `SPI1` NVM (`<= 256 B`).
pub const MAX_RUNTIME_CONFIG_BYTES: usize = 256;

/// Serializes a [`RuntimeConfigBlob`] with `postcard` into `out`, returning written length.
pub fn encode_runtime_config(cfg: &RuntimeConfigBlob, out: &mut [u8]) -> Result<usize, ProtoError> {
    Ok(postcard::to_slice(cfg, out)
        .map_err(|_| ProtoError::SerializeFailed)?
        .len())
}

/// Deserializes and header-validates a persisted [`RuntimeConfigBlob`].
pub fn decode_runtime_config(bytes: &[u8]) -> Option<RuntimeConfigBlob> {
    let cfg: RuntimeConfigBlob = postcard::from_bytes(bytes).ok()?;
    cfg.validate_header().then_some(cfg)
}

#[cfg(test)]
mod tests {
    use ecu_types::{
        ActuatorOutputs, AuxIoConfig, AuxIoTelemetry, FaultFlags, KnockConfig, KnockTelemetry,
        NvmMedium, NvmStatusReport, RuntimeConfigBlob, SensorSnapshot, SyncState,
        TelemetrySnapshot, VtecState, WidebandState,
    };
    use heapless::Vec;

    use super::{
        AuxStatusReport, BusStatusReport, KnockStatusReport, MAX_CHUNK_BYTES, MAX_FRAME_BYTES,
        MAX_RUNTIME_CONFIG_BYTES, ProtoError, Request, Response, SD_FORMAT_CONFIRM_MAGIC,
        SdDirEntry, SdDiskKind, SdStatusReport, cobs_decode, cobs_encode_with_delimiter,
        decode_frame, decode_runtime_config, encode_frame, encode_runtime_config,
    };

    #[test]
    fn roundtrips_ping_and_telemetry_frames_and_rejects_bit_flips() {
        let mut wire = [0u8; MAX_FRAME_BYTES];
        let req = Request::Ping { nonce: 0xdead_beef };
        let len = encode_frame(&req, &mut wire).expect("encode Ping");
        assert_eq!(wire[len - 1], 0x00);
        assert!(
            !wire[..len - 1].contains(&0x00),
            "COBS body must contain zero 0x00 bytes"
        );

        let decoded: Request = decode_frame(&wire[..len]).expect("decode Ping");
        assert_eq!(decoded, req);

        // Full TelemetrySnapshot roundtrip
        let snap = TelemetrySnapshot {
            seq: 42,
            uptime_ms: 12_345,
            sync_state: SyncState::FullSynced720,
            rpm: 5850,
            crank_angle_deg_x10: 3600,
            sync_loss_count: 0,
            sensors: SensorSnapshot::default(),
            vtec_state: VtecState::HighCamActive,
            wideband_state: WidebandState::ClosedLoopReady,
            lambda: 0.875,
            lsu_ri_ohms: 301.2,
            ve_pct: 98.4,
            target_lambda: 0.865,
            stft_pct: 1.2,
            actuators: ActuatorOutputs::default(),
            faults: FaultFlags::default(),
        };
        let resp = Response::Telemetry(snap);
        let rlen = encode_frame(&resp, &mut wire).expect("encode Telemetry");
        let got: Response = decode_frame(&wire[..rlen]).expect("decode Telemetry");
        assert_eq!(got, resp);

        // Flip 1 bit -> CRC32 must catch corruption!
        wire[2] ^= 0x04;
        assert_eq!(
            decode_frame::<Response>(&wire[..rlen]),
            Err(ProtoError::CrcMismatch)
        );
    }

    #[test]
    fn cobs_handles_255_byte_nonzero_runs_chunk_rpc_and_error_paths() {
        // 1. 256 consecutive non-zero bytes triggers COBS 0xff block split without virtual zero!
        let mut data = Vec::<u8, MAX_CHUNK_BYTES>::new();
        for i in 0..MAX_CHUNK_BYTES {
            let byte = ((i as u8) % 250) + 1;
            data.push(byte).expect("push chunk byte");
        }
        let req = Request::WriteCalibrationChunk { offset: 512, data };
        let mut wire = [0u8; MAX_FRAME_BYTES];
        let len = encode_frame(&req, &mut wire).expect("encode 256B non-zero chunk");
        assert!(!wire[..len - 1].contains(&0x00));
        let roundtrip: Request = decode_frame(&wire[..len]).expect("decode 256B non-zero chunk");
        assert_eq!(roundtrip, req);

        // 2. BufferTooSmall & malformed COBS length codes
        let mut tiny = [0u8; 2];
        assert_eq!(
            cobs_encode_with_delimiter(&[1, 2, 3], &mut tiny),
            Err(ProtoError::BufferTooSmall)
        );
        let mut out = [0u8; 16];
        assert_eq!(
            cobs_decode(&[0x05, 0x01, 0x02, 0x00], &mut out),
            Err(ProtoError::BadCobsEncoding)
        );
        assert_eq!(
            decode_frame::<Request>(&[0x02, 0x01, 0x00]),
            Err(ProtoError::CrcMismatch)
        );
    }

    #[test]
    fn sd_and_rtc_frames_fit_within_frame_budget() {
        let mut wire = [0u8; MAX_FRAME_BYTES];
        let fmt_req = Request::FormatSdFat32 {
            confirm_magic: SD_FORMAT_CONFIRM_MAGIC,
        };
        let flen = encode_frame(&fmt_req, &mut wire).expect("encode format req");
        assert_eq!(decode_frame::<Request>(&wire[..flen]), Ok(fmt_req));

        let status = SdStatusReport {
            pd4_pin_high: false,
            pc13_pin_high: false,
            pc13_active_high: true,
            card_initialized: true,
            disk_kind: SdDiskKind::MbrFat32,
            fat32_mounted: true,
            logging_active: true,
            format_in_progress: false,
            last_error_code: 0,
            card_prod_name: *b"SD64G",
            capacity_mb: 59_000,
            active_file: *b"LOG00001.BDL",
            active_file_len: 12,
            bytes_written: 65_536,
            records_logged: 1_024,
            dropped_records: 0,
            write_errors: 0,
            mark_count: 3,
            rtc_year: 2026,
            rtc_month: 11,
            rtc_day: 14,
            rtc_hour: 23,
            rtc_minute: 5,
            rtc_second: 12,
        };
        let s_resp = Response::SdStatus(status);
        let slen = encode_frame(&s_resp, &mut wire).expect("encode SdStatus");
        assert_eq!(decode_frame::<Response>(&wire[..slen]), Ok(s_resp));

        let mut entries = Vec::<SdDirEntry, 8>::new();
        for idx in 0..8 {
            entries
                .push(SdDirEntry {
                    name: *b"LOG00001.BDL",
                    name_len: 12,
                    size_bytes: 1_000_000 + idx,
                })
                .expect("push dir entry");
        }
        let l_resp = Response::SdFileList {
            total_files: 8,
            entries,
        };
        let llen = encode_frame(&l_resp, &mut wire).expect("encode max SdFileList");
        assert_eq!(decode_frame::<Response>(&wire[..llen]), Ok(l_resp));

        let mut data = Vec::<u8, MAX_CHUNK_BYTES>::new();
        for i in 0..MAX_CHUNK_BYTES {
            data.push((i & 0xff) as u8).expect("push file byte");
        }
        let c_resp = Response::SdFileChunk {
            offset: 1024,
            total_size: 123_456,
            data,
        };
        let clen = encode_frame(&c_resp, &mut wire).expect("encode max SdFileChunk");
        assert_eq!(decode_frame::<Response>(&wire[..clen]), Ok(c_resp));

        let knock_cfg = KnockConfig::default_honda_obd1();
        let k_req = Request::SetKnockConfig { config: knock_cfg };
        let klen = encode_frame(&k_req, &mut wire).expect("encode SetKnockConfig");
        assert_eq!(decode_frame::<Request>(&wire[..klen]), Ok(k_req));

        let k_resp = Response::KnockStatus(KnockStatusReport {
            config: knock_cfg,
            telemetry: KnockTelemetry {
                enabled: true,
                retard_enabled: true,
                sensor_fault: false,
                last_cylinder: 2,
                center_freq_hz: 7_500,
                last_rpm: 5_200,
                last_sample_count: 256,
                last_dc_mean_counts: 32_768,
                per_cyl_rms_counts: [140, 130, 2_800, 150],
                per_cyl_noise_floor_counts: [250; 4],
                per_cyl_knock_events: [0, 0, 3, 0],
                per_cyl_retard_deg_x10: [0, 0, 45, 0],
                rpm_noise_bins_counts: [250; 8],
                window_count: 1_024,
                capture_overruns: 0,
            },
        });
        let krlen = encode_frame(&k_resp, &mut wire).expect("encode KnockStatus");
        assert_eq!(decode_frame::<Response>(&wire[..krlen]), Ok(k_resp));

        let clr_len = encode_frame(&Request::ClearFaults, &mut wire).expect("encode ClearFaults");
        assert_eq!(
            decode_frame::<Request>(&wire[..clr_len]),
            Ok(Request::ClearFaults)
        );

        let aux_req = Request::SetAuxConfig {
            config: AuxIoConfig::default_honda_obd1(),
        };
        let alen = encode_frame(&aux_req, &mut wire).expect("encode SetAuxConfig");
        assert_eq!(decode_frame::<Request>(&wire[..alen]), Ok(aux_req));

        let burn_rt_len =
            encode_frame(&Request::CommitRuntimeConfigToNvm, &mut wire).expect("encode Commit");
        assert_eq!(
            decode_frame::<Request>(&wire[..burn_rt_len]),
            Ok(Request::CommitRuntimeConfigToNvm)
        );

        let aux_resp = Response::AuxStatus(AuxStatusReport {
            config: AuxIoConfig::default_honda_obd1(),
            telemetry: AuxIoTelemetry::default(),
        });
        let arlen = encode_frame(&aux_resp, &mut wire).expect("encode AuxStatus");
        assert_eq!(decode_frame::<Response>(&wire[..arlen]), Ok(aux_resp));

        let mut rt_buf = [0u8; MAX_RUNTIME_CONFIG_BYTES];
        let mut rt_cfg = RuntimeConfigBlob::default_honda_obd1();
        rt_cfg.knock.center_freq_hz = 7_800;
        rt_cfg.aux.iab_engage_rpm = 5_900;
        let rt_len = encode_runtime_config(&rt_cfg, &mut rt_buf).expect("encode RuntimeConfigBlob");
        assert_eq!(decode_runtime_config(&rt_buf[..rt_len]), Some(rt_cfg));

        let nvm_req_len =
            encode_frame(&Request::GetNvmStatus, &mut wire).expect("encode GetNvmStatus");
        assert_eq!(
            decode_frame::<Request>(&wire[..nvm_req_len]),
            Ok(Request::GetNvmStatus)
        );
        let nvm_resp = Response::NvmStatus(NvmStatusReport {
            detected: true,
            medium: NvmMedium::SpiFram,
            rdid: [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x22, 0x08],
            rdid_len: 9,
            addr_bytes: 2,
            capacity_bytes: 32_768,
            cal_active_slot: Some(1),
            cal_active_seq: 4,
            cfg_active_slot: Some(0),
            cfg_active_seq: 2,
        });
        let nvm_resp_len = encode_frame(&nvm_resp, &mut wire).expect("encode NvmStatus");
        assert_eq!(
            decode_frame::<Response>(&wire[..nvm_resp_len]),
            Ok(nvm_resp)
        );

        let bus_req_len =
            encode_frame(&Request::GetBusStatus, &mut wire).expect("encode GetBusStatus");
        assert_eq!(
            decode_frame::<Request>(&wire[..bus_req_len]),
            Ok(Request::GetBusStatus)
        );
        let bus_resp = Response::BusStatus(BusStatusReport {
            can_self_test_passed: true,
            can_tx_ok: 120,
            can_tx_drop: 3,
            can_rx_count: 15,
            can_tec: 0,
            dlc_answered_frames: 4,
        });
        let bus_resp_len = encode_frame(&bus_resp, &mut wire).expect("encode BusStatus");
        assert_eq!(
            decode_frame::<Response>(&wire[..bus_resp_len]),
            Ok(bus_resp)
        );
    }
}
