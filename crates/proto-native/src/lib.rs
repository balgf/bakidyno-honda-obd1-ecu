#![no_std]

//! Native framed RPC protocol (`postcard` + `CRC32` + `COBS` `0x00` packet delimiter)
//! shared by `firmware`, `ecu-cli`, and our future custom Rust `tuner-gui`.

use ecu_types::{TelemetrySnapshot, crc32};
use heapless::Vec;
use serde::{Deserialize, Serialize};

pub const PROTO_VERSION: u16 = 1;
pub const MAX_CHUNK_BYTES: usize = 256;
pub const MAX_FRAME_BYTES: usize = 512;

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

#[cfg(test)]
mod tests {
    use ecu_types::{
        ActuatorOutputs, FaultFlags, SensorSnapshot, SyncState, TelemetrySnapshot, VtecState,
        WidebandState,
    };
    use heapless::Vec;

    use super::{
        MAX_CHUNK_BYTES, MAX_FRAME_BYTES, ProtoError, Request, Response, cobs_decode,
        cobs_encode_with_delimiter, decode_frame, encode_frame,
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
}
