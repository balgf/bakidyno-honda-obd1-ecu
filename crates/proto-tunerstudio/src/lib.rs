#![no_std]

//! Optional TunerStudio (`msEnvelope_1.0` `CRC32`-framed) serial adapter.
//!
//! Keeps TunerStudio `.ini` byte-layout concerns completely isolated from `ecu-types`
//! and `ecu-core`, so removing or swapping TunerStudio later touches zero engine code.

use ecu_types::{CalibrationBlob, TelemetrySnapshot, crc32};

pub const TS_SIGNATURE: &[u8] = b"rusEFI CivicOBD1-H743-0.1\0";
pub const TS_VERSION_INFO: &[u8] = b"CivicOBD1 STM32H743 Rust ECU v0.1.0\0";
pub const TS_PROTO_VERSION: &[u8] = b"002";

/// Fixed size (bytes) of TunerStudio Calibration Page 1 (`22 * 128`).
pub const TS_PAGE_SIZE: usize = 2816;
/// Fixed size (bytes) of `OutputChannels` realtime telemetry block.
pub const TS_OCH_SIZE: usize = 24;

pub const TS_FLAG_OK: u8 = 0x00;
pub const TS_FLAG_BURN_OK: u8 = 0x04;
pub const TS_ERR_CRC_FAILURE: u8 = 0x82;
pub const TS_ERR_UNRECOGNIZED_CMD: u8 = 0x83;
pub const TS_ERR_OUT_OF_RANGE: u8 = 0x84;

/// Wraps `[flag_byte | payload]` into a TunerStudio `msEnvelope_1.0` packet:
/// `[len_be_u16 | flag | payload... | crc32_be_u32]`.
pub fn wrap_ts_envelope(flag: u8, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let body_len = 1 + payload.len();
    let total_len = 2 + body_len + 4;
    if out.len() < total_len || body_len > u16::MAX as usize {
        return None;
    }
    out[0..2].copy_from_slice(&(body_len as u16).to_be_bytes());
    out[2] = flag;
    out[3..3 + payload.len()].copy_from_slice(payload);
    let crc = crc32(&out[2..2 + body_len]);
    out[2 + body_len..total_len].copy_from_slice(&crc.to_be_bytes());
    Some(total_len)
}

/// Packs a 24-byte TunerStudio OutputChannel block (`'O'` / `'A'`) from `TelemetrySnapshot`.
pub fn pack_realtime_channels(snap: &TelemetrySnapshot, out: &mut [u8; 24]) {
    let seconds = (snap.uptime_ms / 1_000) as u16;
    let map_x10 = (snap.sensors.map_kpa * 10.0) as u16;
    let ect_x10 = (snap.sensors.ect_c * 10.0) as i16;
    let iat_x10 = (snap.sensors.iat_c * 10.0) as i16;
    let tps_x10 = (snap.sensors.tps_pct * 10.0) as u16;
    let vbatt_x10 = (snap.sensors.vbatt_v * 10.0) as u16;
    let lambda_x1000 = (snap.lambda * 1_000.0) as u16;
    let pw1_us_div10 = (snap.actuators.inj_pw_us[0] / 10).min(u16::MAX as u32) as u16;
    let ve_x10 = (snap.ve_pct * 10.0) as u16;
    let status_bits: u16 = (snap.sync_state as u16)
        | ((snap.vtec_state as u16) << 2)
        | ((snap.actuators.fuel_pump_relay_on as u16) << 4)
        | ((snap.actuators.fan_relay_on as u16) << 5);

    out[0..2].copy_from_slice(&seconds.to_le_bytes());
    out[2..4].copy_from_slice(&snap.rpm.to_le_bytes());
    out[4..6].copy_from_slice(&map_x10.to_le_bytes());
    out[6..8].copy_from_slice(&ect_x10.to_le_bytes());
    out[8..10].copy_from_slice(&iat_x10.to_le_bytes());
    out[10..12].copy_from_slice(&tps_x10.to_le_bytes());
    out[12..14].copy_from_slice(&vbatt_x10.to_le_bytes());
    out[14..16].copy_from_slice(&lambda_x1000.to_le_bytes());
    out[16..18].copy_from_slice(&pw1_us_div10.to_le_bytes());
    out[18..20].copy_from_slice(&snap.actuators.spark_advance_deg_x10.to_le_bytes());
    out[20..22].copy_from_slice(&ve_x10.to_le_bytes());
    out[22..24].copy_from_slice(&status_bits.to_le_bytes());
}

/// Result of handling a framed TunerStudio command packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TsAction {
    /// Total bytes written into `out` (0 if the incoming frame was incomplete).
    pub reply_len: usize,
    /// `true` when RAM `CalibrationBlob` was modified by a `'M'` / `'W'` write chunk.
    pub cal_dirty: bool,
    /// `true` when the host requested `'B'` burn-to-NVM.
    pub burn_requested: bool,
}

fn put_u16(buf: &mut [u8], off: usize, v: u16) {
    buf[off..off + 2].copy_from_slice(&v.to_le_bytes());
}

fn put_i16(buf: &mut [u8], off: usize, v: i16) {
    buf[off..off + 2].copy_from_slice(&v.to_le_bytes());
}

fn put_u32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

fn put_f32(buf: &mut [u8], off: usize, v: f32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

fn get_u16(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

fn get_i16(buf: &[u8], off: usize) -> i16 {
    i16::from_le_bytes([buf[off], buf[off + 1]])
}

fn get_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn get_f32(buf: &[u8], off: usize) -> f32 {
    f32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn pack_u16_slice(buf: &mut [u8], base: usize, vals: &[u16]) {
    for (i, &v) in vals.iter().enumerate() {
        put_u16(buf, base + i * 2, v);
    }
}

fn pack_i16_slice(buf: &mut [u8], base: usize, vals: &[i16]) {
    for (i, &v) in vals.iter().enumerate() {
        put_i16(buf, base + i * 2, v);
    }
}

fn pack_u32_slice(buf: &mut [u8], base: usize, vals: &[u32]) {
    for (i, &v) in vals.iter().enumerate() {
        put_u32(buf, base + i * 4, v);
    }
}

fn unpack_u16_slice(buf: &[u8], base: usize, dst: &mut [u16]) {
    for (i, slot) in dst.iter_mut().enumerate() {
        *slot = get_u16(buf, base + i * 2);
    }
}

fn unpack_i16_slice(buf: &[u8], base: usize, dst: &mut [i16]) {
    for (i, slot) in dst.iter_mut().enumerate() {
        *slot = get_i16(buf, base + i * 2);
    }
}

fn unpack_u32_slice(buf: &[u8], base: usize, dst: &mut [u32]) {
    for (i, slot) in dst.iter_mut().enumerate() {
        *slot = get_u32(buf, base + i * 4);
    }
}

fn pack_u16_table(buf: &mut [u8], base: usize, rows: &[[u16; 16]; 16]) {
    for (r, row) in rows.iter().enumerate() {
        pack_u16_slice(buf, base + r * 32, row);
    }
}

fn pack_i16_table(buf: &mut [u8], base: usize, rows: &[[i16; 16]; 16]) {
    for (r, row) in rows.iter().enumerate() {
        pack_i16_slice(buf, base + r * 32, row);
    }
}

fn unpack_u16_table(buf: &[u8], base: usize, rows: &mut [[u16; 16]; 16]) {
    for (r, row) in rows.iter_mut().enumerate() {
        unpack_u16_slice(buf, base + r * 32, row);
    }
}

fn unpack_i16_table(buf: &[u8], base: usize, rows: &mut [[i16; 16]; 16]) {
    for (r, row) in rows.iter_mut().enumerate() {
        unpack_i16_slice(buf, base + r * 32, row);
    }
}

/// Serializes `cal` into TunerStudio little-endian Page 1 layout (`2816` bytes).
pub fn pack_page(cal: &CalibrationBlob, out: &mut [u8; TS_PAGE_SIZE]) {
    out.fill(0);
    put_u16(out, 0, cal.displacement_cc);
    put_u16(out, 2, cal.injector_flow_cc_min);
    put_u16(out, 4, cal.stoich_afr_x100);
    put_u16(out, 6, cal.rev_limit_soft_rpm);
    put_u16(out, 8, cal.rev_limit_hard_rpm);
    put_i16(out, 10, cal.scs_base_timing_deg_x10);
    put_i16(out, 12, cal.trigger_offset_deg_x10);
    put_i16(out, 14, cal.fan_on_ect_c);
    put_i16(out, 16, cal.fan_off_ect_c);
    put_f32(out, 20, cal.map_cal.kpa_at_0v);
    put_f32(out, 24, cal.map_cal.kpa_at_5v);
    put_u16(out, 28, cal.vtec.engage_rpm);
    put_u16(out, 30, cal.vtec.disengage_rpm);
    put_u16(out, 32, cal.vtec.min_map_kpa);
    out[34] = cal.vtec.min_tps_pct;
    out[35] = cal.vtec.min_vss_kmh;
    put_i16(out, 36, cal.vtec.min_ect_c);
    out[38] = u8::from(cal.vtec.require_vtp_pressure_switch);
    put_u16(out, 40, cal.vtec.vtp_confirm_timeout_ms);

    pack_u16_slice(out, 44, &cal.rpm_bins);
    pack_u16_slice(out, 76, &cal.map_kpa_bins);
    pack_u16_table(out, 108, &cal.ve_low_cam_x10);
    pack_u16_table(out, 620, &cal.ve_vtec_cam_x10);
    pack_i16_table(out, 1132, &cal.spark_low_cam_deg_x10);
    pack_i16_table(out, 1644, &cal.spark_vtec_cam_deg_x10);
    pack_u16_table(out, 2156, &cal.target_lambda_x1000);

    pack_u16_slice(out, 2668, &cal.vbatt_bins_x10);
    pack_u16_slice(out, 2684, &cal.inj_deadtime_us);
    pack_u16_slice(out, 2700, &cal.cop_dwell_us);
    pack_i16_slice(out, 2716, &cal.ect_bins_c);
    pack_u16_slice(out, 2732, &cal.warmup_enrichment_pct);
    pack_u32_slice(out, 2748, &cal.cranking_pw_us);
    pack_u16_slice(out, 2780, &cal.iacv_base_duty_per_mille);
}

/// Applies a validated TunerStudio Page 1 byte buffer back onto `cal`.
pub fn unpack_page_into(page: &[u8; TS_PAGE_SIZE], cal: &mut CalibrationBlob) {
    cal.displacement_cc = get_u16(page, 0);
    cal.injector_flow_cc_min = get_u16(page, 2);
    cal.stoich_afr_x100 = get_u16(page, 4);
    cal.rev_limit_soft_rpm = get_u16(page, 6);
    cal.rev_limit_hard_rpm = get_u16(page, 8);
    cal.scs_base_timing_deg_x10 = get_i16(page, 10);
    cal.trigger_offset_deg_x10 = get_i16(page, 12);
    cal.fan_on_ect_c = get_i16(page, 14);
    cal.fan_off_ect_c = get_i16(page, 16);
    let kpa_0v = get_f32(page, 20);
    let kpa_5v = get_f32(page, 24);
    if kpa_0v.is_finite() && kpa_5v.is_finite() {
        cal.map_cal.kpa_at_0v = kpa_0v;
        cal.map_cal.kpa_at_5v = kpa_5v;
    }
    cal.vtec.engage_rpm = get_u16(page, 28);
    cal.vtec.disengage_rpm = get_u16(page, 30);
    cal.vtec.min_map_kpa = get_u16(page, 32);
    cal.vtec.min_tps_pct = page[34];
    cal.vtec.min_vss_kmh = page[35];
    cal.vtec.min_ect_c = get_i16(page, 36);
    cal.vtec.require_vtp_pressure_switch = page[38] != 0;
    cal.vtec.vtp_confirm_timeout_ms = get_u16(page, 40);

    unpack_u16_slice(page, 44, &mut cal.rpm_bins);
    unpack_u16_slice(page, 76, &mut cal.map_kpa_bins);
    unpack_u16_table(page, 108, &mut cal.ve_low_cam_x10);
    unpack_u16_table(page, 620, &mut cal.ve_vtec_cam_x10);
    unpack_i16_table(page, 1132, &mut cal.spark_low_cam_deg_x10);
    unpack_i16_table(page, 1644, &mut cal.spark_vtec_cam_deg_x10);
    unpack_u16_table(page, 2156, &mut cal.target_lambda_x1000);

    unpack_u16_slice(page, 2668, &mut cal.vbatt_bins_x10);
    unpack_u16_slice(page, 2684, &mut cal.inj_deadtime_us);
    unpack_u16_slice(page, 2700, &mut cal.cop_dwell_us);
    unpack_i16_slice(page, 2716, &mut cal.ect_bins_c);
    unpack_u16_slice(page, 2732, &mut cal.warmup_enrichment_pct);
    unpack_u32_slice(page, 2748, &mut cal.cranking_pw_us);
    unpack_u16_slice(page, 2780, &mut cal.iacv_base_duty_per_mille);
}

fn parse_range_cmd(body: &[u8], default_count: usize) -> Option<(bool, usize, usize)> {
    match body.len() {
        1 => Some((true, 0, default_count)),
        5 => Some((true, get_u16(body, 1) as usize, get_u16(body, 3) as usize)),
        7 => Some((
            get_u16(body, 1) == 0,
            get_u16(body, 3) as usize,
            get_u16(body, 5) as usize,
        )),
        _ => None,
    }
}

fn parse_write_cmd(body: &[u8]) -> Option<(bool, usize, &[u8])> {
    if body.len() >= 7 {
        let count7 = get_u16(body, 5) as usize;
        if body.len() == 7 + count7 {
            let page_ok = get_u16(body, 1) == 0;
            let offset = get_u16(body, 3) as usize;
            return Some((page_ok, offset, &body[7..]));
        }
    }
    if body.len() >= 5 {
        let count5 = get_u16(body, 3) as usize;
        if body.len() == 5 + count5 {
            let offset = get_u16(body, 1) as usize;
            return Some((true, offset, &body[5..]));
        }
    }
    None
}

fn parse_page_only_ok(body: &[u8]) -> bool {
    match body.len() {
        1 => true,
        2 => body[1] == 0,
        3 => get_u16(body, 1) == 0,
        _ => false,
    }
}

/// Handles legacy single-byte unframed TunerStudio auto-detect probes (`'Q'`, `'S'`, `'F'`, `'C'`).
pub fn handle_ts_raw_probe(cmd: u8, out: &mut [u8]) -> Option<usize> {
    let payload = match cmd {
        b'Q' => TS_SIGNATURE,
        b'S' => TS_VERSION_INFO,
        b'F' => TS_PROTO_VERSION,
        b'C' => return None,
        _ => return None,
    };
    if out.len() < payload.len() {
        return None;
    }
    out[..payload.len()].copy_from_slice(payload);
    Some(payload.len())
}

/// Handles a framed TunerStudio command packet (`[len_be_u16 | cmd... | crc32_be_u32]`) with full
/// Page 1 read/write/CRC/burn support against `cal`.
pub fn handle_ts_packet_with_cal(
    packet: &[u8],
    snap: &TelemetrySnapshot,
    cal: &mut CalibrationBlob,
    scratch_page: &mut [u8; TS_PAGE_SIZE],
    out: &mut [u8],
) -> Option<TsAction> {
    if packet.len() < 7 {
        return None;
    }
    let body_len = u16::from_be_bytes([packet[0], packet[1]]) as usize;
    if body_len == 0 || packet.len() < 2 + body_len + 4 {
        return None;
    }
    let body = &packet[2..2 + body_len];
    let rx_crc = u32::from_be_bytes([
        packet[2 + body_len],
        packet[2 + body_len + 1],
        packet[2 + body_len + 2],
        packet[2 + body_len + 3],
    ]);
    if crc32(body) != rx_crc {
        return wrap_ts_envelope(TS_ERR_CRC_FAILURE, &[], out).map(|reply_len| TsAction {
            reply_len,
            cal_dirty: false,
            burn_requested: false,
        });
    }

    let mut cal_dirty = false;
    let mut burn_requested = false;
    let reply_len = match body[0] {
        b'Q' => wrap_ts_envelope(TS_FLAG_OK, TS_SIGNATURE, out)?,
        b'S' => wrap_ts_envelope(TS_FLAG_OK, TS_VERSION_INFO, out)?,
        b'F' => wrap_ts_envelope(TS_FLAG_OK, TS_PROTO_VERSION, out)?,
        b'C' => wrap_ts_envelope(TS_FLAG_OK, &[0x01], out)?,
        b'O' | b'A' => {
            let mut rt = [0u8; TS_OCH_SIZE];
            pack_realtime_channels(snap, &mut rt);
            let Some((page_ok, off, cnt)) = parse_range_cmd(body, TS_OCH_SIZE) else {
                return wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out).map(|reply_len| TsAction {
                    reply_len,
                    cal_dirty: false,
                    burn_requested: false,
                });
            };
            if !page_ok || off.checked_add(cnt).is_none_or(|end| end > TS_OCH_SIZE) {
                wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out)?
            } else {
                wrap_ts_envelope(TS_FLAG_OK, &rt[off..off + cnt], out)?
            }
        }
        b'R' => {
            let Some((page_ok, off, cnt)) = parse_range_cmd(body, TS_PAGE_SIZE) else {
                return wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out).map(|reply_len| TsAction {
                    reply_len,
                    cal_dirty: false,
                    burn_requested: false,
                });
            };
            if !page_ok || off.checked_add(cnt).is_none_or(|end| end > TS_PAGE_SIZE) {
                wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out)?
            } else {
                pack_page(cal, scratch_page);
                wrap_ts_envelope(TS_FLAG_OK, &scratch_page[off..off + cnt], out)?
            }
        }
        b'M' | b'W' => {
            let Some((page_ok, off, data)) = parse_write_cmd(body) else {
                return wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out).map(|reply_len| TsAction {
                    reply_len,
                    cal_dirty: false,
                    burn_requested: false,
                });
            };
            if !page_ok
                || off
                    .checked_add(data.len())
                    .is_none_or(|end| end > TS_PAGE_SIZE)
            {
                wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out)?
            } else {
                pack_page(cal, scratch_page);
                scratch_page[off..off + data.len()].copy_from_slice(data);
                unpack_page_into(scratch_page, cal);
                cal_dirty = true;
                wrap_ts_envelope(TS_FLAG_OK, &[], out)?
            }
        }
        b'k' => {
            let Some((page_ok, off, cnt)) = parse_range_cmd(body, TS_PAGE_SIZE) else {
                return wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out).map(|reply_len| TsAction {
                    reply_len,
                    cal_dirty: false,
                    burn_requested: false,
                });
            };
            if !page_ok || off.checked_add(cnt).is_none_or(|end| end > TS_PAGE_SIZE) {
                wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out)?
            } else {
                pack_page(cal, scratch_page);
                let page_crc = crc32(&scratch_page[off..off + cnt]).to_be_bytes();
                wrap_ts_envelope(TS_FLAG_OK, &page_crc, out)?
            }
        }
        b'B' => {
            if !parse_page_only_ok(body) {
                wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out)?
            } else {
                burn_requested = true;
                wrap_ts_envelope(TS_FLAG_BURN_OK, &[], out)?
            }
        }
        b'P' => {
            if parse_page_only_ok(body) {
                wrap_ts_envelope(TS_FLAG_OK, &[], out)?
            } else {
                wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], out)?
            }
        }
        _ => wrap_ts_envelope(TS_ERR_UNRECOGNIZED_CMD, &[], out)?,
    };

    Some(TsAction {
        reply_len,
        cal_dirty,
        burn_requested,
    })
}

/// Convenience read-only handler for status/telemetry/query packets without calibration edits.
pub fn handle_ts_packet(packet: &[u8], snap: &TelemetrySnapshot, out: &mut [u8]) -> Option<usize> {
    let mut dummy_cal = CalibrationBlob::default_honda_obd1();
    let mut scratch = [0u8; TS_PAGE_SIZE];
    handle_ts_packet_with_cal(packet, snap, &mut dummy_cal, &mut scratch, out).map(|a| a.reply_len)
}

#[cfg(test)]
mod tests {
    use super::{
        TS_ERR_CRC_FAILURE, TS_ERR_OUT_OF_RANGE, TS_ERR_UNRECOGNIZED_CMD, TS_FLAG_BURN_OK,
        TS_FLAG_OK, TS_OCH_SIZE, TS_PAGE_SIZE, TS_PROTO_VERSION, TS_SIGNATURE, TS_VERSION_INFO,
        handle_ts_packet, handle_ts_packet_with_cal, handle_ts_raw_probe, pack_page,
        unpack_page_into,
    };
    use ecu_types::{
        ActuatorOutputs, CalibrationBlob, FaultFlags, SensorSnapshot, SyncState, TelemetrySnapshot,
        VtecState, WidebandState, crc32,
    };

    fn sample_snap() -> TelemetrySnapshot {
        TelemetrySnapshot {
            seq: 1,
            uptime_ms: 12_000,
            sync_state: SyncState::FullSynced720,
            rpm: 900,
            crank_angle_deg_x10: 0,
            sync_loss_count: 0,
            sensors: SensorSnapshot::default(),
            vtec_state: VtecState::LowCam,
            wideband_state: WidebandState::ClosedLoopReady,
            lambda: 1.0,
            lsu_ri_ohms: 300.0,
            ve_pct: 60.0,
            target_lambda: 1.0,
            stft_pct: 0.0,
            actuators: ActuatorOutputs::default(),
            faults: FaultFlags::default(),
        }
    }

    fn build_ts_frame(body: &[u8], out: &mut [u8]) -> usize {
        let len = body.len();
        out[0..2].copy_from_slice(&(len as u16).to_be_bytes());
        out[2..2 + len].copy_from_slice(body);
        let c = crc32(body);
        out[2 + len..2 + len + 4].copy_from_slice(&c.to_be_bytes());
        2 + len + 4
    }

    fn verify_reply(buf: &[u8], len: usize) -> (u8, &[u8]) {
        assert!(len >= 7);
        let body_len = u16::from_be_bytes([buf[0], buf[1]]) as usize;
        assert_eq!(len, 2 + body_len + 4);
        let body = &buf[2..2 + body_len];
        let rx_crc = u32::from_be_bytes([
            buf[2 + body_len],
            buf[2 + body_len + 1],
            buf[2 + body_len + 2],
            buf[2 + body_len + 3],
        ]);
        assert_eq!(crc32(body), rx_crc);
        (body[0], &body[1..])
    }

    #[test]
    fn responds_to_tunerstudio_signature_query_with_valid_crc() {
        let mut req = [0u8; 16];
        let req_len = build_ts_frame(b"Q", &mut req);
        let snap = sample_snap();

        let mut resp = [0u8; 64];
        let n = handle_ts_packet(&req[..req_len], &snap, &mut resp)
            .expect("signature frame should decode");
        let (flag, payload) = verify_reply(&resp, n);
        assert_eq!(flag, TS_FLAG_OK);
        assert_eq!(payload, TS_SIGNATURE);
    }

    #[test]
    fn page_pack_unpack_roundtrips_calibration_losslessly() {
        let orig = CalibrationBlob::default_honda_obd1();
        let mut page = [0u8; TS_PAGE_SIZE];
        pack_page(&orig, &mut page);

        let mut restored = CalibrationBlob::default_honda_obd1();
        restored.displacement_cc = 0;
        restored.ve_low_cam_x10[5][5] = 0;
        restored.spark_vtec_cam_deg_x10[12][10] = 0;
        restored.cranking_pw_us[3] = 0;
        unpack_page_into(&page, &mut restored);

        assert_eq!(restored, orig);
    }

    #[test]
    fn handles_protocol_version_chunk_write_read_crc_and_burn() {
        let snap = sample_snap();
        let mut cal = CalibrationBlob::default_honda_obd1();
        let mut scratch = [0u8; TS_PAGE_SIZE];
        let mut req = [0u8; 256];
        let mut resp = [0u8; 512];

        // 1. Protocol version 'F'
        let n = build_ts_frame(b"F", &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("F command");
        let (flag, payload) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_FLAG_OK);
        assert_eq!(payload, TS_PROTO_VERSION);

        // 2. Write trigger_offset_deg_x10 (offset 12, S16 = 175 -> 17.5 deg) via M%2i%2o%2c%v
        let mut write_body = [0u8; 9];
        write_body[0] = b'M';
        write_body[1..3].copy_from_slice(&0u16.to_le_bytes());
        write_body[3..5].copy_from_slice(&12u16.to_le_bytes());
        write_body[5..7].copy_from_slice(&2u16.to_le_bytes());
        write_body[7..9].copy_from_slice(&175i16.to_le_bytes());
        let n = build_ts_frame(&write_body, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("M command");
        let (flag, _) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_FLAG_OK);
        assert!(act.cal_dirty);
        assert!(!act.burn_requested);
        assert_eq!(cal.trigger_offset_deg_x10, 175);

        // 3. Read back via R%2i%2o%2c
        let mut read_body = [0u8; 7];
        read_body[0] = b'R';
        read_body[1..3].copy_from_slice(&0u16.to_le_bytes());
        read_body[3..5].copy_from_slice(&12u16.to_le_bytes());
        read_body[5..7].copy_from_slice(&2u16.to_le_bytes());
        let n = build_ts_frame(&read_body, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("R command");
        let (flag, payload) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_FLAG_OK);
        assert_eq!(payload, &175i16.to_le_bytes());

        // 4. CRC32 check via k%2i%2o%2c
        let mut crc_body = [0u8; 7];
        crc_body[0] = b'k';
        crc_body[1..3].copy_from_slice(&0u16.to_le_bytes());
        crc_body[3..5].copy_from_slice(&0u16.to_le_bytes());
        crc_body[5..7].copy_from_slice(&(TS_PAGE_SIZE as u16).to_le_bytes());
        let n = build_ts_frame(&crc_body, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("k command");
        let (flag, payload) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_FLAG_OK);
        let mut verify_page = [0u8; TS_PAGE_SIZE];
        pack_page(&cal, &mut verify_page);
        assert_eq!(payload, &crc32(&verify_page).to_be_bytes());

        // 5. Burn via B%2i
        let mut burn_body = [0u8; 3];
        burn_body[0] = b'B';
        burn_body[1..3].copy_from_slice(&0u16.to_le_bytes());
        let n = build_ts_frame(&burn_body, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("B command");
        let (flag, _) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_FLAG_BURN_OK);
        assert!(act.burn_requested);

        // 6. Sliced OutputChannels O%2o%2c
        let mut och_body = [0u8; 5];
        och_body[0] = b'O';
        och_body[1..3].copy_from_slice(&2u16.to_le_bytes());
        och_body[3..5].copy_from_slice(&2u16.to_le_bytes());
        let n = build_ts_frame(&och_body, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("O slice command");
        let (flag, payload) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_FLAG_OK);
        assert_eq!(u16::from_le_bytes([payload[0], payload[1]]), 900);
        assert_eq!(TS_OCH_SIZE, 24);
    }

    #[test]
    fn handles_raw_probes_bad_crc_out_of_range_and_unrecognized_commands() {
        let mut raw_out = [0u8; 64];
        let q_len = handle_ts_raw_probe(b'Q', &mut raw_out).expect("raw Q");
        assert_eq!(&raw_out[..q_len], TS_SIGNATURE);
        let s_len = handle_ts_raw_probe(b'S', &mut raw_out).expect("raw S");
        assert_eq!(&raw_out[..s_len], TS_VERSION_INFO);
        let f_len = handle_ts_raw_probe(b'F', &mut raw_out).expect("raw F");
        assert_eq!(&raw_out[..f_len], TS_PROTO_VERSION);
        assert!(handle_ts_raw_probe(b'C', &mut raw_out).is_none());
        assert!(handle_ts_raw_probe(b'?', &mut raw_out).is_none());

        let snap = sample_snap();
        let mut cal = CalibrationBlob::default_honda_obd1();
        let mut scratch = [0u8; TS_PAGE_SIZE];
        let mut req = [0u8; 64];
        let mut resp = [0u8; 128];

        // 1. Bad CRC returns TS_ERR_CRC_FAILURE without mutating calibration
        let n = build_ts_frame(b"Q", &mut req);
        req[n - 1] ^= 0xff;
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("bad CRC reply");
        let (flag, _) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_ERR_CRC_FAILURE);
        assert!(!act.cal_dirty);

        // 2. Out-of-bounds Page 1 read (`offset = 2800, count = 32 > 2816`) -> TS_ERR_OUT_OF_RANGE
        let mut oob_read = [0u8; 7];
        oob_read[0] = b'R';
        oob_read[3..5].copy_from_slice(&2800u16.to_le_bytes());
        oob_read[5..7].copy_from_slice(&32u16.to_le_bytes());
        let n = build_ts_frame(&oob_read, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("OOB R reply");
        let (flag, _) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_ERR_OUT_OF_RANGE);

        // 3. Invalid page index (`page = 1`) on Burn returns TS_ERR_OUT_OF_RANGE
        let bad_burn = [b'B', 1, 0];
        let n = build_ts_frame(&bad_burn, &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("bad page B reply");
        let (flag, _) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_ERR_OUT_OF_RANGE);
        assert!(!act.burn_requested);

        // 4. Unrecognized command ('Z') returns TS_ERR_UNRECOGNIZED_CMD
        let n = build_ts_frame(b"Z", &mut req);
        let act = handle_ts_packet_with_cal(&req[..n], &snap, &mut cal, &mut scratch, &mut resp)
            .expect("unknown command reply");
        let (flag, _) = verify_reply(&resp, act.reply_len);
        assert_eq!(flag, TS_ERR_UNRECOGNIZED_CMD);
    }
}
