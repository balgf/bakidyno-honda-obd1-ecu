//! Pure `no_std` high-rate (`500 Hz`) standalone datalog record encoding, ring buffer,
//! and RTC FAT32 timestamp conversion (`BakiDyno Log v1` `.BDL` binary format).
//!
//! Record layout (`64` bytes, little-endian, IEEE CRC-32 footer):
//! - Exact `8` records per `512`-byte SDMMC sector (`32,000 B/s` @ `500 Hz`)
//! - Record `0` of every `.BDL` file is a `64`-byte [`DatalogFileHeader`] (`"BDL1"`)
//! - Subsequent records are `64`-byte [`DatalogRecord`] samples capturing engine state,
//!   sensors, actuators, LSU 4.9 diagnostics, switches, aux outputs, and `PC13` mark events.

use ecu_types::{TelemetrySnapshot, crc32};

use crate::aux_io::AuxIoTelemetry;

/// Canonical magic bytes identifying a `BakiDyno Log v1` binary file.
pub const BDL_FILE_MAGIC: [u8; 4] = *b"BDL1";
/// Binary datalog schema version.
pub const BDL_FILE_VERSION: u16 = 1;
/// Fixed byte size of both [`DatalogFileHeader`] and [`DatalogRecord`].
pub const LOG_RECORD_BYTES: usize = 64;
/// Number of `64`-byte records stored in one `512`-byte SDMMC sector.
pub const RECORDS_PER_SECTOR: usize = 512 / LOG_RECORD_BYTES;
/// Nominal standalone SD logger cadence (`Hz`).
pub const LOG_SAMPLE_RATE_HZ: u16 = 500;

/// Flag bit inside [`DatalogRecord::flags`] indicating a newly latched `PC13` mark edge.
pub const LOG_FLAG_MARK_EDGE: u8 = 1 << 0;
/// Flag bit inside [`DatalogRecord::flags`] indicating the manual `PC13` mark button is held.
pub const LOG_FLAG_MARK_HELD: u8 = 1 << 1;

/// Calendar date and wall-clock time sourced from STM32H743 `LSE` `RTC` (or Unix epoch).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RtcTimestamp {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub subsec_ms: u16,
}

impl RtcTimestamp {
    /// Deterministic fallback timestamp (`2026-01-01 00:00:00.000`) when battery-backed RTC
    /// has not yet been synchronized by host USB RPC.
    #[inline]
    pub const fn default_fallback() -> Self {
        Self {
            year: 2026,
            month: 1,
            day: 1,
            hour: 0,
            minute: 0,
            second: 0,
            subsec_ms: 0,
        }
    }

    /// Converts a non-negative Unix epoch timestamp (UTC seconds) into calendar fields.
    ///
    /// Uses Hinnant's branchless Gregorian civil-from-days conversion cleanly in `no_std`.
    pub const fn from_unix_secs(unix_secs: u64) -> Self {
        let secs_of_day = (unix_secs % 86_400) as u32;
        let hour = (secs_of_day / 3_600) as u8;
        let minute = ((secs_of_day % 3_600) / 60) as u8;
        let second = (secs_of_day % 60) as u8;

        let days = (unix_secs / 86_400) as i64 + 719_468;
        let era = if days >= 0 {
            days / 146_097
        } else {
            (days - 146_096) / 146_097
        };
        let doe = (days - era * 146_097) as u32;
        let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
        let mut y = yoe as i64 + era * 400;
        let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        let mp = (5 * doy + 2) / 153;
        let d = doy - (153 * mp + 2) / 5 + 1;
        let m = if mp < 10 { mp + 3 } else { mp - 9 };
        if m <= 2 {
            y += 1;
        }

        Self {
            year: if y < 1980 {
                1980
            } else if y > 2107 {
                2107
            } else {
                y as u16
            },
            month: m as u8,
            day: d as u8,
            hour,
            minute,
            second,
            subsec_ms: 0,
        }
    }

    /// Encodes Microsoft FAT32 MS-DOS date word (`bits 15..9`: year-1980, `8..5`: month, `4..0`: day).
    #[inline]
    pub const fn to_fat_date(&self) -> u16 {
        let yr = if self.year < 1980 {
            0
        } else if self.year > 2107 {
            127
        } else {
            self.year - 1980
        };
        let mo = if self.month < 1 {
            1
        } else if self.month > 12 {
            12
        } else {
            self.month as u16
        };
        let dy = if self.day < 1 {
            1
        } else if self.day > 31 {
            31
        } else {
            self.day as u16
        };
        ((yr & 0x7f) << 9) | ((mo & 0x0f) << 5) | (dy & 0x1f)
    }

    /// Encodes Microsoft FAT32 MS-DOS time word (`bits 15..11`: hour, `10..5`: min, `4..0`: sec/2).
    #[inline]
    pub const fn to_fat_time(&self) -> u16 {
        let hr = if self.hour > 23 { 23 } else { self.hour as u16 };
        let mn = if self.minute > 59 {
            59
        } else {
            self.minute as u16
        };
        let s2 = if self.second > 59 {
            29
        } else {
            (self.second / 2) as u16
        };
        ((hr & 0x1f) << 11) | ((mn & 0x3f) << 5) | (s2 & 0x1f)
    }

    /// Encodes FAT32 `CrtTimeTenth` (`0..=199` representing `10 ms` units within a 2-second tick).
    #[inline]
    pub const fn to_fat_time_tenth(&self) -> u8 {
        let odd_sec_tenths = if self.second <= 59 && (self.second % 2) == 1 {
            100
        } else {
            0
        };
        let sub_tenths = if self.subsec_ms > 999 {
            99
        } else {
            (self.subsec_ms / 10) as u8
        };
        odd_sec_tenths + sub_tenths
    }

    /// Packs `yyyy-mm-dd hh:mm` compactly for deterministic 32-bit volume IDs.
    #[inline]
    pub const fn volume_serial(&self) -> u32 {
        let date = self.to_fat_date() as u32;
        let time = self.to_fat_time() as u32;
        ((date & 0xffff) << 16) ^ time.wrapping_mul(0x9e37_79b9) ^ 0x4244_594e
    }
}

/// `64`-byte header written at offset `0` of every `LOGnnnnn.BDL` binary log file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DatalogFileHeader {
    pub version: u16,
    pub record_size: u16,
    pub sample_rate_hz: u16,
    pub created: RtcTimestamp,
    pub cal_crc32: u32,
}

impl DatalogFileHeader {
    /// Creates a new v1 header stamped with `500 Hz` sample cadence and calibration CRC.
    pub const fn new(created: RtcTimestamp, cal_crc32: u32) -> Self {
        Self {
            version: BDL_FILE_VERSION,
            record_size: LOG_RECORD_BYTES as u16,
            sample_rate_hz: LOG_SAMPLE_RATE_HZ,
            created,
            cal_crc32,
        }
    }

    /// Serializes the file header into an exact `64`-byte little-endian block with CRC32 footer.
    pub fn to_bytes(&self) -> [u8; LOG_RECORD_BYTES] {
        let mut buf = [0u8; LOG_RECORD_BYTES];
        buf[0..4].copy_from_slice(&BDL_FILE_MAGIC);
        buf[4..6].copy_from_slice(&self.version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.record_size.to_le_bytes());
        buf[8..10].copy_from_slice(&self.sample_rate_hz.to_le_bytes());
        buf[10..12].copy_from_slice(&self.created.year.to_le_bytes());
        buf[12] = self.created.month;
        buf[13] = self.created.day;
        buf[14] = self.created.hour;
        buf[15] = self.created.minute;
        buf[16] = self.created.second;
        buf[20..24].copy_from_slice(&self.cal_crc32.to_le_bytes());
        let sum = crc32(&buf[0..60]);
        buf[60..64].copy_from_slice(&sum.to_le_bytes());
        buf
    }

    /// Decodes and CRC32-verifies a `64`-byte `BDL1` file header.
    pub fn from_bytes(bytes: &[u8; LOG_RECORD_BYTES]) -> Option<Self> {
        if bytes[0..4] != BDL_FILE_MAGIC {
            return None;
        }
        let expected_crc = u32::from_le_bytes([bytes[60], bytes[61], bytes[62], bytes[63]]);
        if crc32(&bytes[0..60]) != expected_crc {
            return None;
        }
        let version = u16::from_le_bytes([bytes[4], bytes[5]]);
        let record_size = u16::from_le_bytes([bytes[6], bytes[7]]);
        if version != BDL_FILE_VERSION || record_size as usize != LOG_RECORD_BYTES {
            return None;
        }
        Some(Self {
            version,
            record_size,
            sample_rate_hz: u16::from_le_bytes([bytes[8], bytes[9]]),
            created: RtcTimestamp {
                year: u16::from_le_bytes([bytes[10], bytes[11]]),
                month: bytes[12],
                day: bytes[13],
                hour: bytes[14],
                minute: bytes[15],
                second: bytes[16],
                subsec_ms: 0,
            },
            cal_crc32: u32::from_le_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]),
        })
    }
}

/// Fixed `64`-byte telemetry sample packed at `500 Hz` without dynamic allocation or float parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DatalogRecord {
    pub uptime_ms: u32,
    pub faults: u32,
    pub inj_pw1_us: u32,
    pub mark_seq: u32,
    pub rpm: u16,
    pub crank_angle_deg_x10: u16,
    pub map_kpa_x10: u16,
    pub tps_pct_x10: i16,
    pub ect_c_x10: i16,
    pub iat_c_x10: i16,
    pub vbatt_mv: u16,
    pub vcc_5v_mv: u16,
    pub vss_kmh_x10: u16,
    pub eld_amps_x10: u16,
    pub lambda_x1000: u16,
    pub target_lambda_x1000: u16,
    pub lsu_ri_ohms_x10: u16,
    pub stft_pct_x10: i16,
    pub ve_pct_x10: u16,
    pub spark_adv_deg_x10: i16,
    pub cop_dwell_us: u16,
    pub iacv_duty_per_mille: u16,
    pub lsu_heater_duty_per_mille: u16,
    pub sync_state: u8,
    pub vtec_state: u8,
    pub wideband_state: u8,
    pub switch_bits: u8,
    pub aux_outputs_bits: u8,
    pub flags: u8,
}

#[inline]
fn f32_to_u16_scaled(value: f32, scale: f32) -> u16 {
    if !value.is_finite() || value <= 0.0 {
        return 0;
    }
    let scaled = value * scale + 0.5;
    if scaled >= u16::MAX as f32 {
        u16::MAX
    } else {
        scaled as u16
    }
}

#[inline]
fn f32_to_i16_scaled(value: f32, scale: f32) -> i16 {
    if !value.is_finite() {
        return 0;
    }
    let scaled = value * scale;
    let rounded = if scaled >= 0.0 {
        scaled + 0.5
    } else {
        scaled - 0.5
    };
    if rounded >= i16::MAX as f32 {
        i16::MAX
    } else if rounded <= i16::MIN as f32 {
        i16::MIN
    } else {
        rounded as i16
    }
}

impl DatalogRecord {
    /// Packs a [`TelemetrySnapshot`] + auxiliary IO + manual `PC13` mark state into `64` bytes.
    pub fn from_telemetry(
        snap: &TelemetrySnapshot,
        aux: &AuxIoTelemetry,
        mark_seq: u32,
        mark_edge: bool,
        mark_held: bool,
    ) -> Self {
        let sw = snap.sensors.switches;
        let switch_bits = u8::from(sw.sts_cranking)
            | (u8::from(sw.vtp_pressurized) << 1)
            | (u8::from(sw.scs_service_jumper) << 2)
            | (u8::from(sw.acs_ac_request) << 3)
            | (u8::from(sw.psp_steering_load) << 4)
            | (u8::from(sw.bksw_brake) << 5)
            | (u8::from(sw.clutch_pressed) << 6)
            | (u8::from(mark_held) << 7);

        let flags = (if mark_edge { LOG_FLAG_MARK_EDGE } else { 0 })
            | (if mark_held { LOG_FLAG_MARK_HELD } else { 0 });

        Self {
            uptime_ms: snap.uptime_ms,
            faults: snap.faults.0,
            inj_pw1_us: snap.actuators.inj_pw_us[0],
            mark_seq,
            rpm: snap.rpm,
            crank_angle_deg_x10: snap.crank_angle_deg_x10,
            map_kpa_x10: f32_to_u16_scaled(snap.sensors.map_kpa, 10.0),
            tps_pct_x10: f32_to_i16_scaled(snap.sensors.tps_pct, 10.0),
            ect_c_x10: f32_to_i16_scaled(snap.sensors.ect_c, 10.0),
            iat_c_x10: f32_to_i16_scaled(snap.sensors.iat_c, 10.0),
            vbatt_mv: f32_to_u16_scaled(snap.sensors.vbatt_v, 1000.0),
            vcc_5v_mv: f32_to_u16_scaled(snap.sensors.vcc_5v_v, 1000.0),
            vss_kmh_x10: f32_to_u16_scaled(snap.sensors.vss_kmh, 10.0),
            eld_amps_x10: f32_to_u16_scaled(snap.sensors.eld_amps, 10.0),
            lambda_x1000: f32_to_u16_scaled(snap.lambda, 1000.0),
            target_lambda_x1000: f32_to_u16_scaled(snap.target_lambda, 1000.0),
            lsu_ri_ohms_x10: f32_to_u16_scaled(snap.lsu_ri_ohms, 10.0),
            stft_pct_x10: f32_to_i16_scaled(snap.stft_pct, 10.0),
            ve_pct_x10: f32_to_u16_scaled(snap.ve_pct, 10.0),
            spark_adv_deg_x10: snap.actuators.spark_advance_deg_x10,
            cop_dwell_us: snap.actuators.cop_dwell_us.min(u16::MAX as u32) as u16,
            iacv_duty_per_mille: snap.actuators.iacv_duty_per_mille,
            lsu_heater_duty_per_mille: snap.actuators.lsu_heater_duty_per_mille,
            sync_state: snap.sync_state as u8,
            vtec_state: snap.vtec_state as u8,
            wideband_state: snap.wideband_state as u8,
            switch_bits,
            aux_outputs_bits: aux.outputs.bitmask(),
            flags,
        }
    }

    /// Serializes this record into an exact `64`-byte little-endian payload with CRC32 footer.
    pub fn to_bytes(&self) -> [u8; LOG_RECORD_BYTES] {
        let mut b = [0u8; LOG_RECORD_BYTES];
        b[0..4].copy_from_slice(&self.uptime_ms.to_le_bytes());
        b[4..8].copy_from_slice(&self.faults.to_le_bytes());
        b[8..12].copy_from_slice(&self.inj_pw1_us.to_le_bytes());
        b[12..16].copy_from_slice(&self.mark_seq.to_le_bytes());
        b[16..18].copy_from_slice(&self.rpm.to_le_bytes());
        b[18..20].copy_from_slice(&self.crank_angle_deg_x10.to_le_bytes());
        b[20..22].copy_from_slice(&self.map_kpa_x10.to_le_bytes());
        b[22..24].copy_from_slice(&self.tps_pct_x10.to_le_bytes());
        b[24..26].copy_from_slice(&self.ect_c_x10.to_le_bytes());
        b[26..28].copy_from_slice(&self.iat_c_x10.to_le_bytes());
        b[28..30].copy_from_slice(&self.vbatt_mv.to_le_bytes());
        b[30..32].copy_from_slice(&self.vcc_5v_mv.to_le_bytes());
        b[32..34].copy_from_slice(&self.vss_kmh_x10.to_le_bytes());
        b[34..36].copy_from_slice(&self.eld_amps_x10.to_le_bytes());
        b[36..38].copy_from_slice(&self.lambda_x1000.to_le_bytes());
        b[38..40].copy_from_slice(&self.target_lambda_x1000.to_le_bytes());
        b[40..42].copy_from_slice(&self.lsu_ri_ohms_x10.to_le_bytes());
        b[42..44].copy_from_slice(&self.stft_pct_x10.to_le_bytes());
        b[44..46].copy_from_slice(&self.ve_pct_x10.to_le_bytes());
        b[46..48].copy_from_slice(&self.spark_adv_deg_x10.to_le_bytes());
        b[48..50].copy_from_slice(&self.cop_dwell_us.to_le_bytes());
        b[50..52].copy_from_slice(&self.iacv_duty_per_mille.to_le_bytes());
        b[52..54].copy_from_slice(&self.lsu_heater_duty_per_mille.to_le_bytes());
        b[54] = self.sync_state;
        b[55] = self.vtec_state;
        b[56] = self.wideband_state;
        b[57] = self.switch_bits;
        b[58] = self.aux_outputs_bits;
        b[59] = self.flags;
        let sum = crc32(&b[0..60]);
        b[60..64].copy_from_slice(&sum.to_le_bytes());
        b
    }

    /// Decodes and CRC32-verifies one `64`-byte record from disk.
    pub fn from_bytes(b: &[u8; LOG_RECORD_BYTES]) -> Option<Self> {
        let expected_crc = u32::from_le_bytes([b[60], b[61], b[62], b[63]]);
        if crc32(&b[0..60]) != expected_crc {
            return None;
        }
        Some(Self {
            uptime_ms: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            faults: u32::from_le_bytes([b[4], b[5], b[6], b[7]]),
            inj_pw1_us: u32::from_le_bytes([b[8], b[9], b[10], b[11]]),
            mark_seq: u32::from_le_bytes([b[12], b[13], b[14], b[15]]),
            rpm: u16::from_le_bytes([b[16], b[17]]),
            crank_angle_deg_x10: u16::from_le_bytes([b[18], b[19]]),
            map_kpa_x10: u16::from_le_bytes([b[20], b[21]]),
            tps_pct_x10: i16::from_le_bytes([b[22], b[23]]),
            ect_c_x10: i16::from_le_bytes([b[24], b[25]]),
            iat_c_x10: i16::from_le_bytes([b[26], b[27]]),
            vbatt_mv: u16::from_le_bytes([b[28], b[29]]),
            vcc_5v_mv: u16::from_le_bytes([b[30], b[31]]),
            vss_kmh_x10: u16::from_le_bytes([b[32], b[33]]),
            eld_amps_x10: u16::from_le_bytes([b[34], b[35]]),
            lambda_x1000: u16::from_le_bytes([b[36], b[37]]),
            target_lambda_x1000: u16::from_le_bytes([b[38], b[39]]),
            lsu_ri_ohms_x10: u16::from_le_bytes([b[40], b[41]]),
            stft_pct_x10: i16::from_le_bytes([b[42], b[43]]),
            ve_pct_x10: u16::from_le_bytes([b[44], b[45]]),
            spark_adv_deg_x10: i16::from_le_bytes([b[46], b[47]]),
            cop_dwell_us: u16::from_le_bytes([b[48], b[49]]),
            iacv_duty_per_mille: u16::from_le_bytes([b[50], b[51]]),
            lsu_heater_duty_per_mille: u16::from_le_bytes([b[52], b[53]]),
            sync_state: b[54],
            vtec_state: b[55],
            wideband_state: b[56],
            switch_bits: b[57],
            aux_outputs_bits: b[58],
            flags: b[59],
        })
    }
}

/// Fixed-capacity static RAM ring buffer bridging the `500 Hz` producer to async multi-block SD writes.
///
/// Capacity `CAP` should be a multiple of [`RECORDS_PER_SECTOR`] (e.g., `512` = `32 KB` = `1.024 s`
/// headroom at `500 Hz` so SD flash erase stalls never drop telemetry samples).
pub struct DatalogRing<const CAP: usize> {
    slots: [[u8; LOG_RECORD_BYTES]; CAP],
    head: usize,
    tail: usize,
    len: usize,
    total_pushed: u32,
    dropped: u32,
}

impl<const CAP: usize> Default for DatalogRing<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> DatalogRing<CAP> {
    /// Creates an empty statically initializable `64`-byte record ring buffer.
    pub const fn new() -> Self {
        Self {
            slots: [[0u8; LOG_RECORD_BYTES]; CAP],
            head: 0,
            tail: 0,
            len: 0,
            total_pushed: 0,
            dropped: 0,
        }
    }

    /// Number of queued `64`-byte records awaiting SDMMC flush.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` when zero records are buffered.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Cumulative records dropped because the ring filled while SD card was busy or missing.
    #[inline]
    pub const fn dropped_count(&self) -> u32 {
        self.dropped
    }

    /// Cumulative records accepted into the ring buffer.
    #[inline]
    pub const fn total_pushed(&self) -> u32 {
        self.total_pushed
    }

    /// Drops all queued records (used when starting a fresh log file after format/hot-plug).
    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.len = 0;
    }

    /// Pushes a single `64`-byte record or header block; never blocks or overwrites unread entries.
    pub fn push_block(&mut self, block: [u8; LOG_RECORD_BYTES]) -> bool {
        if CAP == 0 || self.len >= CAP {
            self.dropped = self.dropped.saturating_add(1);
            return false;
        }
        self.slots[self.head] = block;
        self.head = (self.head + 1) % CAP;
        self.len += 1;
        self.total_pushed = self.total_pushed.saturating_add(1);
        true
    }

    /// Pops up to `out_sectors.len()` complete `512`-byte sectors (`8` records each).
    ///
    /// Leaves any partial `< 8`-record tail queued so data sectors stay 100% packed and aligned.
    pub fn pop_sectors(&mut self, out_sectors: &mut [[u8; 512]]) -> usize {
        let ready_sectors = (self.len / RECORDS_PER_SECTOR).min(out_sectors.len());
        for sector in out_sectors.iter_mut().take(ready_sectors) {
            for slot_idx in 0..RECORDS_PER_SECTOR {
                let off = slot_idx * LOG_RECORD_BYTES;
                sector[off..off + LOG_RECORD_BYTES].copy_from_slice(&self.slots[self.tail]);
                self.tail = (self.tail + 1) % CAP;
                self.len -= 1;
            }
        }
        ready_sectors
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{
        ActuatorOutputs, FaultFlags, SensorSnapshot, SwitchState, SyncState, TelemetrySnapshot,
        VtecState, WidebandState,
    };

    use super::{
        DatalogFileHeader, DatalogRecord, DatalogRing, LOG_FLAG_MARK_EDGE, LOG_FLAG_MARK_HELD,
        LOG_RECORD_BYTES, RECORDS_PER_SECTOR, RtcTimestamp,
    };
    use crate::aux_io::{AltfState, AuxIoTelemetry, AuxOutputs, FlexFuelState};

    #[test]
    fn fat32_timestamp_and_unix_conversion_match_msdos_encoding() {
        let ts = RtcTimestamp {
            year: 2026,
            month: 11,
            day: 14,
            hour: 22,
            minute: 55,
            second: 35,
            subsec_ms: 420,
        };
        // Date: (2026-1980)=46 -> 0x2e << 9 | 11 << 5 | 14 = 0x5d6e
        assert_eq!(ts.to_fat_date(), 0x5d6e);
        // Time: 22 << 11 | 55 << 5 | (35/2=17) = 0xb6f1
        assert_eq!(ts.to_fat_time(), 0xb6f1);
        // Tenths: odd second (100) + 420/10 (42) = 142
        assert_eq!(ts.to_fat_time_tenth(), 142);

        // 2026-11-14T14:55:35Z -> unix seconds 1_794_668_135
        let from_unix = RtcTimestamp::from_unix_secs(1_794_668_135);
        assert_eq!(from_unix.year, 2026);
        assert_eq!(from_unix.month, 11);
        assert_eq!(from_unix.day, 14);
        assert_eq!(from_unix.hour, 14);
        assert_eq!(from_unix.minute, 55);
        assert_eq!(from_unix.second, 35);
    }

    #[test]
    fn bdl1_header_and_record_roundtrip_with_crc32_detection() {
        let hdr = DatalogFileHeader::new(RtcTimestamp::default_fallback(), 0x1234_5678);
        let mut hdr_bytes = hdr.to_bytes();
        let decoded_hdr = DatalogFileHeader::from_bytes(&hdr_bytes).expect("valid BDL1 header");
        assert_eq!(decoded_hdr, hdr);
        hdr_bytes[5] ^= 0x01;
        assert!(DatalogFileHeader::from_bytes(&hdr_bytes).is_none());

        let snap = TelemetrySnapshot {
            seq: 12,
            uptime_ms: 98_765,
            sync_state: SyncState::FullSynced720,
            rpm: 6_250,
            crank_angle_deg_x10: 1_800,
            sync_loss_count: 1,
            sensors: SensorSnapshot {
                map_kpa: 99.4,
                tps_pct: 84.2,
                ect_c: 88.5,
                iat_c: 31.0,
                vbatt_v: 13.85,
                vcc_5v_v: 5.01,
                eld_amps: 18.2,
                vss_kmh: 72.5,
                switches: SwitchState {
                    sts_cranking: false,
                    vtp_pressurized: true,
                    scs_service_jumper: false,
                    acs_ac_request: false,
                    psp_steering_load: false,
                    bksw_brake: false,
                    clutch_pressed: true,
                },
            },
            vtec_state: VtecState::HighCamActive,
            wideband_state: WidebandState::ClosedLoopReady,
            lambda: 0.855,
            lsu_ri_ohms: 300.4,
            ve_pct: 102.3,
            target_lambda: 0.850,
            stft_pct: -1.4,
            actuators: ActuatorOutputs {
                inj_pw_us: [11_250; 4],
                spark_advance_deg_x10: 265,
                cop_dwell_us: 2_850,
                vts_high_side_on: true,
                fuel_pump_relay_on: true,
                fan_relay_on: false,
                ac_clutch_relay_on: false,
                mil_lamp_on: false,
                iacv_duty_per_mille: 360,
                lsu_heater_duty_per_mille: 420,
                lsu_ip_dac_code: 2_100,
            },
            faults: FaultFlags(FaultFlags::WIDEBAND_FAULT),
        };
        let aux = AuxIoTelemetry {
            outputs: AuxOutputs {
                altc_on: true,
                iab_on: true,
                evap_on: false,
                evap_duty_per_mille: 0,
                pe0_on: false,
                pe1_on: false,
            },
            flex: FlexFuelState::new(),
            altf: AltfState::new(),
        };

        let rec = DatalogRecord::from_telemetry(&snap, &aux, 7, true, true);
        let mut raw = rec.to_bytes();
        let dec = DatalogRecord::from_bytes(&raw).expect("valid BDL1 record");
        assert_eq!(dec, rec);
        assert_eq!(dec.mark_seq, 7);
        assert_ne!(dec.flags & LOG_FLAG_MARK_EDGE, 0);
        assert_ne!(dec.flags & LOG_FLAG_MARK_HELD, 0);

        raw[16] ^= 0x80;
        assert!(DatalogRecord::from_bytes(&raw).is_none());
    }

    #[test]
    fn ring_packs_exact_sectors_and_counts_drops_without_overwriting() {
        let mut ring = DatalogRing::<16>::new();
        for idx in 0..16u8 {
            assert!(ring.push_block([idx; LOG_RECORD_BYTES]));
        }
        assert_eq!(ring.len(), 16);
        assert!(!ring.push_block([0xff; LOG_RECORD_BYTES]));
        assert_eq!(ring.dropped_count(), 1);

        let mut sectors = [[0u8; 512]; 4];
        let popped = ring.pop_sectors(&mut sectors);
        assert_eq!(popped, 2);
        assert_eq!(ring.len(), 0);
        for slot in 0..RECORDS_PER_SECTOR {
            assert_eq!(
                sectors[0][slot * LOG_RECORD_BYTES],
                slot as u8,
                "first sector record ordering"
            );
            assert_eq!(
                sectors[1][slot * LOG_RECORD_BYTES],
                (slot + RECORDS_PER_SECTOR) as u8,
                "second sector record ordering"
            );
        }

        // Partial tail (< 8 records) stays buffered until a full sector completes.
        for idx in 20..25u8 {
            assert!(ring.push_block([idx; LOG_RECORD_BYTES]));
        }
        assert_eq!(ring.pop_sectors(&mut sectors), 0);
        assert_eq!(ring.len(), 5);
    }
}
