//! Honda OBD1/OBD2 `D7 DLC` `9600 bps 8N1` 1-wire diagnostic serial codec (`PA9` TX / `PA10` RX).
//!
//! Implements the classic Honda two's-complement checksummed query/response framing used on `D7`:
//! - `0x20 0x05 <offset> <count> <chk>`: Read contiguous slice of live 32-byte telemetry table
//! - `0xfe 0x04 <subcmd> <chk>`: Identify ECU hardware (`"H743"`)
//! - `0x60 0x04 <subcmd> <chk>`: Clear latched faults / ping acknowledge

use ecu_types::{SyncState, TelemetrySnapshot, VtecState, WidebandState};

use crate::aux_io::AuxIoTelemetry;

/// Baud rate specified by the Honda `D7 DLC` diagnostic line.
pub const DLC_BAUDRATE_BPS: u32 = 9_600;
/// Number of bytes exposed in the live Honda DLC telemetry register table (`0x00..=0x1f`).
pub const DLC_TABLE_LEN: usize = 32;
/// Maximum response frame length (`[0x00, len_byte, offset, 32 bytes data, chk]` = 36 bytes).
pub const DLC_MAX_RESPONSE_BYTES: usize = 40;

const CMD_READ_TABLE: u8 = 0x20;
const CMD_CLEAR_FAULTS: u8 = 0x60;
const CMD_IDENTIFY: u8 = 0xfe;

/// Decoded inbound Honda `D7 DLC` request command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DlcCommand {
    /// Read `count` bytes starting at `offset` in the live `32`-byte telemetry table.
    ReadTable { offset: u8, count: u8 },
    /// Acknowledge fault-clear request (`0x60`).
    ClearFaults,
    /// Query ECU identification string (`0xfe`).
    Identify,
}

/// Fixed-capacity outbound `DLC` response frame ready to write to `USART1`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DlcResponse {
    buf: [u8; DLC_MAX_RESPONSE_BYTES],
    len: usize,
}

impl DlcResponse {
    /// Valid response byte slice including Honda trailer checksum.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// Computes Honda two's-complement checksum such that `sum(frame) & 0xff == 0`.
#[inline]
pub fn honda_checksum(payload: &[u8]) -> u8 {
    let sum = payload.iter().copied().fold(0u8, u8::wrapping_add);
    0u8.wrapping_sub(sum)
}

/// Validates that a complete Honda DLC frame sums to `0 (mod 256)`.
#[inline]
pub fn verify_honda_checksum(frame: &[u8]) -> bool {
    !frame.is_empty() && frame.iter().copied().fold(0u8, u8::wrapping_add) == 0
}

/// Packs live ECU telemetry into the `32`-byte `D7 DLC` diagnostic register table (`0x00..=0x1f`).
pub fn build_dlc_table(snap: &TelemetrySnapshot, aux: &AuxIoTelemetry) -> [u8; DLC_TABLE_LEN] {
    let mut table = [0u8; DLC_TABLE_LEN];
    let rpm_be = snap.rpm.to_be_bytes();
    table[0x00] = rpm_be[0];
    table[0x01] = rpm_be[1];
    table[0x02] = snap.sensors.vss_kmh.clamp(0.0, 255.0) as u8;
    table[0x03] = (snap.sensors.map_kpa.clamp(0.0, 255.0)) as u8;
    table[0x04] = (snap.sensors.ect_c + 40.0).clamp(0.0, 255.0) as u8;
    table[0x05] = (snap.sensors.iat_c + 40.0).clamp(0.0, 255.0) as u8;
    table[0x06] = (snap.sensors.tps_pct * 2.0).clamp(0.0, 200.0) as u8; // 0.5%/bit
    table[0x07] = (snap.sensors.vbatt_v * 10.0).clamp(0.0, 255.0) as u8; // 0.1V/bit
    table[0x08] = (snap.lambda * 100.0).clamp(0.0, 255.0) as u8; // 0.01 lambda/bit

    let inj_01ms = (snap.actuators.inj_pw_us[0] / 100).min(255) as u8;
    table[0x09] = inj_01ms;
    table[0x0a] = ((snap.actuators.spark_advance_deg_x10 + 200) / 10).clamp(0, 255) as u8;
    table[0x0b] = ((snap.vtec_state == VtecState::HighCamActive) as u8)
        | (((snap.sync_state == SyncState::FullSynced720) as u8) << 1)
        | (((snap.wideband_state == WidebandState::ClosedLoopReady) as u8) << 2)
        | ((snap.actuators.fuel_pump_relay_on as u8) << 3)
        | ((snap.actuators.fan_relay_on as u8) << 4)
        | ((snap.actuators.ac_clutch_relay_on as u8) << 5)
        | ((snap.actuators.mil_lamp_on as u8) << 6)
        | ((snap.sensors.switches.scs_service_jumper as u8) << 7);

    table[0x0c] = (snap.sensors.eld_amps * 2.0).clamp(0.0, 255.0) as u8;
    table[0x0d] = if aux.altf.valid {
        aux.altf.field_duty_pct.clamp(0.0, 100.0) as u8
    } else {
        0xff
    };
    table[0x0e] = if aux.flex.is_valid() {
        aux.flex.ethanol_pct.clamp(0.0, 100.0) as u8
    } else {
        0xff
    };
    table[0x0f] = aux.outputs.bitmask();

    let faults_be = snap.faults.0.to_be_bytes();
    table[0x10..0x14].copy_from_slice(&faults_be);
    table
}

/// Serializes the response frame for a validated [`DlcCommand`].
pub fn encode_dlc_response(
    cmd: DlcCommand,
    snap: &TelemetrySnapshot,
    aux: &AuxIoTelemetry,
) -> DlcResponse {
    let mut buf = [0u8; DLC_MAX_RESPONSE_BYTES];
    let len = match cmd {
        DlcCommand::ReadTable { offset, count } => {
            let start = (offset as usize).min(DLC_TABLE_LEN);
            let max_avail = DLC_TABLE_LEN.saturating_sub(start);
            let n = (count as usize).min(max_avail);
            let table = build_dlc_table(snap, aux);

            buf[0] = 0x00;
            buf[1] = (n + 4) as u8; // total byte count including header, offset, data, chk
            buf[2] = offset;
            buf[3..3 + n].copy_from_slice(&table[start..start + n]);
            buf[3 + n] = honda_checksum(&buf[..3 + n]);
            4 + n
        }
        DlcCommand::ClearFaults => {
            buf[0] = 0x00;
            buf[1] = 0x04;
            buf[2] = 0x01;
            buf[3] = honda_checksum(&buf[..3]);
            4
        }
        DlcCommand::Identify => {
            buf[0] = CMD_IDENTIFY;
            buf[1] = 0x07;
            buf[2..6].copy_from_slice(b"H743");
            buf[6] = honda_checksum(&buf[..6]);
            7
        }
    };

    DlcResponse { buf, len }
}

/// Incremental byte stream parser for inbound Honda `D7 DLC` frames with automatic resync.
#[derive(Debug, Clone)]
pub struct DlcParser {
    rx: [u8; 8],
    len: usize,
}

impl Default for DlcParser {
    fn default() -> Self {
        Self::new()
    }
}

impl DlcParser {
    pub const fn new() -> Self {
        Self { rx: [0; 8], len: 0 }
    }

    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Feeds one byte from `USART1` RX and yields a validated [`DlcCommand`] when complete.
    pub fn push_byte(&mut self, byte: u8) -> Option<DlcCommand> {
        if self.len == 0 && !matches!(byte, CMD_READ_TABLE | CMD_CLEAR_FAULTS | CMD_IDENTIFY) {
            return None;
        }

        if self.len < self.rx.len() {
            self.rx[self.len] = byte;
            self.len += 1;
        } else {
            self.len = 0;
            return None;
        }

        if self.len < 2 {
            return None;
        }

        let expected_len = self.rx[1] as usize;
        if !(4..=8).contains(&expected_len) {
            self.shift_resync();
            return None;
        }

        if self.len < expected_len {
            return None;
        }

        let frame = &self.rx[..expected_len];
        let cmd = if verify_honda_checksum(frame) {
            match (frame[0], expected_len) {
                (CMD_READ_TABLE, 5) => Some(DlcCommand::ReadTable {
                    offset: frame[2],
                    count: frame[3],
                }),
                (CMD_CLEAR_FAULTS, 4) => Some(DlcCommand::ClearFaults),
                (CMD_IDENTIFY, 4) => Some(DlcCommand::Identify),
                _ => None,
            }
        } else {
            None
        };

        if cmd.is_some() {
            self.len = 0;
        } else {
            self.shift_resync();
        }
        cmd
    }

    fn shift_resync(&mut self) {
        if let Some(pos) = self.rx[1..self.len]
            .iter()
            .position(|&b| matches!(b, CMD_READ_TABLE | CMD_CLEAR_FAULTS | CMD_IDENTIFY))
        {
            let start = 1 + pos;
            let rem = self.len - start;
            self.rx.copy_within(start..self.len, 0);
            self.len = rem;
        } else {
            self.len = 0;
        }
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{
        ActuatorOutputs, FaultFlags, SensorSnapshot, SyncState, TelemetrySnapshot, VtecState,
        WidebandState,
    };

    use crate::aux_io::AuxIoTelemetry;

    use super::{
        DlcCommand, DlcParser, encode_dlc_response, honda_checksum, verify_honda_checksum,
    };

    #[test]
    fn dlc_parser_decodes_requests_and_resyncs_after_noise() {
        let mut parser = DlcParser::new();

        // Noise bytes before valid 0x20 ReadTable command (`offset=0`, `count=16`)
        let mut req = [0x20, 0x05, 0x00, 0x10, 0x00];
        req[4] = honda_checksum(&req[..4]);

        assert_eq!(parser.push_byte(0x99), None);
        assert_eq!(parser.push_byte(0xaa), None);
        // False start byte followed by real command -> shift_resync recovers!
        assert_eq!(parser.push_byte(0x20), None);
        assert_eq!(parser.push_byte(0x99), None);

        let mut parsed = None;
        for b in req {
            if let Some(c) = parser.push_byte(b) {
                parsed = Some(c);
            }
        }
        assert_eq!(
            parsed,
            Some(DlcCommand::ReadTable {
                offset: 0x00,
                count: 0x10
            })
        );
    }

    #[test]
    fn dlc_response_encodes_live_table_with_valid_checksum() {
        let mut faults = FaultFlags::default();
        faults.insert(FaultFlags::ECT_RANGE);
        let snap = TelemetrySnapshot {
            seq: 1,
            uptime_ms: 100,
            sync_state: SyncState::FullSynced720,
            rpm: 2_850,
            crank_angle_deg_x10: 0,
            sync_loss_count: 0,
            sensors: SensorSnapshot {
                map_kpa: 45.0,
                ect_c: 88.0,
                iat_c: 30.0,
                tps_pct: 25.0,
                vbatt_v: 14.2,
                ..Default::default()
            },
            vtec_state: VtecState::LowCam,
            wideband_state: WidebandState::ClosedLoopReady,
            lambda: 0.99,
            lsu_ri_ohms: 300.0,
            ve_pct: 75.0,
            target_lambda: 1.0,
            stft_pct: 0.0,
            actuators: ActuatorOutputs {
                inj_pw_us: [3_400; 4],
                spark_advance_deg_x10: 250,
                ..Default::default()
            },
            faults,
        };
        let aux = AuxIoTelemetry::default();

        let resp = encode_dlc_response(
            DlcCommand::ReadTable {
                offset: 0x00,
                count: 16,
            },
            &snap,
            &aux,
        );
        let bytes = resp.as_slice();
        assert_eq!(bytes.len(), 20);
        assert!(verify_honda_checksum(bytes));
        assert_eq!(u16::from_be_bytes([bytes[3], bytes[4]]), 2_850);
        assert_eq!(bytes[7], 128); // ECT 88°C + 40

        let id_resp = encode_dlc_response(DlcCommand::Identify, &snap, &aux);
        assert!(verify_honda_checksum(id_resp.as_slice()));
        assert_eq!(&id_resp.as_slice()[2..6], b"H743");
    }

    #[test]
    fn dlc_handles_clear_faults_identify_bad_checksum_and_clamped_reads() {
        let mut parser = DlcParser::new();

        // 1. Corrupted checksum frame is rejected, followed immediately by valid ClearFaults (0x60)
        let bad_req = [0x60, 0x04, 0x00, 0xff];
        for b in bad_req {
            assert_eq!(parser.push_byte(b), None);
        }
        let mut clear_req = [0x60, 0x04, 0x00, 0x00];
        clear_req[3] = honda_checksum(&clear_req[..3]);
        let mut got = None;
        for b in clear_req {
            got = parser.push_byte(b);
        }
        assert_eq!(got, Some(DlcCommand::ClearFaults));

        // 2. Identify (0xfe) request parses cleanly
        let mut id_req = [0xfe, 0x04, 0x00, 0x00];
        id_req[3] = honda_checksum(&id_req[..3]);
        for b in id_req {
            got = parser.push_byte(b);
        }
        assert_eq!(got, Some(DlcCommand::Identify));

        // 3. Out-of-bounds ReadTable (offset=30, count=20) clamps to the remaining 2 bytes of
        // the 32B table
        let snap = TelemetrySnapshot {
            seq: 1,
            uptime_ms: 0,
            sync_state: SyncState::Unsynced,
            rpm: 0,
            crank_angle_deg_x10: 0,
            sync_loss_count: 0,
            sensors: SensorSnapshot::default(),
            vtec_state: VtecState::LowCam,
            wideband_state: WidebandState::WaitForEngine,
            lambda: 1.0,
            lsu_ri_ohms: 2_500.0,
            ve_pct: 0.0,
            target_lambda: 1.0,
            stft_pct: 0.0,
            actuators: ActuatorOutputs::default(),
            faults: FaultFlags::default(),
        };
        let aux = AuxIoTelemetry::default();
        let clamped = encode_dlc_response(
            DlcCommand::ReadTable {
                offset: 30,
                count: 20,
            },
            &snap,
            &aux,
        );
        assert_eq!(clamped.as_slice().len(), 6); // 4 framing bytes + 2 data bytes
        assert!(verify_honda_checksum(clamped.as_slice()));

        let clear_resp = encode_dlc_response(DlcCommand::ClearFaults, &snap, &aux);
        assert_eq!(clear_resp.as_slice().len(), 4);
        assert!(verify_honda_checksum(clear_resp.as_slice()));
    }
}
