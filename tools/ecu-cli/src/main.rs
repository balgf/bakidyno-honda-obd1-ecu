use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::process::Command;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use ecu_core::EngineCoordinator;
use ecu_core::sensors::{ADC_FULL_SCALE, VREF_VOLTS};
use ecu_types::{CalibrationBlob, FaultFlags, RawSensorAdc, SwitchState, crc32};
use proto_native::{MAX_FRAME_BYTES, Request, Response, decode_frame, encode_frame};
use proto_tunerstudio::{
    TS_FLAG_BURN_OK, TS_FLAG_OK, TS_OCH_SIZE, TS_PAGE_SIZE, TS_PROTO_VERSION, TS_SIGNATURE,
};

#[derive(Parser)]
#[command(
    name = "ecu-cli",
    about = "Honda Civic OBD1 STM32H743 Rust ECU — Bench Simulator & Calibration Tool"
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run a deterministic closed-loop Honda 24/4/1 sweep (Crank -> Idle -> Cruise -> VTEC WOT)
    Sim,
    /// Inspect the default Honda OBD1 CalibrationBlob byte size, CRC32, and VE/Spark tables
    InspectCal,
    /// Verify framed `proto-native` (`COBS` + `CRC32` + `postcard`) telemetry wire serialization
    FrameDemo,
    /// Probe a live STM32H743 ECU over USB CDC-ACM using `proto-native` RPC
    Probe {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Verify TunerStudio `msEnvelope_1.0` commands (`Q/S/F/C/O/R/k` and `M/B`) on live USB CDC
    TsCheck {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        #[arg(long)]
        burn_test: bool,
    },
}

fn volts_5v_to_adc(v_harness: f32) -> u16 {
    let v_pin = v_harness * 0.625; // 12.0k / 20.0k divider
    ((v_pin / VREF_VOLTS) * ADC_FULL_SCALE) as u16
}

fn kpa_to_honda_map_adc(kpa: f32, cal: &CalibrationBlob) -> u16 {
    let span = cal.map_cal.kpa_at_5v - cal.map_cal.kpa_at_0v;
    let v_5v = ((kpa - cal.map_cal.kpa_at_0v) / span * 5.0).clamp(0.2, 4.9);
    volts_5v_to_adc(v_5v)
}

fn tps_pct_to_adc(tps_pct: f32) -> u16 {
    let v_5v = 0.45 + (tps_pct / 100.0) * (4.50 - 0.45);
    volts_5v_to_adc(v_5v)
}

fn run_bench_sim() -> Result<()> {
    let cal = CalibrationBlob::default_honda_obd1();
    let mut ecu = EngineCoordinator::new();

    struct Stage {
        label: &'static str,
        rpm: u16,
        map_kpa: f32,
        tps_pct: f32,
        vss_kmh: f32,
        cranking: bool,
        vtp_oil_ok: bool,
        scs_jumper: bool,
    }

    let stages = [
        Stage {
            label: "1. Cold/Warm Cranking (STS)",
            rpm: 240,
            map_kpa: 92.0,
            tps_pct: 2.0,
            vss_kmh: 0.0,
            cranking: true,
            vtp_oil_ok: false,
            scs_jumper: false,
        },
        Stage {
            label: "2. SCS Timing-Light Lock",
            rpm: 850,
            map_kpa: 32.0,
            tps_pct: 0.0,
            vss_kmh: 0.0,
            cranking: false,
            vtp_oil_ok: false,
            scs_jumper: true,
        },
        Stage {
            label: "3. Warm Stoich Idle",
            rpm: 850,
            map_kpa: 32.0,
            tps_pct: 0.0,
            vss_kmh: 0.0,
            cranking: false,
            vtp_oil_ok: false,
            scs_jumper: false,
        },
        Stage {
            label: "4. Part-Throttle Cruise",
            rpm: 3200,
            map_kpa: 52.0,
            tps_pct: 22.0,
            vss_kmh: 80.0,
            cranking: false,
            vtp_oil_ok: false,
            scs_jumper: false,
        },
        Stage {
            label: "5. WOT Low-Cam Pull",
            rpm: 4400,
            map_kpa: 99.0,
            tps_pct: 100.0,
            vss_kmh: 95.0,
            cranking: false,
            vtp_oil_ok: false,
            scs_jumper: false,
        },
        Stage {
            label: "6. WOT VTEC Crossover!",
            rpm: 5600,
            map_kpa: 100.0,
            tps_pct: 100.0,
            vss_kmh: 118.0,
            cranking: false,
            vtp_oil_ok: true,
            scs_jumper: false,
        },
        Stage {
            label: "7. High-RPM VTEC Peak",
            rpm: 7100,
            map_kpa: 100.0,
            tps_pct: 100.0,
            vss_kmh: 148.0,
            cranking: false,
            vtp_oil_ok: true,
            scs_jumper: false,
        },
    ];

    println!(
        "{:<27} | {:>5} | {:>6} | {:>5} | {:>12} | {:>7} | {:>7} | {:>6} | {:>5}",
        "Operating Stage", "RPM", "MAP", "VE%", "VTEC State", "Inj PW", "Spark", "Dwell", "λ"
    );
    println!("{}", "-".repeat(104));

    let mut now_us: u32 = 1_000;
    let mut tooth_cursor: u8 = 0;

    for st in stages {
        let tooth_period_us = 5_000_000 / (st.rpm as u32);
        let mut next_tooth_us = now_us + tooth_period_us;

        let raw_adc = RawSensorAdc {
            vcc_5v_raw: volts_5v_to_adc(5.00),
            map_raw: kpa_to_honda_map_adc(st.map_kpa, &cal),
            tps_raw: tps_pct_to_adc(st.tps_pct),
            ect_raw: volts_5v_to_adc(0.403), // ~85 °C warm engine
            iat_raw: volts_5v_to_adc(2.050), // ~28 °C intake air
            vbatt_raw: (((14.1 * 15.0 / 115.0) / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
            eld_raw: volts_5v_to_adc(3.80),
            lsu_un_dc_raw: ((2.100 / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
            lsu_un_ac_delta_raw: ((0.0961 / VREF_VOLTS) * ADC_FULL_SCALE) as u16, // Ri = 300 Ω
            lsu_ip_raw: ((1.650 / VREF_VOLTS) * ADC_FULL_SCALE) as u16,
        };

        // Run 250 ms of real-time simulation per operating stage so IIR filters & VTEC settle
        for _ms in 0..250 {
            let target_end_us = now_us.wrapping_add(1_000);
            while next_tooth_us.wrapping_sub(now_us) <= 1_000 {
                if tooth_cursor == 0 {
                    ecu.on_cyp_capture(next_tooth_us.wrapping_sub(50));
                }
                if tooth_cursor.is_multiple_of(6) {
                    ecu.on_tdc_capture(next_tooth_us.wrapping_sub(25));
                }
                let _ = ecu.on_ckp_capture(next_tooth_us, &cal);
                tooth_cursor = (tooth_cursor + 1) % 24;
                next_tooth_us = next_tooth_us.wrapping_add(tooth_period_us);
            }
            now_us = target_end_us;

            let switches = SwitchState {
                sts_cranking: st.cranking,
                vtp_pressurized: st.vtp_oil_ok,
                scs_service_jumper: st.scs_jumper,
                ..Default::default()
            };
            let _ = ecu.tick_1khz(
                now_us, &raw_adc, switches, st.vss_kmh, /* vts_feedback_fault= */ false, &cal,
            );
        }

        let snap = ecu.snapshot();
        println!(
            "{:<27} | {:>5} | {:>5.1} | {:>5.1} | {:>12?} | {:>5.2}ms | {:>5.1}° | \
             {:>4.2}ms | {:>5.3}",
            st.label,
            snap.rpm,
            snap.sensors.map_kpa,
            snap.ve_pct,
            snap.vtec_state,
            snap.actuators.inj_pw_us[0] as f32 / 1_000.0,
            snap.actuators.spark_advance_deg_x10 as f32 / 10.0,
            snap.actuators.cop_dwell_us as f32 / 1_000.0,
            snap.target_lambda,
        );
    }

    Ok(())
}

fn inspect_cal() -> Result<()> {
    let cal = CalibrationBlob::default_honda_obd1();
    let mut buf = [0u8; 8192];
    let serialized = postcard::to_slice(&cal, &mut buf)
        .map_err(|e| anyhow::anyhow!("postcard serialize error: {e}"))?;
    let digest = crc32(serialized);

    println!("Honda Civic OBD1 Default Calibration Summary");
    println!("============================================");
    println!("Magic Header       : 0x{:08x} (\"HON1\")", cal.magic);
    println!("Schema Version     : {}", cal.version);
    println!(
        "Postcard Wire Size : {} bytes (fits dual ping-pong slots in 32 KB SPI1 FRAM)",
        serialized.len()
    );
    println!("CRC-32 Checksum    : 0x{:08x}", digest);
    println!("Engine Displacement: {} cc", cal.displacement_cc);
    println!("Injector Flow      : {} cc/min", cal.injector_flow_cc_min);
    println!(
        "VTEC Window        : Engage {} RPM / Disengage {} RPM (Min MAP {} kPa, ECT {} °C)",
        cal.vtec.engage_rpm, cal.vtec.disengage_rpm, cal.vtec.min_map_kpa, cal.vtec.min_ect_c
    );
    println!(
        "Rev Limiter        : Soft {} RPM / Hard {} RPM",
        cal.rev_limit_soft_rpm, cal.rev_limit_hard_rpm
    );
    Ok(())
}

fn frame_demo() -> Result<()> {
    let mut ecu = EngineCoordinator::new();
    let cal = CalibrationBlob::default_honda_obd1();
    let _ = ecu.tick_1khz(
        1_000,
        &RawSensorAdc::default(),
        SwitchState::default(),
        0.0,
        false,
        &cal,
    );

    let mut frame = [0u8; MAX_FRAME_BYTES];
    let req = Request::GetTelemetry;
    let req_len = encode_frame(&req, &mut frame)
        .map_err(|e| anyhow::anyhow!("encode request failed: {e:?}"))?;
    let decoded_req: Request = decode_frame(&frame[..req_len])
        .map_err(|e| anyhow::anyhow!("decode request failed: {e:?}"))?;

    let resp = Response::Telemetry(ecu.snapshot());
    let resp_len = encode_frame(&resp, &mut frame)
        .map_err(|e| anyhow::anyhow!("encode response failed: {e:?}"))?;
    let decoded_resp: Response = decode_frame(&frame[..resp_len])
        .map_err(|e| anyhow::anyhow!("decode response failed: {e:?}"))?;

    println!("proto-native COBS + CRC32 + Postcard Wire Verification");
    println!("======================================================");
    println!(
        "Encoded Request  ({:?}) -> {} wire bytes",
        decoded_req, req_len
    );
    println!(
        "Encoded Response (TelemetrySnapshot) -> {} wire bytes",
        resp_len
    );
    if let Response::Telemetry(t) = decoded_resp {
        println!(
            "Verified CRC32 Roundtrip: seq={}, uptime={}ms, FLR_prime={}",
            t.seq, t.uptime_ms, t.actuators.fuel_pump_relay_on
        );
    }
    Ok(())
}

fn open_serial(port_path: &str) -> Result<File> {
    let status = Command::new("stty")
        .args([
            "-F", port_path, "115200", "raw", "-echo", "-echoe", "-echok", "-onlcr", "-icrnl",
            "-ixon", "-ixoff", "min", "0", "time", "5",
        ])
        .status()
        .with_context(|| format!("failed to invoke stty on {port_path}"))?;
    if !status.success() {
        bail!("stty exited with {status} while configuring {port_path}");
    }
    OpenOptions::new()
        .read(true)
        .write(true)
        .open(port_path)
        .with_context(|| format!("failed to open serial port {port_path}"))
}

fn native_rpc(port: &mut File, req: &Request) -> Result<Response> {
    let mut tx = [0u8; MAX_FRAME_BYTES];
    let n = encode_frame(req, &mut tx).map_err(|e| anyhow::anyhow!("encode error: {e:?}"))?;
    port.write_all(&tx[..n])?;
    port.flush()?;

    let deadline = Instant::now() + Duration::from_secs(2);
    let mut rx = Vec::with_capacity(MAX_FRAME_BYTES);
    let mut byte = [0u8; 64];
    loop {
        if Instant::now() > deadline {
            bail!("timed out waiting for proto-native response");
        }
        let read_n = port.read(&mut byte)?;
        if read_n == 0 {
            continue;
        }
        rx.extend_from_slice(&byte[..read_n]);
        if let Some(end) = rx.iter().position(|&b| b == 0x00) {
            return decode_frame(&rx[..=end]).map_err(|e| anyhow::anyhow!("decode error: {e:?}"));
        }
    }
}

fn ts_rpc(port: &mut File, body: &[u8]) -> Result<(u8, Vec<u8>)> {
    let mut tx = Vec::with_capacity(2 + body.len() + 4);
    tx.extend_from_slice(&(body.len() as u16).to_be_bytes());
    tx.extend_from_slice(body);
    tx.extend_from_slice(&crc32(body).to_be_bytes());
    port.write_all(&tx)?;
    port.flush()?;

    let deadline = Instant::now() + Duration::from_secs(2);
    let mut rx = Vec::with_capacity(256);
    let mut chunk = [0u8; 256];
    let expected_len = loop {
        if Instant::now() > deadline {
            bail!("timed out waiting for TunerStudio header");
        }
        let n = port.read(&mut chunk)?;
        if n > 0 {
            rx.extend_from_slice(&chunk[..n]);
        }
        if rx.len() >= 2 {
            let body_len = u16::from_be_bytes([rx[0], rx[1]]) as usize;
            break 2 + body_len + 4;
        }
    };

    while rx.len() < expected_len {
        if Instant::now() > deadline {
            bail!(
                "timed out reading TunerStudio payload ({} of {} bytes)",
                rx.len(),
                expected_len
            );
        }
        let n = port.read(&mut chunk)?;
        if n > 0 {
            rx.extend_from_slice(&chunk[..n]);
        }
    }

    let body_len = expected_len - 6;
    let resp_body = &rx[2..2 + body_len];
    let rx_crc = u32::from_be_bytes([
        rx[2 + body_len],
        rx[2 + body_len + 1],
        rx[2 + body_len + 2],
        rx[2 + body_len + 3],
    ]);
    if crc32(resp_body) != rx_crc {
        bail!("TunerStudio reply CRC32 mismatch");
    }
    if resp_body.is_empty() {
        bail!("empty TunerStudio response body");
    }
    Ok((resp_body[0], resp_body[1..].to_vec()))
}

fn run_probe(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    let pong = native_rpc(&mut port, &Request::Ping { nonce: 0x5648_3743 })?;
    let telemetry = native_rpc(&mut port, &Request::GetTelemetry)?;

    println!("STM32H743 Live USB CDC Probe ({port_path})");
    println!("===========================================");
    match pong {
        Response::Pong {
            proto_version,
            nonce,
            uptime_ms,
        } => println!(
            "Ping Reply  : proto=v{proto_version}, nonce=0x{nonce:08x}, uptime={uptime_ms}ms"
        ),
        other => bail!("unexpected Ping response: {other:?}"),
    }
    match telemetry {
        Response::Telemetry(t) => {
            println!(
                "Live Engine : sync={:?}, rpm={}, adv={}deg, inj1={}us, ve={:.1}%",
                t.sync_state,
                t.rpm,
                t.actuators.spark_advance_deg_x10 as f32 / 10.0,
                t.actuators.inj_pw_us[0],
                t.ve_pct
            );
            println!(
                "Sensors     : MAP={:.1}kPa, TPS={:.1}%, ECT={:.1}C, IAT={:.1}C, \
                 VBatt={:.2}V, Vcc5={:.2}V, ELD={:.1}A, VSS={:.0}km/h",
                t.sensors.map_kpa,
                t.sensors.tps_pct,
                t.sensors.ect_c,
                t.sensors.iat_c,
                t.sensors.vbatt_v,
                t.sensors.vcc_5v_v,
                t.sensors.eld_amps,
                t.sensors.vss_kmh,
            );
            let sw = t.sensors.switches;
            println!(
                "Switches    : sts={}, vtp={}, scs={}, acs={}, psp={}, bksw={}, clutch={}",
                sw.sts_cranking,
                sw.vtp_pressurized,
                sw.scs_service_jumper,
                sw.acs_ac_request,
                sw.psp_steering_load,
                sw.bksw_brake,
                sw.clutch_pressed,
            );
            let act = t.actuators;
            println!(
                "Outputs/Flag: flr={}, vts={}, fan={}, acc={}, mil={}, iacv={:.1}%, \
                 vtec={:?}, faults=0x{:08x} [{}]",
                act.fuel_pump_relay_on,
                act.vts_high_side_on,
                act.fan_relay_on,
                act.ac_clutch_relay_on,
                act.mil_lamp_on,
                act.iacv_duty_per_mille as f32 / 10.0,
                t.vtec_state,
                t.faults.0,
                format_fault_flags(t.faults),
            );
        }
        other => bail!("unexpected Telemetry response: {other:?}"),
    }
    Ok(())
}

fn format_fault_flags(faults: FaultFlags) -> String {
    if faults.is_empty() {
        return String::from("NONE");
    }
    let flags = [
        (FaultFlags::MAP_RANGE, "MAP_RANGE"),
        (FaultFlags::ECT_RANGE, "ECT_RANGE"),
        (FaultFlags::TPS_RANGE, "TPS_RANGE"),
        (FaultFlags::TDC_SENSOR, "TDC_SENSOR"),
        (FaultFlags::CYP_SENSOR, "CYP_SENSOR"),
        (FaultFlags::IAT_RANGE, "IAT_RANGE"),
        (FaultFlags::CKP_SYNC_LOSS, "CKP_SYNC_LOSS"),
        (FaultFlags::VTS_SOLENOID_HW, "VTS_SOLENOID_HW"),
        (FaultFlags::VTP_OIL_PRESSURE, "VTP_OIL_PRESSURE"),
        (FaultFlags::KNOCK_SENSOR, "KNOCK_SENSOR"),
        (FaultFlags::WIDEBAND_FAULT, "WIDEBAND_FAULT"),
    ];
    flags
        .iter()
        .filter(|&&(bit, _)| faults.contains(bit))
        .map(|&(_, name)| name)
        .collect::<Vec<_>>()
        .join("|")
}

fn build_ts_page_cmd(cmd: u8, off: u16, cnt: u16) -> [u8; 7] {
    let mut req = [0u8; 7];
    req[0] = cmd;
    req[3..5].copy_from_slice(&off.to_le_bytes());
    req[5..7].copy_from_slice(&cnt.to_le_bytes());
    req
}

fn read_ts_chunk(port: &mut File, off: u16, cnt: u16) -> Result<Vec<u8>> {
    let req = build_ts_page_cmd(b'R', off, cnt);
    let (flag, payload) = ts_rpc(port, &req)?;
    if flag != TS_FLAG_OK || payload.len() != cnt as usize {
        bail!(
            "R({off}, {cnt}) failed: flag=0x{flag:02x}, len={}",
            payload.len()
        );
    }
    Ok(payload)
}

fn write_ts_i16(port: &mut File, off: u16, val: i16) -> Result<()> {
    let mut req = [0u8; 9];
    req[0] = b'M';
    req[3..5].copy_from_slice(&off.to_le_bytes());
    req[5..7].copy_from_slice(&2u16.to_le_bytes());
    req[7..9].copy_from_slice(&val.to_le_bytes());
    let (flag, _) = ts_rpc(port, &req)?;
    if flag != TS_FLAG_OK {
        bail!("M({off}) failed: flag=0x{flag:02x}");
    }
    Ok(())
}

fn run_ts_check(port_path: &str, burn_test: bool) -> Result<()> {
    let mut port = open_serial(port_path)?;
    println!("TunerStudio msEnvelope_1.0 Live Verification ({port_path})");
    println!("=========================================================");

    let (fq, pq) = ts_rpc(&mut port, b"Q")?;
    if fq != TS_FLAG_OK || pq != TS_SIGNATURE {
        bail!("signature mismatch: flag=0x{fq:02x}, payload={pq:?}");
    }
    println!(
        "Query 'Q'   : OK ({})",
        String::from_utf8_lossy(&pq).trim_end_matches('\0')
    );

    let (fs, ps) = ts_rpc(&mut port, b"S")?;
    if fs != TS_FLAG_OK {
        bail!("versionInfo failed: flag=0x{fs:02x}");
    }
    println!(
        "Version 'S' : {}",
        String::from_utf8_lossy(&ps).trim_end_matches('\0')
    );

    let (ff, pf) = ts_rpc(&mut port, b"F")?;
    if ff != TS_FLAG_OK || pf != TS_PROTO_VERSION {
        bail!("protocol 'F' failed: flag=0x{ff:02x}, payload={pf:?}");
    }
    println!("Proto 'F'   : OK ({})", String::from_utf8_lossy(&pf));

    let (fc, _) = ts_rpc(&mut port, b"C")?;
    if fc != TS_FLAG_OK {
        bail!("comms test 'C' failed: flag=0x{fc:02x}");
    }
    println!("Comms 'C'   : OK");

    let (fo, po) = ts_rpc(&mut port, b"O")?;
    if fo != TS_FLAG_OK || po.len() != TS_OCH_SIZE {
        bail!("och 'O' failed: flag=0x{fo:02x}, len={}", po.len());
    }
    let secl = u16::from_le_bytes([po[0], po[1]]);
    let rpm = u16::from_le_bytes([po[2], po[3]]);
    let adv_x10 = i16::from_le_bytes([po[18], po[19]]);
    println!(
        "Live 'O'    : secl={secl}s, rpm={rpm}, adv={:.1}deg (24B OK)",
        adv_x10 as f32 / 10.0
    );

    let mut page = Vec::with_capacity(TS_PAGE_SIZE);
    for off in (0..TS_PAGE_SIZE).step_by(128) {
        let cnt = 128.min(TS_PAGE_SIZE - off);
        page.extend_from_slice(&read_ts_chunk(&mut port, off as u16, cnt as u16)?);
    }
    let local_crc = crc32(&page);
    let k_req = build_ts_page_cmd(b'k', 0, TS_PAGE_SIZE as u16);
    let (fk, pk) = ts_rpc(&mut port, &k_req)?;
    if fk != TS_FLAG_OK || pk.len() != 4 {
        bail!("CRC check 'k' failed: flag=0x{fk:02x}");
    }
    let ecu_crc = u32::from_be_bytes([pk[0], pk[1], pk[2], pk[3]]);
    if ecu_crc != local_crc {
        bail!("Page 1 CRC32 mismatch: ECU=0x{ecu_crc:08x}, Host=0x{local_crc:08x}");
    }
    println!("Page 1 'R/k': {TS_PAGE_SIZE}B read & CRC32 matched (0x{ecu_crc:08x})");

    if burn_test {
        let orig_bytes = read_ts_chunk(&mut port, 12, 2)?;
        let orig_off = i16::from_le_bytes([orig_bytes[0], orig_bytes[1]]);
        write_ts_i16(&mut port, 12, orig_off + 5)?;
        let mod_bytes = read_ts_chunk(&mut port, 12, 2)?;
        let mod_val = i16::from_le_bytes([mod_bytes[0], mod_bytes[1]]);
        if mod_val != orig_off + 5 {
            bail!(
                "write verification failed: expected {}, got {mod_val}",
                orig_off + 5
            );
        }
        write_ts_i16(&mut port, 12, orig_off)?;
        let (fb, _) = ts_rpc(&mut port, &[b'B', 0, 0])?;
        if fb != TS_FLAG_BURN_OK {
            bail!("Burn 'B' failed: expected flag 0x{TS_FLAG_BURN_OK:02x}, got 0x{fb:02x}");
        }
        println!("Write/Burn  : Live M-write verified & 'B' NVM burn acknowledged (0x{fb:02x})");
    }

    Ok(())
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Commands::Sim => run_bench_sim(),
        Commands::InspectCal => inspect_cal(),
        Commands::FrameDemo => frame_demo(),
        Commands::Probe { port } => run_probe(&port),
        Commands::TsCheck { port, burn_test } => run_ts_check(&port, burn_test),
    }
}

#[cfg(test)]
mod tests {
    use ecu_types::{CalibrationBlob, FaultFlags};

    use super::{
        build_ts_page_cmd, format_fault_flags, frame_demo, inspect_cal, kpa_to_honda_map_adc,
        run_bench_sim, tps_pct_to_adc,
    };

    #[test]
    fn cli_bench_sim_inspect_cal_and_frame_demo_succeed() {
        run_bench_sim().expect("bench sim should succeed");
        inspect_cal().expect("inspect cal should succeed");
        frame_demo().expect("frame demo should succeed");
    }

    #[test]
    fn fault_flag_formatter_and_adc_helpers_are_consistent() {
        assert_eq!(format_fault_flags(FaultFlags::default()), "NONE");
        let mut f = FaultFlags::default();
        f.insert(FaultFlags::MAP_RANGE);
        f.insert(FaultFlags::WIDEBAND_FAULT);
        assert_eq!(format_fault_flags(f), "MAP_RANGE|WIDEBAND_FAULT");

        let cal = CalibrationBlob::default_honda_obd1();
        assert!(kpa_to_honda_map_adc(100.0, &cal) > kpa_to_honda_map_adc(30.0, &cal));
        assert!(tps_pct_to_adc(100.0) > tps_pct_to_adc(0.0));

        let cmd = build_ts_page_cmd(b'R', 12, 2);
        assert_eq!(cmd, [b'R', 0, 0, 12, 0, 2, 0]);
    }
}
