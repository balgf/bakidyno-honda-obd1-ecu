use std::fmt::Write as _;
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use ecu_core::EngineCoordinator;
use ecu_core::datalog::{
    DatalogFileHeader, DatalogRecord, LOG_FLAG_MARK_EDGE, LOG_RECORD_BYTES, RtcTimestamp,
};
use ecu_core::sensors::{ADC_FULL_SCALE, VREF_VOLTS};
use ecu_types::{CalibrationBlob, FaultFlags, RawSensorAdc, SwitchState, crc32};
use proto_native::{
    AuxStatusReport, BoostMode, BoostStatusReport, BusStatusReport, KnockConfig, KnockStatusReport,
    MAX_CHUNK_BYTES, MAX_FRAME_BYTES, NvmStatusReport, ProtoError, Request, Response,
    SD_FORMAT_CONFIRM_MAGIC, SdDiskKind, SdStatusReport, decode_frame, encode_frame,
};
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
    /// Query MicroSD card, FAT32 logger, PD4/PC13 pins, and LSE RTC status over `proto-native`
    SdStatus {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Format the inserted MicroSD card cleanly as MBR + FAT32 (requires `--yes` and `RPM == 0`)
    SdFormat {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        /// Required explicit confirmation to wipe existing partition/FAT structures
        #[arg(long)]
        yes: bool,
    },
    /// List files in the FAT32 root directory on the ECU MicroSD card
    SdLs {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Download a `.BDL` datalog (or root FAT32 file) from the ECU MicroSD card over USB CDC
    SdPull {
        /// Root 8.3 filename (for example `LOG00001.BDL`)
        file: String,
        /// Destination local file path
        out: PathBuf,
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Decode and CRC32-verify a `.BDL` binary datalog into MegaLogViewer / Tuner CSV
    LogToCsv {
        /// Input `.BDL` file pulled from SD card or via `sd-pull`
        input: PathBuf,
        /// Output `.csv` path
        out: PathBuf,
    },
    /// Synchronize the STM32H743 LSE RTC date/time from host system time
    RtcSync {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        /// Optional fixed UTC offset in hours (e.g. `8`); defaults to local system offset
        #[arg(long)]
        tz_offset_hours: Option<i8>,
    },
    /// Query live ADC3 Knock Window DSP config, per-cylinder RMS, noise floor, and retard
    KnockStatus {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Update runtime Knock Window DSP configuration in ECU RAM over `proto-native`
    KnockCfg {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        #[arg(long)]
        enabled: Option<bool>,
        #[arg(long)]
        retard_enabled: Option<bool>,
        #[arg(long)]
        sensor_diag: Option<bool>,
        #[arg(long)]
        freq_hz: Option<u16>,
        #[arg(long)]
        min_rpm: Option<u16>,
        #[arg(long)]
        threshold_x100: Option<u16>,
        #[arg(long)]
        step_deg_x10: Option<i16>,
        #[arg(long)]
        max_deg_x10: Option<i16>,
        /// Also persist active Knock + Aux configs to SPI1 Flash NVM after updating RAM
        #[arg(long)]
        burn: bool,
    },
    /// Clear latched MIL fault flags in ECU RAM over `proto-native`
    ClearFaults {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Query live Phase D.4 Auxiliary I/O, Flex Fuel, and `ALTF*` status
    AuxStatus {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Query live Boost Control, overboost safety, Flex blend, 2-step launch, and FFS status
    BoostStatus {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Update Boost Control, overboost safety, 2-step launch, or FFS parameters in RAM
    BoostCfg {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
        /// `off`, `open`, or `closed`
        #[arg(long)]
        mode: Option<String>,
        /// Uniform target MAP (`kPa`) applied across all 8 RPM bins
        #[arg(long)]
        target_kpa: Option<u16>,
        /// Uniform base duty (`0..=1000` per-mille) applied across all 8 RPM bins
        #[arg(long)]
        base_duty: Option<u16>,
        /// Warm overboost fuel/spark cut threshold (`kPa`)
        #[arg(long)]
        overboost_kpa: Option<f32>,
        #[arg(long)]
        overboost_enabled: Option<bool>,
        #[arg(long)]
        flex_comp_enabled: Option<bool>,
        #[arg(long)]
        flex_boost_kpa: Option<f32>,
        #[arg(long)]
        launch_enabled: Option<bool>,
        #[arg(long)]
        launch_rpm: Option<u16>,
        #[arg(long)]
        launch_retard_deg_x10: Option<i16>,
        #[arg(long)]
        ffs_enabled: Option<bool>,
        #[arg(long)]
        ffs_min_rpm: Option<u16>,
        #[arg(long)]
        lean_prot_enabled: Option<bool>,
        /// Also persist active Knock + Aux + Boost configs to SPI1 Flash NVM after updating RAM
        #[arg(long)]
        burn: bool,
    },
    /// Persist active Knock DSP, Auxiliary I/O, and Boost configurations to SPI1 NVM
    BurnConfig {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Query auto-detected SPI1 NOR Flash / FRAM hardware identity and slot state
    NvmStatus {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
    },
    /// Query live FDCAN1 (PD0/PD1) and Honda D7 DLC (PA9/PA10) bus counters
    BusStatus {
        #[arg(long, default_value = "/dev/ttyACM0")]
        port: String,
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

fn native_rpc_with_timeout(port: &mut File, req: &Request, timeout: Duration) -> Result<Response> {
    let mut tx = [0u8; MAX_FRAME_BYTES];
    let n = encode_frame(req, &mut tx).map_err(|e| anyhow::anyhow!("encode error: {e:?}"))?;
    port.write_all(&tx[..n])?;
    port.flush()?;

    let deadline = Instant::now() + timeout;
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

fn native_rpc(port: &mut File, req: &Request) -> Result<Response> {
    native_rpc_with_timeout(port, req, Duration::from_secs(2))
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
    if let Ok(Response::Capabilities(cap)) = native_rpc(&mut port, &Request::GetCapabilities) {
        println!(
            "Capabilities: {:?} | trigger={:?} | ign={:?} | cyl={} | hw=rev{}.{} | features=0x{:08x}",
            cap.platform,
            cap.trigger,
            cap.ignition,
            cap.cylinders,
            cap.board_rev_major,
            cap.board_rev_minor,
            cap.features.0
        );
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
        (FaultFlags::OVERBOOST_CUT, "OVERBOOST_CUT"),
        (FaultFlags::LEAN_BOOST_CUT, "LEAN_BOOST_CUT"),
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

fn format_proto_error(err: ProtoError) -> &'static str {
    match err {
        ProtoError::SdNotReady => "MicroSD card not detected or not ready on SDMMC1",
        ProtoError::SdBusy => "SD task busy (try again shortly)",
        ProtoError::SdFormatRejectedEngineRunning => {
            "format rejected because engine RPM > 0 (stop engine / dizzy simulator first)"
        }
        ProtoError::SdFileNotFound => "requested file not found in FAT32 root directory",
        ProtoError::SdIoFailed => "MicroSD block I/O or FAT32 filesystem error",
        ProtoError::OutOfBounds => "request parameter out of bounds",
        _ => "protocol error",
    }
}

fn format_disk_kind(kind: SdDiskKind) -> String {
    match kind {
        SdDiskKind::Unknown => "Unknown / Unread".to_string(),
        SdDiskKind::Blank => "Blank (all-zero sector 0 — run sd-format --yes)".to_string(),
        SdDiskKind::MbrFat32 => "MBR + FAT32 (ECU ready)".to_string(),
        SdDiskKind::MbrExFat => {
            "MBR + exFAT (typical SDXC 64GB factory format — run sd-format --yes)".to_string()
        }
        SdDiskKind::MbrNtfs => "MBR + NTFS (unsupported — run sd-format --yes)".to_string(),
        SdDiskKind::MbrLinux => "MBR + Linux (unsupported — run sd-format --yes)".to_string(),
        SdDiskKind::MbrOther(pt) => {
            format!("MBR partition type 0x{pt:02x} (run sd-format --yes)")
        }
        SdDiskKind::Gpt => "GPT protective MBR (unsupported — run sd-format --yes)".to_string(),
        SdDiskKind::SuperfloppyFat32 => {
            "Superfloppy FAT32 (no MBR partition table — run sd-format --yes)".to_string()
        }
        SdDiskKind::SuperfloppyExFat => {
            "Superfloppy exFAT (unsupported — run sd-format --yes)".to_string()
        }
    }
}

fn format_ascii_field(bytes: &[u8], len: usize) -> String {
    let clamped = len.min(bytes.len());
    String::from_utf8_lossy(&bytes[..clamped])
        .trim()
        .to_string()
}

fn print_sd_status(st: &SdStatusReport) {
    let prod = format_ascii_field(&st.card_prod_name, st.card_prod_name.len());
    let active = if st.active_file_len > 0 {
        format_ascii_field(&st.active_file, st.active_file_len as usize)
    } else {
        "<none>".to_string()
    };
    let cap_gib = st.capacity_mb as f64 / 1024.0;
    println!(
        "MicroSD Card   : {}",
        if st.card_initialized {
            "INITIALIZED"
        } else {
            "NOT READY"
        }
    );
    println!(
        "  Product Name : {}",
        if prod.is_empty() { "<n/a>" } else { &prod }
    );
    println!(
        "  Capacity     : {} MB ({:.2} GiB)",
        st.capacity_mb, cap_gib
    );
    println!("  Disk Layout  : {}", format_disk_kind(st.disk_kind));
    println!(
        "  FAT32 / Log  : mounted={} | logging_active={} | formatting={}",
        st.fat32_mounted, st.logging_active, st.format_in_progress
    );
    println!("  Active File  : {active} ({} B written)", st.bytes_written);
    println!(
        "  500 Hz Stats : logged={} rec | drops={} | io_err={} | marks={} | last_err={}",
        st.records_logged, st.dropped_records, st.write_errors, st.mark_count, st.last_error_code
    );
    println!(
        "  GPIO State   : PD4(detect)={} | PC13(KEY)={} (active_{})",
        if st.pd4_pin_high { "HIGH" } else { "LOW" },
        if st.pc13_pin_high { "HIGH" } else { "LOW" },
        if st.pc13_active_high { "high" } else { "low" }
    );
    println!(
        "  LSE RTC Time : {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        st.rtc_year, st.rtc_month, st.rtc_day, st.rtc_hour, st.rtc_minute, st.rtc_second
    );
}

fn query_sd_status(port: &mut File) -> Result<SdStatusReport> {
    match native_rpc(port, &Request::GetSdStatus)? {
        Response::SdStatus(st) => Ok(st),
        Response::Error(err) => bail!("GetSdStatus failed: {err:?} ({})", format_proto_error(err)),
        other => bail!("unexpected GetSdStatus response: {other:?}"),
    }
}

fn run_sd_status(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    let st = query_sd_status(&mut port)?;
    println!("STM32H743 SDMMC1 Status ({port_path})");
    println!("=======================================");
    print_sd_status(&st);
    Ok(())
}

fn run_sd_format(port_path: &str, yes: bool) -> Result<()> {
    if !yes {
        bail!(
            "Refusing to wipe MicroSD without `--yes`. Inspect first with `ecu-cli sd-status`, \
             then run `ecu-cli sd-format --port {port_path} --yes`."
        );
    }
    let mut port = open_serial(port_path)?;
    println!("Formatting MicroSD on {port_path} as MBR + FAT32 (waiting up to 30 s)...");
    match native_rpc_with_timeout(
        &mut port,
        &Request::FormatSdFat32 {
            confirm_magic: SD_FORMAT_CONFIRM_MAGIC,
        },
        Duration::from_secs(30),
    )? {
        Response::Ack => println!("Format & FAT32 read-back verification succeeded."),
        Response::Error(err) => bail!("sd-format failed: {err:?} ({})", format_proto_error(err)),
        other => bail!("unexpected sd-format response: {other:?}"),
    }

    let st = query_sd_status(&mut port)?;
    print_sd_status(&st);
    if st.rtc_year <= 2026 && st.rtc_month == 1 && st.rtc_day == 1 {
        println!(
            "Tip: RTC is still at default fallback date; run `ecu-cli rtc-sync --port \
             {port_path}` to timestamp new logs accurately."
        );
    }
    Ok(())
}

fn run_sd_ls(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    println!("Root Directory Listing ({port_path})");
    println!("====================================");
    let mut offset: u16 = 0;
    let mut total_listed = 0usize;
    for _ in 0..1024 {
        let (total_files, entries) = match native_rpc(&mut port, &Request::ListSdFiles { offset })?
        {
            Response::SdFileList {
                total_files,
                entries,
            } => (total_files, entries),
            Response::Error(err) => bail!("sd-ls failed: {err:?} ({})", format_proto_error(err)),
            other => bail!("unexpected ListSdFiles response: {other:?}"),
        };

        if total_listed == 0 {
            println!("Total root files: {total_files}");
        }
        if entries.is_empty() {
            break;
        }
        for entry in &entries {
            let name = format_ascii_field(&entry.name, entry.name_len as usize);
            total_listed += 1;
            println!(
                "  {total_listed:>3}. {name:<12}  {:>10} B",
                entry.size_bytes
            );
        }
        offset = offset.saturating_add(entries.len() as u16);
        if total_listed >= total_files as usize {
            break;
        }
    }
    Ok(())
}

fn run_sd_pull(file: &str, out: &Path, port_path: &str) -> Result<()> {
    let trimmed = file.trim();
    if trimmed.is_empty() || trimmed.len() > 12 {
        bail!("filename `{trimmed}` must be 1..=12 ASCII characters (8.3 FAT name)");
    }
    let mut name = [0u8; 12];
    name[..trimmed.len()].copy_from_slice(trimmed.as_bytes());
    let name_len = trimmed.len() as u8;

    let mut port = open_serial(port_path)?;
    let started = Instant::now();
    let mut offset = 0u32;
    let mut total_size: Option<u32> = None;
    let mut out_buf = Vec::new();

    loop {
        let req_len = match total_size {
            Some(total) => total.saturating_sub(offset).min(MAX_CHUNK_BYTES as u32) as u16,
            None => MAX_CHUNK_BYTES as u16,
        };
        if req_len == 0 && total_size.is_some() {
            break;
        }
        let req = Request::ReadSdFileChunk {
            name,
            name_len,
            offset,
            len: req_len,
        };
        match native_rpc_with_timeout(&mut port, &req, Duration::from_secs(5))? {
            Response::SdFileChunk {
                offset: chunk_off,
                total_size: file_total,
                data,
            } => {
                if chunk_off != offset {
                    bail!("chunk offset mismatch: expected {offset}, got {chunk_off}");
                }
                if total_size.is_none() {
                    total_size = Some(file_total);
                    out_buf.reserve(file_total as usize);
                    if file_total == 0 {
                        break;
                    }
                }
                if data.is_empty() {
                    bail!("unexpected empty chunk at offset {offset} of {file_total}");
                }
                out_buf.extend_from_slice(&data);
                offset = offset.saturating_add(data.len() as u32);
                if offset >= file_total {
                    break;
                }
            }
            Response::Error(err) => {
                bail!(
                    "sd-pull failed at offset {offset}: {err:?} ({})",
                    format_proto_error(err)
                )
            }
            other => bail!("unexpected ReadSdFileChunk response: {other:?}"),
        }
    }

    fs::write(out, &out_buf).with_context(|| format!("failed to write {}", out.display()))?;
    let elapsed = started.elapsed().as_secs_f64().max(0.001);
    let kib_s = (out_buf.len() as f64 / 1024.0) / elapsed;
    println!(
        "Pulled {trimmed} -> {} ({} bytes in {:.2}s, {:.1} KiB/s)",
        out.display(),
        out_buf.len(),
        elapsed,
        kib_s
    );
    Ok(())
}

struct CsvConvertSummary {
    header: DatalogFileHeader,
    csv: String,
    records_decoded: usize,
    bad_crc_records: usize,
    zero_pad_slots: usize,
    marks_seen: usize,
}

fn convert_bdl_bytes_to_csv(bytes: &[u8]) -> Result<CsvConvertSummary> {
    if bytes.len() < LOG_RECORD_BYTES {
        bail!(
            ".BDL file too short ({} bytes, need at least {LOG_RECORD_BYTES} byte header)",
            bytes.len()
        );
    }
    let hdr_bytes: &[u8; LOG_RECORD_BYTES] = bytes[..LOG_RECORD_BYTES]
        .try_into()
        .expect("exact header slice");
    let header = DatalogFileHeader::from_bytes(hdr_bytes)
        .context("invalid BDL1 header magic, version, record size, or CRC32")?;

    let body = &bytes[LOG_RECORD_BYTES..];
    let mut csv = String::with_capacity((body.len() / LOG_RECORD_BYTES) * 112 + 256);
    csv.push_str(
        "time_s,rpm,map_kpa,tps_pct,lambda,target_lambda,spark_deg,inj_pw_ms,ve_pct,ect_c,\
         iat_c,vbatt_v,vss_kmh,vtec_state,stft_pct,faults_hex,mark_seq,mark_edge,sync_state,\
         wideband_state,lsu_ri_ohms,switch_bits,aux_bits\n",
    );

    let mut base_uptime_ms: Option<u32> = None;
    let mut records_decoded = 0usize;
    let mut bad_crc_records = 0usize;
    let mut zero_pad_slots = 0usize;
    let mut marks_seen = 0usize;

    let (slots, _) = body.as_chunks::<LOG_RECORD_BYTES>();
    for slot in slots {
        if slot.iter().all(|&b| b == 0) {
            zero_pad_slots += 1;
            continue;
        }
        let Some(rec) = DatalogRecord::from_bytes(slot) else {
            bad_crc_records += 1;
            continue;
        };
        let t0 = *base_uptime_ms.get_or_insert(rec.uptime_ms);
        let time_s = rec.uptime_ms.saturating_sub(t0) as f64 / 1000.0;
        let mark_edge = u8::from((rec.flags & LOG_FLAG_MARK_EDGE) != 0);
        if mark_edge != 0 {
            marks_seen += 1;
        }
        let _ = writeln!(
            csv,
            "{:.3},{},{:.2},{:.2},{:.4},{:.4},{:.1},{:.3},{:.2},{:.1},{:.1},{:.2},{:.1},{},{:.2},0x{:08x},{},{},{},{},{:.1},0x{:02x},0x{:02x}",
            time_s,
            rec.rpm,
            rec.map_kpa_x10 as f32 / 10.0,
            rec.tps_pct_x10 as f32 / 10.0,
            rec.lambda_x1000 as f32 / 1000.0,
            rec.target_lambda_x1000 as f32 / 1000.0,
            rec.spark_adv_deg_x10 as f32 / 10.0,
            rec.inj_pw1_us as f32 / 1000.0,
            rec.ve_pct_x10 as f32 / 10.0,
            rec.ect_c_x10 as f32 / 10.0,
            rec.iat_c_x10 as f32 / 10.0,
            rec.vbatt_mv as f32 / 1000.0,
            rec.vss_kmh_x10 as f32 / 10.0,
            rec.vtec_state,
            rec.stft_pct_x10 as f32 / 10.0,
            rec.faults,
            rec.mark_seq,
            mark_edge,
            rec.sync_state,
            rec.wideband_state,
            rec.lsu_ri_ohms_x10 as f32 / 10.0,
            rec.switch_bits,
            rec.aux_outputs_bits
        );
        records_decoded += 1;
    }

    Ok(CsvConvertSummary {
        header,
        csv,
        records_decoded,
        bad_crc_records,
        zero_pad_slots,
        marks_seen,
    })
}

fn run_log_to_csv(input: &Path, out: &Path) -> Result<()> {
    let raw = fs::read(input).with_context(|| format!("failed to read {}", input.display()))?;
    let trailing_bytes = raw.len() % LOG_RECORD_BYTES;
    let summary = convert_bdl_bytes_to_csv(&raw)?;
    fs::write(out, &summary.csv).with_context(|| format!("failed to write {}", out.display()))?;
    let c = summary.header.created;
    println!(
        "BDL1 Header : v{} | {} Hz | created {:04}-{:02}-{:02} {:02}:{:02}:{:02} | cal_crc=0x{:08x}",
        summary.header.version,
        summary.header.sample_rate_hz,
        c.year,
        c.month,
        c.day,
        c.hour,
        c.minute,
        c.second,
        summary.header.cal_crc32
    );
    println!(
        "Decoded CSV : {} rows -> {} (marks={}, bad_crc={}, zero_pad={}, trailing_bytes={})",
        summary.records_decoded,
        out.display(),
        summary.marks_seen,
        summary.bad_crc_records,
        summary.zero_pad_slots,
        trailing_bytes
    );
    Ok(())
}

fn parse_tz_offset_hhmm(spec: &str) -> Option<i32> {
    let s = spec.trim();
    let sign = match s.as_bytes().first().copied()? {
        b'+' => 1i32,
        b'-' => -1i32,
        _ => return None,
    };
    let digits = &s[1..];
    if digits.len() != 4 || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let hours: i32 = digits[0..2].parse().ok()?;
    let mins: i32 = digits[2..4].parse().ok()?;
    Some(sign * (hours * 3600 + mins * 60))
}

fn detect_local_tz_offset_secs() -> i32 {
    if let Ok(out) = Command::new("date").arg("+%z").output()
        && out.status.success()
        && let Ok(text) = core::str::from_utf8(&out.stdout)
        && let Some(secs) = parse_tz_offset_hhmm(text)
    {
        return secs;
    }
    0
}

fn compute_rtc_fields(unix_secs: i64, tz_offset_secs: i32) -> (u16, u8, u8, u8, u8, u8, u8) {
    let local_secs = unix_secs.saturating_add(tz_offset_secs as i64).max(0) as u64;
    let ts = RtcTimestamp::from_unix_secs(local_secs);
    // 1970-01-01 was Thursday (`4` where `0` = Sun .. `6` = Sat).
    let days = (local_secs / 86_400) as i64;
    let weekday = (days + 4).rem_euclid(7) as u8;
    (
        ts.year, ts.month, ts.day, weekday, ts.hour, ts.minute, ts.second,
    )
}

fn run_rtc_sync(port_path: &str, tz_offset_hours: Option<i8>) -> Result<()> {
    let tz_secs = match tz_offset_hours {
        Some(h) => i32::from(h) * 3600,
        None => detect_local_tz_offset_secs(),
    };
    let unix_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .context("system clock is before UNIX_EPOCH")?
        .as_secs() as i64;
    let (year, month, day, weekday, hour, minute, second) = compute_rtc_fields(unix_secs, tz_secs);

    let mut port = open_serial(port_path)?;
    match native_rpc(
        &mut port,
        &Request::SetRtcDateTime {
            year,
            month,
            day,
            weekday,
            hour,
            minute,
            second,
        },
    )? {
        Response::Ack => {
            println!(
                "Synchronized STM32H743 LSE RTC -> {year:04}-{month:02}-{day:02} \
                 {hour:02}:{minute:02}:{second:02} (tz {:+.1}h)",
                tz_secs as f32 / 3600.0
            );
        }
        Response::Error(err) => bail!("rtc-sync failed: {err:?} ({})", format_proto_error(err)),
        other => bail!("unexpected SetRtcDateTime response: {other:?}"),
    }
    Ok(())
}

fn print_knock_status(st: &KnockStatusReport) {
    let t = &st.telemetry;
    let c = &st.config;
    println!("Honda OBD1 Knock Window DSP (ADC3 PC2_C @ 200 kS/s BDMA -> SRAM4)");
    println!("==================================================================");
    println!(
        "Config   : enabled={} retard={} diag={} center={}Hz Q={:.1} win={:.1}..{:.1}deg \
         min_rpm={} thr={:.2}x step={:.1}deg max={:.1}deg",
        c.enabled,
        c.retard_enabled,
        c.sensor_diag_enabled,
        c.center_freq_hz,
        f32::from(c.q_x10) / 10.0,
        f32::from(c.window_start_deg_x10) / 10.0,
        f32::from(c.window_end_deg_x10) / 10.0,
        c.min_rpm,
        f32::from(c.threshold_ratio_x100) / 100.0,
        f32::from(c.step_retard_deg_x10) / 10.0,
        f32::from(c.max_retard_deg_x10) / 10.0
    );
    println!(
        "Activity : windows={} overruns={} fault={} last=Cyl{} rpm={} n={} dc_mean={}",
        t.window_count,
        t.capture_overruns,
        t.sensor_fault,
        t.last_cylinder + 1,
        t.last_rpm,
        t.last_sample_count,
        t.last_dc_mean_counts
    );
    for cyl in 0..4 {
        println!(
            "  Cyl {}  : rms={:>5} floor={:>5} events={:>4} retard={:.1} deg",
            cyl + 1,
            t.per_cyl_rms_counts[cyl],
            t.per_cyl_noise_floor_counts[cyl],
            t.per_cyl_knock_events[cyl],
            f32::from(t.per_cyl_retard_deg_x10[cyl]) / 10.0
        );
    }
    println!("RPM Floor: {:?}", t.rpm_noise_bins_counts);
}

#[allow(clippy::too_many_arguments)]
fn apply_knock_cfg_overrides(
    mut cfg: KnockConfig,
    enabled: Option<bool>,
    retard_enabled: Option<bool>,
    sensor_diag: Option<bool>,
    freq_hz: Option<u16>,
    min_rpm: Option<u16>,
    threshold_x100: Option<u16>,
    step_deg_x10: Option<i16>,
    max_deg_x10: Option<i16>,
) -> KnockConfig {
    if let Some(v) = enabled {
        cfg.enabled = v;
    }
    if let Some(v) = retard_enabled {
        cfg.retard_enabled = v;
    }
    if let Some(v) = sensor_diag {
        cfg.sensor_diag_enabled = v;
    }
    if let Some(v) = freq_hz {
        cfg.center_freq_hz = v;
    }
    if let Some(v) = min_rpm {
        cfg.min_rpm = v;
    }
    if let Some(v) = threshold_x100 {
        cfg.threshold_ratio_x100 = v;
    }
    if let Some(v) = step_deg_x10 {
        cfg.step_retard_deg_x10 = v;
    }
    if let Some(v) = max_deg_x10 {
        cfg.max_retard_deg_x10 = v;
    }
    cfg
}

fn fetch_knock_status(port: &mut File) -> Result<KnockStatusReport> {
    match native_rpc(port, &Request::GetKnockStatus)? {
        Response::KnockStatus(report) => Ok(report),
        Response::Error(err) => bail!("GetKnockStatus failed: {err:?}"),
        other => bail!("unexpected GetKnockStatus response: {other:?}"),
    }
}

fn run_knock_status(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    let report = fetch_knock_status(&mut port)?;
    print_knock_status(&report);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn run_knock_cfg(
    port_path: &str,
    enabled: Option<bool>,
    retard_enabled: Option<bool>,
    sensor_diag: Option<bool>,
    freq_hz: Option<u16>,
    min_rpm: Option<u16>,
    threshold_x100: Option<u16>,
    step_deg_x10: Option<i16>,
    max_deg_x10: Option<i16>,
    burn: bool,
) -> Result<()> {
    let mut port = open_serial(port_path)?;
    let current = fetch_knock_status(&mut port)?;
    let updated = apply_knock_cfg_overrides(
        current.config,
        enabled,
        retard_enabled,
        sensor_diag,
        freq_hz,
        min_rpm,
        threshold_x100,
        step_deg_x10,
        max_deg_x10,
    );
    match native_rpc(&mut port, &Request::SetKnockConfig { config: updated })? {
        Response::Ack => {
            if burn {
                commit_runtime_cfg(&mut port)?;
            }
            let confirmed = fetch_knock_status(&mut port)?;
            print_knock_status(&confirmed);
            if burn {
                println!("Persisted Knock + Aux runtime config to SPI1 Flash NVM.");
            }
            Ok(())
        }
        Response::Error(err) => bail!("SetKnockConfig failed: {err:?}"),
        other => bail!("unexpected SetKnockConfig response: {other:?}"),
    }
}

fn commit_runtime_cfg(port: &mut File) -> Result<()> {
    match native_rpc_with_timeout(
        port,
        &Request::CommitRuntimeConfigToNvm,
        Duration::from_secs(5),
    )? {
        Response::Ack => Ok(()),
        Response::Error(err) => bail!("CommitRuntimeConfigToNvm failed: {err:?}"),
        other => bail!("unexpected CommitRuntimeConfigToNvm response: {other:?}"),
    }
}

fn run_clear_faults(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    match native_rpc(&mut port, &Request::ClearFaults)? {
        Response::Ack => {
            println!("Cleared latched MIL fault codes in ECU RAM.");
            Ok(())
        }
        Response::Error(err) => bail!("ClearFaults failed: {err:?}"),
        other => bail!("unexpected ClearFaults response: {other:?}"),
    }
}

fn print_aux_status(report: &AuxStatusReport) {
    let out = &report.telemetry.outputs;
    let flex = &report.telemetry.flex;
    let altf = &report.telemetry.altf;
    println!("Aux I/O & Flex Fuel Report");
    println!("--------------------------");
    println!(
        "Outputs    : ALTC*={} IAB*={} EVAP*={} ({:.1}%) PE0={} PE1={}",
        out.altc_on,
        out.iab_on,
        out.evap_on,
        f32::from(out.evap_duty_per_mille) / 10.0,
        out.pe0_on,
        out.pe1_on
    );
    println!(
        "Flex Fuel  : {:?} | {:.1} Hz | {:.1}% E | {:.1} C | {:.2} AFR | {:.3}x fuel | +{:.1} deg",
        flex.status,
        flex.freq_hz,
        flex.ethanol_pct,
        flex.fuel_temp_c,
        flex.stoich_afr(),
        flex.fuel_volume_multiplier(),
        flex.spark_advance_offset_deg()
    );
    println!(
        "ALTF Field : valid={} | {:.1} Hz | {:.1}% duty",
        altf.valid, altf.freq_hz, altf.field_duty_pct
    );
    println!(
        "Config     : ALTC={} (Eco {:.1}..{:.1}V) | IAB={} (inv={}, {}/{} RPM) | EVAP={} | PE0={:?} | PE1={:?}",
        report.config.altc_enabled,
        report.config.altc_eco_vbatt_off_v,
        report.config.altc_eco_vbatt_on_v,
        report.config.iab_enabled,
        report.config.iab_invert,
        report.config.iab_engage_rpm,
        report.config.iab_disengage_rpm,
        report.config.evap_enabled,
        report.config.pe0_mode,
        report.config.pe1_mode
    );
}

fn run_aux_status(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    match native_rpc(&mut port, &Request::GetAuxStatus)? {
        Response::AuxStatus(report) => {
            print_aux_status(&report);
            Ok(())
        }
        Response::Error(err) => bail!("GetAuxStatus failed: {err:?}"),
        other => bail!("unexpected GetAuxStatus response: {other:?}"),
    }
}

fn print_boost_status(report: &BoostStatusReport) {
    let tel = &report.telemetry;
    let cfg = &report.config;
    println!("Boost, Safety & Launch Report");
    println!("-----------------------------");
    println!(
        "Live Boost : mode={:?} | baro={:.1} kPa | boost={:+.1} kPa ({:+.1} psi) | \
         target={:.1} kPa | peak={:+.1} kPa",
        tel.mode,
        tel.baro_kpa,
        tel.boost_kpa,
        tel.boost_kpa * 0.145_038,
        tel.target_kpa,
        tel.peak_boost_kpa
    );
    println!(
        "Solenoid   : PE1/A19 duty={:.1}% (on={}) @ {} Hz | flex={:.3}x fuel / {:+.1} deg",
        f32::from(tel.solenoid_duty_per_mille) / 10.0,
        tel.solenoid_on,
        cfg.solenoid_freq_hz,
        tel.flex_fuel_mult,
        tel.flex_spark_add_deg
    );
    println!(
        "Safety     : overboost_cut={} ({:.0} kPa) | lean_cut={} | knock_drop={} | \
         cold_limit={} | stft_neg_max={:.0} kPa",
        tel.overboost_cut_active,
        cfg.overboost_cut_kpa,
        tel.lean_cut_active,
        tel.knock_drop_active,
        tel.cold_limit_active,
        cfg.stft_max_map_kpa
    );
    println!(
        "Motorsport : 2-step={} (en={}, {} RPM, {} deg/10) | FFS={} (en={}, min {} RPM)",
        tel.launch_active,
        cfg.launch_enabled,
        cfg.launch_rpm,
        cfg.launch_retard_deg_x10,
        tel.ffs_active,
        cfg.ffs_enabled,
        cfg.ffs_min_rpm
    );
}

fn run_boost_status(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    match native_rpc(&mut port, &Request::GetBoostStatus)? {
        Response::BoostStatus(report) => {
            print_boost_status(&report);
            Ok(())
        }
        Response::Error(err) => bail!("GetBoostStatus failed: {err:?}"),
        other => bail!("unexpected GetBoostStatus response: {other:?}"),
    }
}

#[allow(clippy::too_many_arguments)]
fn run_boost_cfg(
    port_path: &str,
    mode: Option<&str>,
    target_kpa: Option<u16>,
    base_duty: Option<u16>,
    overboost_kpa: Option<f32>,
    overboost_enabled: Option<bool>,
    flex_comp_enabled: Option<bool>,
    flex_boost_kpa: Option<f32>,
    launch_enabled: Option<bool>,
    launch_rpm: Option<u16>,
    launch_retard_deg_x10: Option<i16>,
    ffs_enabled: Option<bool>,
    ffs_min_rpm: Option<u16>,
    lean_prot_enabled: Option<bool>,
    burn: bool,
) -> Result<()> {
    let mut port = open_serial(port_path)?;
    let mut cfg = match native_rpc(&mut port, &Request::GetBoostStatus)? {
        Response::BoostStatus(report) => report.config,
        Response::Error(err) => bail!("GetBoostStatus failed: {err:?}"),
        other => bail!("unexpected GetBoostStatus response: {other:?}"),
    };
    if let Some(m) = mode {
        cfg.mode = match m.to_ascii_lowercase().as_str() {
            "off" => BoostMode::Off,
            "open" | "open_loop" => BoostMode::OpenLoop,
            "closed" | "closed_loop" | "pid" => BoostMode::ClosedLoop,
            _ => bail!("invalid boost mode '{m}'; expected off, open, or closed"),
        };
    }
    if let Some(kpa) = target_kpa {
        cfg.target_map_kpa = [kpa; 8];
    }
    if let Some(duty) = base_duty {
        cfg.base_duty_per_mille = [duty; 8];
    }
    if let Some(v) = overboost_kpa {
        cfg.overboost_cut_kpa = v;
    }
    if let Some(v) = overboost_enabled {
        cfg.overboost_cut_enabled = v;
    }
    if let Some(v) = flex_comp_enabled {
        cfg.flex_comp_enabled = v;
    }
    if let Some(v) = flex_boost_kpa {
        cfg.flex_boost_adder_kpa = v;
    }
    if let Some(v) = launch_enabled {
        cfg.launch_enabled = v;
    }
    if let Some(v) = launch_rpm {
        cfg.launch_rpm = v;
    }
    if let Some(v) = launch_retard_deg_x10 {
        cfg.launch_retard_deg_x10 = v;
    }
    if let Some(v) = ffs_enabled {
        cfg.ffs_enabled = v;
    }
    if let Some(v) = ffs_min_rpm {
        cfg.ffs_min_rpm = v;
    }
    if let Some(v) = lean_prot_enabled {
        cfg.lean_prot_enabled = v;
    }
    match native_rpc(&mut port, &Request::SetBoostConfig { config: cfg })? {
        Response::Ack => {}
        Response::Error(err) => bail!("SetBoostConfig failed: {err:?}"),
        other => bail!("unexpected SetBoostConfig response: {other:?}"),
    }
    if burn {
        commit_runtime_cfg(&mut port)?;
    }
    run_boost_status(port_path)
}

fn format_rdid_hex(rdid: &[u8; 9], len: u8) -> String {
    let count = usize::from(len).min(rdid.len());
    if count == 0 {
        return String::from("none");
    }
    let mut out = String::new();
    for (idx, b) in rdid[..count].iter().enumerate() {
        if idx > 0 {
            out.push('-');
        }
        let _ = write!(out, "{b:02x}");
    }
    out
}

fn print_nvm_status(report: &NvmStatusReport) {
    println!("SPI1 NVM Status Report");
    println!("----------------------");
    println!(
        "Device     : {:?} | detected={} | capacity={} KB ({} B)",
        report.medium,
        report.detected,
        report.capacity_bytes / 1024,
        report.capacity_bytes
    );
    println!(
        "SPI Format : RDID=[{}] | addr_bytes={} ({})",
        format_rdid_hex(&report.rdid, report.rdid_len),
        report.addr_bytes,
        if report.addr_bytes == 2 {
            "15/16-bit FRAM"
        } else if report.addr_bytes == 3 {
            "24-bit"
        } else {
            "unconfigured"
        }
    );
    println!(
        "Cal Region : active_slot={:?} | active_seq={}",
        report.cal_active_slot, report.cal_active_seq
    );
    println!(
        "Cfg Region : active_slot={:?} | active_seq={}",
        report.cfg_active_slot, report.cfg_active_seq
    );
}

fn run_nvm_status(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    match native_rpc(&mut port, &Request::GetNvmStatus)? {
        Response::NvmStatus(report) => {
            print_nvm_status(&report);
            Ok(())
        }
        Response::Error(err) => bail!("GetNvmStatus failed: {err:?}"),
        other => bail!("unexpected GetNvmStatus response: {other:?}"),
    }
}

fn print_bus_status(report: &BusStatusReport) {
    println!("Vehicle Bus & Honda DLC Status Report");
    println!("-------------------------------------");
    println!(
        "FDCAN1 (500k): self_test={} | tx_ok={} | tx_drop={} | rx_count={} | tec={}",
        report.can_self_test_passed,
        report.can_tx_ok,
        report.can_tx_drop,
        report.can_rx_count,
        report.can_tec
    );
    println!(
        "Honda D7 DLC : answered_frames={}",
        report.dlc_answered_frames
    );
}

fn run_bus_status(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    match native_rpc(&mut port, &Request::GetBusStatus)? {
        Response::BusStatus(report) => {
            print_bus_status(&report);
            Ok(())
        }
        Response::Error(err) => bail!("GetBusStatus failed: {err:?}"),
        other => bail!("unexpected GetBusStatus response: {other:?}"),
    }
}

fn run_burn_config(port_path: &str) -> Result<()> {
    let mut port = open_serial(port_path)?;
    commit_runtime_cfg(&mut port)?;
    println!("Persisted active Knock DSP + Aux I/O configurations to SPI1 NVM.");
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
        Commands::SdStatus { port } => run_sd_status(&port),
        Commands::SdFormat { port, yes } => run_sd_format(&port, yes),
        Commands::SdLs { port } => run_sd_ls(&port),
        Commands::SdPull { file, out, port } => run_sd_pull(&file, &out, &port),
        Commands::LogToCsv { input, out } => run_log_to_csv(&input, &out),
        Commands::RtcSync {
            port,
            tz_offset_hours,
        } => run_rtc_sync(&port, tz_offset_hours),
        Commands::KnockStatus { port } => run_knock_status(&port),
        Commands::KnockCfg {
            port,
            enabled,
            retard_enabled,
            sensor_diag,
            freq_hz,
            min_rpm,
            threshold_x100,
            step_deg_x10,
            max_deg_x10,
            burn,
        } => run_knock_cfg(
            &port,
            enabled,
            retard_enabled,
            sensor_diag,
            freq_hz,
            min_rpm,
            threshold_x100,
            step_deg_x10,
            max_deg_x10,
            burn,
        ),
        Commands::ClearFaults { port } => run_clear_faults(&port),
        Commands::AuxStatus { port } => run_aux_status(&port),
        Commands::BoostStatus { port } => run_boost_status(&port),
        Commands::BoostCfg {
            port,
            mode,
            target_kpa,
            base_duty,
            overboost_kpa,
            overboost_enabled,
            flex_comp_enabled,
            flex_boost_kpa,
            launch_enabled,
            launch_rpm,
            launch_retard_deg_x10,
            ffs_enabled,
            ffs_min_rpm,
            lean_prot_enabled,
            burn,
        } => run_boost_cfg(
            &port,
            mode.as_deref(),
            target_kpa,
            base_duty,
            overboost_kpa,
            overboost_enabled,
            flex_comp_enabled,
            flex_boost_kpa,
            launch_enabled,
            launch_rpm,
            launch_retard_deg_x10,
            ffs_enabled,
            ffs_min_rpm,
            lean_prot_enabled,
            burn,
        ),
        Commands::BurnConfig { port } => run_burn_config(&port),
        Commands::NvmStatus { port } => run_nvm_status(&port),
        Commands::BusStatus { port } => run_bus_status(&port),
    }
}

#[cfg(test)]
mod tests {
    use ecu_core::EngineCoordinator;
    use ecu_core::aux_io::AuxIoTelemetry;
    use ecu_core::datalog::{DatalogFileHeader, DatalogRecord, LOG_RECORD_BYTES, RtcTimestamp};
    use ecu_types::{CalibrationBlob, FaultFlags};

    use super::{
        build_ts_page_cmd, compute_rtc_fields, convert_bdl_bytes_to_csv, format_fault_flags,
        frame_demo, inspect_cal, kpa_to_honda_map_adc, parse_tz_offset_hhmm, run_bench_sim,
        tps_pct_to_adc,
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
        f.insert(FaultFlags::OVERBOOST_CUT);
        assert_eq!(
            format_fault_flags(f),
            "MAP_RANGE|WIDEBAND_FAULT|OVERBOOST_CUT"
        );

        let cal = CalibrationBlob::default_honda_obd1();
        assert!(kpa_to_honda_map_adc(100.0, &cal) > kpa_to_honda_map_adc(30.0, &cal));
        assert!(tps_pct_to_adc(100.0) > tps_pct_to_adc(0.0));

        let cmd = build_ts_page_cmd(b'R', 12, 2);
        assert_eq!(cmd, [b'R', 0, 0, 12, 0, 2, 0]);
    }

    #[test]
    fn bdl_to_csv_decodes_records_marks_and_rejects_bad_crc() {
        let created = RtcTimestamp {
            year: 2026,
            month: 11,
            day: 14,
            hour: 23,
            minute: 15,
            second: 4,
            subsec_ms: 0,
        };
        let hdr = DatalogFileHeader::new(created, 0x1234_5678);
        let mut raw = Vec::new();
        raw.extend_from_slice(&hdr.to_bytes());

        let mut snap = EngineCoordinator::new().snapshot();
        snap.uptime_ms = 5_000;
        snap.rpm = 3_200;
        snap.sensors.map_kpa = 98.4;
        snap.sensors.tps_pct = 55.2;
        snap.lambda = 0.875;
        snap.target_lambda = 0.860;
        snap.actuators.spark_advance_deg_x10 = 245;
        snap.actuators.inj_pw_us[0] = 6_420;
        snap.ve_pct = 88.5;
        snap.sensors.ect_c = 86.2;
        snap.sensors.iat_c = 34.0;
        snap.sensors.vbatt_v = 13.85;
        snap.sensors.vss_kmh = 72.0;

        let aux = AuxIoTelemetry::default();
        let rec0 = DatalogRecord::from_telemetry(&snap, &aux, 1, true, true);
        raw.extend_from_slice(&rec0.to_bytes());

        snap.uptime_ms = 5_002;
        snap.rpm = 3_250;
        let rec1 = DatalogRecord::from_telemetry(&snap, &aux, 1, false, false);
        raw.extend_from_slice(&rec1.to_bytes());

        // Add one corrupted record and one trailing zero pad slot.
        let mut corrupt = rec1.to_bytes();
        corrupt[0] ^= 0xff;
        raw.extend_from_slice(&corrupt);
        raw.extend_from_slice(&[0u8; LOG_RECORD_BYTES]);

        let summary = convert_bdl_bytes_to_csv(&raw).expect("valid BDL file should convert");
        assert_eq!(summary.header.cal_crc32, 0x1234_5678);
        assert_eq!(summary.records_decoded, 2);
        assert_eq!(summary.marks_seen, 1);
        assert_eq!(summary.bad_crc_records, 1);
        assert_eq!(summary.zero_pad_slots, 1);

        let mut lines = summary.csv.lines();
        let header_line = lines.next().expect("CSV header line");
        assert!(header_line.starts_with(
            "time_s,rpm,map_kpa,tps_pct,lambda,target_lambda,spark_deg,inj_pw_ms,ve_pct,ect_c,\
             iat_c,vbatt_v,vss_kmh,vtec_state,stft_pct,faults_hex,"
        ));
        let row0 = lines.next().expect("row 0");
        assert!(row0.starts_with(
            "0.000,3200,98.40,55.20,0.8750,0.8600,24.5,6.420,88.50,86.2,34.0,13.85,72.0,"
        ));
        let row1 = lines.next().expect("row 1");
        assert!(row1.starts_with("0.002,3250,"));
        assert!(lines.next().is_none());
    }

    #[test]
    fn rtc_fields_and_tz_offset_parser_match_calendar() {
        assert_eq!(parse_tz_offset_hhmm("+0800\n"), Some(28_800));
        assert_eq!(parse_tz_offset_hhmm("-0530"), Some(-19_800));
        assert_eq!(parse_tz_offset_hhmm("invalid"), None);

        // 2026-11-14 00:00:00 UTC (Saturday = 6) -> +8h = 2026-11-14 08:00:00 (Saturday = 6)
        let utc_2026_11_14 = 1_794_614_400i64;
        assert_eq!(
            compute_rtc_fields(utc_2026_11_14, 28_800),
            (2026, 11, 14, 6, 8, 0, 0)
        );

        let rdid = [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x22, 0x08];
        assert_eq!(
            super::format_rdid_hex(&rdid, 9),
            "7f-7f-7f-7f-7f-7f-c2-22-08"
        );
        assert_eq!(super::format_rdid_hex(&rdid, 0), "none");
    }
}
