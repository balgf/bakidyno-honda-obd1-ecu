//! Full-Speed `USB_OTG_FS` (`PA11` D-, `PA12` D+) CDC-ACM virtual serial interface
//! with zero-conflict protocol multiplexing for both TunerStudio (`msEnvelope_1.0`)
//! and `proto-native` (`COBS` + `CRC32` + `postcard` RPC).

use defmt::{info, warn};
use ecu_core::aux_io::sanitize_aux_config;
use ecu_types::{CalibrationBlob, RuntimeConfigBlob};
use embassy_stm32::Peri;
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::peripherals;
use embassy_stm32::usb::{self, Config as OtgConfig, Driver, InterruptHandler};
use embassy_time::{Duration, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as CdcState};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config as UsbConfig, UsbDevice};
use heapless::Vec;
use proto_native::{
    AuxStatusReport, KnockStatusReport, MAX_CHUNK_BYTES, ProtoError, Request, Response,
    SD_FORMAT_CONFIRM_MAGIC, decode_frame, encode_frame,
};
use proto_tunerstudio::{
    TS_ERR_OUT_OF_RANGE, TS_FLAG_BURN_OK, TS_PAGE_SIZE, handle_ts_packet,
    handle_ts_packet_with_cal, handle_ts_raw_probe, pack_page, unpack_page_into, wrap_ts_envelope,
};
use static_cell::StaticCell;

use crate::ECU_STATE;
use crate::nvm::NvmFlash;
use crate::sd_logger_hw::{
    SdCmd, SdCmdReply, parse_rpc_filename, run_sd_command, set_rtc_datetime, status_report,
    trigger_mark_event, update_cal_crc,
};

const USB_MAX_PACKET_SIZE: usize = 64;
const MAX_RX_BUF: usize = 600;
const MAX_TX_BUF: usize = 2900;
const INTER_BYTE_TIMEOUT_MS: u64 = 200;

static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<CdcState<'static>> = StaticCell::new();

pub type UsbOtgFsDriver = Driver<'static, peripherals::USB_OTG_FS>;

/// Initialized USB device runner and CDC-ACM data endpoint ready to spawn onto Embassy tasks.
pub struct UsbCommInit {
    pub usb_dev: UsbDevice<'static, UsbOtgFsDriver>,
    pub cdc: CdcAcmClass<'static, UsbOtgFsDriver>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum RxMode {
    Idle,
    TunerStudio,
    NativeCobs,
}

/// Initializes `USB_OTG_FS` (`PA12` D+, `PA11` D-) as a standard CDC-ACM virtual COM port.
pub fn init_cdc_acm(
    usb_peri: Peri<'static, peripherals::USB_OTG_FS>,
    irq: impl Binding<
        <peripherals::USB_OTG_FS as usb::Instance>::Interrupt,
        InterruptHandler<peripherals::USB_OTG_FS>,
    > + 'static,
    pa12_dp: Peri<'static, peripherals::PA12>,
    pa11_dm: Peri<'static, peripherals::PA11>,
) -> UsbCommInit {
    let mut otg_cfg = OtgConfig::default();
    // PA9 is wired to Honda DLC_TX and SB6 is open on WeAct H743; disable hardware VBUS sense.
    otg_cfg.vbus_detection = false;

    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 256]);
    let driver = Driver::new_fs(usb_peri, irq, pa12_dp, pa11_dm, ep_out_buffer, otg_cfg);

    let mut usb_cfg = UsbConfig::new(0x1209, 0x4837);
    usb_cfg.manufacturer = Some("Honda OBD1 OpenECU");
    usb_cfg.product = Some("CivicOBD1 STM32H743 ECU");
    usb_cfg.serial_number = Some("H743-OBD1-0001");
    usb_cfg.max_power = 100;
    usb_cfg.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CONFIG_DESC.init([0u8; 256]),
        BOS_DESC.init([0u8; 256]),
        MSOS_DESC.init([0u8; 256]),
        CONTROL_BUF.init([0u8; 64]),
    );
    let cdc = CdcAcmClass::new(&mut builder, CDC_STATE.init(CdcState::new()), 64);
    let usb_dev = builder.build();
    UsbCommInit { usb_dev, cdc }
}

#[embassy_executor::task]
pub async fn usb_device_task(mut usb: UsbDevice<'static, UsbOtgFsDriver>) -> ! {
    usb.run().await
}

struct ProtocolBuffers {
    mode: RxMode,
    rx_buf: [u8; MAX_RX_BUF],
    rx_len: usize,
    tx_buf: [u8; MAX_TX_BUF],
    page_buf: [u8; TS_PAGE_SIZE],
}

impl ProtocolBuffers {
    const fn new() -> Self {
        Self {
            mode: RxMode::Idle,
            rx_buf: [0u8; MAX_RX_BUF],
            rx_len: 0,
            tx_buf: [0u8; MAX_TX_BUF],
            page_buf: [0u8; TS_PAGE_SIZE],
        }
    }

    fn reset(&mut self) {
        self.mode = RxMode::Idle;
        self.rx_len = 0;
    }
}

#[embassy_executor::task]
pub async fn usb_protocol_task(
    mut cdc: CdcAcmClass<'static, UsbOtgFsDriver>,
    mut nvm: NvmFlash<'static>,
) {
    let mut pkt = [0u8; USB_MAX_PACKET_SIZE];
    let mut bufs = ProtocolBuffers::new();

    loop {
        cdc.wait_connection().await;
        info!("USB CDC-ACM host connected");
        bufs.reset();

        loop {
            let read_res = if bufs.mode == RxMode::Idle {
                cdc.read_packet(&mut pkt).await
            } else {
                match with_timeout(
                    Duration::from_millis(INTER_BYTE_TIMEOUT_MS),
                    cdc.read_packet(&mut pkt),
                )
                .await
                {
                    Ok(res) => res,
                    Err(_) => {
                        bufs.reset();
                        continue;
                    }
                }
            };

            let n = match read_res {
                Ok(n) => n,
                Err(EndpointError::Disabled) => break,
                Err(EndpointError::BufferOverflow) => {
                    warn!("USB CDC OUT buffer overflow; resetting parser");
                    bufs.reset();
                    continue;
                }
            };

            // Legacy unframed single-byte TunerStudio auto-detect probes ('Q', 'S', 'F').
            if bufs.mode == RxMode::Idle
                && n == 1
                && let Some(reply_len) = handle_ts_raw_probe(pkt[0], &mut bufs.tx_buf)
            {
                if write_cdc_all(&mut cdc, &bufs.tx_buf[..reply_len])
                    .await
                    .is_err()
                {
                    break;
                }
                continue;
            }

            if process_usb_chunk(&pkt[..n], &mut bufs, &mut cdc, &mut nvm)
                .await
                .is_err()
            {
                break;
            }
        }

        info!("USB CDC-ACM host disconnected");
    }
}

async fn process_usb_chunk(
    chunk: &[u8],
    bufs: &mut ProtocolBuffers,
    cdc: &mut CdcAcmClass<'static, UsbOtgFsDriver>,
    nvm: &mut NvmFlash<'static>,
) -> Result<(), EndpointError> {
    for &byte in chunk {
        match bufs.mode {
            RxMode::Idle => {
                bufs.rx_buf[0] = byte;
                bufs.rx_len = 1;
                bufs.mode = if byte == 0x00 {
                    RxMode::TunerStudio
                } else {
                    RxMode::NativeCobs
                };
            }
            RxMode::TunerStudio => {
                if bufs.rx_len >= MAX_RX_BUF {
                    bufs.reset();
                    continue;
                }
                bufs.rx_buf[bufs.rx_len] = byte;
                bufs.rx_len += 1;

                if bufs.rx_len >= 2 {
                    let body_len = u16::from_be_bytes([bufs.rx_buf[0], bufs.rx_buf[1]]) as usize;
                    if body_len == 0 {
                        // Drop superfluous leading 0x00 alignment byte.
                        bufs.rx_buf[0] = bufs.rx_buf[1];
                        bufs.rx_len = 1;
                        continue;
                    }
                    let expected = 2usize.saturating_add(body_len).saturating_add(4);
                    if expected > MAX_RX_BUF {
                        bufs.reset();
                        continue;
                    }
                    if bufs.rx_len == expected {
                        let frame_len = bufs.rx_len;
                        bufs.reset();
                        let reply_len = handle_tunerstudio_frame(
                            &bufs.rx_buf[..frame_len],
                            &mut bufs.tx_buf,
                            &mut bufs.page_buf,
                            nvm,
                        )
                        .await;
                        if reply_len > 0 {
                            write_cdc_all(cdc, &bufs.tx_buf[..reply_len]).await?;
                        }
                    }
                }
            }
            RxMode::NativeCobs => {
                if bufs.rx_len >= MAX_RX_BUF {
                    bufs.reset();
                    continue;
                }
                bufs.rx_buf[bufs.rx_len] = byte;
                bufs.rx_len += 1;
                if byte == 0x00 {
                    let frame_len = bufs.rx_len;
                    bufs.reset();
                    let reply_len = handle_native_frame(
                        &bufs.rx_buf[..frame_len],
                        &mut bufs.tx_buf,
                        &mut bufs.page_buf,
                        nvm,
                    )
                    .await;
                    if reply_len > 0 {
                        write_cdc_all(cdc, &bufs.tx_buf[..reply_len]).await?;
                    }
                }
            }
        }
    }
    Ok(())
}

async fn handle_tunerstudio_frame(
    frame: &[u8],
    tx_buf: &mut [u8; MAX_TX_BUF],
    page_buf: &mut [u8; TS_PAGE_SIZE],
    nvm: &mut NvmFlash<'static>,
) -> usize {
    let touches_cal = frame.len() >= 3 && matches!(frame[2], b'R' | b'M' | b'W' | b'k' | b'B');

    if !touches_cal {
        let snap = ECU_STATE.lock().await.ecu.snapshot();
        return handle_ts_packet(frame, &snap, tx_buf).unwrap_or(0);
    }

    let (snap, mut cal_opt) = {
        let state = ECU_STATE.lock().await;
        (state.ecu.snapshot(), state.cal.clone())
    };
    let Some(ref mut cal) = cal_opt else {
        return 0;
    };

    let Some(action) = handle_ts_packet_with_cal(frame, &snap, cal, page_buf, tx_buf) else {
        return 0;
    };

    if action.cal_dirty {
        ECU_STATE.lock().await.cal = Some(cal.clone());
    }

    if action.burn_requested {
        match nvm.save_calibration(cal, page_buf).await {
            Ok(()) => return wrap_ts_envelope(TS_FLAG_BURN_OK, &[], tx_buf).unwrap_or(0),
            Err(err) => {
                warn!("TunerStudio Burn NVM write failed: {}", err);
                return wrap_ts_envelope(TS_ERR_OUT_OF_RANGE, &[], tx_buf).unwrap_or(0);
            }
        }
    }

    action.reply_len
}

async fn handle_native_frame(
    frame: &[u8],
    tx_buf: &mut [u8; MAX_TX_BUF],
    page_buf: &mut [u8; TS_PAGE_SIZE],
    nvm: &mut NvmFlash<'static>,
) -> usize {
    let resp = match decode_frame(frame) {
        Ok(req) => dispatch_native_request(req, page_buf, nvm).await,
        Err(err) => Response::Error(err),
    };
    encode_frame(&resp, tx_buf).unwrap_or(0)
}

async fn dispatch_native_request(
    req: Request,
    page_buf: &mut [u8; TS_PAGE_SIZE],
    nvm: &mut NvmFlash<'static>,
) -> Response {
    match req {
        Request::Ping { nonce } => {
            let uptime_ms = ECU_STATE.lock().await.ecu.snapshot().uptime_ms;
            Response::Pong {
                proto_version: proto_native::PROTO_VERSION,
                nonce,
                uptime_ms,
            }
        }
        Request::GetTelemetry => {
            let snap = ECU_STATE.lock().await.ecu.snapshot();
            Response::Telemetry(snap)
        }
        Request::ReadCalibrationChunk { offset, len } => {
            let off = offset as usize;
            let cnt = (len as usize).min(MAX_CHUNK_BYTES);
            if off.checked_add(cnt).is_none_or(|end| end > TS_PAGE_SIZE) {
                return Response::Error(ProtoError::OutOfBounds);
            }
            {
                let state = ECU_STATE.lock().await;
                let Some(ref cal) = state.cal else {
                    return Response::Error(ProtoError::OutOfBounds);
                };
                pack_page(cal, page_buf);
            }
            let mut data = Vec::<u8, MAX_CHUNK_BYTES>::new();
            if data.extend_from_slice(&page_buf[off..off + cnt]).is_err() {
                return Response::Error(ProtoError::BufferTooSmall);
            }
            Response::CalibrationChunk {
                offset,
                total_len: TS_PAGE_SIZE as u16,
                data,
            }
        }
        Request::WriteCalibrationChunk { offset, data } => {
            let off = offset as usize;
            if off
                .checked_add(data.len())
                .is_none_or(|end| end > TS_PAGE_SIZE)
            {
                return Response::Error(ProtoError::OutOfBounds);
            }
            let mut cal_opt: Option<CalibrationBlob> = {
                let state = ECU_STATE.lock().await;
                state.cal.clone()
            };
            let Some(ref mut cal) = cal_opt else {
                return Response::Error(ProtoError::OutOfBounds);
            };
            pack_page(cal, page_buf);
            page_buf[off..off + data.len()].copy_from_slice(&data);
            unpack_page_into(page_buf, cal);
            ECU_STATE.lock().await.cal = Some(cal.clone());
            Response::Ack
        }
        Request::CommitCalibrationToNvm => {
            let cal_opt = ECU_STATE.lock().await.cal.clone();
            let Some(ref cal) = cal_opt else {
                return Response::Error(ProtoError::NvmWriteFailed);
            };
            pack_page(cal, page_buf);
            update_cal_crc(ecu_types::crc32(page_buf));
            match nvm.save_calibration(cal, page_buf).await {
                Ok(()) => Response::Ack,
                Err(_) => Response::Error(ProtoError::NvmWriteFailed),
            }
        }
        Request::SetTimingOverride { advance_deg_x10 } => {
            ECU_STATE
                .lock()
                .await
                .ecu
                .set_timing_override_deg_x10(advance_deg_x10);
            Response::Ack
        }
        Request::GetSdStatus => Response::SdStatus(status_report()),
        Request::ListSdFiles { offset } => {
            match run_sd_command(SdCmd::ListFiles { offset }, 3_000).await {
                SdCmdReply::FileList {
                    total_files,
                    entries,
                } => Response::SdFileList {
                    total_files,
                    entries,
                },
                SdCmdReply::Err(err) => Response::Error(err),
                _ => Response::Error(ProtoError::SdIoFailed),
            }
        }
        Request::FormatSdFat32 { confirm_magic } => {
            if confirm_magic != SD_FORMAT_CONFIRM_MAGIC {
                return Response::Error(ProtoError::OutOfBounds);
            }
            match run_sd_command(SdCmd::FormatFat32, 25_000).await {
                SdCmdReply::Ok => Response::Ack,
                SdCmdReply::Err(err) => Response::Error(err),
                _ => Response::Error(ProtoError::SdIoFailed),
            }
        }
        Request::ReadSdFileChunk {
            name,
            name_len,
            offset,
            len,
        } => {
            let Some(name_8_3) = parse_rpc_filename(&name, name_len) else {
                return Response::Error(ProtoError::OutOfBounds);
            };
            match run_sd_command(
                SdCmd::ReadFileChunk {
                    name_8_3,
                    offset,
                    len,
                },
                3_000,
            )
            .await
            {
                SdCmdReply::FileChunk {
                    offset,
                    total_size,
                    data,
                } => Response::SdFileChunk {
                    offset,
                    total_size,
                    data,
                },
                SdCmdReply::Err(err) => Response::Error(err),
                _ => Response::Error(ProtoError::SdIoFailed),
            }
        }
        Request::SetRtcDateTime {
            year,
            month,
            day,
            weekday,
            hour,
            minute,
            second,
        } => match set_rtc_datetime(year, month, day, weekday, hour, minute, second) {
            Ok(()) => Response::Ack,
            Err(err) => Response::Error(err),
        },
        Request::TriggerLogMark => {
            trigger_mark_event();
            Response::Ack
        }
        Request::GetKnockStatus => {
            let overruns = crate::knock_hw::overrun_count();
            let state = ECU_STATE.lock().await;
            Response::KnockStatus(KnockStatusReport {
                config: state.ecu.knock_config(),
                telemetry: state.ecu.knock_telemetry(overruns),
            })
        }
        Request::SetKnockConfig { config } => {
            crate::knock_hw::set_hw_enabled(config.enabled);
            ECU_STATE.lock().await.ecu.set_knock_config(config);
            Response::Ack
        }
        Request::ClearFaults => {
            ECU_STATE.lock().await.ecu.clear_faults();
            Response::Ack
        }
        Request::GetAuxStatus => {
            let state = ECU_STATE.lock().await;
            Response::AuxStatus(AuxStatusReport {
                config: state.aux_cfg,
                telemetry: state.aux_telemetry,
            })
        }
        Request::SetAuxConfig { config } => {
            let sanitized = sanitize_aux_config(config);
            ECU_STATE.lock().await.aux_cfg = sanitized;
            crate::aux_io_hw::set_capture_polarity(
                sanitized.flex_active_low_pulse,
                sanitized.altf_active_low_duty,
            );
            Response::Ack
        }
        Request::CommitRuntimeConfigToNvm => {
            let rt_cfg = {
                let state = ECU_STATE.lock().await;
                RuntimeConfigBlob {
                    knock: state.ecu.knock_config(),
                    aux: state.aux_cfg,
                    ..RuntimeConfigBlob::default_honda_obd1()
                }
            };
            match nvm.save_runtime_config(&rt_cfg).await {
                Ok(()) => Response::Ack,
                Err(err) => {
                    warn!("NVM runtime config write failed: {}", err);
                    Response::Error(ProtoError::NvmWriteFailed)
                }
            }
        }
    }
}

async fn write_cdc_all(
    cdc: &mut CdcAcmClass<'static, UsbOtgFsDriver>,
    data: &[u8],
) -> Result<(), EndpointError> {
    for chunk in data.chunks(USB_MAX_PACKET_SIZE) {
        cdc.write_packet(chunk).await?;
    }
    // USB bulk transfers whose total length is a multiple of 64 bytes require a ZLP to flush.
    if !data.is_empty() && data.len().is_multiple_of(USB_MAX_PACKET_SIZE) {
        cdc.write_packet(&[]).await?;
    }
    Ok(())
}
