//! Hardware `SDMMC1` 4-bit (`PC8–PC12`, `PD2`), `PD4` (`SD_DETECT_N`), `PC13` (`KEY` mark),
//! and `LSE` (`32.768 kHz`) `RTC` `500 Hz` standalone datalogger driver.
//!
//! Safety & concurrency design:
//! - Producer (`1 kHz` control loop) pushes `64`-byte [`DatalogRecord`]s at `500 Hz` into a static
//!   `32 KB` (`512`-record) ring buffer in `AXI SRAM` using only non-blocking critical sections.
//! - Consumer (`sd_logger_task`) exclusively owns `SDMMC1`, writes `16`-sector (`8 KB`) multi-block
//!   IDMA bursts, commits FAT32 metadata every `2 s`, and services USB RPC requests (`sd-status`,
//!   `sd-ls`, `sd-pull`, and explicit `sd-format`).
//! - Never auto-formats on boot or button press: formatting is only executed on explicit
//!   `Request::FormatSdFat32` confirmation while `RPM == 0`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering, fence};

use defmt::{info, warn};
use ecu_core::aux_io::AuxIoTelemetry;
use ecu_core::datalog::{
    DatalogFileHeader, DatalogRecord, DatalogRing, LOG_RECORD_BYTES, RECORDS_PER_SECTOR,
    RtcTimestamp,
};
use ecu_core::fat32::{
    AsyncBlock512, DirEntryInfo, DiskLayoutKind, Fat32Error, Fat32FileWriter, Fat32Volume,
    format_8_3_name, format_log_filename_8_3, format_mbr_fat32, parse_filename_to_8_3, probe_disk,
};
use ecu_types::TelemetrySnapshot;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Flex, Input, Pull};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::pac;
use embassy_stm32::peripherals::{PC8, PC9, PC10, PC11, PC12, PC13, PD2, PD4, RTC, SDMMC1};
use embassy_stm32::rtc::{DateTime, DayOfWeek, Rtc, RtcConfig, RtcTimeProvider};
use embassy_stm32::sdmmc::sd::{CmdBlock, DataBlock, StorageDevice};
use embassy_stm32::sdmmc::{Config as SdmmcConfig, Error as SdError, InterruptHandler, Sdmmc};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use heapless::Vec;
use proto_native::{MAX_CHUNK_BYTES, ProtoError, SdDirEntry, SdDiskKind, SdStatusReport};
use static_cell::StaticCell;

const RING_CAP_RECORDS: usize = 512; // 32 KB (~1.024 s buffer at 500 Hz)
const DMA_BATCH_SECTORS: usize = 16; // 8 KB IDMA multi-block burst
const METADATA_SYNC_MS: u64 = 2_000;
const FLUSH_IDLE_MS: u64 = 250;
const RETRY_CARD_MS: u64 = 1_500;
const BUTTON_DEBOUNCE_TICKS: u8 = 25; // 25 ms @ 1 kHz poll
const RTC_MAGIC_BKP_REG: usize = 0;
const RTC_VALID_MAGIC: u32 = 0x4244_5254; // "BDRT"

static LOG_RING: BlockingMutex<CriticalSectionRawMutex, RefCell<DatalogRing<RING_CAP_RECORDS>>> =
    BlockingMutex::new(RefCell::new(DatalogRing::new()));
static STATUS_SNAPSHOT: BlockingMutex<CriticalSectionRawMutex, RefCell<SdLiveState>> =
    BlockingMutex::new(RefCell::new(SdLiveState::new()));

static LOGGING_ENABLED: AtomicBool = AtomicBool::new(false);
static FORMAT_RUNNING: AtomicBool = AtomicBool::new(false);
static PD4_PIN_HIGH: AtomicBool = AtomicBool::new(true);
static PC13_PIN_HIGH: AtomicBool = AtomicBool::new(false);
static PC13_ACTIVE_HIGH: AtomicBool = AtomicBool::new(true);
static MARK_HELD: AtomicBool = AtomicBool::new(false);
static MARK_PENDING_EDGE: AtomicBool = AtomicBool::new(false);
static MARK_COUNT: AtomicU32 = AtomicU32::new(0);
static LAST_RPM: AtomicU32 = AtomicU32::new(0);
static LAST_CAL_CRC: AtomicU32 = AtomicU32::new(0);
static LAST_ERROR_CODE: AtomicU8 = AtomicU8::new(0);

static RTC_HANDLE: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<Rtc>>> =
    BlockingMutex::new(RefCell::new(None));
static RTC_TIME: BlockingMutex<CriticalSectionRawMutex, RefCell<Option<RtcTimeProvider>>> =
    BlockingMutex::new(RefCell::new(None));

static DMA_SCRATCH: StaticCell<[DataBlock; DMA_BATCH_SECTORS]> = StaticCell::new();
static IO_SCRATCH: StaticCell<[[u8; 512]; DMA_BATCH_SECTORS]> = StaticCell::new();

static SD_CMD_CH: Channel<CriticalSectionRawMutex, SdCmd, 1> = Channel::new();
static SD_RESP_SIG: Signal<CriticalSectionRawMutex, SdCmdReply> = Signal::new();

#[derive(Clone, Copy)]
struct SdLiveState {
    card_initialized: bool,
    disk_kind: SdDiskKind,
    fat32_mounted: bool,
    logging_active: bool,
    card_prod_name: [u8; 5],
    capacity_mb: u32,
    active_file: [u8; 12],
    active_file_len: u8,
    bytes_written: u32,
    records_logged: u32,
    write_errors: u32,
}

impl SdLiveState {
    const fn new() -> Self {
        Self {
            card_initialized: false,
            disk_kind: SdDiskKind::Unknown,
            fat32_mounted: false,
            logging_active: false,
            card_prod_name: [b' '; 5],
            capacity_mb: 0,
            active_file: [0u8; 12],
            active_file_len: 0,
            bytes_written: 0,
            records_logged: 0,
            write_errors: 0,
        }
    }
}

/// Asynchronous commands dispatched from `usb_comm` to `sd_logger_task`.
pub enum SdCmd {
    ListFiles {
        offset: u16,
    },
    FormatFat32,
    ReadFileChunk {
        name_8_3: [u8; 11],
        offset: u32,
        len: u16,
    },
}

/// Reply returned over [`SD_RESP_SIG`] after `sd_logger_task` finishes a command.
pub enum SdCmdReply {
    Ok,
    Err(ProtoError),
    FileList {
        total_files: u16,
        entries: Vec<SdDirEntry, 8>,
    },
    FileChunk {
        offset: u32,
        total_size: u32,
        data: Vec<u8, MAX_CHUNK_BYTES>,
    },
}

/// Hardware peripherals bundled for [`init_and_spawn`] or manual task wiring.
pub struct SdLoggerPeripherals {
    pub sdmmc1: Peri<'static, SDMMC1>,
    pub pc12_ck: Peri<'static, PC12>,
    pub pd2_cmd: Peri<'static, PD2>,
    pub pc8_d0: Peri<'static, PC8>,
    pub pc9_d1: Peri<'static, PC9>,
    pub pc10_d2: Peri<'static, PC10>,
    pub pc11_d3: Peri<'static, PC11>,
    pub pd4_detect: Peri<'static, PD4>,
    pub pc13_key: Peri<'static, PC13>,
    pub rtc: Peri<'static, RTC>,
}

/// Debounced `PC13` (`KEY` button) state machine polled from the `1 kHz` control loop.
pub struct LogMarkButton {
    pin: Input<'static>,
    active_high: bool,
    stable_pressed: bool,
    debounce_ticks: u8,
}

impl LogMarkButton {
    /// Auto-detects external pull polarity on `PC13` (`KEY`) so both WeAct active-high (`10 kΩ`
    /// pull-down to GND, button -> `3.3V`) and active-low clones read deterministically.
    pub fn init(pc13: Peri<'static, PC13>) -> Self {
        let mut flex = Flex::new(pc13);
        flex.set_as_input(Pull::Down);
        for _ in 0..1_000 {
            cortex_m::asm::nop();
        }
        let down_high = flex.is_high();

        flex.set_as_input(Pull::Up);
        for _ in 0..1_000 {
            cortex_m::asm::nop();
        }
        let up_high = flex.is_high();

        // External strong pull-up keeps pin HIGH even under internal Pull::Down -> active-low key.
        let (pull, active_high) = if down_high && up_high {
            (Pull::Up, false)
        } else {
            (Pull::Down, true)
        };
        flex.set_as_input(pull);
        let pin = Input::from_flex(flex);
        PC13_ACTIVE_HIGH.store(active_high, Ordering::Relaxed);
        PC13_PIN_HIGH.store(pin.is_high(), Ordering::Relaxed);

        Self {
            pin,
            active_high,
            stable_pressed: false,
            debounce_ticks: 0,
        }
    }

    /// Samples `PC13` at `1 kHz` (`25 ms` debounce) and latches manual mark events.
    pub fn poll_1khz(&mut self) {
        let high = self.pin.is_high();
        PC13_PIN_HIGH.store(high, Ordering::Relaxed);
        let raw_pressed = if self.active_high { high } else { !high };

        if raw_pressed != self.stable_pressed {
            self.debounce_ticks = self.debounce_ticks.saturating_add(1);
            if self.debounce_ticks >= BUTTON_DEBOUNCE_TICKS {
                self.stable_pressed = raw_pressed;
                self.debounce_ticks = 0;
                MARK_HELD.store(raw_pressed, Ordering::Relaxed);
                if raw_pressed {
                    trigger_mark_event();
                }
            }
        } else {
            self.debounce_ticks = 0;
        }
    }
}

/// Latches one manual event mark (invoked from `PC13` button press or USB RPC).
pub fn trigger_mark_event() {
    let seq = MARK_COUNT.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    MARK_PENDING_EDGE.store(true, Ordering::Relaxed);
    info!("Datalog manual mark #{} latched", seq);
}

/// Initializes STM32 `LSE` (`32.768 kHz`) RTC hardware, seeding fallback time if uninitialized.
pub fn init_rtc(rtc_peri: Peri<'static, RTC>) {
    let (mut rtc, provider) = Rtc::new(rtc_peri, RtcConfig::default());
    let has_valid_time = rtc.read_backup_register(RTC_MAGIC_BKP_REG) == Some(RTC_VALID_MAGIC)
        && provider.now().is_ok_and(|dt| dt.year() >= 2024);

    if !has_valid_time
        && let Ok(default_dt) = DateTime::from(2026, 1, 1, DayOfWeek::Thursday, 0, 0, 0, 0)
    {
        let _ = rtc.set_datetime(default_dt);
    }

    RTC_HANDLE.lock(|cell| {
        *cell.borrow_mut() = Some(rtc);
    });
    RTC_TIME.lock(|cell| {
        *cell.borrow_mut() = Some(provider);
    });
}

/// Updates the `STM32H743` backup-domain `RTC` calendar & clock from USB RPC (`weekday`: `0`=Sun..`6`=Sat).
pub fn set_rtc_datetime(
    year: u16,
    month: u8,
    day: u8,
    weekday_sun0: u8,
    hour: u8,
    minute: u8,
    second: u8,
) -> Result<(), ProtoError> {
    let dow = match weekday_sun0 {
        0 => DayOfWeek::Sunday,
        1 => DayOfWeek::Monday,
        2 => DayOfWeek::Tuesday,
        3 => DayOfWeek::Wednesday,
        4 => DayOfWeek::Thursday,
        5 => DayOfWeek::Friday,
        6 => DayOfWeek::Saturday,
        _ => return Err(ProtoError::OutOfBounds),
    };
    let dt = DateTime::from(year, month, day, dow, hour, minute, second, 0)
        .map_err(|_| ProtoError::OutOfBounds)?;

    let mut applied = false;
    RTC_HANDLE.lock(|cell| {
        if let Some(ref mut rtc) = *cell.borrow_mut()
            && rtc.set_datetime(dt).is_ok()
        {
            rtc.write_backup_register(RTC_MAGIC_BKP_REG, RTC_VALID_MAGIC);
            applied = true;
        }
    });
    if applied {
        Ok(())
    } else {
        Err(ProtoError::NvmWriteFailed)
    }
}

/// Reads current `LSE` RTC time or returns deterministic fallback (`2026-01-01`).
pub fn now_timestamp() -> RtcTimestamp {
    RTC_TIME.lock(|cell| {
        if let Some(ref prov) = *cell.borrow()
            && let Ok(dt) = prov.now()
        {
            return RtcTimestamp {
                year: dt.year(),
                month: dt.month(),
                day: dt.day(),
                hour: dt.hour(),
                minute: dt.minute(),
                second: dt.second(),
                subsec_ms: (dt.microsecond() / 1_000) as u16,
            };
        }
        RtcTimestamp::default_fallback()
    })
}

/// Records current calibration CRC32 for `.BDL` headers and latest `RPM` safety gate.
pub fn update_cal_crc(crc32_val: u32) {
    LAST_CAL_CRC.store(crc32_val, Ordering::Relaxed);
}

/// Queues one `500 Hz` sample into the `32 KB` ring buffer without blocking the `1 kHz` loop.
pub fn push_500hz_sample(snap: &TelemetrySnapshot, aux: &AuxIoTelemetry) {
    LAST_RPM.store(snap.rpm as u32, Ordering::Relaxed);
    if !LOGGING_ENABLED.load(Ordering::Relaxed) || FORMAT_RUNNING.load(Ordering::Relaxed) {
        return;
    }
    let mark_edge = MARK_PENDING_EDGE.swap(false, Ordering::Relaxed);
    let mark_held = MARK_HELD.load(Ordering::Relaxed);
    let mark_seq = MARK_COUNT.load(Ordering::Relaxed);
    let rec = DatalogRecord::from_telemetry(snap, aux, mark_seq, mark_edge, mark_held).to_bytes();
    LOG_RING.lock(|cell| {
        cell.borrow_mut().push_block(rec);
    });
}

/// Builds a compact USB-serializable status snapshot covering SD, FAT32, `PD4`, `PC13`, and RTC.
pub fn status_report() -> SdStatusReport {
    let ts = now_timestamp();
    let dropped = LOG_RING.lock(|cell| cell.borrow().dropped_count());
    let live = STATUS_SNAPSHOT.lock(|cell| *cell.borrow());
    SdStatusReport {
        pd4_pin_high: PD4_PIN_HIGH.load(Ordering::Relaxed),
        pc13_pin_high: PC13_PIN_HIGH.load(Ordering::Relaxed),
        pc13_active_high: PC13_ACTIVE_HIGH.load(Ordering::Relaxed),
        card_initialized: live.card_initialized,
        disk_kind: live.disk_kind,
        fat32_mounted: live.fat32_mounted,
        logging_active: live.logging_active,
        format_in_progress: FORMAT_RUNNING.load(Ordering::Relaxed),
        last_error_code: LAST_ERROR_CODE.load(Ordering::Relaxed),
        card_prod_name: live.card_prod_name,
        capacity_mb: live.capacity_mb,
        active_file: live.active_file,
        active_file_len: live.active_file_len,
        bytes_written: live.bytes_written,
        records_logged: live.records_logged,
        dropped_records: dropped,
        write_errors: live.write_errors,
        mark_count: MARK_COUNT.load(Ordering::Relaxed),
        rtc_year: ts.year,
        rtc_month: ts.month,
        rtc_day: ts.day,
        rtc_hour: ts.hour,
        rtc_minute: ts.minute,
        rtc_second: ts.second,
    }
}

/// Sends an RPC command to `sd_logger_task` and awaits completion cleanly.
pub async fn run_sd_command(cmd: SdCmd, timeout_ms: u64) -> SdCmdReply {
    if matches!(cmd, SdCmd::FormatFat32) && LAST_RPM.load(Ordering::Relaxed) > 0 {
        return SdCmdReply::Err(ProtoError::SdFormatRejectedEngineRunning);
    }
    SD_RESP_SIG.reset();
    if SD_CMD_CH.try_send(cmd).is_err() {
        return SdCmdReply::Err(ProtoError::SdBusy);
    }
    match with_timeout(Duration::from_millis(timeout_ms), SD_RESP_SIG.wait()).await {
        Ok(reply) => reply,
        Err(_) => SdCmdReply::Err(ProtoError::SdBusy),
    }
}

/// Initializes `SDMMC1` 4-bit, `PD4`, `PC13`, and `RTC`, returning peripherals & debounced button.
pub fn init_hardware(
    p: SdLoggerPeripherals,
    irq: impl Binding<<SDMMC1 as embassy_stm32::sdmmc::Instance>::Interrupt, InterruptHandler<SDMMC1>>
    + 'static,
) -> (Sdmmc<'static>, Input<'static>, LogMarkButton) {
    init_rtc(p.rtc);
    let mark_btn = LogMarkButton::init(p.pc13_key);
    let pd4 = Input::new(p.pd4_detect, Pull::Up);
    PD4_PIN_HIGH.store(pd4.is_high(), Ordering::Relaxed);
    let sdmmc = Sdmmc::new_4bit(
        p.sdmmc1,
        irq,
        p.pc12_ck,
        p.pd2_cmd,
        p.pc8_d0,
        p.pc9_d1,
        p.pc10_d2,
        p.pc11_d3,
        SdmmcConfig::default(),
    );
    (sdmmc, pd4, mark_btn)
}

const SD_CMD_SPIN_LIMIT: u32 = 2_000_000;
const SD_WRITE_TIMEOUT_MS: u64 = 1_500;
const SD_R1_READY_FOR_DATA: u32 = 0x100;

fn clear_sdmmc1_flags(regs: pac::sdmmc::Sdmmc) {
    regs.icr().write(|w| {
        w.set_ccrcfailc(true);
        w.set_dcrcfailc(true);
        w.set_ctimeoutc(true);
        w.set_dtimeoutc(true);
        w.set_txunderrc(true);
        w.set_rxoverrc(true);
        w.set_cmdrendc(true);
        w.set_cmdsentc(true);
        w.set_dataendc(true);
        w.set_dholdc(true);
        w.set_dbckendc(true);
        w.set_dabortc(true);
        w.set_busyd0endc(true);
        w.set_sdioitc(true);
        w.set_ackfailc(true);
        w.set_acktimeoutc(true);
        w.set_vswendc(true);
        w.set_ckstopc(true);
        w.set_idmatec(true);
        w.set_idmabtcc(true);
    });
}

fn send_sdmmc1_cmd(
    regs: pac::sdmmc::Sdmmc,
    cmdindex: u8,
    arg: u32,
    waitresp: u8,
    check_crc: bool,
    cmdtrans: bool,
    cmdstop: bool,
) -> Result<u32, SdError> {
    clear_sdmmc1_flags(regs);
    let mut spin = SD_CMD_SPIN_LIMIT;
    while regs.star().read().cpsmact() {
        spin -= 1;
        if spin == 0 {
            return Err(SdError::SoftwareTimeout);
        }
    }

    regs.argr().write(|w| w.set_cmdarg(arg));
    regs.cmdr().write(|w| {
        w.set_waitint(false);
        w.set_waitresp(waitresp);
        w.set_cmdindex(cmdindex);
        w.set_cpsmen(true);
        w.set_cmdstop(cmdstop);
        w.set_cmdtrans(cmdtrans);
    });

    let mut status;
    spin = SD_CMD_SPIN_LIMIT;
    loop {
        status = regs.star().read();
        if waitresp == 0 {
            if status.ctimeout() || status.cmdsent() {
                break;
            }
        } else if status.ctimeout() || status.cmdrend() || status.ccrcfail() {
            break;
        }
        spin -= 1;
        if spin == 0 {
            return Err(SdError::SoftwareTimeout);
        }
    }

    if status.ctimeout() {
        return Err(SdError::Timeout);
    }
    if check_crc && status.ccrcfail() {
        return Err(SdError::Crc);
    }
    Ok(if waitresp == 0 {
        0
    } else {
        regs.respr(0).read().cardstatus()
    })
}

fn abort_sdmmc1_write(regs: pac::sdmmc::Sdmmc) {
    regs.maskr().write(|_| {});
    regs.idmactrlr().modify(|w| w.set_idmaen(/* val= */ false));
    if regs.star().read().dpsmact() {
        let _ = send_sdmmc1_cmd(
            regs, /* cmdindex= */ 12, /* arg= */ 0, /* waitresp= */ 1,
            /* check_crc= */ false, /* cmdtrans= */ false, /* cmdstop= */ true,
        );
        let mut spin = SD_CMD_SPIN_LIMIT;
        while regs.star().read().dpsmact() && spin > 0 {
            spin -= 1;
        }
    }
    clear_sdmmc1_flags(regs);
}

async fn wait_sdmmc1_idle(regs: pac::sdmmc::Sdmmc) -> Result<(), SdError> {
    let start = Instant::now();
    while regs.star().read().dpsmact() || regs.star().read().cpsmact() {
        if start.elapsed() > Duration::from_millis(SD_WRITE_TIMEOUT_MS) {
            return Err(SdError::SoftwareTimeout);
        }
        Timer::after_micros(200).await;
    }
    Ok(())
}

async fn wait_card_ready_yield(regs: pac::sdmmc::Sdmmc, rca: u16) -> Result<(), SdError> {
    let start = Instant::now();
    loop {
        if !regs.star().read().busyd0() {
            let r1 = send_sdmmc1_cmd(
                regs,
                /* cmdindex= */ 13,
                /* arg= */ (rca as u32) << 16,
                /* waitresp= */ 1,
                /* check_crc= */ true,
                /* cmdtrans= */ false,
                /* cmdstop= */ false,
            )?;
            if (r1 & SD_R1_READY_FOR_DATA) != 0 {
                return Ok(());
            }
        }
        if start.elapsed() > Duration::from_millis(SD_WRITE_TIMEOUT_MS) {
            return Err(SdError::SoftwareTimeout);
        }
        Timer::after_micros(250).await;
    }
}

struct SdStorageIo<'d, 's, 'a> {
    storage: &'d mut StorageDevice<'s, 'a, embassy_stm32::sdmmc::sd::Card>,
    dma_buf: &'d mut [DataBlock; DMA_BATCH_SECTORS],
    yield_every_batch: bool,
}

impl SdStorageIo<'_, '_, '_> {
    /// Non-blocking `SDMMC1` IDMA + yielding `CMD13` write replacement for `embassy_stm32`'s
    /// synchronous `0x00ff_ffff` `read_status()` busy-wait loop, so SD NAND programming stalls
    /// (`10-130+ ms`) yield the Thread-Mode executor without freezing `1 kHz` control or `USB`.
    async fn write_chunk_nonblocking(&mut self, lba: u32, n: usize) -> Result<(), SdError> {
        let card = self.storage.card();
        let address = if card.ocr.high_capacity() {
            lba
        } else {
            lba.saturating_mul(512)
        };
        let regs = pac::SDMMC1;

        wait_sdmmc1_idle(regs).await?;
        regs.maskr().write(|_| {});
        if regs.star().read().busyd0() {
            wait_card_ready_yield(regs, card.rca).await?;
        }

        // CMD16: ensure 512-byte card block length matches IDMA transfer size.
        send_sdmmc1_cmd(
            regs, /* cmdindex= */ 16, /* arg= */ 512, /* waitresp= */ 1,
            /* check_crc= */ true, /* cmdtrans= */ false, /* cmdstop= */ false,
        )?;

        clear_sdmmc1_flags(regs);
        let byte_len = (n * 512) as u32;
        regs.dlenr().write(|w| w.set_datalength(byte_len));
        regs.idmabase0r()
            .write(|w| w.set_idmabase0(self.dma_buf.as_ptr() as u32));
        regs.idmactrlr().modify(|w| w.set_idmaen(/* val= */ true));
        regs.dctrl().modify(|w| {
            w.set_dtmode(0);
            w.set_dblocksize(9); // 2^9 = 512 bytes per SD sector
            w.set_dtdir(false); // Host -> Card
        });
        fence(Ordering::SeqCst);

        let cmd_idx = if n == 1 { 24 } else { 25 };
        if let Err(err) = send_sdmmc1_cmd(
            regs, cmd_idx, address, /* waitresp= */ 1, /* check_crc= */ true,
            /* cmdtrans= */ true, /* cmdstop= */ false,
        ) {
            abort_sdmmc1_write(regs);
            return Err(err);
        }

        Timer::after_micros(if n == 1 { 60 } else { 300 }).await;
        let xfer_start = Instant::now();
        loop {
            let st = regs.star().read();
            if st.dcrcfail() {
                abort_sdmmc1_write(regs);
                return Err(SdError::Crc);
            }
            if st.dtimeout() {
                abort_sdmmc1_write(regs);
                return Err(SdError::Timeout);
            }
            if st.txunderr() {
                abort_sdmmc1_write(regs);
                return Err(SdError::Underrun);
            }
            if st.dabort() || st.idmate() {
                abort_sdmmc1_write(regs);
                return Err(SdError::SoftwareTimeout);
            }
            if st.dataend() {
                break;
            }
            if xfer_start.elapsed() > Duration::from_millis(SD_WRITE_TIMEOUT_MS) {
                abort_sdmmc1_write(regs);
                return Err(SdError::SoftwareTimeout);
            }
            Timer::after_micros(200).await;
        }

        fence(Ordering::Acquire);
        clear_sdmmc1_flags(regs);
        regs.idmactrlr().modify(|w| w.set_idmaen(/* val= */ false));

        if n > 1 {
            if let Err(err) = send_sdmmc1_cmd(
                regs, /* cmdindex= */ 12, /* arg= */ 0, /* waitresp= */ 1,
                /* check_crc= */ true, /* cmdtrans= */ false, /* cmdstop= */ true,
            ) {
                abort_sdmmc1_write(regs);
                return Err(err);
            }
            clear_sdmmc1_flags(regs);
        }

        wait_card_ready_yield(regs, card.rca).await
    }
}

impl AsyncBlock512 for SdStorageIo<'_, '_, '_> {
    type Error = SdError;

    async fn read_blocks(
        &mut self,
        start_lba: u32,
        out: &mut [[u8; 512]],
    ) -> Result<(), Self::Error> {
        let mut lba = start_lba;
        for chunk in out.chunks_mut(DMA_BATCH_SECTORS) {
            let n = chunk.len();
            if n == 1 {
                self.storage.read_block(lba, &mut self.dma_buf[0]).await?;
            } else {
                self.storage
                    .read_blocks(lba, &mut self.dma_buf[..n])
                    .await?;
            }
            for (dst, src) in chunk.iter_mut().zip(self.dma_buf[..n].iter()) {
                dst.copy_from_slice(&src[..]);
            }
            lba = lba.saturating_add(n as u32);
        }
        Ok(())
    }

    async fn write_blocks(
        &mut self,
        start_lba: u32,
        data: &[[u8; 512]],
    ) -> Result<(), Self::Error> {
        let mut lba = start_lba;
        for chunk in data.chunks(DMA_BATCH_SECTORS) {
            let n = chunk.len();
            for (dst, src) in self.dma_buf[..n].iter_mut().zip(chunk.iter()) {
                dst.copy_from_slice(src);
            }
            self.write_chunk_nonblocking(lba, n).await?;
            lba = lba.saturating_add(n as u32);
            if self.yield_every_batch {
                // Keeps USB CDC and 1 kHz tasks responsive during ~1,800-batch 64 GB FAT clears.
                Timer::after_ticks(1).await;
            }
        }
        Ok(())
    }
}

fn map_disk_kind(kind: DiskLayoutKind) -> SdDiskKind {
    match kind {
        DiskLayoutKind::Blank => SdDiskKind::Blank,
        DiskLayoutKind::MbrFat32 => SdDiskKind::MbrFat32,
        DiskLayoutKind::MbrExFat => SdDiskKind::MbrExFat,
        DiskLayoutKind::MbrNtfs => SdDiskKind::MbrNtfs,
        DiskLayoutKind::MbrLinux => SdDiskKind::MbrLinux,
        DiskLayoutKind::MbrOther(t) => SdDiskKind::MbrOther(t),
        DiskLayoutKind::Gpt => SdDiskKind::Gpt,
        DiskLayoutKind::SuperfloppyFat32 => SdDiskKind::SuperfloppyFat32,
        DiskLayoutKind::SuperfloppyExFat => SdDiskKind::SuperfloppyExFat,
        DiskLayoutKind::Unknown => SdDiskKind::Unknown,
    }
}

fn sd_err_code(err: SdError) -> u8 {
    match err {
        SdError::Timeout => 1,
        SdError::SoftwareTimeout => 2,
        SdError::UnsupportedCardVersion => 3,
        SdError::UnsupportedCardType => 4,
        SdError::UnsupportedVoltage => 5,
        SdError::Crc => 6,
        SdError::NoCard => 7,
        SdError::BusWidth => 8,
        SdError::BadClock => 9,
        SdError::SignalingSwitchFailed => 10,
        SdError::Underrun => 11,
        _ => 12,
    }
}

fn fat_err_code(err: Fat32Error<SdError>) -> u8 {
    match err {
        Fat32Error::Io(e) => sd_err_code(e),
        Fat32Error::CardTooSmall => 20,
        Fat32Error::NotFat32(_) => 21,
        Fat32Error::InvalidBpb => 22,
        Fat32Error::DiskFull => 23,
        Fat32Error::DirectoryFull => 24,
        Fat32Error::FileNotFound => 25,
        Fat32Error::CorruptFat => 26,
        Fat32Error::VerifyFailed => 27,
        _ => 29,
    }
}

fn set_status_card_absent() {
    LOGGING_ENABLED.store(false, Ordering::Relaxed);
    STATUS_SNAPSHOT.lock(|cell| {
        let mut s = *cell.borrow();
        s.card_initialized = false;
        s.fat32_mounted = false;
        s.logging_active = false;
        s.active_file_len = 0;
        *cell.borrow_mut() = s;
    });
}

async fn try_acquire_card<'s>(
    sdmmc: &'s mut Sdmmc<'static>,
    cmd_block: &mut CmdBlock,
) -> Result<StorageDevice<'s, 'static, embassy_stm32::sdmmc::sd::Card>, SdError> {
    // Wait: borrow cannot return out of a retry loop neatly in a single branch without re-acquiring,
    // so run 25 MHz probe first via StorageDevice::new_uninit_sd_card + reacquire fallback at 12 MHz.
    let mut dev = StorageDevice::new_uninit_sd_card(sdmmc);
    match dev.reacquire(cmd_block, Hertz(25_000_000)).await {
        Ok(()) => Ok(dev),
        Err(SdError::Crc | SdError::Timeout | SdError::SoftwareTimeout) => {
            dev.reacquire(cmd_block, Hertz(12_000_000)).await?;
            Ok(dev)
        }
        Err(err) => Err(err),
    }
}

async fn handle_list_files(
    vol: Option<&Fat32Volume>,
    io: &mut SdStorageIo<'_, '_, '_>,
    offset: u16,
) -> SdCmdReply {
    let Some(v) = vol else {
        return SdCmdReply::Err(ProtoError::SdNotReady);
    };
    let mut raw_entries = [DirEntryInfo {
        name_8_3: [0; 11],
        first_cluster: 0,
        size_bytes: 0,
        is_dir: false,
    }; 8];
    match v.list_root(io, offset as usize, &mut raw_entries).await {
        Ok((total, count)) => {
            let mut entries = Vec::<SdDirEntry, 8>::new();
            for item in raw_entries.iter().take(count) {
                let (name, name_len) = format_8_3_name(&item.name_8_3);
                let _ = entries.push(SdDirEntry {
                    name,
                    name_len,
                    size_bytes: item.size_bytes,
                });
            }
            SdCmdReply::FileList {
                total_files: total.min(u16::MAX as usize) as u16,
                entries,
            }
        }
        Err(_) => SdCmdReply::Err(ProtoError::SdIoFailed),
    }
}

async fn handle_read_chunk(
    vol: Option<&Fat32Volume>,
    io: &mut SdStorageIo<'_, '_, '_>,
    name_8_3: [u8; 11],
    offset: u32,
    len: u16,
) -> SdCmdReply {
    let Some(v) = vol else {
        return SdCmdReply::Err(ProtoError::SdNotReady);
    };
    let req_len = (len as usize).min(MAX_CHUNK_BYTES);
    let mut buf = [0u8; MAX_CHUNK_BYTES];
    match v
        .read_file_chunk(io, name_8_3, offset, &mut buf[..req_len])
        .await
    {
        Ok((read_n, total_size)) => {
            let mut data = Vec::<u8, MAX_CHUNK_BYTES>::new();
            let _ = data.extend_from_slice(&buf[..read_n]);
            SdCmdReply::FileChunk {
                offset,
                total_size,
                data,
            }
        }
        Err(Fat32Error::FileNotFound) => SdCmdReply::Err(ProtoError::SdFileNotFound),
        Err(_) => SdCmdReply::Err(ProtoError::SdIoFailed),
    }
}

async fn open_next_bdl_file(
    vol: &mut Fat32Volume,
    io: &mut SdStorageIo<'_, '_, '_>,
) -> Result<(Fat32FileWriter, [u8; 12], u8), Fat32Error<SdError>> {
    let seq = vol.next_log_sequence(io).await?;
    let name_8_3 = format_log_filename_8_3(seq);
    let (pretty_name, pretty_len) = format_8_3_name(&name_8_3);
    let ts = now_timestamp();
    let writer = vol.create_file(io, name_8_3, ts).await?;

    let hdr = DatalogFileHeader::new(ts, LAST_CAL_CRC.load(Ordering::Relaxed));
    LOG_RING.lock(|cell| {
        let mut ring = cell.borrow_mut();
        ring.clear();
        ring.push_block(hdr.to_bytes());
    });

    Ok((writer, pretty_name, pretty_len))
}

/// Parses an ASCII slice filename sent by RPC (`LOG00001.BDL`) into 8.3 format.
pub fn parse_rpc_filename(raw: &[u8; 12], len: u8) -> Option<[u8; 11]> {
    let n = (len as usize).min(raw.len());
    let s = core::str::from_utf8(&raw[..n]).ok()?;
    parse_filename_to_8_3(s)
}

/// Async MicroSD task owning `SDMMC1`, handling hot-plug, FAT32 formatting, and `500 Hz` writes.
#[embassy_executor::task]
pub async fn sd_logger_task(mut sdmmc: Sdmmc<'static>, pd4_detect: Input<'static>) -> ! {
    let dma_buf = DMA_SCRATCH.init([const { DataBlock::new() }; DMA_BATCH_SECTORS]);
    let io_scratch = IO_SCRATCH.init([[0u8; 512]; DMA_BATCH_SECTORS]);
    let mut cmd_block = CmdBlock::new();
    let mut logged_no_card_notice = false;

    loop {
        let pd4_high = pd4_detect.is_high();
        PD4_PIN_HIGH.store(pd4_high, Ordering::Relaxed);

        // Drain pending RPC commands cleanly while card init isn't ready yet.
        let mut pending_fmt = false;
        while let Ok(cmd) = SD_CMD_CH.try_receive() {
            match cmd {
                SdCmd::FormatFat32 => {
                    pending_fmt = true;
                    break;
                }
                _ => SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdNotReady)),
            }
        }

        let err = match try_acquire_card(&mut sdmmc, &mut cmd_block).await {
            Ok(mut storage) => {
                logged_no_card_notice = false;
                LAST_ERROR_CODE.store(0, Ordering::Relaxed);
                let initial_cmd = if pending_fmt {
                    Some(SdCmd::FormatFat32)
                } else {
                    None
                };
                run_card_session(&mut storage, dma_buf, io_scratch, &pd4_detect, initial_cmd).await;
                set_status_card_absent();
                Timer::after_millis(250).await;
                continue;
            }
            Err(err) => err,
        };

        LAST_ERROR_CODE.store(sd_err_code(err), Ordering::Relaxed);
        set_status_card_absent();
        if pending_fmt {
            SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdNotReady));
        }
        if !logged_no_card_notice {
            info!(
                "SDMMC1 waiting for MicroSD card (pd4_high={}, err={})",
                pd4_high, err
            );
            logged_no_card_notice = true;
        }
        if let Ok(cmd) =
            with_timeout(Duration::from_millis(RETRY_CARD_MS), SD_CMD_CH.receive()).await
        {
            if matches!(cmd, SdCmd::FormatFat32) {
                if let Ok(mut dev) = try_acquire_card(&mut sdmmc, &mut cmd_block).await {
                    run_card_session(
                        &mut dev,
                        dma_buf,
                        io_scratch,
                        &pd4_detect,
                        Some(SdCmd::FormatFat32),
                    )
                    .await;
                    set_status_card_absent();
                } else {
                    SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdNotReady));
                }
            } else {
                SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdNotReady));
            }
        }
    }
}

async fn run_card_session(
    storage: &mut StorageDevice<'_, 'static, embassy_stm32::sdmmc::sd::Card>,
    dma_buf: &mut [DataBlock; DMA_BATCH_SECTORS],
    io_scratch: &mut [[u8; 512]; DMA_BATCH_SECTORS],
    pd4_detect: &Input<'static>,
    initial_cmd: Option<SdCmd>,
) {
    let card = storage.card();
    let total_sectors = card.csd.block_count();
    let capacity_mb = (total_sectors / 2_048).min(u32::MAX as u64) as u32;
    let mut prod_name = [b' '; 5];
    for (dst, src) in prod_name
        .iter_mut()
        .zip(card.cid.product_name().as_bytes().iter())
    {
        *dst = *src;
    }

    let mut io = SdStorageIo {
        storage,
        dma_buf,
        yield_every_batch: false,
    };

    let probe = match probe_disk(&mut io, total_sectors).await {
        Ok(p) => p,
        Err(err) => {
            LAST_ERROR_CODE.store(fat_err_code(err), Ordering::Relaxed);
            if initial_cmd.is_some() {
                SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdIoFailed));
            }
            return;
        }
    };
    let disk_kind = map_disk_kind(probe.layout);

    STATUS_SNAPSHOT.lock(|cell| {
        let mut s = *cell.borrow();
        s.card_initialized = true;
        s.disk_kind = disk_kind;
        s.fat32_mounted = false;
        s.logging_active = false;
        s.card_prod_name = prod_name;
        s.capacity_mb = capacity_mb;
        s.bytes_written = 0;
        s.records_logged = 0;
        s.write_errors = 0;
        *cell.borrow_mut() = s;
    });

    info!(
        "MicroSD detected: {} MB | layout={:?} | pd4_high={}",
        capacity_mb,
        disk_kind,
        pd4_detect.is_high()
    );

    let mut mounted: Option<(Fat32Volume, Fat32FileWriter)> = None;
    let mut consecutive_io_errors = 0u8;
    let mut last_write_at = Instant::now();
    let mut last_sync_at = Instant::now();
    let mut prev_pd4 = pd4_detect.is_high();

    if let Some(cmd) = initial_cmd {
        execute_session_cmd(
            cmd,
            &mut io,
            total_sectors,
            io_scratch,
            &mut mounted,
            prod_name,
            capacity_mb,
        )
        .await;
    } else if matches!(
        probe.layout,
        DiskLayoutKind::MbrFat32 | DiskLayoutKind::SuperfloppyFat32
    ) {
        try_mount_and_open_log(&mut io, total_sectors, &mut mounted).await;
    } else {
        warn!(
            "MicroSD is not FAT32 ({:?}); run `ecu-cli sd-format --yes` to format",
            disk_kind
        );
    }

    loop {
        let now_pd4 = pd4_detect.is_high();
        PD4_PIN_HIGH.store(now_pd4, Ordering::Relaxed);
        // Only treat LOW -> HIGH transition as physical eject if pin actually started LOW (`SB2` closed).
        if !prev_pd4 && now_pd4 {
            info!("PD4 card-detect went HIGH — MicroSD ejected");
            if let Some((ref vol, ref writer)) = mounted {
                let _ = writer.sync_metadata(vol, &mut io, now_timestamp()).await;
            }
            LOGGING_ENABLED.store(false, Ordering::Relaxed);
            break;
        }
        prev_pd4 = now_pd4;

        if let Some((ref mut vol, ref mut writer)) = mounted {
            let queued = LOG_RING.lock(|cell| cell.borrow().len());
            let should_flush = queued >= RECORDS_PER_SECTOR * DMA_BATCH_SECTORS
                || (queued >= RECORDS_PER_SECTOR
                    && last_write_at.elapsed() >= Duration::from_millis(FLUSH_IDLE_MS));

            if should_flush {
                let sec_count = LOG_RING.lock(|cell| cell.borrow_mut().pop_sectors(io_scratch));
                if sec_count > 0 {
                    let ts = now_timestamp();
                    match writer
                        .write_sectors(vol, &mut io, &io_scratch[..sec_count], ts)
                        .await
                    {
                        Ok(()) => {
                            consecutive_io_errors = 0;
                            last_write_at = Instant::now();
                            let recs =
                                (writer.bytes_written / LOG_RECORD_BYTES as u32).saturating_sub(1);
                            STATUS_SNAPSHOT.lock(|cell| {
                                let mut s = *cell.borrow();
                                s.bytes_written = writer.bytes_written;
                                s.records_logged = recs;
                                *cell.borrow_mut() = s;
                            });
                        }
                        Err(err) => {
                            LAST_ERROR_CODE.store(fat_err_code(err), Ordering::Relaxed);
                            consecutive_io_errors = consecutive_io_errors.saturating_add(1);
                            STATUS_SNAPSHOT.lock(|cell| {
                                let mut s = *cell.borrow();
                                s.write_errors = s.write_errors.saturating_add(1);
                                *cell.borrow_mut() = s;
                            });
                            if consecutive_io_errors >= 3 {
                                warn!("SDMMC1 write failed 3x — resetting card session");
                                LOGGING_ENABLED.store(false, Ordering::Relaxed);
                                break;
                            }
                        }
                    }
                }
            }

            if last_sync_at.elapsed() >= Duration::from_millis(METADATA_SYNC_MS) {
                last_sync_at = Instant::now();
                let ts = now_timestamp();
                if let Err(err) = writer.sync_metadata(vol, &mut io, ts).await {
                    LAST_ERROR_CODE.store(fat_err_code(err), Ordering::Relaxed);
                }
            }
        }

        if let Ok(cmd) = with_timeout(Duration::from_millis(25), SD_CMD_CH.receive()).await {
            execute_session_cmd(
                cmd,
                &mut io,
                total_sectors,
                io_scratch,
                &mut mounted,
                prod_name,
                capacity_mb,
            )
            .await;
            last_write_at = Instant::now();
            last_sync_at = Instant::now();
        }
    }
}

async fn try_mount_and_open_log(
    io: &mut SdStorageIo<'_, '_, '_>,
    total_sectors: u64,
    mounted: &mut Option<(Fat32Volume, Fat32FileWriter)>,
) {
    match Fat32Volume::mount(io, total_sectors).await {
        Ok(mut vol) => match open_next_bdl_file(&mut vol, io).await {
            Ok((writer, pretty_name, pretty_len)) => {
                LOGGING_ENABLED.store(true, Ordering::Relaxed);
                STATUS_SNAPSHOT.lock(|cell| {
                    let mut s = *cell.borrow();
                    s.disk_kind = SdDiskKind::MbrFat32;
                    s.fat32_mounted = true;
                    s.logging_active = true;
                    s.active_file = pretty_name;
                    s.active_file_len = pretty_len;
                    s.bytes_written = writer.bytes_written;
                    s.records_logged = 0;
                    *cell.borrow_mut() = s;
                });
                let name_str =
                    core::str::from_utf8(&pretty_name[..pretty_len as usize]).unwrap_or("LOG.BDL");
                info!(
                    "FAT32 mounted ({} clusters, {} KB/clus); logging @ 500 Hz -> {}",
                    vol.cluster_count,
                    (vol.sec_per_clus as u32) / 2,
                    name_str
                );
                *mounted = Some((vol, writer));
            }
            Err(err) => {
                LAST_ERROR_CODE.store(fat_err_code(err), Ordering::Relaxed);
                warn!("FAT32 mounted but file create failed");
            }
        },
        Err(err) => {
            LAST_ERROR_CODE.store(fat_err_code(err), Ordering::Relaxed);
            warn!("FAT32 mount failed");
        }
    }
}

async fn execute_session_cmd(
    cmd: SdCmd,
    io: &mut SdStorageIo<'_, '_, '_>,
    total_sectors: u64,
    io_scratch: &mut [[u8; 512]; DMA_BATCH_SECTORS],
    mounted: &mut Option<(Fat32Volume, Fat32FileWriter)>,
    prod_name: [u8; 5],
    capacity_mb: u32,
) {
    // Sync metadata before reading directory/chunks so host sees latest written byte count.
    if let Some((ref vol, ref writer)) = *mounted {
        let _ = writer.sync_metadata(vol, io, now_timestamp()).await;
    }

    match cmd {
        SdCmd::ListFiles { offset } => {
            let reply = handle_list_files(mounted.as_ref().map(|(v, _)| v), io, offset).await;
            SD_RESP_SIG.signal(reply);
        }
        SdCmd::ReadFileChunk {
            name_8_3,
            offset,
            len,
        } => {
            let reply =
                handle_read_chunk(mounted.as_ref().map(|(v, _)| v), io, name_8_3, offset, len)
                    .await;
            SD_RESP_SIG.signal(reply);
        }
        SdCmd::FormatFat32 => {
            if LAST_RPM.load(Ordering::Relaxed) > 0 {
                SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdFormatRejectedEngineRunning));
                return;
            }
            LOGGING_ENABLED.store(false, Ordering::Relaxed);
            FORMAT_RUNNING.store(true, Ordering::Relaxed);
            *mounted = None;
            LOG_RING.lock(|cell| cell.borrow_mut().clear());
            STATUS_SNAPSHOT.lock(|cell| {
                let mut s = *cell.borrow();
                s.fat32_mounted = false;
                s.logging_active = false;
                s.active_file_len = 0;
                s.bytes_written = 0;
                s.records_logged = 0;
                *cell.borrow_mut() = s;
            });

            info!("Formatting MicroSD ({} MB) as MBR + FAT32...", capacity_mb);
            let ts = now_timestamp();
            io.yield_every_batch = true;
            let fmt_res = format_mbr_fat32(io, total_sectors, ts, io_scratch).await;
            io.yield_every_batch = false;
            FORMAT_RUNNING.store(false, Ordering::Relaxed);

            match fmt_res {
                Ok(vol) => {
                    LAST_ERROR_CODE.store(0, Ordering::Relaxed);
                    STATUS_SNAPSHOT.lock(|cell| {
                        let mut s = *cell.borrow();
                        s.card_initialized = true;
                        s.card_prod_name = prod_name;
                        s.capacity_mb = capacity_mb;
                        s.disk_kind = SdDiskKind::MbrFat32;
                        s.fat32_mounted = true;
                        *cell.borrow_mut() = s;
                    });
                    info!(
                        "FAT32 format & read-back verify PASSED ({} clusters)!",
                        vol.cluster_count
                    );
                    try_mount_and_open_log(io, total_sectors, mounted).await;
                    SD_RESP_SIG.signal(SdCmdReply::Ok);
                }
                Err(err) => {
                    LAST_ERROR_CODE.store(fat_err_code(err), Ordering::Relaxed);
                    warn!("FAT32 format FAILED (code={})", fat_err_code(err));
                    SD_RESP_SIG.signal(SdCmdReply::Err(ProtoError::SdIoFailed));
                }
            }
        }
    }
}
