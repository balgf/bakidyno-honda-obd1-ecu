//! Non-volatile calibration storage on onboard `SPI1` JEDEC NOR Flash
//! (`Puya PY25Q64HA` / `Winbond W25Q64`, 8 MiB, 4 KB erase sectors).
//!
//! Uses ping-pong dual sectors (`0x00_0000` and `0x00_1000`) with monotonic generation sequence
//! numbers and `CRC32` verification so sudden power loss during flash erase/program never
//! corrupts the previously committed tune.

use defmt::{info, warn};
use ecu_types::{CalibrationBlob, crc32};
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::{Config as SpiConfig, Error as SpiError, MODE_0, Spi};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use proto_tunerstudio::{TS_PAGE_SIZE, pack_page, unpack_page_into};

const CMD_WRITE_STATUS_DISABLE_NA: u8 = 0x04;
const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_READ_DATA: u8 = 0x03;
const CMD_READ_STATUS_1: u8 = 0x05;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_SECTOR_ERASE_4K: u8 = 0x20;
const CMD_READ_JEDEC_ID: u8 = 0x9f;
const CMD_RELEASE_POWERDOWN: u8 = 0xab;

const STATUS_BUSY_MASK: u8 = 0x01;
const STATUS_WEL_MASK: u8 = 0x02;

const NVM_MAGIC: u32 = 0x4349_564e;
const NVM_SCHEMA_VERSION: u16 = 1;
const HEADER_SIZE: usize = 16;
const SLOT_SECTOR_ADDRS: [u32; 2] = [0x00_0000, 0x00_1000];
const FLASH_PAGE_SIZE: usize = 256;
const WRITE_CHUNK_SIZE: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum NvmError {
    NotDetected,
    SpiBus,
    WriteEnableRejected,
    EraseProgramTimeout,
    VerifyFailed,
}

#[derive(Debug, Clone, Copy)]
struct SlotHeader {
    schema_version: u16,
    payload_len: u16,
    seq_gen: u32,
    payload_crc32: u32,
}

impl SlotHeader {
    fn from_bytes(buf: &[u8; HEADER_SIZE]) -> Option<Self> {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic != NVM_MAGIC {
            return None;
        }
        Some(Self {
            schema_version: u16::from_le_bytes([buf[4], buf[5]]),
            payload_len: u16::from_le_bytes([buf[6], buf[7]]),
            seq_gen: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            payload_crc32: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
        })
    }

    fn to_bytes(self) -> [u8; HEADER_SIZE] {
        let mut buf = [0u8; HEADER_SIZE];
        buf[0..4].copy_from_slice(&NVM_MAGIC.to_le_bytes());
        buf[4..6].copy_from_slice(&self.schema_version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.payload_len.to_le_bytes());
        buf[8..12].copy_from_slice(&self.seq_gen.to_le_bytes());
        buf[12..16].copy_from_slice(&self.payload_crc32.to_le_bytes());
        buf
    }
}

/// Manages dual-sector ping-pong calibration persistence over `SPI1` NOR Flash.
pub struct NvmFlash<'d> {
    spi: Spi<'d, Blocking, Master>,
    cs_n: Output<'d>,
    jedec_id: [u8; 3],
    detected: bool,
    active_slot: Option<usize>,
    active_seq: u32,
}

impl<'d> NvmFlash<'d> {
    /// Initializes `SPI1` (`PB3` SCK, `PD7` MOSI, `PB4` MISO, `PD6` CS#) and probes JEDEC ID.
    pub async fn init(
        spi1: Peri<'d, peripherals::SPI1>,
        sck: Peri<'d, peripherals::PB3>,
        mosi: Peri<'d, peripherals::PD7>,
        miso: Peri<'d, peripherals::PB4>,
        cs_pin: Peri<'d, peripherals::PD6>,
    ) -> Self {
        let mut cfg = SpiConfig::default();
        cfg.mode = MODE_0;
        cfg.frequency = Hertz(12_000_000);
        cfg.miso_pull = Pull::Up;
        cfg.gpio_speed = Speed::High;

        let spi = Spi::new_blocking(spi1, sck, mosi, miso, cfg);
        let cs_n = Output::new(cs_pin, Level::High, Speed::High);

        let mut nvm = Self {
            spi,
            cs_n,
            jedec_id: [0; 3],
            detected: false,
            active_slot: None,
            active_seq: 0,
        };

        // Wake flash cleanly in case a brownout left it in Deep Power-Down mode.
        let _ = nvm.with_cs(|spi| spi.blocking_write(&[CMD_RELEASE_POWERDOWN, 0, 0, 0]));
        Timer::after_micros(50).await;

        let mut id = [0u8; 3];
        if nvm.read_jedec_id(&mut id).is_ok() && id[0] != 0x00 && id[0] != 0xff {
            nvm.jedec_id = id;
            nvm.detected = true;
            info!(
                "SPI1 NOR Flash detected | JEDEC={=u8:02x}-{=u8:02x}-{=u8:02x}",
                id[0], id[1], id[2]
            );
        } else {
            warn!(
                "SPI1 NOR Flash absent or unreadable \
                 (JEDEC={=u8:02x}-{=u8:02x}-{=u8:02x}); running RAM-only",
                id[0], id[1], id[2]
            );
        }
        let _ = CMD_WRITE_STATUS_DISABLE_NA;
        nvm
    }

    /// Loads the newest CRC-validated `CalibrationBlob` from Flash, or `Default` on blank chip.
    pub async fn load_calibration(&mut self, scratch: &mut [u8; TS_PAGE_SIZE]) -> CalibrationBlob {
        if !self.detected {
            return CalibrationBlob::default_honda_obd1();
        }

        let mut best: Option<(usize, u32)> = None;
        for (slot_idx, &sector_addr) in SLOT_SECTOR_ADDRS.iter().enumerate() {
            let mut hdr_bytes = [0u8; HEADER_SIZE];
            if self
                .read_chunked(sector_addr, &mut hdr_bytes)
                .await
                .is_err()
            {
                continue;
            }
            let Some(hdr) = SlotHeader::from_bytes(&hdr_bytes) else {
                continue;
            };
            if hdr.schema_version != NVM_SCHEMA_VERSION || hdr.payload_len as usize != TS_PAGE_SIZE
            {
                continue;
            }
            if self
                .read_chunked(sector_addr + HEADER_SIZE as u32, scratch)
                .await
                .is_err()
            {
                continue;
            }
            if crc32(scratch) != hdr.payload_crc32 {
                warn!("NVM slot {} failed CRC32; ignoring", slot_idx);
                continue;
            }
            if best.is_none_or(|(_, best_seq)| hdr.seq_gen > best_seq) {
                best = Some((slot_idx, hdr.seq_gen));
            }
        }

        if let Some((slot_idx, seq_gen)) = best {
            let addr = SLOT_SECTOR_ADDRS[slot_idx] + HEADER_SIZE as u32;
            if self.read_chunked(addr, scratch).await.is_ok() {
                let mut cal = CalibrationBlob::default_honda_obd1();
                unpack_page_into(scratch, &mut cal);
                if cal.validate_header() {
                    self.active_slot = Some(slot_idx);
                    self.active_seq = seq_gen;
                    info!("NVM loaded slot {} (seq={}, crc=ok)", slot_idx, seq_gen);
                    return cal;
                }
            }
        }

        info!("NVM blank or uninitialized; using default Honda OBD1 base map");
        CalibrationBlob::default_honda_obd1()
    }

    /// Erases and programs the alternate 4 KB Flash sector non-blockingly (`after_millis(2)`).
    pub async fn save_calibration(
        &mut self,
        cal: &CalibrationBlob,
        scratch: &mut [u8; TS_PAGE_SIZE],
    ) -> Result<(), NvmError> {
        if !self.detected {
            return Err(NvmError::NotDetected);
        }

        let next_slot = match self.active_slot {
            Some(0) => 1,
            _ => 0,
        };
        let next_seq = self.active_seq.wrapping_add(1).max(1);
        let sector_addr = SLOT_SECTOR_ADDRS[next_slot];

        pack_page(cal, scratch);
        let payload_crc32 = crc32(scratch);
        let hdr = SlotHeader {
            schema_version: NVM_SCHEMA_VERSION,
            payload_len: TS_PAGE_SIZE as u16,
            seq_gen: next_seq,
            payload_crc32,
        }
        .to_bytes();

        self.erase_sector_4k(sector_addr).await?;
        self.program_range(sector_addr, &hdr).await?;
        self.program_range(sector_addr + HEADER_SIZE as u32, scratch)
            .await?;

        // Read-back verification guarantees flash bits committed before swapping active slot.
        let mut verify_buf = [0u8; WRITE_CHUNK_SIZE];
        let mut offset = 0usize;
        while offset < TS_PAGE_SIZE {
            let chunk_len = WRITE_CHUNK_SIZE.min(TS_PAGE_SIZE - offset);
            self.read_bytes(
                sector_addr + HEADER_SIZE as u32 + offset as u32,
                &mut verify_buf[..chunk_len],
            )?;
            if verify_buf[..chunk_len] != scratch[offset..offset + chunk_len] {
                return Err(NvmError::VerifyFailed);
            }
            offset += chunk_len;
            Timer::after_millis(1).await;
        }

        self.active_slot = Some(next_slot);
        self.active_seq = next_seq;
        info!(
            "NVM committed slot {} (seq={}, crc=0x{:08x})",
            next_slot, next_seq, payload_crc32
        );
        Ok(())
    }

    pub fn jedec_id(&self) -> [u8; 3] {
        self.jedec_id
    }

    pub fn is_detected(&self) -> bool {
        self.detected
    }

    fn with_cs<R>(
        &mut self,
        op: impl FnOnce(&mut Spi<'d, Blocking, Master>) -> Result<R, SpiError>,
    ) -> Result<R, NvmError> {
        self.cs_n.set_low();
        let res = op(&mut self.spi);
        self.cs_n.set_high();
        res.map_err(|_| NvmError::SpiBus)
    }

    fn read_jedec_id(&mut self, id: &mut [u8; 3]) -> Result<(), NvmError> {
        let mut buf = [CMD_READ_JEDEC_ID, 0, 0, 0];
        self.with_cs(|spi| spi.blocking_transfer_in_place(&mut buf))?;
        id.copy_from_slice(&buf[1..4]);
        Ok(())
    }

    fn read_status(&mut self) -> Result<u8, NvmError> {
        let mut buf = [CMD_READ_STATUS_1, 0];
        self.with_cs(|spi| spi.blocking_transfer_in_place(&mut buf))?;
        Ok(buf[1])
    }

    fn write_enable(&mut self) -> Result<(), NvmError> {
        self.with_cs(|spi| spi.blocking_write(&[CMD_WRITE_ENABLE]))?;
        let sr = self.read_status()?;
        if (sr & STATUS_WEL_MASK) == 0 {
            return Err(NvmError::WriteEnableRejected);
        }
        Ok(())
    }

    async fn wait_not_busy(&mut self, max_polls: u16) -> Result<(), NvmError> {
        for _ in 0..max_polls {
            if (self.read_status()? & STATUS_BUSY_MASK) == 0 {
                return Ok(());
            }
            Timer::after_millis(2).await;
        }
        Err(NvmError::EraseProgramTimeout)
    }

    async fn erase_sector_4k(&mut self, addr: u32) -> Result<(), NvmError> {
        self.write_enable()?;
        let a = addr.to_be_bytes();
        self.with_cs(|spi| spi.blocking_write(&[CMD_SECTOR_ERASE_4K, a[1], a[2], a[3]]))?;
        self.wait_not_busy(250).await
    }

    async fn program_range(&mut self, start_addr: u32, data: &[u8]) -> Result<(), NvmError> {
        let mut offset = 0usize;
        while offset < data.len() {
            let addr = start_addr + offset as u32;
            let page_rem = FLASH_PAGE_SIZE - ((addr as usize) % FLASH_PAGE_SIZE);
            let chunk_len = page_rem.min(WRITE_CHUNK_SIZE).min(data.len() - offset);

            self.write_enable()?;
            let a = addr.to_be_bytes();
            self.with_cs(|spi| {
                spi.blocking_write(&[CMD_PAGE_PROGRAM, a[1], a[2], a[3]])?;
                spi.blocking_write(&data[offset..offset + chunk_len])
            })?;
            self.wait_not_busy(25).await?;
            offset += chunk_len;
        }
        Ok(())
    }

    fn read_bytes(&mut self, addr: u32, dst: &mut [u8]) -> Result<(), NvmError> {
        let a = addr.to_be_bytes();
        self.with_cs(|spi| {
            spi.blocking_write(&[CMD_READ_DATA, a[1], a[2], a[3]])?;
            spi.blocking_read(dst)
        })
    }

    async fn read_chunked(&mut self, start_addr: u32, dst: &mut [u8]) -> Result<(), NvmError> {
        let mut offset = 0usize;
        while offset < dst.len() {
            let len = WRITE_CHUNK_SIZE.min(dst.len() - offset);
            self.read_bytes(start_addr + offset as u32, &mut dst[offset..offset + len])?;
            offset += len;
            Timer::after_micros(10).await;
        }
        Ok(())
    }
}
