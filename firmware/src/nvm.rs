//! Non-volatile calibration & runtime configuration storage on onboard `SPI1` JEDEC NOR Flash
//! (`Puya PY25Q64HA` / `Winbond W25Q64`) or Stage 2 SPI FRAM (`FM25V02A` / `MB85RS256B`).
//!
//! Uses dual-slot ping-pong regions with monotonic generation sequence numbers, `CRC32`,
//! commit-last headers, and read-back verification so sudden power loss during erase/program or
//! FRAM byte writes never corrupts the active slot:
//! - Calibration (`2,816 B` `CalibrationBlob`): `0x0000` & `0x1000`
//! - Runtime Config (`KnockConfig` + `AuxIoConfig`): `0x2000` & `0x3000`

use defmt::{info, warn};
use ecu_core::nvm_chip::{
    CAL_SLOT_ADDRS, CFG_SLOT_ADDRS, SLOT_HEADER_BYTES, SpiNvmGeometry, identify_rdid,
};
use ecu_types::{
    CalibrationBlob, NvmMedium, NvmStatusReport, RUNTIME_CONFIG_MAGIC, RUNTIME_CONFIG_VERSION,
    RuntimeConfigBlob, crc32,
};
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::{Config as SpiConfig, Error as SpiError, MODE_0, Spi};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use proto_native::{MAX_RUNTIME_CONFIG_BYTES, decode_runtime_config, encode_runtime_config};
use proto_tunerstudio::{TS_PAGE_SIZE, pack_page, unpack_page_into};

const CMD_PAGE_PROGRAM: u8 = 0x02;
const CMD_READ_DATA: u8 = 0x03;
const CMD_READ_STATUS_1: u8 = 0x05;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_SECTOR_ERASE_4K: u8 = 0x20;
const CMD_READ_JEDEC_ID: u8 = 0x9f;
const CMD_RELEASE_POWERDOWN: u8 = 0xab;

const STATUS_BUSY_MASK: u8 = 0x01;
const STATUS_WEL_MASK: u8 = 0x02;

const CAL_NVM_MAGIC: u32 = 0x4349_564e;
const CAL_SCHEMA_VERSION: u16 = 1;
const HEADER_SIZE: usize = SLOT_HEADER_BYTES;
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
    magic: u32,
    schema_version: u16,
    payload_len: u16,
    seq_gen: u32,
    payload_crc32: u32,
}

impl SlotHeader {
    fn from_bytes(buf: &[u8; HEADER_SIZE], expected_magic: u32) -> Option<Self> {
        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        if magic != expected_magic {
            return None;
        }
        Some(Self {
            magic,
            schema_version: u16::from_le_bytes([buf[4], buf[5]]),
            payload_len: u16::from_le_bytes([buf[6], buf[7]]),
            seq_gen: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            payload_crc32: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
        })
    }

    fn to_bytes(self) -> [u8; HEADER_SIZE] {
        let mut buf = [0u8; HEADER_SIZE];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4..6].copy_from_slice(&self.schema_version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.payload_len.to_le_bytes());
        buf[8..12].copy_from_slice(&self.seq_gen.to_le_bytes());
        buf[12..16].copy_from_slice(&self.payload_crc32.to_le_bytes());
        buf
    }
}

#[derive(Clone, Copy)]
struct PingPongRegion {
    sectors: [u32; 2],
    magic: u32,
    schema_version: u16,
    active_slot: Option<usize>,
    active_seq: u32,
}

impl PingPongRegion {
    const fn new(sectors: [u32; 2], magic: u32, schema_version: u16) -> Self {
        Self {
            sectors,
            magic,
            schema_version,
            active_slot: None,
            active_seq: 0,
        }
    }
}

#[derive(Clone, Copy)]
enum RegionId {
    Calibration,
    RuntimeConfig,
}

/// Manages auto-detected `SPI1` NOR Flash (`PY25Q64HA`/`W25Q64`) or FRAM (`FM25V02A`/`MB85RS256B`)
/// dual-slot persistence.
pub struct NvmStorage<'d> {
    spi: Spi<'d, Blocking, Master>,
    cs_n: Output<'d>,
    rdid: [u8; 9],
    geometry: Option<SpiNvmGeometry>,
    cal_region: PingPongRegion,
    cfg_region: PingPongRegion,
}

impl<'d> NvmStorage<'d> {
    /// Initializes `SPI1` (`PB3` SCK, `PD7` MOSI, `PB4` MISO, `PD6` CS#) and auto-detects Flash vs FRAM.
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
            rdid: [0; 9],
            geometry: None,
            cal_region: PingPongRegion::new(CAL_SLOT_ADDRS, CAL_NVM_MAGIC, CAL_SCHEMA_VERSION),
            cfg_region: PingPongRegion::new(
                CFG_SLOT_ADDRS,
                RUNTIME_CONFIG_MAGIC,
                RUNTIME_CONFIG_VERSION,
            ),
        };

        // Release Flash deep power-down (`0xAB`) AND pulse `CS#` to wake FRAM (`FM25V02A` tREC <= 400 us).
        // `0xAB` is an unassigned non-write opcode on `FM25V02A` / `MB85RS256B`.
        let _ = nvm.with_cs(|spi| spi.blocking_write(&[CMD_RELEASE_POWERDOWN, 0, 0, 0]));
        Timer::after_micros(500).await;

        let mut raw_id = [0u8; 9];
        if nvm.read_rdid_9(&mut raw_id).is_ok()
            && let Some(geom) = identify_rdid(&raw_id)
        {
            nvm.rdid = raw_id;
            nvm.geometry = Some(geom);
            match geom.medium {
                NvmMedium::SpiFram => info!(
                    "SPI1 FRAM detected | size={} KB addr={}B rdid={=u8:02x}-{=u8:02x}-{=u8:02x}-{=u8:02x}",
                    geom.capacity_bytes / 1024,
                    geom.addr_bytes,
                    raw_id[0],
                    raw_id[6],
                    raw_id[7],
                    raw_id[2],
                ),
                NvmMedium::SpiNorFlash => info!(
                    "SPI1 NOR Flash detected | size={} KB JEDEC={=u8:02x}-{=u8:02x}-{=u8:02x}",
                    geom.capacity_bytes / 1024,
                    raw_id[0],
                    raw_id[1],
                    raw_id[2]
                ),
                NvmMedium::None => {}
            }
        } else {
            warn!(
                "SPI1 NVM absent or unsupported \
                 (RDID={=u8:02x}-{=u8:02x}-{=u8:02x}-{=u8:02x}); running RAM-only",
                raw_id[0], raw_id[1], raw_id[2], raw_id[3]
            );
        }
        nvm
    }

    /// Loads the newest CRC-validated `CalibrationBlob` from Flash, or `Default` on blank chip.
    pub async fn load_calibration(&mut self, scratch: &mut [u8; TS_PAGE_SIZE]) -> CalibrationBlob {
        if let Some((slot_idx, seq_gen, _)) = self
            .load_best_slot(RegionId::Calibration, scratch, Some(TS_PAGE_SIZE))
            .await
        {
            let mut cal = CalibrationBlob::default_honda_obd1();
            unpack_page_into(scratch, &mut cal);
            if cal.validate_header() {
                self.cal_region.active_slot = Some(slot_idx);
                self.cal_region.active_seq = seq_gen;
                info!("NVM cal loaded slot {} (seq={}, crc=ok)", slot_idx, seq_gen);
                return cal;
            }
        }

        info!("NVM cal blank or uninitialized; using default Honda OBD1 base map");
        CalibrationBlob::default_honda_obd1()
    }

    /// Erases and programs the alternate 4 KB Flash sector for `CalibrationBlob`.
    pub async fn save_calibration(
        &mut self,
        cal: &CalibrationBlob,
        scratch: &mut [u8; TS_PAGE_SIZE],
    ) -> Result<(), NvmError> {
        pack_page(cal, scratch);
        self.save_region_payload(RegionId::Calibration, scratch)
            .await
    }

    /// Loads the newest CRC-validated [`RuntimeConfigBlob`] (`KnockConfig` + `AuxIoConfig`), or
    /// stock defaults on a blank chip.
    pub async fn load_runtime_config(&mut self) -> RuntimeConfigBlob {
        let mut buf = [0u8; MAX_RUNTIME_CONFIG_BYTES];
        if let Some((slot_idx, seq_gen, len)) = self
            .load_best_slot(RegionId::RuntimeConfig, &mut buf, None)
            .await
            && let Some(cfg) = decode_runtime_config(&buf[..len])
        {
            self.cfg_region.active_slot = Some(slot_idx);
            self.cfg_region.active_seq = seq_gen;
            info!(
                "NVM runtime cfg loaded slot {} (seq={}, crc=ok)",
                slot_idx, seq_gen
            );
            return cfg;
        }

        info!("NVM runtime cfg blank; using default Knock & Aux I/O settings");
        RuntimeConfigBlob::default_honda_obd1()
    }

    /// Persists [`RuntimeConfigBlob`] (`KnockConfig` + `AuxIoConfig`) into dedicated sectors.
    pub async fn save_runtime_config(&mut self, cfg: &RuntimeConfigBlob) -> Result<(), NvmError> {
        let mut buf = [0u8; MAX_RUNTIME_CONFIG_BYTES];
        let len = encode_runtime_config(cfg, &mut buf).map_err(|_| NvmError::VerifyFailed)?;
        self.save_region_payload(RegionId::RuntimeConfig, &buf[..len])
            .await
    }

    pub fn status_report(&self) -> NvmStatusReport {
        let (detected, medium, rdid_len, addr_bytes, capacity_bytes) = match self.geometry {
            Some(g) => (true, g.medium, g.rdid_len, g.addr_bytes, g.capacity_bytes),
            None => (false, NvmMedium::None, 0, 0, 0),
        };
        NvmStatusReport {
            detected,
            medium,
            rdid: self.rdid,
            rdid_len,
            addr_bytes,
            capacity_bytes,
            cal_active_slot: self.cal_region.active_slot.map(|s| s as u8),
            cal_active_seq: self.cal_region.active_seq,
            cfg_active_slot: self.cfg_region.active_slot.map(|s| s as u8),
            cfg_active_seq: self.cfg_region.active_seq,
        }
    }

    fn region(&self, id: RegionId) -> PingPongRegion {
        match id {
            RegionId::Calibration => self.cal_region,
            RegionId::RuntimeConfig => self.cfg_region,
        }
    }

    fn region_mut(&mut self, id: RegionId) -> &mut PingPongRegion {
        match id {
            RegionId::Calibration => &mut self.cal_region,
            RegionId::RuntimeConfig => &mut self.cfg_region,
        }
    }

    async fn load_best_slot(
        &mut self,
        id: RegionId,
        scratch: &mut [u8],
        exact_len: Option<usize>,
    ) -> Option<(usize, u32, usize)> {
        self.geometry?;

        let region = self.region(id);
        let mut best: Option<(usize, u32, usize)> = None;
        for (slot_idx, &sector_addr) in region.sectors.iter().enumerate() {
            let mut hdr_bytes = [0u8; HEADER_SIZE];
            if self
                .read_chunked(sector_addr, &mut hdr_bytes)
                .await
                .is_err()
            {
                continue;
            }
            let Some(hdr) = SlotHeader::from_bytes(&hdr_bytes, region.magic) else {
                continue;
            };
            let len = hdr.payload_len as usize;
            let len_ok = match exact_len {
                Some(req) => len == req,
                None => len > 0 && len <= scratch.len(),
            };
            let version_ok = match id {
                RegionId::Calibration => hdr.schema_version == region.schema_version,
                // Allow legacy v1/v2 runtime config slots to migrate losslessly via
                // `decode_runtime_config`.
                RegionId::RuntimeConfig => {
                    (1..=region.schema_version).contains(&hdr.schema_version)
                }
            };
            if !version_ok || !len_ok {
                continue;
            }
            if self
                .read_chunked(sector_addr + HEADER_SIZE as u32, &mut scratch[..len])
                .await
                .is_err()
            {
                continue;
            }
            if crc32(&scratch[..len]) != hdr.payload_crc32 {
                warn!("NVM slot {} failed CRC32; ignoring", slot_idx);
                continue;
            }
            if best.is_none_or(|(_, best_seq, _)| hdr.seq_gen > best_seq) {
                best = Some((slot_idx, hdr.seq_gen, len));
            }
        }

        // Re-populate `scratch` with the highest-sequence slot in case slot 0 won then slot 1 had
        // stale bytes read into `scratch` (even if CRC failed).
        if let Some((slot_idx, _, len)) = best {
            let addr = region.sectors[slot_idx] + HEADER_SIZE as u32;
            self.read_chunked(addr, &mut scratch[..len]).await.ok()?;
        }
        best
    }

    async fn save_region_payload(&mut self, id: RegionId, payload: &[u8]) -> Result<(), NvmError> {
        let geom = self.geometry.ok_or(NvmError::NotDetected)?;

        let region = self.region(id);
        let next_slot = match region.active_slot {
            Some(0) => 1,
            _ => 0,
        };
        let next_seq = region.active_seq.wrapping_add(1).max(1);
        let sector_addr = region.sectors[next_slot];
        if sector_addr.saturating_add(HEADER_SIZE as u32 + payload.len() as u32)
            > geom.capacity_bytes
        {
            return Err(NvmError::VerifyFailed);
        }

        let payload_crc32 = crc32(payload);
        let hdr = SlotHeader {
            magic: region.magic,
            schema_version: region.schema_version,
            payload_len: payload.len() as u16,
            seq_gen: next_seq,
            payload_crc32,
        }
        .to_bytes();

        if geom.requires_erase {
            self.erase_sector_4k(sector_addr).await?;
        } else {
            // Invalidate the inactive FRAM slot header before writing payload so a mid-write
            // brownout leaves the alternate slot intact and cleanly rejects the torn header.
            self.program_range(sector_addr, &[0xffu8; HEADER_SIZE])
                .await?;
        }
        // Write payload before committing the magic/CRC header.
        self.program_range(sector_addr + HEADER_SIZE as u32, payload)
            .await?;
        self.program_range(sector_addr, &hdr).await?;

        let mut verify_hdr = [0u8; HEADER_SIZE];
        self.read_bytes(sector_addr, &mut verify_hdr)?;
        if verify_hdr != hdr {
            return Err(NvmError::VerifyFailed);
        }

        let mut verify_buf = [0u8; WRITE_CHUNK_SIZE];
        let mut offset = 0usize;
        while offset < payload.len() {
            let chunk_len = WRITE_CHUNK_SIZE.min(payload.len() - offset);
            self.read_bytes(
                sector_addr + HEADER_SIZE as u32 + offset as u32,
                &mut verify_buf[..chunk_len],
            )?;
            if verify_buf[..chunk_len] != payload[offset..offset + chunk_len] {
                return Err(NvmError::VerifyFailed);
            }
            offset += chunk_len;
            Timer::after_micros(50).await;
        }

        let reg_mut = self.region_mut(id);
        reg_mut.active_slot = Some(next_slot);
        reg_mut.active_seq = next_seq;
        info!(
            "NVM committed slot {} (seq={}, len={}, crc=0x{:08x})",
            next_slot,
            next_seq,
            payload.len(),
            payload_crc32
        );
        Ok(())
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

    fn send_cmd_addr(
        spi: &mut Spi<'d, Blocking, Master>,
        geom: SpiNvmGeometry,
        opcode: u8,
        addr: u32,
    ) -> Result<(), SpiError> {
        let mut cmd = [0u8; 4];
        let len = geom.format_cmd_addr(opcode, addr, &mut cmd);
        spi.blocking_write(&cmd[..len])
    }

    fn read_rdid_9(&mut self, id: &mut [u8; 9]) -> Result<(), NvmError> {
        let mut buf = [CMD_READ_JEDEC_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        self.with_cs(|spi| spi.blocking_transfer_in_place(&mut buf))?;
        id.copy_from_slice(&buf[1..10]);
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
        let geom = self.geometry.ok_or(NvmError::NotDetected)?;
        self.write_enable()?;
        self.with_cs(|spi| Self::send_cmd_addr(spi, geom, CMD_SECTOR_ERASE_4K, addr))?;
        self.wait_not_busy(250).await
    }

    async fn program_range(&mut self, start_addr: u32, data: &[u8]) -> Result<(), NvmError> {
        let geom = self.geometry.ok_or(NvmError::NotDetected)?;
        let mut offset = 0usize;
        while offset < data.len() {
            let addr = start_addr + offset as u32;
            let chunk_len = geom.write_chunk_limit(addr, WRITE_CHUNK_SIZE, data.len() - offset);

            self.write_enable()?;
            self.with_cs(|spi| {
                Self::send_cmd_addr(spi, geom, CMD_PAGE_PROGRAM, addr)?;
                spi.blocking_write(&data[offset..offset + chunk_len])
            })?;
            if geom.requires_erase {
                self.wait_not_busy(25).await?;
            } else {
                // FRAM writes latch on the last SCK edge with zero busy wait; yield briefly.
                Timer::after_micros(10).await;
            }
            offset += chunk_len;
        }
        Ok(())
    }

    fn read_bytes(&mut self, addr: u32, dst: &mut [u8]) -> Result<(), NvmError> {
        let geom = self.geometry.ok_or(NvmError::NotDetected)?;
        self.with_cs(|spi| {
            Self::send_cmd_addr(spi, geom, CMD_READ_DATA, addr)?;
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
