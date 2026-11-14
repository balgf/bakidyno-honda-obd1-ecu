//! Pure `no_std`, zero-allocation, zero-`unsafe` MBR + FAT32 disk inspector,
//! formatter (`fatgen103`), and streaming async multi-block log writer.
//!
//! Designed specifically for high-rate (`500 Hz`) `SDMMC1` 4-bit logging on `STM32H743`
//! while remaining 100% host unit-testable over an in-memory block device.

use crate::datalog::RtcTimestamp;

const SECTOR_BYTES: usize = 512;
const PARTITION_START_LBA: u32 = 2048;
const RESERVED_SECTORS: u16 = 32;
const NUM_FATS: u8 = 2;
const BACKUP_BOOT_SEC: u16 = 6;
const FSINFO_SEC: u16 = 1;
const MIN_FAT32_CLUSTERS: u32 = 65_525;
const FAT32_EOC: u32 = 0x0fff_ffff;
const FAT32_MASK: u32 = 0x0fff_ffff;
const MAX_DIR_CLUSTERS: u32 = 256;

/// Abstract `512`-byte multi-block storage interface implemented by `embassy_stm32::sdmmc::sd::StorageDevice`
/// in firmware and by RAM-backed block devices in host unit tests.
#[allow(async_fn_in_trait)]
pub trait AsyncBlock512 {
    type Error: Copy;

    async fn read_blocks(
        &mut self,
        start_lba: u32,
        out: &mut [[u8; 512]],
    ) -> Result<(), Self::Error>;
    async fn write_blocks(&mut self, start_lba: u32, data: &[[u8; 512]])
    -> Result<(), Self::Error>;
}

/// Partition table / filesystem layout detected read-only from LBA `0` and partition VBR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DiskLayoutKind {
    Blank,
    MbrFat32,
    MbrExFat,
    MbrNtfs,
    MbrLinux,
    MbrOther(u8),
    Gpt,
    SuperfloppyFat32,
    SuperfloppyExFat,
    Unknown,
}

/// Read-only inspection result produced by [`probe_disk`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiskProbe {
    pub layout: DiskLayoutKind,
    pub total_sectors: u64,
    pub partition_lba_start: u32,
    pub partition_sectors: u32,
}

/// Errors returned by FAT32 inspection, formatting, mounting, and streaming I/O.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Fat32Error<E> {
    Io(E),
    CardTooSmall,
    EmptyScratchBuffer,
    NotFat32(DiskLayoutKind),
    InvalidBpb,
    DiskFull,
    DirectoryFull,
    FileNotFound,
    CorruptFat,
    VerifyFailed,
    InvalidFilename,
}

impl<E> From<E> for Fat32Error<E> {
    fn from(err: E) -> Self {
        Self::Io(err)
    }
}

/// Compact metadata for one FAT 8.3 root directory file entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirEntryInfo {
    pub name_8_3: [u8; 11],
    pub first_cluster: u32,
    pub size_bytes: u32,
    pub is_dir: bool,
}

/// Mounted FAT32 filesystem geometry and mutable `FSInfo` allocation hint.
#[derive(Debug, Clone, Copy)]
pub struct Fat32Volume {
    pub part_start_lba: u32,
    pub part_sectors: u32,
    pub sec_per_clus: u8,
    pub rsvd_sec_cnt: u16,
    pub num_fats: u8,
    pub fat_sz32: u32,
    pub root_clus: u32,
    pub fsinfo_lba: u32,
    pub fat1_start_lba: u32,
    pub data_start_lba: u32,
    pub cluster_count: u32,
    pub free_count: u32,
    pub nxt_free: u32,
}

/// Open sequential writer targeting one file in the root directory.
#[derive(Debug, Clone, Copy)]
pub struct Fat32FileWriter {
    pub name_8_3: [u8; 11],
    pub dir_entry_lba: u32,
    pub dir_entry_offset: usize,
    pub first_cluster: u32,
    pub current_cluster: u32,
    pub sector_in_cluster: u8,
    pub bytes_written: u32,
}

#[inline]
fn read_le_u16(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

#[inline]
fn read_le_u32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

#[inline]
fn write_le_u16(buf: &mut [u8], off: usize, val: u16) {
    buf[off..off + 2].copy_from_slice(&val.to_le_bytes());
}

#[inline]
fn write_le_u32(buf: &mut [u8], off: usize, val: u32) {
    buf[off..off + 4].copy_from_slice(&val.to_le_bytes());
}

#[inline]
fn is_eoc(cluster_val: u32) -> bool {
    (cluster_val & FAT32_MASK) >= 0x0fff_fff8
}

/// Converts `"LOG00001.BDL"` into canonical uppercase `[u8; 11]` FAT 8.3 storage format.
pub fn parse_filename_to_8_3(name: &str) -> Option<[u8; 11]> {
    let bytes = name.as_bytes();
    if bytes.is_empty() || bytes.len() > 12 {
        return None;
    }
    let dot_pos = bytes.iter().position(|&b| b == b'.');
    let (stem, ext) = match dot_pos {
        Some(pos) => {
            if bytes[pos + 1..].contains(&b'.') {
                return None;
            }
            (&bytes[..pos], &bytes[pos + 1..])
        }
        None => (bytes, &[][..]),
    };
    if stem.is_empty() || stem.len() > 8 || ext.len() > 3 {
        return None;
    }
    let mut out = [b' '; 11];
    for (idx, &ch) in stem.iter().enumerate() {
        let up = ch.to_ascii_uppercase();
        if !(up.is_ascii_alphanumeric() || up == b'_' || up == b'-') {
            return None;
        }
        out[idx] = up;
    }
    for (idx, &ch) in ext.iter().enumerate() {
        let up = ch.to_ascii_uppercase();
        if !(up.is_ascii_alphanumeric() || up == b'_' || up == b'-') {
            return None;
        }
        out[8 + idx] = up;
    }
    Some(out)
}

/// Formats `[u8; 11]` FAT 8.3 name into trimmed `"NAME.EXT"` buffer + length.
pub fn format_8_3_name(raw: &[u8; 11]) -> ([u8; 12], u8) {
    let stem_len = raw[0..8]
        .iter()
        .rposition(|&b| b != b' ')
        .map_or(0, |p| p + 1);
    let ext_len = raw[8..11]
        .iter()
        .rposition(|&b| b != b' ')
        .map_or(0, |p| p + 1);
    let mut out = [0u8; 12];
    let mut n = 0usize;
    out[..stem_len].copy_from_slice(&raw[..stem_len]);
    n += stem_len;
    if ext_len > 0 {
        out[n] = b'.';
        n += 1;
        out[n..n + ext_len].copy_from_slice(&raw[8..8 + ext_len]);
        n += ext_len;
    }
    (out, n as u8)
}

/// Builds sequential `LOGnnnnn.BDL` FAT 8.3 filename (`1..=99_999`).
pub fn format_log_filename_8_3(seq: u32) -> [u8; 11] {
    let clamped = seq.clamp(1, 99_999);
    let mut out = *b"LOG00000BDL";
    let mut rem = clamped;
    for pos in (3..8).rev() {
        out[pos] = b'0' + (rem % 10) as u8;
        rem /= 10;
    }
    out
}

fn parse_log_filename_seq(raw: &[u8; 11]) -> Option<u32> {
    if &raw[0..3] != b"LOG" || &raw[8..11] != b"BDL" {
        return None;
    }
    let mut val = 0u32;
    for &b in &raw[3..8] {
        if !b.is_ascii_digit() {
            return None;
        }
        val = val * 10 + (b - b'0') as u32;
    }
    Some(val)
}

/// Inspects sector `0` (and partition 1 VBR / GPT header) without issuing any writes.
pub async fn probe_disk<IO: AsyncBlock512>(
    io: &mut IO,
    total_sectors: u64,
) -> Result<DiskProbe, Fat32Error<IO::Error>> {
    if total_sectors == 0 {
        return Err(Fat32Error::CardTooSmall);
    }
    let mut sec0 = [[0u8; SECTOR_BYTES]; 1];
    io.read_blocks(0, &mut sec0).await?;
    let s0 = &sec0[0];

    let all_zero = s0.iter().all(|&b| b == 0x00);
    let all_ff = s0.iter().all(|&b| b == 0xff);
    if all_zero || all_ff {
        return Ok(DiskProbe {
            layout: DiskLayoutKind::Blank,
            total_sectors,
            partition_lba_start: 0,
            partition_sectors: 0,
        });
    }

    if s0[510] != 0x55 || s0[511] != 0xaa {
        return Ok(DiskProbe {
            layout: DiskLayoutKind::Unknown,
            total_sectors,
            partition_lba_start: 0,
            partition_sectors: 0,
        });
    }

    // Check unpartitioned superfloppy VBR first before treating offset 446 as MBR entries.
    if matches!(s0[0], 0xeb | 0xe9) {
        if &s0[3..11] == b"EXFAT   " {
            return Ok(DiskProbe {
                layout: DiskLayoutKind::SuperfloppyExFat,
                total_sectors,
                partition_lba_start: 0,
                partition_sectors: total_sectors.min(u32::MAX as u64) as u32,
            });
        }
        if &s0[82..90] == b"FAT32   " {
            return Ok(DiskProbe {
                layout: DiskLayoutKind::SuperfloppyFat32,
                total_sectors,
                partition_lba_start: 0,
                partition_sectors: read_le_u32(s0, 32),
            });
        }
    }

    // Parse 4 primary MBR partition slots.
    let mut chosen: Option<(u8, u32, u32)> = None;
    for slot in 0..4 {
        let base = 446 + slot * 16;
        let ptype = s0[base + 4];
        let lba_start = read_le_u32(s0, base + 8);
        let lba_len = read_le_u32(s0, base + 12);
        if ptype == 0 || lba_len == 0 {
            continue;
        }
        if matches!(ptype, 0x0b | 0x0c) {
            chosen = Some((ptype, lba_start, lba_len));
            break;
        }
        if chosen.is_none() {
            chosen = Some((ptype, lba_start, lba_len));
        }
    }

    let Some((ptype, part_lba, part_len)) = chosen else {
        return Ok(DiskProbe {
            layout: DiskLayoutKind::Unknown,
            total_sectors,
            partition_lba_start: 0,
            partition_sectors: 0,
        });
    };

    if ptype == 0xee {
        return Ok(DiskProbe {
            layout: DiskLayoutKind::Gpt,
            total_sectors,
            partition_lba_start: part_lba,
            partition_sectors: part_len,
        });
    }

    let layout = if (part_lba as u64) < total_sectors {
        let mut vbr = [[0u8; SECTOR_BYTES]; 1];
        io.read_blocks(part_lba, &mut vbr).await?;
        let v = &vbr[0];
        if matches!(ptype, 0x0b | 0x0c) && &v[82..90] == b"FAT32   " {
            DiskLayoutKind::MbrFat32
        } else if ptype == 0x07 && &v[3..11] == b"EXFAT   " {
            DiskLayoutKind::MbrExFat
        } else if ptype == 0x07 && &v[3..11] == b"NTFS    " {
            DiskLayoutKind::MbrNtfs
        } else if &v[82..90] == b"FAT32   " {
            DiskLayoutKind::MbrFat32
        } else if ptype == 0x83 {
            DiskLayoutKind::MbrLinux
        } else {
            DiskLayoutKind::MbrOther(ptype)
        }
    } else {
        DiskLayoutKind::MbrOther(ptype)
    };

    Ok(DiskProbe {
        layout,
        total_sectors,
        partition_lba_start: part_lba,
        partition_sectors: part_len,
    })
}

fn pick_fat32_geometry(part_sectors: u32) -> Option<(u8, u32, u32)> {
    // Try larger cluster sizes first (keeps FAT tables compact & multi-block writes fast)
    // while guaranteeing >= 65,525 clusters (`fatgen103` minimum for FAT32).
    let candidates: [u8; 4] = if part_sectors >= 67_108_864 {
        [64, 32, 16, 8]
    } else if part_sectors >= 16_777_216 {
        [32, 16, 8, 4]
    } else {
        [16, 8, 4, 1]
    };

    for spc in candidates {
        if part_sectors <= RESERVED_SECTORS as u32 {
            return None;
        }
        let tmp1 = (part_sectors - RESERVED_SECTORS as u32) as u64;
        let tmp2 = (256u64 * spc as u64 + NUM_FATS as u64) / 2;
        let fat_sz32 = tmp1.div_ceil(tmp2) as u32;
        let overhead = RESERVED_SECTORS as u64 + NUM_FATS as u64 * fat_sz32 as u64;
        if overhead >= part_sectors as u64 {
            continue;
        }
        let data_sectors = (part_sectors as u64 - overhead) as u32;
        let cluster_count = data_sectors / spc as u32;
        if cluster_count >= MIN_FAT32_CLUSTERS && fat_sz32 * 128 >= cluster_count + 2 {
            return Some((spc, fat_sz32, cluster_count));
        }
    }
    None
}

async fn zero_sector_range<IO: AsyncBlock512>(
    io: &mut IO,
    start_lba: u32,
    count: u32,
    scratch: &mut [[u8; 512]],
) -> Result<(), Fat32Error<IO::Error>> {
    for blk in scratch.iter_mut() {
        blk.fill(0);
    }
    let chunk_cap = scratch.len() as u32;
    let mut done = 0u32;
    while done < count {
        let step = (count - done).min(chunk_cap);
        io.write_blocks(start_lba + done, &scratch[..step as usize])
            .await?;
        done += step;
    }
    Ok(())
}

/// Formats a card cleanly as **MBR Partition 1 (`0x0c` LBA, 1 MiB aligned) + FAT32 (`fatgen103`)**.
///
/// Clears both FAT tables and Root Directory cluster in multi-block chunks (`scratch`), writes
/// Boot + Backup Boot + FSInfo + Backup FSInfo sectors, creates `BAKIDYNO` volume label, and
/// reads back critical structures to detect silent card write failures on aged flash.
pub async fn format_mbr_fat32<IO: AsyncBlock512>(
    io: &mut IO,
    total_sectors: u64,
    ts: RtcTimestamp,
    scratch: &mut [[u8; 512]],
) -> Result<Fat32Volume, Fat32Error<IO::Error>> {
    if scratch.is_empty() {
        return Err(Fat32Error::EmptyScratchBuffer);
    }
    if total_sectors <= (PARTITION_START_LBA as u64 + RESERVED_SECTORS as u64) {
        return Err(Fat32Error::CardTooSmall);
    }
    let part_sectors = (total_sectors - PARTITION_START_LBA as u64).min(u32::MAX as u64) as u32;
    let Some((sec_per_clus, fat_sz32, cluster_count)) = pick_fat32_geometry(part_sectors) else {
        return Err(Fat32Error::CardTooSmall);
    };

    // 1. Write MBR at LBA 0.
    let mut mbr = [0u8; SECTOR_BYTES];
    mbr[446] = 0x00;
    mbr[447..450].copy_from_slice(&[0x20, 0x21, 0x00]); // LBA 2048 CHS start
    mbr[450] = 0x0c; // WIN95 OSR2 FAT32, LBA
    mbr[451..454].copy_from_slice(&[0xfe, 0xff, 0xff]);
    write_le_u32(&mut mbr, 454, PARTITION_START_LBA);
    write_le_u32(&mut mbr, 458, part_sectors);
    mbr[510] = 0x55;
    mbr[511] = 0xaa;
    io.write_blocks(0, &[mbr]).await?;

    // 2. Clear remaining reserved sectors (2..32) before writing VBR/FSInfo & backups.
    zero_sector_range(
        io,
        PARTITION_START_LBA + 2,
        (RESERVED_SECTORS - 2) as u32,
        scratch,
    )
    .await?;

    // 3. Build & write VBR + Backup VBR (+6).
    let mut vbr = [0u8; SECTOR_BYTES];
    vbr[0..3].copy_from_slice(&[0xeb, 0x58, 0x90]);
    vbr[3..11].copy_from_slice(b"MSWIN4.1");
    write_le_u16(&mut vbr, 11, SECTOR_BYTES as u16);
    vbr[13] = sec_per_clus;
    write_le_u16(&mut vbr, 14, RESERVED_SECTORS);
    vbr[16] = NUM_FATS;
    write_le_u16(&mut vbr, 17, 0); // RootEntCnt = 0 on FAT32
    write_le_u16(&mut vbr, 19, 0); // TotSec16 = 0 on FAT32
    vbr[21] = 0xf8; // Fixed/non-volatile media
    write_le_u16(&mut vbr, 22, 0); // FATSz16 = 0 on FAT32
    write_le_u16(&mut vbr, 24, 63);
    write_le_u16(&mut vbr, 26, 255);
    write_le_u32(&mut vbr, 28, PARTITION_START_LBA);
    write_le_u32(&mut vbr, 32, part_sectors);
    write_le_u32(&mut vbr, 36, fat_sz32);
    write_le_u16(&mut vbr, 40, 0); // ExtFlags (mirror active FAT)
    write_le_u16(&mut vbr, 42, 0); // FSVer 0.0
    write_le_u32(&mut vbr, 44, 2); // RootClus = 2
    write_le_u16(&mut vbr, 48, FSINFO_SEC);
    write_le_u16(&mut vbr, 50, BACKUP_BOOT_SEC);
    vbr[64] = 0x80; // BS_DrvNum
    vbr[66] = 0x29; // BS_BootSig
    write_le_u32(&mut vbr, 67, ts.volume_serial());
    vbr[71..82].copy_from_slice(b"BAKIDYNO   ");
    vbr[82..90].copy_from_slice(b"FAT32   ");
    vbr[510] = 0x55;
    vbr[511] = 0xaa;
    io.write_blocks(PARTITION_START_LBA, &[vbr]).await?;
    io.write_blocks(PARTITION_START_LBA + BACKUP_BOOT_SEC as u32, &[vbr])
        .await?;

    // 4. Write FSInfo (+1) and backup FSInfo (+7).
    let mut fsi = [0u8; SECTOR_BYTES];
    write_le_u32(&mut fsi, 0, 0x4161_5252);
    write_le_u32(&mut fsi, 484, 0x6141_7272);
    write_le_u32(&mut fsi, 488, cluster_count.saturating_sub(1));
    write_le_u32(&mut fsi, 492, 3);
    write_le_u32(&mut fsi, 508, 0xaa55_0000);
    io.write_blocks(PARTITION_START_LBA + FSINFO_SEC as u32, &[fsi])
        .await?;
    io.write_blocks(PARTITION_START_LBA + BACKUP_BOOT_SEC as u32 + 1, &[fsi])
        .await?;

    // 5. Clear FAT1 and FAT2, then initialize FAT[0..=2].
    let fat1_lba = PARTITION_START_LBA + RESERVED_SECTORS as u32;
    let fat2_lba = fat1_lba + fat_sz32;
    zero_sector_range(io, fat1_lba, fat_sz32 * 2, scratch).await?;

    let mut fat0 = [0u8; SECTOR_BYTES];
    write_le_u32(&mut fat0, 0, 0x0fff_fff8);
    write_le_u32(&mut fat0, 4, FAT32_EOC);
    write_le_u32(&mut fat0, 8, FAT32_EOC); // Cluster 2 = Root Directory EOC
    io.write_blocks(fat1_lba, &[fat0]).await?;
    io.write_blocks(fat2_lba, &[fat0]).await?;

    // 6. Zero Root Directory cluster (cluster 2) and write volume label entry.
    let data_start_lba = fat1_lba + 2 * fat_sz32;
    zero_sector_range(io, data_start_lba, sec_per_clus as u32, scratch).await?;
    let mut root0 = [0u8; SECTOR_BYTES];
    root0[0..11].copy_from_slice(b"BAKIDYNO   ");
    root0[11] = 0x08; // ATTR_VOLUME_ID
    write_le_u16(&mut root0, 22, ts.to_fat_time());
    write_le_u16(&mut root0, 24, ts.to_fat_date());
    io.write_blocks(data_start_lba, &[root0]).await?;

    // 7. Read back critical metadata sectors to prove card actually retained writes.
    let mut check = [[0u8; SECTOR_BYTES]; 1];
    io.read_blocks(0, &mut check).await?;
    if check[0] != mbr {
        return Err(Fat32Error::VerifyFailed);
    }
    io.read_blocks(PARTITION_START_LBA, &mut check).await?;
    if check[0] != vbr {
        return Err(Fat32Error::VerifyFailed);
    }
    io.read_blocks(fat1_lba, &mut check).await?;
    if check[0] != fat0 {
        return Err(Fat32Error::VerifyFailed);
    }

    Ok(Fat32Volume {
        part_start_lba: PARTITION_START_LBA,
        part_sectors,
        sec_per_clus,
        rsvd_sec_cnt: RESERVED_SECTORS,
        num_fats: NUM_FATS,
        fat_sz32,
        root_clus: 2,
        fsinfo_lba: PARTITION_START_LBA + FSINFO_SEC as u32,
        fat1_start_lba: fat1_lba,
        data_start_lba,
        cluster_count,
        free_count: cluster_count.saturating_sub(1),
        nxt_free: 3,
    })
}

impl Fat32Volume {
    /// Mounts an existing MBR or superfloppy FAT32 volume read-only until writes begin.
    pub async fn mount<IO: AsyncBlock512>(
        io: &mut IO,
        total_sectors: u64,
    ) -> Result<Self, Fat32Error<IO::Error>> {
        let probe = probe_disk(io, total_sectors).await?;
        let part_start_lba = match probe.layout {
            DiskLayoutKind::MbrFat32 | DiskLayoutKind::SuperfloppyFat32 => {
                probe.partition_lba_start
            }
            other => return Err(Fat32Error::NotFat32(other)),
        };

        let mut sec = [[0u8; SECTOR_BYTES]; 1];
        io.read_blocks(part_start_lba, &mut sec).await?;
        let vbr = &sec[0];

        let bytes_per_sec = read_le_u16(vbr, 11);
        let sec_per_clus = vbr[13];
        let rsvd_sec_cnt = read_le_u16(vbr, 14);
        let num_fats = vbr[16];
        let tot_sec32 = read_le_u32(vbr, 32);
        let fat_sz32 = read_le_u32(vbr, 36);
        let root_clus = read_le_u32(vbr, 44);
        let fsinfo_off = read_le_u16(vbr, 48);

        if vbr[510] != 0x55
            || vbr[511] != 0xaa
            || &vbr[82..90] != b"FAT32   "
            || bytes_per_sec != SECTOR_BYTES as u16
            || sec_per_clus == 0
            || !sec_per_clus.is_power_of_two()
            || rsvd_sec_cnt == 0
            || num_fats == 0
            || fat_sz32 == 0
            || root_clus < 2
        {
            return Err(Fat32Error::InvalidBpb);
        }

        let fat1_start_lba = part_start_lba.saturating_add(rsvd_sec_cnt as u32);
        let total_fat_secs = fat_sz32.saturating_mul(num_fats as u32);
        let data_start_lba = fat1_start_lba.saturating_add(total_fat_secs);
        let part_sectors = if tot_sec32 != 0 {
            tot_sec32
        } else {
            probe.partition_sectors
        };
        let overhead = (rsvd_sec_cnt as u32).saturating_add(total_fat_secs);
        if part_sectors <= overhead {
            return Err(Fat32Error::InvalidBpb);
        }
        let cluster_count = (part_sectors - overhead) / sec_per_clus as u32;
        if cluster_count < MIN_FAT32_CLUSTERS {
            return Err(Fat32Error::InvalidBpb);
        }

        let fsinfo_lba = part_start_lba.saturating_add(fsinfo_off as u32);
        let (free_count, nxt_free) = if fsinfo_off != 0 && fsinfo_off < rsvd_sec_cnt {
            io.read_blocks(fsinfo_lba, &mut sec).await?;
            let f = &sec[0];
            if read_le_u32(f, 0) == 0x4161_5252 && read_le_u32(f, 484) == 0x6141_7272 {
                let fc = read_le_u32(f, 488);
                let nf = read_le_u32(f, 492);
                let valid_nf = if nf >= 2 && nf < cluster_count + 2 {
                    nf
                } else {
                    2
                };
                (fc, valid_nf)
            } else {
                (u32::MAX, 2)
            }
        } else {
            (u32::MAX, 2)
        };

        Ok(Self {
            part_start_lba,
            part_sectors,
            sec_per_clus,
            rsvd_sec_cnt,
            num_fats,
            fat_sz32,
            root_clus,
            fsinfo_lba,
            fat1_start_lba,
            data_start_lba,
            cluster_count,
            free_count,
            nxt_free,
        })
    }

    #[inline]
    fn max_cluster_index(&self) -> u32 {
        self.cluster_count.saturating_add(2)
    }

    #[inline]
    fn cluster_to_lba(&self, cluster: u32) -> Option<u32> {
        if cluster < 2 || cluster >= self.max_cluster_index() {
            return None;
        }
        let offset = (cluster - 2) * self.sec_per_clus as u32;
        self.data_start_lba.checked_add(offset)
    }

    async fn read_fat_entry<IO: AsyncBlock512>(
        &self,
        io: &mut IO,
        cluster: u32,
    ) -> Result<u32, Fat32Error<IO::Error>> {
        if cluster >= self.max_cluster_index() {
            return Err(Fat32Error::CorruptFat);
        }
        let fat_sec = cluster / 128;
        let off = ((cluster % 128) * 4) as usize;
        let mut sec = [[0u8; SECTOR_BYTES]; 1];
        io.read_blocks(self.fat1_start_lba + fat_sec, &mut sec)
            .await?;
        Ok(read_le_u32(&sec[0], off) & FAT32_MASK)
    }

    async fn write_fat_entry<IO: AsyncBlock512>(
        &self,
        io: &mut IO,
        cluster: u32,
        new_val: u32,
    ) -> Result<(), Fat32Error<IO::Error>> {
        if cluster < 2 || cluster >= self.max_cluster_index() {
            return Err(Fat32Error::CorruptFat);
        }
        let fat_sec = cluster / 128;
        let off = ((cluster % 128) * 4) as usize;
        let mut sec = [[0u8; SECTOR_BYTES]; 1];
        io.read_blocks(self.fat1_start_lba + fat_sec, &mut sec)
            .await?;
        let existing = read_le_u32(&sec[0], off);
        write_le_u32(
            &mut sec[0],
            off,
            (existing & 0xf000_0000) | (new_val & FAT32_MASK),
        );
        for fat_idx in 0..self.num_fats {
            let lba = self.fat1_start_lba + fat_idx as u32 * self.fat_sz32 + fat_sec;
            io.write_blocks(lba, &sec).await?;
        }
        Ok(())
    }

    async fn alloc_cluster<IO: AsyncBlock512>(
        &mut self,
        io: &mut IO,
        link_from: Option<u32>,
    ) -> Result<u32, Fat32Error<IO::Error>> {
        let max_clus = self.max_cluster_index();
        let total_usable = self.cluster_count;
        if total_usable == 0 {
            return Err(Fat32Error::DiskFull);
        }

        let start = self.nxt_free.clamp(2, max_clus - 1);
        let mut sec = [[0u8; SECTOR_BYTES]; 1];
        let mut cached_fat_sec: Option<u32> = None;

        for step in 0..total_usable {
            let candidate = 2 + ((start - 2 + step) % total_usable);
            let fat_sec = candidate / 128;
            if cached_fat_sec != Some(fat_sec) {
                io.read_blocks(self.fat1_start_lba + fat_sec, &mut sec)
                    .await?;
                cached_fat_sec = Some(fat_sec);
            }
            let off = ((candidate % 128) * 4) as usize;
            let raw = read_le_u32(&sec[0], off);
            if (raw & FAT32_MASK) == 0 {
                // Mark newly allocated cluster as EOC in all FAT copies.
                write_le_u32(&mut sec[0], off, (raw & 0xf000_0000) | FAT32_EOC);
                for fat_idx in 0..self.num_fats {
                    let lba = self.fat1_start_lba + fat_idx as u32 * self.fat_sz32 + fat_sec;
                    io.write_blocks(lba, &sec).await?;
                }
                if let Some(prev) = link_from {
                    self.write_fat_entry(io, prev, candidate).await?;
                }
                if self.free_count != u32::MAX && self.free_count > 0 {
                    self.free_count -= 1;
                }
                self.nxt_free = if candidate + 1 < max_clus {
                    candidate + 1
                } else {
                    2
                };
                return Ok(candidate);
            }
        }
        Err(Fat32Error::DiskFull)
    }

    async fn flush_fsinfo<IO: AsyncBlock512>(
        &self,
        io: &mut IO,
    ) -> Result<(), Fat32Error<IO::Error>> {
        if self.fsinfo_lba == self.part_start_lba {
            return Ok(());
        }
        let mut sec = [[0u8; SECTOR_BYTES]; 1];
        io.read_blocks(self.fsinfo_lba, &mut sec).await?;
        if read_le_u32(&sec[0], 0) != 0x4161_5252 || read_le_u32(&sec[0], 484) != 0x6141_7272 {
            return Ok(());
        }
        write_le_u32(&mut sec[0], 488, self.free_count);
        write_le_u32(&mut sec[0], 492, self.nxt_free);
        io.write_blocks(self.fsinfo_lba, &sec).await?;
        Ok(())
    }

    /// Scans the root directory cluster chain, returning total valid entries and populating `out`
    /// starting from entry index `skip`.
    pub async fn list_root<IO: AsyncBlock512>(
        &self,
        io: &mut IO,
        skip: usize,
        out: &mut [DirEntryInfo],
    ) -> Result<(usize, usize), Fat32Error<IO::Error>> {
        let mut total_found = 0usize;
        let mut written = 0usize;
        let mut clus = self.root_clus;
        let mut sec = [[0u8; SECTOR_BYTES]; 1];

        for _ in 0..MAX_DIR_CLUSTERS {
            let Some(base_lba) = self.cluster_to_lba(clus) else {
                return Err(Fat32Error::CorruptFat);
            };
            for sec_off in 0..self.sec_per_clus as u32 {
                io.read_blocks(base_lba + sec_off, &mut sec).await?;
                for slot in 0..16 {
                    let e = &sec[0][slot * 32..(slot + 1) * 32];
                    let first = e[0];
                    if first == 0x00 {
                        return Ok((total_found, written));
                    }
                    let attr = e[11];
                    if first == 0xe5 || (attr & 0x0f) == 0x0f || (attr & 0x08) != 0 {
                        continue;
                    }
                    if total_found >= skip && written < out.len() {
                        let mut name_8_3 = [0u8; 11];
                        name_8_3.copy_from_slice(&e[0..11]);
                        let hi = read_le_u16(e, 20) as u32;
                        let lo = read_le_u16(e, 26) as u32;
                        out[written] = DirEntryInfo {
                            name_8_3,
                            first_cluster: (hi << 16) | lo,
                            size_bytes: read_le_u32(e, 28),
                            is_dir: (attr & 0x10) != 0,
                        };
                        written += 1;
                    }
                    total_found += 1;
                }
            }
            let next = self.read_fat_entry(io, clus).await?;
            if is_eoc(next) {
                return Ok((total_found, written));
            }
            clus = next;
        }
        Err(Fat32Error::CorruptFat)
    }

    /// Scans all `LOGnnnnn.BDL` entries in root dir and picks `max + 1` (minimum `1`).
    pub async fn next_log_sequence<IO: AsyncBlock512>(
        &self,
        io: &mut IO,
    ) -> Result<u32, Fat32Error<IO::Error>> {
        let mut max_seq = 0u32;
        let mut clus = self.root_clus;
        let mut sec = [[0u8; SECTOR_BYTES]; 1];

        for _ in 0..MAX_DIR_CLUSTERS {
            let Some(base_lba) = self.cluster_to_lba(clus) else {
                return Err(Fat32Error::CorruptFat);
            };
            for sec_off in 0..self.sec_per_clus as u32 {
                io.read_blocks(base_lba + sec_off, &mut sec).await?;
                for slot in 0..16 {
                    let e = &sec[0][slot * 32..(slot + 1) * 32];
                    if e[0] == 0x00 {
                        return Ok(max_seq.saturating_add(1).min(99_999));
                    }
                    if e[0] == 0xe5 || (e[11] & 0x0f) == 0x0f || (e[11] & 0x08) != 0 {
                        continue;
                    }
                    let mut raw = [0u8; 11];
                    raw.copy_from_slice(&e[0..11]);
                    if let Some(seq) = parse_log_filename_seq(&raw) {
                        max_seq = max_seq.max(seq);
                    }
                }
            }
            let next = self.read_fat_entry(io, clus).await?;
            if is_eoc(next) {
                return Ok(max_seq.saturating_add(1).min(99_999));
            }
            clus = next;
        }
        Err(Fat32Error::CorruptFat)
    }

    /// Creates a new 8.3 file in the root directory with pre-allocated initial cluster.
    pub async fn create_file<IO: AsyncBlock512>(
        &mut self,
        io: &mut IO,
        name_8_3: [u8; 11],
        ts: RtcTimestamp,
    ) -> Result<Fat32FileWriter, Fat32Error<IO::Error>> {
        let mut clus = self.root_clus;
        let mut sec = [[0u8; SECTOR_BYTES]; 1];

        let mut found_slot: Option<(u32, usize)> = None;
        for _ in 0..MAX_DIR_CLUSTERS {
            let Some(base_lba) = self.cluster_to_lba(clus) else {
                return Err(Fat32Error::CorruptFat);
            };
            for sec_off in 0..self.sec_per_clus as u32 {
                let lba = base_lba + sec_off;
                io.read_blocks(lba, &mut sec).await?;
                for slot in 0..16 {
                    let first = sec[0][slot * 32];
                    if first == 0x00 || first == 0xe5 {
                        found_slot = Some((lba, slot * 32));
                        break;
                    }
                }
                if found_slot.is_some() {
                    break;
                }
            }
            if found_slot.is_some() {
                break;
            }
            let next = self.read_fat_entry(io, clus).await?;
            if is_eoc(next) {
                let new_dir_clus = self.alloc_cluster(io, Some(clus)).await?;
                let Some(new_lba) = self.cluster_to_lba(new_dir_clus) else {
                    return Err(Fat32Error::CorruptFat);
                };
                let zero = [[0u8; SECTOR_BYTES]; 1];
                for z in 0..self.sec_per_clus as u32 {
                    io.write_blocks(new_lba + z, &zero).await?;
                }
                io.read_blocks(new_lba, &mut sec).await?;
                found_slot = Some((new_lba, 0));
                break;
            }
            clus = next;
        }
        let Some((entry_lba, entry_off)) = found_slot else {
            return Err(Fat32Error::CorruptFat);
        };

        let data_clus = self.alloc_cluster(io, None).await?;
        let e = &mut sec[0][entry_off..entry_off + 32];
        e.fill(0);
        e[0..11].copy_from_slice(&name_8_3);
        e[11] = 0x20; // ATTR_ARCHIVE
        e[13] = ts.to_fat_time_tenth();
        let fat_t = ts.to_fat_time();
        let fat_d = ts.to_fat_date();
        write_le_u16(e, 14, fat_t);
        write_le_u16(e, 16, fat_d);
        write_le_u16(e, 18, fat_d);
        write_le_u16(e, 20, ((data_clus >> 16) & 0xffff) as u16);
        write_le_u16(e, 22, fat_t);
        write_le_u16(e, 24, fat_d);
        write_le_u16(e, 26, (data_clus & 0xffff) as u16);
        write_le_u32(e, 28, 0);
        io.write_blocks(entry_lba, &sec).await?;
        self.flush_fsinfo(io).await?;

        Ok(Fat32FileWriter {
            name_8_3,
            dir_entry_lba: entry_lba,
            dir_entry_offset: entry_off,
            first_cluster: data_clus,
            current_cluster: data_clus,
            sector_in_cluster: 0,
            bytes_written: 0,
        })
    }

    /// Reads up to `out.len()` bytes from an existing 8.3 file at byte `offset`.
    ///
    /// Returns `(bytes_read, total_file_size)`.
    pub async fn read_file_chunk<IO: AsyncBlock512>(
        &self,
        io: &mut IO,
        name_8_3: [u8; 11],
        offset: u32,
        out: &mut [u8],
    ) -> Result<(usize, u32), Fat32Error<IO::Error>> {
        let mut entry: Option<DirEntryInfo> = None;
        let mut clus = self.root_clus;
        let mut sec = [[0u8; SECTOR_BYTES]; 1];

        'search: for _ in 0..MAX_DIR_CLUSTERS {
            let Some(base_lba) = self.cluster_to_lba(clus) else {
                return Err(Fat32Error::CorruptFat);
            };
            for sec_off in 0..self.sec_per_clus as u32 {
                io.read_blocks(base_lba + sec_off, &mut sec).await?;
                for slot in 0..16 {
                    let e = &sec[0][slot * 32..(slot + 1) * 32];
                    if e[0] == 0x00 {
                        break 'search;
                    }
                    if e[0] == 0xe5 || (e[11] & 0x0f) == 0x0f || (e[11] & 0x18) != 0 {
                        continue;
                    }
                    if e[0..11] == name_8_3 {
                        let hi = read_le_u16(e, 20) as u32;
                        let lo = read_le_u16(e, 26) as u32;
                        entry = Some(DirEntryInfo {
                            name_8_3,
                            first_cluster: (hi << 16) | lo,
                            size_bytes: read_le_u32(e, 28),
                            is_dir: false,
                        });
                        break 'search;
                    }
                }
            }
            let next = self.read_fat_entry(io, clus).await?;
            if is_eoc(next) {
                break;
            }
            clus = next;
        }

        let Some(info) = entry else {
            return Err(Fat32Error::FileNotFound);
        };
        if offset >= info.size_bytes || out.is_empty() || info.first_cluster < 2 {
            return Ok((0, info.size_bytes));
        }

        let cluster_bytes = SECTOR_BYTES as u32 * self.sec_per_clus as u32;
        let mut cur_clus = info.first_cluster;
        let skip_clusters = offset / cluster_bytes;
        for _ in 0..skip_clusters {
            let next = self.read_fat_entry(io, cur_clus).await?;
            if is_eoc(next) {
                return Ok((0, info.size_bytes));
            }
            cur_clus = next;
        }

        let mut remaining = ((info.size_bytes - offset) as usize).min(out.len());
        let mut pos_in_clus = (offset % cluster_bytes) as usize;
        let mut copied = 0usize;

        for _ in 0..=self.cluster_count {
            let Some(clus_lba) = self.cluster_to_lba(cur_clus) else {
                return Err(Fat32Error::CorruptFat);
            };
            let mut sec_in_clus = pos_in_clus / SECTOR_BYTES;
            let mut sec_off = pos_in_clus % SECTOR_BYTES;
            while sec_in_clus < self.sec_per_clus as usize && remaining > 0 {
                io.read_blocks(clus_lba + sec_in_clus as u32, &mut sec)
                    .await?;
                let take = remaining.min(SECTOR_BYTES - sec_off);
                out[copied..copied + take].copy_from_slice(&sec[0][sec_off..sec_off + take]);
                copied += take;
                remaining -= take;
                sec_in_clus += 1;
                sec_off = 0;
            }
            if remaining == 0 {
                return Ok((copied, info.size_bytes));
            }
            let next = self.read_fat_entry(io, cur_clus).await?;
            if is_eoc(next) {
                return Ok((copied, info.size_bytes));
            }
            cur_clus = next;
            pos_in_clus = 0;
        }
        Err(Fat32Error::CorruptFat)
    }
}

impl Fat32FileWriter {
    /// Streams contiguous `512`-byte sectors via multi-block writes across FAT32 clusters.
    pub async fn write_sectors<IO: AsyncBlock512>(
        &mut self,
        vol: &mut Fat32Volume,
        io: &mut IO,
        sectors: &[[u8; 512]],
        _ts: RtcTimestamp,
    ) -> Result<(), Fat32Error<IO::Error>> {
        let mut idx = 0usize;
        while idx < sectors.len() {
            if self.sector_in_cluster >= vol.sec_per_clus {
                let next_clus = vol.alloc_cluster(io, Some(self.current_cluster)).await?;
                self.current_cluster = next_clus;
                self.sector_in_cluster = 0;
            }
            let Some(clus_lba) = vol.cluster_to_lba(self.current_cluster) else {
                return Err(Fat32Error::CorruptFat);
            };
            let start_lba = clus_lba + self.sector_in_cluster as u32;
            let room = (vol.sec_per_clus - self.sector_in_cluster) as usize;
            let batch = (sectors.len() - idx).min(room);
            io.write_blocks(start_lba, &sectors[idx..idx + batch])
                .await?;
            self.sector_in_cluster += batch as u8;
            self.bytes_written = self
                .bytes_written
                .saturating_add((batch * SECTOR_BYTES) as u32);
            idx += batch;
        }
        Ok(())
    }

    /// Commits current file byte length, write timestamp, and `FSInfo` hint back to FAT32 metadata.
    pub async fn sync_metadata<IO: AsyncBlock512>(
        &self,
        vol: &Fat32Volume,
        io: &mut IO,
        ts: RtcTimestamp,
    ) -> Result<(), Fat32Error<IO::Error>> {
        let mut sec = [[0u8; SECTOR_BYTES]; 1];
        io.read_blocks(self.dir_entry_lba, &mut sec).await?;
        let e = &mut sec[0][self.dir_entry_offset..self.dir_entry_offset + 32];
        write_le_u16(e, 18, ts.to_fat_date());
        write_le_u16(e, 22, ts.to_fat_time());
        write_le_u16(e, 24, ts.to_fat_date());
        write_le_u32(e, 28, self.bytes_written);
        io.write_blocks(self.dir_entry_lba, &sec).await?;
        vol.flush_fsinfo(io).await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use std::vec;
    use std::vec::Vec;

    use super::{
        AsyncBlock512, DirEntryInfo, DiskLayoutKind, Fat32Error, Fat32Volume, format_8_3_name,
        format_log_filename_8_3, format_mbr_fat32, parse_filename_to_8_3, probe_disk,
    };
    use crate::datalog::RtcTimestamp;

    struct MemDisk {
        sectors: Vec<[u8; 512]>,
    }

    impl MemDisk {
        fn new(sector_count: usize) -> Self {
            Self {
                sectors: vec![[0u8; 512]; sector_count],
            }
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MemErr;

    impl AsyncBlock512 for MemDisk {
        type Error = MemErr;

        async fn read_blocks(
            &mut self,
            start_lba: u32,
            out: &mut [[u8; 512]],
        ) -> Result<(), MemErr> {
            let start = start_lba as usize;
            let end = start.checked_add(out.len()).ok_or(MemErr)?;
            if end > self.sectors.len() {
                return Err(MemErr);
            }
            out.copy_from_slice(&self.sectors[start..end]);
            Ok(())
        }

        async fn write_blocks(&mut self, start_lba: u32, data: &[[u8; 512]]) -> Result<(), MemErr> {
            let start = start_lba as usize;
            let end = start.checked_add(data.len()).ok_or(MemErr)?;
            if end > self.sectors.len() {
                return Err(MemErr);
            }
            self.sectors[start..end].copy_from_slice(data);
            Ok(())
        }
    }

    fn block_on<F: Future>(fut: F) -> F::Output {
        let mut fut = pin!(fut);
        let mut cx = Context::from_waker(Waker::noop());
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(val) => val,
            Poll::Pending => panic!("MemDisk future should always resolve synchronously"),
        }
    }

    #[test]
    fn probe_detects_blank_exfat_and_fat32_after_format() {
        // ~270 MiB virtual card (> 65,525 clusters @ spc=4)
        let total_sectors = 560_000u64;
        let mut disk = MemDisk::new(total_sectors as usize);
        let mut scratch = vec![[0u8; 512]; 32];

        // 1. Blank disk check
        let p0 = block_on(probe_disk(&mut disk, total_sectors)).expect("probe blank");
        assert_eq!(p0.layout, DiskLayoutKind::Blank);

        // 2. Simulated CCTV exFAT layout (MBR type 0x07 + "EXFAT   " VBR at 2048)
        disk.sectors[0][450] = 0x07;
        disk.sectors[0][454..458].copy_from_slice(&2048u32.to_le_bytes());
        disk.sectors[0][458..462].copy_from_slice(&(total_sectors as u32 - 2048).to_le_bytes());
        disk.sectors[0][510] = 0x55;
        disk.sectors[0][511] = 0xaa;
        disk.sectors[2048][3..11].copy_from_slice(b"EXFAT   ");
        disk.sectors[2048][510] = 0x55;
        disk.sectors[2048][511] = 0xaa;
        let p1 = block_on(probe_disk(&mut disk, total_sectors)).expect("probe exfat");
        assert_eq!(p1.layout, DiskLayoutKind::MbrExFat);

        // 3. Reformat in-place to MBR + FAT32
        let ts = RtcTimestamp::default_fallback();
        let formatted = block_on(format_mbr_fat32(&mut disk, total_sectors, ts, &mut scratch))
            .expect("format FAT32");
        assert!(formatted.cluster_count >= 65_525);
        assert_eq!(formatted.part_start_lba, 2048);

        let p2 = block_on(probe_disk(&mut disk, total_sectors)).expect("probe fat32");
        assert_eq!(p2.layout, DiskLayoutKind::MbrFat32);
    }

    #[test]
    fn fat32_geometry_scales_cleanly_to_64gb_card() {
        // Kingston 64 GB SDXC ~ 121_000_000 sectors (~57.7 GiB usable)
        let sectors_64gb: u32 = 121_000_000;
        let (spc, fat_sz32, clusters) =
            super::pick_fat32_geometry(sectors_64gb).expect("64GB geometry");
        assert_eq!(spc, 64, "64GB card should select 32 KB clusters");
        assert!(clusters >= 1_800_000);
        assert!(u64::from(fat_sz32) * 128 >= u64::from(clusters) + 2);
    }

    #[test]
    fn create_stream_across_clusters_sync_remount_and_read_chunk_roundtrip() {
        let total_sectors = 560_000u64;
        let mut disk = MemDisk::new(total_sectors as usize);
        let mut scratch = vec![[0u8; 512]; 32];
        let ts = RtcTimestamp::default_fallback();
        let mut vol =
            block_on(format_mbr_fat32(&mut disk, total_sectors, ts, &mut scratch)).expect("format");

        let seq1 = block_on(vol.next_log_sequence(&mut disk)).expect("seq1");
        assert_eq!(seq1, 1);
        let name1 = format_log_filename_8_3(seq1);
        let (pretty, pretty_len) = format_8_3_name(&name1);
        assert_eq!(&pretty[..pretty_len as usize], b"LOG00001.BDL");
        assert_eq!(parse_filename_to_8_3("log00001.bdl"), Some(name1));

        let mut writer = block_on(vol.create_file(&mut disk, name1, ts)).expect("create LOG00001");
        // Write 19 sectors (= 9,728 bytes) spanning multiple 4-sector (2 KB) clusters!
        let mut payload = vec![[0u8; 512]; 19];
        for (sec_idx, blk) in payload.iter_mut().enumerate() {
            for (b_idx, byte) in blk.iter_mut().enumerate() {
                *byte = ((sec_idx * 37 + b_idx) & 0xff) as u8;
            }
        }
        block_on(writer.write_sectors(&mut vol, &mut disk, &payload[..7], ts)).expect("write 7");
        block_on(writer.write_sectors(&mut vol, &mut disk, &payload[7..19], ts)).expect("write 12");
        block_on(writer.sync_metadata(&vol, &mut disk, ts)).expect("sync metadata");

        // Remount fresh from disk, verify sequence increments, listing, and cross-cluster readback.
        let remount = block_on(Fat32Volume::mount(&mut disk, total_sectors)).expect("remount");
        assert_eq!(
            block_on(remount.next_log_sequence(&mut disk)).expect("next seq"),
            2
        );
        let mut entries = [DirEntryInfo {
            name_8_3: [0; 11],
            first_cluster: 0,
            size_bytes: 0,
            is_dir: false,
        }; 8];
        let (total, count) = block_on(remount.list_root(&mut disk, 0, &mut entries)).expect("list");
        assert_eq!(total, 1);
        assert_eq!(count, 1);
        assert_eq!(entries[0].size_bytes, 19 * 512);

        let mut readback = vec![0u8; 19 * 512];
        let mut off = 0usize;
        while off < readback.len() {
            let mut chunk = [0u8; 256];
            let (got, tot) =
                block_on(remount.read_file_chunk(&mut disk, name1, off as u32, &mut chunk))
                    .expect("read chunk");
            assert_eq!(tot, 19 * 512);
            assert!(got > 0);
            readback[off..off + got].copy_from_slice(&chunk[..got]);
            off += got;
        }
        for sec_idx in 0..19 {
            assert_eq!(
                &readback[sec_idx * 512..(sec_idx + 1) * 512],
                &payload[sec_idx]
            );
        }

        // Verify unaligned cross-cluster boundary read (offset 2040 spans cluster 0 -> cluster 1 @ 2048).
        let mut cross_buf = [0u8; 256];
        let (cross_got, _) =
            block_on(remount.read_file_chunk(&mut disk, name1, 2040, &mut cross_buf))
                .expect("cross-cluster unaligned read");
        assert_eq!(cross_got, 256);
        assert_eq!(&cross_buf[..], &readback[2040..2040 + 256]);

        // Fill remaining slots in cluster 2 with deleted (0xe5) markers so list_root walks the FAT chain,
        // then inject a circular root_clus -> root_clus FAT entry to verify CorruptFat termination.
        let root_lba = remount.data_start_lba;
        for sec_off in 0..remount.sec_per_clus as u32 {
            let lba = (root_lba + sec_off) as usize;
            let start_slot = if sec_off == 0 { 2 } else { 0 };
            for slot in start_slot..16 {
                disk.sectors[lba][slot * 32] = 0xe5;
            }
        }
        block_on(remount.write_fat_entry(&mut disk, remount.root_clus, remount.root_clus))
            .expect("inject loop");
        let err = block_on(remount.list_root(&mut disk, 0, &mut entries)).unwrap_err();
        assert_eq!(err, Fat32Error::CorruptFat);
    }
}
