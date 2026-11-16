//! Pure `no_std` identification and addressing rules for `SPI1` non-volatile memories.
//!
//! Supports both Stage 1 breadboard JEDEC SPI NOR Flash (`PY25Q64HA`, `W25Q64`, etc.; 24-bit
//! addressing + 4 KB sector erase) and Stage 2 PCB SPI FRAM (`FM25V02A`, `MB85RS256B`, etc.;
//! instant byte writes with `15/16-bit` 2-byte or `24-bit` 3-byte addressing and zero sector erase).

use ecu_types::NvmMedium;

/// Dual-slot header length prepended to each `CalibrationBlob` and `RuntimeConfigBlob` slot.
pub const SLOT_HEADER_BYTES: usize = 16;

/// Dual-slot addresses for the persistent `2,816 B` [`ecu_types::CalibrationBlob`] page.
///
/// Each slot occupies up to `2,832 B` (`0x0B10 B`) within its `4 KB` (`0x1000`) allocation.
pub const CAL_SLOT_ADDRS: [u32; 2] = [0x00_0000, 0x00_1000];

/// Dual-slot addresses for [`ecu_types::RuntimeConfigBlob`] (`KnockConfig` + `AuxIoConfig`).
///
/// Fits cleanly alongside [`CAL_SLOT_ADDRS`] inside the first `16 KB` (`0x4000`) of a `32 KB`
/// `FM25V02A` / `MB85RS256B` FRAM without address wrapping or overlap.
pub const CFG_SLOT_ADDRS: [u32; 2] = [0x00_2000, 0x00_3000];

/// Minimum usable capacity (`16 KB`) needed to store both dual-slot ping-pong regions safely.
pub const MIN_REQUIRED_NVM_BYTES: u32 = 0x00_4000;

const CYPRESS_BANK7_CONT: [u8; 6] = [0x7f; 6];
const CYPRESS_MFG_RAMTRON: u8 = 0xc2;
const FUJITSU_MFG_ID: u8 = 0x04;
const JEDEC_CONT_BYTE: u8 = 0x7f;
const NOR_FLASH_PAGE_BYTES: usize = 256;

/// Hardware geometry & protocol parameters derived from a `9-byte` `RDID (0x9F)` response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiNvmGeometry {
    pub medium: NvmMedium,
    /// Number of valid bytes returned by `RDID (0x9F)` (`3` Flash, `4` Fujitsu, `9` Cypress).
    pub rdid_len: u8,
    /// Address bytes sent after `READ (0x03)` / `WRITE (0x02)` (`2` for <=64 KB FRAM, else `3`).
    pub addr_bytes: u8,
    /// True for NOR Flash requiring `0x20` 4 KB sector erase and `WIP` status polling.
    pub requires_erase: bool,
    /// Write page boundary (`256` for NOR Flash, `0` when contiguous writes never wrap).
    pub max_program_page: usize,
    /// Total decoded chip capacity in bytes.
    pub capacity_bytes: u32,
}

impl SpiNvmGeometry {
    /// Encodes `[opcode, addr...]` into `out` using `2-byte` (`15/16-bit`) or `3-byte` (`24-bit`)
    /// SPI big-endian address framing and returns the total command prefix length (`3` or `4`).
    pub const fn format_cmd_addr(self, opcode: u8, addr: u32, out: &mut [u8; 4]) -> usize {
        let a = addr.to_be_bytes();
        out[0] = opcode;
        if self.addr_bytes == 2 {
            out[1] = a[2];
            out[2] = a[3];
            3
        } else {
            out[1] = a[1];
            out[2] = a[2];
            out[3] = a[3];
            4
        }
    }

    /// Computes the maximum safe byte count for a single `WRITE`/`PAGE_PROGRAM` transfer.
    pub fn write_chunk_limit(self, addr: u32, max_chunk: usize, remaining: usize) -> usize {
        let bounded = max_chunk.min(remaining);
        if self.max_program_page > 0 {
            let page_rem = self.max_program_page - ((addr as usize) % self.max_program_page);
            bounded.min(page_rem)
        } else {
            bounded
        }
    }
}

/// Identifies a `SPI1` NOR Flash or SPI FRAM chip from a raw `9-byte` `RDID (0x9F)` read.
///
/// Returns `None` when the bus floats (`0x00`/`0xFF`), belongs to an unknown bank continuation,
/// or reports fewer than [`MIN_REQUIRED_NVM_BYTES`] (`16 KB`).
pub fn identify_rdid(rdid: &[u8; 9]) -> Option<SpiNvmGeometry> {
    if let Some(fram) = identify_cypress_fram(rdid).or_else(|| identify_fujitsu_fram(rdid)) {
        return (fram.capacity_bytes >= MIN_REQUIRED_NVM_BYTES).then_some(fram);
    }
    identify_jedec_nor_flash(rdid).filter(|flash| flash.capacity_bytes >= MIN_REQUIRED_NVM_BYTES)
}

fn identify_cypress_fram(rdid: &[u8; 9]) -> Option<SpiNvmGeometry> {
    if rdid[0..6] != CYPRESS_BANK7_CONT || rdid[6] != CYPRESS_MFG_RAMTRON {
        return None;
    }
    let prod1 = rdid[7];
    if (prod1 & 0xe0) != 0x20 {
        return None;
    }
    let density_code = prod1 & 0x0f;
    let capacity_bytes = match density_code {
        1..=5 => 8_192u32 << density_code, // 16 KB (128Kb) .. 256 KB (2Mb)
        6 | 0x0e => 524_288,               // 512 KB (4Mb CY15B104Q)
        _ => return None,
    };
    Some(SpiNvmGeometry {
        medium: NvmMedium::SpiFram,
        rdid_len: 9,
        addr_bytes: if capacity_bytes <= 65_536 { 2 } else { 3 },
        requires_erase: false,
        max_program_page: 0,
        capacity_bytes,
    })
}

fn identify_fujitsu_fram(rdid: &[u8; 9]) -> Option<SpiNvmGeometry> {
    if rdid[0] != FUJITSU_MFG_ID || rdid[1] != JEDEC_CONT_BYTE {
        return None;
    }
    if rdid[2] == 0xff || rdid[3] == 0xff {
        return None;
    }
    // Lower nibble of Product ID Byte 1 holds density exponent (`4=128Kb, 5=256Kb, 6=512Kb, ...`).
    let density_code = rdid[2] & 0x0f;
    if !(4..=9).contains(&density_code) {
        return None;
    }
    let capacity_bytes = 1_024u32 << density_code;
    Some(SpiNvmGeometry {
        medium: NvmMedium::SpiFram,
        rdid_len: 4,
        addr_bytes: if capacity_bytes <= 65_536 { 2 } else { 3 },
        requires_erase: false,
        max_program_page: 0,
        capacity_bytes,
    })
}

fn identify_jedec_nor_flash(rdid: &[u8; 9]) -> Option<SpiNvmGeometry> {
    let [mfg, mem_type, cap_exp, ..] = *rdid;
    if mfg == 0x00 || mfg == 0xff || mfg == JEDEC_CONT_BYTE {
        return None;
    }
    if mem_type == 0x00 || mem_type == 0xff || !(0x0e..=0x1c).contains(&cap_exp) {
        return None;
    }
    Some(SpiNvmGeometry {
        medium: NvmMedium::SpiNorFlash,
        rdid_len: 3,
        addr_bytes: 3,
        requires_erase: true,
        max_program_page: NOR_FLASH_PAGE_BYTES,
        capacity_bytes: 1u32 << cap_exp,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifies_stage1_nor_flash_and_avoids_macronix_fram_confusion() {
        let puya = [0x85, 0x20, 0x17, 0x85, 0x20, 0x17, 0x85, 0x20, 0x17];
        let g = identify_rdid(&puya).expect("PY25Q64HA should be recognized");
        assert_eq!(g.medium, NvmMedium::SpiNorFlash);
        assert_eq!(g.rdid_len, 3);
        assert_eq!(g.addr_bytes, 3);
        assert!(g.requires_erase);
        assert_eq!(g.max_program_page, 256);
        assert_eq!(g.capacity_bytes, 8 * 1024 * 1024);

        let winbond = [0xef, 0x40, 0x17, 0, 0, 0, 0, 0, 0];
        let wg = identify_rdid(&winbond).expect("W25Q64 should be recognized");
        assert_eq!(wg.medium, NvmMedium::SpiNorFlash);
        assert_eq!(wg.capacity_bytes, 8 * 1024 * 1024);

        // Macronix NOR flash uses mfg 0xC2 at byte 0; must not match Cypress FRAM (0xC2 at byte 6).
        let macronix = [0xc2, 0x20, 0x17, 0xc2, 0x20, 0x17, 0xc2, 0x20, 0x17];
        let mg = identify_rdid(&macronix).expect("MX25L64 should be recognized as NOR flash");
        assert_eq!(mg.medium, NvmMedium::SpiNorFlash);
        assert_eq!(mg.addr_bytes, 3);
    }

    #[test]
    fn identifies_cypress_and_fujitsu_spi_frams_with_exact_address_widths() {
        let fm25v02a = [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x22, 0x08];
        let fg = identify_rdid(&fm25v02a).expect("FM25V02A should be recognized");
        assert_eq!(fg.medium, NvmMedium::SpiFram);
        assert_eq!(fg.rdid_len, 9);
        assert_eq!(fg.addr_bytes, 2);
        assert!(!fg.requires_erase);
        assert_eq!(fg.max_program_page, 0);
        assert_eq!(fg.capacity_bytes, 32 * 1024);

        let fm25v05 = [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x23, 0x00];
        let fg5 = identify_rdid(&fm25v05).expect("FM25V05 should be recognized");
        assert_eq!(fg5.addr_bytes, 2);
        assert_eq!(fg5.capacity_bytes, 64 * 1024);

        let fm25v10 = [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x24, 0x00];
        let fg10 = identify_rdid(&fm25v10).expect("FM25V10 should use 3-byte addressing");
        assert_eq!(fg10.addr_bytes, 3);
        assert_eq!(fg10.capacity_bytes, 128 * 1024);

        let mb85rs256b = [0x04, 0x7f, 0x05, 0x09, 0, 0, 0, 0, 0];
        let fuj256b = identify_rdid(&mb85rs256b).expect("MB85RS256B should be recognized");
        assert_eq!(fuj256b.medium, NvmMedium::SpiFram);
        assert_eq!(fuj256b.rdid_len, 4);
        assert_eq!(fuj256b.addr_bytes, 2);
        assert!(!fuj256b.requires_erase);
        assert_eq!(fuj256b.capacity_bytes, 32 * 1024);

        let mb85rs256ty = [0x04, 0x7f, 0x25, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff];
        let fuj256ty = identify_rdid(&mb85rs256ty).expect("MB85RS256TY should be recognized");
        assert_eq!(fuj256ty.addr_bytes, 2);
        assert_eq!(fuj256ty.capacity_bytes, 32 * 1024);

        let mb85rs1mt = [0x04, 0x7f, 0x27, 0x03, 0, 0, 0, 0, 0];
        let fuj1m = identify_rdid(&mb85rs1mt).expect("MB85RS1MT should use 3-byte addressing");
        assert_eq!(fuj1m.addr_bytes, 3);
        assert_eq!(fuj1m.capacity_bytes, 128 * 1024);
    }

    #[test]
    fn rejects_floating_bus_and_undersized_devices() {
        assert_eq!(identify_rdid(&[0x00; 9]), None);
        assert_eq!(identify_rdid(&[0xff; 9]), None);
        // Unknown Bank N continuation must not masquerade as FRAM or NOR Flash.
        assert_eq!(
            identify_rdid(&[0x7f, 0x7f, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]),
            None
        );
        // 8 KB Fujitsu MB85RS64T cannot fit dual 0x0000..0x3110 calibration + runtime config regions.
        assert_eq!(
            identify_rdid(&[0x04, 0x7f, 0x23, 0x03, 0, 0, 0, 0, 0]),
            None
        );
    }

    #[test]
    fn formats_address_prefixes_and_chunk_limits_per_medium() {
        let fram = identify_rdid(&[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xc2, 0x22, 0x08]).unwrap();
        let flash = identify_rdid(&[0x85, 0x20, 0x17, 0, 0, 0, 0, 0, 0]).unwrap();

        let mut cmd = [0u8; 4];
        assert_eq!(fram.format_cmd_addr(0x03, 0x3010, &mut cmd), 3);
        assert_eq!(&cmd[..3], &[0x03, 0x30, 0x10]);

        assert_eq!(flash.format_cmd_addr(0x02, 0x00_3010, &mut cmd), 4);
        assert_eq!(cmd, [0x02, 0x00, 0x30, 0x10]);

        // Crossing a 256-byte page boundary splits NOR flash writes cleanly, whereas FRAM bursts.
        assert_eq!(flash.write_chunk_limit(0x00_00f0, 128, 64), 16);
        assert_eq!(fram.write_chunk_limit(0x00_00f0, 128, 64), 64);
    }
}
