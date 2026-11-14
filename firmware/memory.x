MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K /* BANK_1 + BANK_2 */
  RAM   : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
  SRAM4 : ORIGIN = 0x38000000, LENGTH = 64K   /* D3 Domain SRAM4 (BDMA accessible) */
}

SECTIONS
{
  .sram4 (NOLOAD) : ALIGN(4)
  {
    *(.sram4 .sram4.*);
  } > SRAM4
} INSERT AFTER .got;
