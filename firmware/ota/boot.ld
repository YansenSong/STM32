ENTRY(Boot_Reset)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
}
SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) } > FLASH
}
