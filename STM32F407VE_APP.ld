/* 占位型号 STM32F407VE：512KB Flash / 128KB RAM。
   应用区从 0x08010000 开始（前 64KB 让给原厂 bootloader），长度 512-64=448KB。
   芯片型号确认后只需改这里的 ORIGIN/LENGTH 与启动代码的 RAM 顶。 */
ENTRY(Reset_Handler)
_estack = 0x20006538;   /* 与原厂 dl1824.bin 初始栈值一致（2026-11-11 台架实证：镜像形态须对齐原厂 Bootloader 才放行） */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08010000, LENGTH = 448K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}
SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH
  .text : { . = ALIGN(4); *(.text) *(.text*) *(.rodata) *(.rodata*) . = ALIGN(4); _etext = .; } >FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .; } >RAM AT> FLASH
  .bss  : { . = ALIGN(4); _sbss = .; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } >RAM
}
