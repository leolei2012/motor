/* MEMORY 区域示例（Cortex-M4，256K Flash / 64K SRAM）
 * 根据实际芯片修改 ORIGIN 和 LENGTH
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    .heap (NOLOAD) :
    {
        _heap_start = .;
        . = . + 0x1000;   /* 4K heap，按需调整 */
        _heap_end = .;
    } > RAM

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _stack_end = .;
        . = . + 0x800;    /* 2K stack，按需调整 */
        _stack_top = .;
    } > RAM
}
