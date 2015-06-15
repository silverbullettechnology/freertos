ENTRY(Vector_Init);

MEMORY
{
	sdram (rwx) : ORIGIN = 0x80010000, LENGTH = ( 512M - 0x10000 )
	//flash (rx) : ORIGIN = 0x10000, LENGTH = ( 64M - 0x10000 )
	//ram (rwx) : ORIGIN = 0x4001000, LENGTH = ( 1M - 0x1000 )
}

PROVIDE(__stack = 0x10000);

SECTIONS
{
    .text :
    {
        _text = .;
        __isr_vector_start = .;
        *(.isr_vector)
        __isr_vector_end = .;
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > sdram
    .data : /*AT(ADDR(.text) + SIZEOF(.text))*/
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > sdram
    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > sdram
}

