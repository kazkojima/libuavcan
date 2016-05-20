/*
 * ST32F446 memory setup.
 */
__main_stack_size__     = 0x0400;  /* Exception handlers     */
__process_stack_size__  = 0x2000;  /* Main program           */
__process0_stack_size__  = 0x100;

MEMORY
{
    flash0 : org = 0x08000000, len = 4k
    flash  : org = 0x08000000+0x1000, len = 508k
    ram : org = 0x20000000, len = 128k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = 128k;
__ram_end__             = __ram_start__ + __ram_size__;

SECTIONS
{
    . = 0;

    .sys : ALIGN(4) SUBALIGN(4)
    {
	_sys = .;
	KEEP(*(.vectors))
	. = ALIGN(16);
	KEEP(*(.sys.version))
	KEEP(*(.sys.board_id))
	KEEP(*(.sys.board_name))
	KEEP(*(.sys.reset))
	. = ALIGN(1024);
    } > flash0 =0xffffffff

    _text = .;

    .startup : ALIGN(128) SUBALIGN(128)
    {
        KEEP(*(.startup.vectors))
        . = ALIGN (16);
    } > flash =0xffffffff

    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
	. = ALIGN(8);
    } > flash

    .ARM.extab : {*(.ARM.extab* .gnu.linkonce.armextab.*)} > flash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash

    .eh_frame_hdr : {*(.eh_frame_hdr)} > flash

    .eh_frame : ONLY_IF_RO {*(.eh_frame)} > flash

    .textalign : ONLY_IF_RO { . = ALIGN(8); } > flash

    _etext = .;
    _textdata = _etext;

    .vectors_in_ram :
    {
        . = ALIGN(8);
        __vector_ram_addr__ = .;
	KEEP(*(.bss.startup.*))
    } > ram

    .process_stack :
    {
        . = ALIGN(8);
        __stack_heap_can1__ = .;
        . += 0x300;
        . = ALIGN(8);
        __stack_heap_can1_end__ = .;
        __stack_heap_can2__ = .;
        . += 0x300;
        . = ALIGN(8);
        __stack_heap_can2_end__ = .;
        __stack_heap_tim__ = .;
        . += 0x100;
        . = ALIGN(8);
        __stack_heap_tim_end__ = .;
        __process0_stack_base__ = .;
        . += __process0_stack_size__;
        . = ALIGN(8);
        __process0_stack_end__ = .;
        __process_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
    } > ram

    .main_stack :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
    } > ram

    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > flash

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
    } > ram

    PROVIDE(end = .);
    _end            = .;
    . = ALIGN(128);
    PROVIDE(__heap_base__ = .);
}

__heap_end__    = __ram_end__;
