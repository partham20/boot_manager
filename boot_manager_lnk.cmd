/*
 * boot_manager_lnk.cmd
 *
 * Linker command file for Boot Manager.
 * Constrains ALL flash code/data to Bank 0 Sectors 0-3 (4KB).
 * Flash API runs from RAM (mandatory — can't execute from flash
 * while erasing/programming flash).
 *
 * Memory map:
 *   0x080000-0x080001  codestart (2 words, jump to _c_int00)
 *   0x080002-0x083FFF  boot manager code + const (sectors 0-15, 16KB)
 *   0x084000+           APPLICATION (not touched by this project)
 */

MEMORY
{
    /* Boot manager flash — sectors 0-15 (16KB) */
    BEGIN           : origin = 0x080000, length = 0x000002
    BOOT_FLASH      : origin = 0x080002, length = 0x003FFE  /* 16KB - 2 words */

    /* RAM for stack, data, and flash API execution */
    BOOT_RSVD       : origin = 0x000002, length = 0x000126  /* ROM boot stack */
    RAMM0           : origin = 0x000128, length = 0x0002D8
    RAMM1           : origin = 0x000400, length = 0x000400

    RAMLS0          : origin = 0x008000, length = 0x000800
    RAMLS1          : origin = 0x008800, length = 0x000800
    RAMLS2          : origin = 0x009000, length = 0x000800  /* Ring buffer for FW receive */

    RESET           : origin = 0x3FFFC0, length = 0x000002
}

SECTIONS
{
    codestart       : > BEGIN

    .text           : > BOOT_FLASH, ALIGN(8)
    .cinit          : > BOOT_FLASH, ALIGN(8)
    .switch         : > BOOT_FLASH, ALIGN(8)
    .const          : > BOOT_FLASH, ALIGN(8)

    .reset          : > RESET, TYPE = DSECT

    .stack          : > RAMM1

#if defined(__TI_EABI__)
    .bss            : > RAMM0
    .init_array     : > BOOT_FLASH, ALIGN(8)
    .data           : > RAMM0
    .sysmem         : > RAMM0

    /* FW receive ring buffer (32 x 64 bytes = 2KB) */
    .boot_ring      : > RAMLS2
#endif

    /*
     * Flash API library + any .TI.ramfunc code.
     * LOAD from boot manager flash (sectors 0-3).
     * RUN from RAM (mandatory for flash operations).
     * C runtime copies this at startup via memcpy.
     */
#if defined(__TI_EABI__)
    GROUP
    {
        .TI.ramfunc :
        { -l FAPI_F28P55x_EABI_v4.00.00.lib }
    }   LOAD = BOOT_FLASH,
        RUN  = RAMLS0 | RAMLS1,
        LOAD_START(RamfuncsLoadStart),
        LOAD_SIZE(RamfuncsLoadSize),
        RUN_START(RamfuncsRunStart),
        ALIGN(8)
#endif
}
