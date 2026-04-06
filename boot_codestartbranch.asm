;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; boot_codestartbranch.asm
;;
;; Boot Manager entry point — placed at 0x080000 by linker.
;; ROM bootloader always jumps here after reset.
;; Disables watchdog, then branches to C runtime (_c_int00 → main).
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    .ref _c_int00
    .global code_start

    .sect "codestart"
    .retain

code_start:
    LB  wd_disable

    .text
wd_disable:
    SETC    OBJMODE             ; 28x object mode
    EALLOW                      ; Enable EALLOW
    MOVZ    DP, #7029h>>6       ; Data page for WDCR
    MOV     @7029h, #0068h      ; Disable watchdog (WDDIS bit)
    EDIS                        ; Disable EALLOW
    LB      _c_int00            ; C runtime init → main()

    .end
