# Boot Manager - TMS320F28P55x

Brick-proof boot manager for OTA firmware updates on the TI TMS320F28P55x (C2000) microcontroller. Resides in Bank 0 Sectors 0-7 (8 KB) and is never erased during OTA -- loaded once via JTAG, it stays permanently to guarantee the device can always boot.

Part of the **GEN3 Distributed Power Monitoring System** (S-Board controller).

## How It Works

On every power-up or reset, the boot manager:

1. Disables the watchdog and initializes flash wait states
2. Reads a boot flag from Bank 3 (`0x0E0000`)
3. If an update is pending (`0xA5A5`) and CRC is pre-validated (`0x5A5A`):
   - Verifies CRC32 of the staged firmware in Bank 2
   - Erases Bank 0 sectors 8-127 (application area)
   - Copies Bank 2 to Bank 0
   - Clears the boot flag and resets
4. Otherwise, jumps to the application at `0x082000`

```
Power On --> ROM Bootloader --> 0x080000 (Boot Manager)
                                    |
                                    +-- Read Bank 3 flag
                                    |   Flag == 0xA5A5 && CRC valid?
                                    |       |
                                    |   NO  |  YES
                                    |   |   v
                                    |   |  Verify CRC32 of Bank 2
                                    |   |   |
                                    |   |   +-- CRC OK --> Erase app area
                                    |   |   |              Copy Bank 2 --> Bank 0
                                    |   |   |              Clear flag, reset
                                    |   |   |
                                    |   |   +-- CRC FAIL --> Clear flag
                                    |   |
                                    |   v
                                    +-- Jump to 0x082000 (Application)
```

## Memory Map

| Region | Address Range | Purpose |
|--------|--------------|---------|
| Boot Manager (codestart) | `0x080000 - 0x080001` | Entry point, jump to C runtime |
| Boot Manager (code+const) | `0x080002 - 0x081FFF` | Boot logic, CRC table, Flash API image |
| Application | `0x082000 - 0x09FFFF` | Main firmware (sectors 8-127) |
| Bank 2 (staging) | `0x0C0000+` | OTA firmware image staged here |
| Bank 3 (flag) | `0x0E0000` | Boot flag: update pending, CRC, image size |

## Files

| File | Purpose |
|------|---------|
| `boot_manager.c` | Main logic -- flag check, CRC32 verify, erase, copy, jump |
| `boot_codestartbranch.asm` | Entry at `0x080000` -- disables watchdog, branches to C runtime |
| `boot_manager_lnk.cmd` | Linker script -- constrains code to sectors 0-7, Flash API runs from RAM |
| `flash_programming_f28p55x.h` | TI Flash API constants for F28P55x |
| `device/` | TI C2000Ware device support (device.c, device.h, driverlib) |
| `targetConfigs/` | CCS debug target configurations (XDS110) |
| `Sender.ipynb` | Python/CAN-FD firmware sender tool (PCAN USB) |

## Size Budget

The boot manager fits in 8 KB (sectors 0-7):

| Component | Approximate Size |
|-----------|-----------------|
| `boot_manager.c` code | ~400 words |
| CRC32 lookup table | 512 words |
| Flash API library (RAM copy image) | ~1,500 words |
| C runtime + cinit | ~100 words |
| **Total** | **~2,500 words (~5 KB)** |

## Boot Flag Layout

The boot flag at `0x0E0000` (Bank 3, sector 0) is written by the application after receiving an OTA image:

| Word | Field | Value |
|------|-------|-------|
| 0 | `updatePending` | `0xA5A5` = update ready |
| 1 | `crcValid` | `0x5A5A` = CRC confirmed by app |
| 2-3 | `imageSize` | Firmware size in bytes (uint32) |
| 4-5 | `imageCRC` | Expected CRC32 (uint32) |

## Building

**Prerequisites:**
- Code Composer Studio v12.8.0+
- TI C2000 Compiler v22.6.1.LTS
- C2000Ware SDK 5.03.00.00+
- `FAPI_F28P55x_EABI_v4.00.00.lib` (TI Flash API library)

### CCS Project Setup

1. **File > New > CCS Project**
   - Target: `TMS320F28P550SG9`
   - Compiler: TI v22.6.1.LTS
   - Empty project

2. **Add source files** to the project: `boot_manager.c`, `boot_codestartbranch.asm`, `boot_manager_lnk.cmd`

3. **Compiler include paths:**
   - `${C2000WARE}/driverlib/f28p55x/driverlib`
   - `${C2000WARE}/device_support/f28p55x/common/include`
   - Path to Flash API headers

4. **Predefined symbols:** `_FLASH`, `CPU1`

5. **Linker settings:**
   - Entry point (`-e`): `code_start`
   - Stack size: `0x200`
   - Link against `FAPI_F28P55x_EABI_v4.00.00.lib` and driverlib

6. **Build** with `CPU1_FLASH` configuration. Verify in the `.map` file that all `.text` is within `0x080000-0x081FFF`.

### Flashing

1. Connect XDS110 JTAG
2. Debug > Load `boot_manager.out`
3. Verify jump to `0x082000`

**After loading the boot manager, load the application firmware** with the updated linker file that starts at `0x082000`.

## Firmware Sender Tool

`Sender.ipynb` is a Jupyter notebook for sending firmware images to the S-Board over CAN-FD using a PCAN USB adapter. It supports:

- Full protocol mode (with ACK/NAK handshaking)
- Fire-and-forget streaming mode
- CRC32 verification
- 500 kbps nominal / 2 Mbps data bit rate

## Updating the Application Project

When using this boot manager, the main S-Board application must be re-linked to start at `0x082000` instead of `0x080000`:

1. In the S-Board CCS project, update the linker command file so the application entry point is at `0x082000`
2. Rebuild -- the `.bin` file for OTA will also be linked at the new address

## License

This project is licensed under the [MIT License](LICENSE).

TI C2000Ware files (`device/`, `flash_programming_f28p55x.h`) are subject to the Texas Instruments BSD-style license. See individual file headers for terms.
