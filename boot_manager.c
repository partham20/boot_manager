/*
 * boot_manager.c
 *
 * Boot Manager for F28P55x â€” lives in Bank 0 Sectors 0-3 (0x080000-0x080FFF).
 * NEVER erased by OTA. Loaded via JTAG once.
 *
 * On every boot:
 *   1. Init flash module (required to read/write flash)
 *   2. Read boot flag from Bank 3 (0x0E0000)
 *   3. If update pending (0xA5A5) and CRC valid (0x5A5A):
 *      a. Re-verify CRC32 of Bank 2
 *      b. Erase Bank 0 sectors 4-127 (app area)
 *      c. Copy Bank 2 â†’ Bank 0 sectors 4-127
 *      d. Clear boot flag
 *      e. Reset
 *   4. Otherwise: jump to application at 0x081000
 *
 * Author: Parthasarathy.M
 * Date:   03-Apr-2026
 */

#include "driverlib.h"
#include "device.h"
#include "FlashTech_F28P55x_C28x.h"
#include "flash_programming_f28p55x.h"

/* â”€â”€ Addresses â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
#define APP_ENTRY_ADDR      0x082000UL  /* Application starts here (sector 8) */
#define BANK0_APP_START     0x082000UL  /* Bank 0 sector 8 */
#define BANK0_APP_END       0x09FFFFUL  /* Bank 0 sector 127 end */
#define BANK2_START         0x0C0000UL
#define BANK3_FLAG_ADDR     0x0E0000UL  /* Boot flag location */

#define SECTOR_SIZE_WORDS   0x400U      /* 2KB sector = 0x400 uint16 words */
#define APP_SECTOR_COUNT    120         /* Sectors 8-127 */
#define APP_FIRST_SECTOR    8

/* Flash write-enable */
#define SEC0TO31            0x00000000U
#define SEC32To127          0xFFFFFFFFU

/* â”€â”€ Boot flag layout (must match fw_image_rx.h) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
/* Word 0: updatePending  (0xA5A5 = update ready)
 * Word 1: crcValid       (0x5A5A = CRC confirmed by app)
 * Word 2: imageSize low  (uint16)
 * Word 3: imageSize high (uint16)
 * Word 4: imageCRC low   (uint16)
 * Word 5: imageCRC high  (uint16)
 */
#define FLAG_UPDATE_PENDING 0xA5A5U
#define FLAG_CRC_VALID      0x5A5AU

/* â”€â”€ CRC32 lookup table â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static const uint32_t crc32Table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
    0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
    0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
    0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
    0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
    0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
    0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
    0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
    0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
    0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
    0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/* â”€â”€ Forward declarations â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€ */
static uint32_t  computeCRC32(uint32_t startAddr, uint32_t numBytes);
static void      eraseSector(uint32_t sectorAddr);
static void      programWords(uint32_t destAddr, const uint16_t *src);
static void      clearBootFlag(void);
static void      copyBank2ToBank0(uint32_t imageSize);
static void      jumpToApp(void);

volatile uint16_t debugHalt;

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  MAIN â€” Boot manager entry point
 *  Runs from flash 0x080000 on every power-up/reset.
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
void main(void)
{

    debugHalt = 1;


  while(debugHalt);  // ← pause here, set debugHalt=0 in CCS to continue


    /* Disable watchdog first â€” Device_init does too much for boot manager */
    SysCtl_disableWatchdog();

    /* Set flash wait states (required before any flash read at speed) */
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 3);

    /* Read boot flag from Bank 3 first sector */
    volatile uint16_t *flag = (volatile uint16_t *)BANK3_FLAG_ADDR;

    uint16_t updatePending = flag[0];
    uint16_t crcValid      = flag[1];
    uint32_t imageSize     = (uint32_t)flag[2] | ((uint32_t)flag[3] << 16);
    uint32_t imageCRC      = (uint32_t)flag[4] | ((uint32_t)flag[5] << 16);

    if (updatePending == FLAG_UPDATE_PENDING && crcValid == FLAG_CRC_VALID)
    {
        /* Init Flash API â€” only needed when we actually erase/program */
        Fapi_initializeAPI(FlashTech_CPU0_BASE_ADDRESS,
                           DEVICE_SYSCLK_FREQ / 1000000U);
        Fapi_setActiveFlashBank(Fapi_FlashBank0);

        /* Update is pending â€” verify Bank 2 CRC before copying */
        uint32_t computedCRC = computeCRC32(BANK2_START, imageSize);

        if (computedCRC == imageCRC)
        {
            /* CRC matches â€” safe to copy */
            copyBank2ToBank0(imageSize);
            clearBootFlag();

            /* Reset to boot fresh with new firmware */
            SysCtl_resetDevice();
            /* Never reaches here */
        }
        else
        {
            /* CRC mismatch â€” Bank 2 is corrupt.
             * Clear the flag so we don't retry forever.
             * Boot the old app (which may be partially erased
             * from a previous failed attempt â€” but at least
             * we won't loop). */
            clearBootFlag();
        }
    }

    /* No update pending (or CRC failed) â€” jump to application */
    jumpToApp();
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  JUMP TO APPLICATION at 0x081000
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
static void jumpToApp(void)
{
    /* Cast address to function pointer and call.
     * The app has its own codestart at 0x081000 which
     * branches to _c_int00 (C runtime init â†’ main). */
    ((void (*)(void))APP_ENTRY_ADDR)();
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  COPY Bank 2 â†’ Bank 0 (sectors 4-127 only)
 *
 *  Erases app area first, then copies word-by-word.
 *  Flash writes are 8 x uint16 (512-bit) per call.
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
static void copyBank2ToBank0(uint32_t imageSize)
{
    uint32_t sectorAddr;
    uint32_t srcAddr, dstAddr;
    uint32_t wordsRemaining;
    uint16_t i;

    /* Step 1: Erase Bank 0 sectors 4-127 (application area) */
    for (i = 0; i < APP_SECTOR_COUNT; i++)
    {
        sectorAddr = BANK0_APP_START + ((uint32_t)i * SECTOR_SIZE_WORDS);
        eraseSector(sectorAddr);
    }

    /* Step 2: Copy from Bank 2 to Bank 0 app area, 8 words at a time */
    srcAddr = BANK2_START;
    dstAddr = BANK0_APP_START;
    wordsRemaining = (imageSize + 1) / 2;  /* bytes â†’ 16-bit words */

    while (wordsRemaining >= 8)
    {
        programWords(dstAddr, (const uint16_t *)srcAddr);
        srcAddr += 8;
        dstAddr += 8;
        wordsRemaining -= 8;
    }

    /* Handle remaining words (pad with 0xFFFF) */
    if (wordsRemaining > 0)
    {
        uint16_t padBuf[8];
        for (i = 0; i < 8; i++)
        {
            if (i < wordsRemaining)
                padBuf[i] = *((volatile uint16_t *)(srcAddr + i));
            else
                padBuf[i] = 0xFFFF;
        }
        programWords(dstAddr, padBuf);
    }
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  ERASE one 2KB sector
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
static void eraseSector(uint32_t sectorAddr)
{
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) {}

    Fapi_issueAsyncCommand(Fapi_ClearStatus);
    while (Fapi_getFsmStatus() != 0) {}

    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC0TO31);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC32To127);

    Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector, (uint32_t *)sectorAddr);

    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) {}
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  PROGRAM 8 x uint16 words (512-bit aligned block)
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
static void programWords(uint32_t destAddr, const uint16_t *src)
{
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) {}

    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC0TO31);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC32To127);

    Fapi_issueProgrammingCommand((uint32_t *)destAddr,
                                 (uint16_t *)src, 8,
                                 0, 0, Fapi_AutoEccGeneration);

    while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy) {}
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  CLEAR BOOT FLAG â€” erase Bank 3 first sector
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
static void clearBootFlag(void)
{
    eraseSector(BANK3_FLAG_ADDR);
}

/* â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�
 *  CRC32 over flash memory
 * â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•�â•� */
static uint32_t computeCRC32(uint32_t startAddr, uint32_t numBytes)
{
    uint32_t crc = 0xFFFFFFFF;
    volatile uint16_t *wordPtr = (volatile uint16_t *)startAddr;
    uint32_t numWords = (numBytes + 1) / 2;
    uint32_t i;

    for (i = 0; i < numWords; i++)
    {
        uint16_t word = wordPtr[i];
        crc = (crc >> 8) ^ crc32Table[(crc ^ (word & 0xFF)) & 0xFF];
        crc = (crc >> 8) ^ crc32Table[(crc ^ (word >> 8)) & 0xFF];
    }

    return crc ^ 0xFFFFFFFF;
}
