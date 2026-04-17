/*
 * boot_manager.c  —  S-Board variant (expanded 16KB)
 *
 * Boot Manager for the S-Board (F28P550SG9). Lives in Bank 0 Sectors
 * 0-15 (0x080000-0x083FFF).
 * NEVER erased by OTA. Loaded via JTAG once at manufacturing.
 *
 * MCANA (GPIO 1 TX / GPIO 0 RX) is the BU-Board bus (relay/debug).
 * MCANB (GPIO 2 TX / GPIO 3 RX) is the M-Board bus (heartbeat + OTA).
 *
 * On every boot:
 *   1. Device_init() — 150 MHz PLL, flash wait-states, watchdog off.
 *   2. LED blink on GPIO 21.
 *   3. BOOT_MGR_HELLO CAN frame on MCANB (ID 0x09).
 *   4. Read boot flag at Bank 3, sector 0 (0x0E0000).
 *   5. If flag = 0xA5A5/0x5A5A and Bank 2 CRC matches:
 *        erase Bank 0 sectors 16..127, copy Bank 2 -> Bank 0, clear flag, reset.
 *   6. Check OTA receive-mode flag at Bank 3, sector 1 (0x0E0400).
 *      If flag = 0xB007: enter FW receive mode (bootFwReceiveMode).
 *   7. Otherwise: jump to application at 0x084000.
 *
 * OTA FW Receive Mode:
 *   - Reconfigures MCANB with RX FIFO + filters for OTA CAN IDs.
 *   - State machine: IDLE -> PREPARING -> WAITING_HEADER -> RECEIVING ->
 *     VERIFYING -> DONE/ABORT.
 *   - Receives firmware data frames on CAN ID 6 into a ring buffer.
 *   - Drains ring buffer to Bank 2 flash via RAM-resident write functions.
 *   - On successful verify + activate: writes copy-flag and resets so the
 *     boot manager copies Bank 2 -> Bank 0 on next boot.
 *
 * CRITICAL: eraseSector / programEightWords / copyBank2ToBank0 / clearBootFlag /
 *           writePayloadToBank2 / eraseOtaFlag / writeBootFlagForOTA / bootMcanISR
 *           MUST execute from RAM (.TI.ramfunc), because the flash controller
 *           locks the entire flash module while an erase/program is in progress.
 *           Code running from flash would stall the CPU.
 *
 * Debug CAN IDs:
 *   0x09  HELLO         — boot manager started
 *   0x0A  FLAG_STATUS   — flag values read from Bank 3
 *   0x19  CRC_RESULT    — CRC comparison
 *   0x1A  ERASE_PROG    — erase/program progress
 *   0x1B  COPY_DONE     — copy complete
 *   0x1C  NO_UPDATE     — no flag, jumping to app
 *   0x1D  CRC_FAIL      — CRC mismatch
 *   0x29  OTA_MODE      — entered OTA receive mode
 *
 * Author: Parthasarathy.M
 */

#include <string.h>
#include "driverlib.h"
#include "device.h"
#include "can_config.h"
#include "FlashTech_F28P55x_C28x.h"
#include "flash_programming_f28p55x.h"

/* ── Memory map ───────────────────────────────────────────────── */
#define APP_ENTRY_ADDR          0x084000UL  /* Sector 16 (sectors 0-15 = boot manager) */
#define BANK0_APP_START         0x084000UL
#define BANK2_START             0x0C0000UL
#define BANK3_FLAG_ADDR         0x0E0000UL  /* Copy flag (Bank 3, sector 0) */
#define BANK3_OTA_FLAG_ADDR     0x0E0400UL  /* OTA receive-mode flag (Bank 3, sector 1) */

#define SECTOR_SIZE_WORDS       0x400U      /* 1KB sector = 0x400 16-bit words */
#define APP_SECTOR_COUNT        112U        /* Sectors 16..127 */
#define BANK2_SECTOR_COUNT      128U        /* Full Bank 2 for staging */

/* ── OTA receive-mode flag ────────────────────────────────────── */
#define FLAG_OTA_MODE_MAGIC     0xB007U
#define FLAG_OTA_PROTO_SBOARD   0x01U

#define SEC_UNLOCK_A            0x00000000U
#define SEC_UNLOCK_B            0x00000000U

/* ── Boot flag layout ─────────────────────────────────────────── */
#define FLAG_UPDATE_PENDING     0xA5A5U
#define FLAG_CRC_VALID          0x5A5AU

/* ── CAN debug IDs ────────────────────────────────────────────── */
#define CAN_ID_HELLO            0x09U
#define CAN_ID_FLAG_STATUS      0x0AU
#define CAN_ID_CRC_RESULT       0x19U
#define CAN_ID_ERASE_PROG       0x1AU
#define CAN_ID_COPY_DONE        0x1BU
#define CAN_ID_NO_UPDATE        0x1CU
#define CAN_ID_CRC_FAIL         0x1DU

#define BOOT_MCAN_TX_BUF_ADDR   0x0000U
#define BOOT_MCAN_TX_BUF_NUM    1U

/* ── Heartbeat ────────────────────────────────────────────────── */
#define HEARTBEAT_BOOT_CAN_ID   0x7FFU

/* ── S-Board OTA CAN IDs (M-Board <-> S-Board, on MCANB) ──────── */
#define SB_FW_DATA_CAN_ID      0x06U
#define SB_FW_CMD_CAN_ID       0x07U
#define SB_FW_RESP_CAN_ID      0x08U

/* ── S-Board OTA command codes ──────────────────────────────────── */
#define SB_CMD_FW_START         0x30U
#define SB_CMD_FW_HEADER        0x31U
#define SB_CMD_FW_COMPLETE      0x33U

/* ── S-Board OTA response codes ─────────────────────────────────── */
#define SB_RESP_FW_ACK          0x25U
#define SB_RESP_FW_NAK          0x26U
#define SB_RESP_FW_CRC_PASS     0x27U
#define SB_RESP_FW_CRC_FAIL     0x28U

/* ── Ring buffer ─────────────────────────────────────────────── */
#define BOOT_RX_RING_SIZE       32U
#define BOOT_RX_RING_MASK       (BOOT_RX_RING_SIZE - 1U)

/* ── Burst ACK ───────────────────────────────────────────────── */
#define SB_FW_BURST_SIZE        16U

/* ── Timeouts ────────────────────────────────────────────────── */
#define OTA_SESSION_TIMEOUT_MS  120000UL
#define OTA_IDLE_TIMEOUT_MS     30000UL

/* ── Ring buffer type & storage ───────────────────────────────── */
typedef struct { uint8_t payload[64]; } BootRxEntry;
#pragma DATA_SECTION(bootRxRing, ".boot_ring")
static BootRxEntry bootRxRing[BOOT_RX_RING_SIZE];
static volatile uint16_t bootRxHead = 0;
static uint16_t bootRxTail = 0;

/* ── FW receive state machine ────────────────────────────────── */
typedef enum {
    BOOT_RX_IDLE,
    BOOT_RX_PREPARING,
    BOOT_RX_WAITING_HEADER,
    BOOT_RX_RECEIVING,
    BOOT_RX_VERIFYING,
    BOOT_RX_DONE,
    BOOT_RX_ABORT
} BootRxState;

/* ── OTA receive state variables ─────────────────────────────── */
static volatile BootRxState bootRxState = BOOT_RX_IDLE;
static uint32_t bootRxImageSize = 0;
static uint32_t bootRxImageCRC = 0;
static uint32_t bootRxWriteAddr = 0;
static uint32_t bootRxBytesReceived = 0;
static uint16_t bootRxBurstCount = 0;
static uint16_t bootRxSeq = 0;
static volatile uint8_t bootRxPendingCmd = 0;
static volatile uint8_t bootRxCmdData[64];

/* ── Prototypes ───────────────────────────────────────────────── */
static void      ledInit(void);
static void      ledSet(uint16_t on);
static void      configureMCANA(void);
static void      configureMCANB(void);
static void      canSendMsg(uint16_t canId, const uint8_t *payload, uint16_t len);
static void      canSendHello(void);
static void      canSendDebug(uint16_t canId, uint8_t b0, uint8_t b1,
                               uint32_t val1, uint32_t val2);
static void      sendBootHeartbeat(void);
static uint32_t  computeCRC32(uint32_t startAddr, uint32_t numBytes);
static void      delayCycles(uint32_t cycles);
static void      jumpToApp(void);

/* These MUST run from RAM — declared here, pragma'd below */
static void      clearFSM(void);
static void      eraseSector(uint32_t sectorAddr);
static void      programEightWords(uint32_t destAddr, const uint16_t *src);
static void      copyBank2ToBank0(uint32_t imageSize);
static void      clearBootFlag(void);

/* OTA FW receive functions */
__interrupt void         bootMcanISR(void);
static void              configureMCANB_RX(void);
static void              bootFwSendResponse(uint8_t respCode, uint16_t seq);
static Fapi_StatusType   writePayloadToBank2(const uint8_t *payload);
static void              eraseOtaFlag(void);
static void              writeBootFlagForOTA(void);
static void              eraseStagingBank(void);
static void              bootFwReceiveMode(void);

/* ── CRC32 table ──────────────────────────────────────────────── */
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

/* ══════════════════════════════════════════════════════════════
 *  MAIN
 * ══════════════════════════════════════════════════════════════ */
void main(void)
{
    volatile uint16_t *flag;
    uint16_t updatePending, crcValid;
    uint32_t imageSize, imageCRC, computedCRC;

    Device_init();

    ledInit();
    ledSet(1U);
    canSendHello();         /* Inits MCANA (BU bus) + MCANB (M-Board bus), sends hello */
    sendBootHeartbeat();    /* 0x7FF on MCANB — tells M-Board we are in boot mode */
    delayCycles(DEVICE_SYSCLK_FREQ / 7U);
    ledSet(0U);

    /* Read boot flag */
    flag          = (volatile uint16_t *)BANK3_FLAG_ADDR;
    updatePending = flag[0];
    crcValid      = flag[1];
    imageSize     = (uint32_t)flag[2] | ((uint32_t)flag[3] << 16);
    imageCRC      = (uint32_t)flag[4] | ((uint32_t)flag[5] << 16);

    /* Report flag over CAN */
    canSendDebug(CAN_ID_FLAG_STATUS,
                 (uint8_t)(updatePending & 0xFF),
                 (uint8_t)(updatePending >> 8),
                 imageSize, imageCRC);

    if ((updatePending == FLAG_UPDATE_PENDING) &&
        (crcValid      == FLAG_CRC_VALID))
    {
        /* LED on steady = update in progress */
        ledSet(1U);

        /* Init Flash API */
        EALLOW;
        Fapi_initializeAPI(FlashTech_CPU0_BASE_ADDRESS,
                           DEVICE_SYSCLK_FREQ / 1000000U);
        Fapi_setActiveFlashBank(Fapi_FlashBank0);
        EDIS;

        /* Verify Bank 2 CRC */
        computedCRC = computeCRC32(BANK2_START, imageSize);

        canSendDebug(CAN_ID_CRC_RESULT,
                     (computedCRC == imageCRC) ? 0x01 : 0x00, 0x00,
                     imageCRC, computedCRC);

        if (computedCRC == imageCRC)
        {
            canSendDebug(CAN_ID_ERASE_PROG, 0x01, 0x00, APP_SECTOR_COUNT, 0);

            /* ALL flash operations run from RAM with EALLOW active */
            EALLOW;
            copyBank2ToBank0(imageSize);
            EDIS;

            canSendDebug(CAN_ID_COPY_DONE, 0x01, 0x00, imageSize, 0);

            EALLOW;
            clearBootFlag();
            EDIS;

            canSendDebug(CAN_ID_COPY_DONE, 0x02, 0x00, 0, 0);

            ledSet(0U);
            SysCtl_resetDevice();
            /* unreachable */
        }
        else
        {
            canSendDebug(CAN_ID_CRC_FAIL, 0xFF, 0x00, imageCRC, computedCRC);
            EALLOW;
            clearBootFlag();
            EDIS;
        }

        ledSet(0U);
    }
    else
    {
        canSendDebug(CAN_ID_NO_UPDATE,
                     (uint8_t)(updatePending & 0xFF),
                     (uint8_t)(crcValid & 0xFF), 0, 0);
    }

    /* Check OTA receive-mode flag at Bank 3 sector 1 */
    {
        volatile uint16_t *otaFlag = (volatile uint16_t *)BANK3_OTA_FLAG_ADDR;
        if (otaFlag[0] == FLAG_OTA_MODE_MAGIC)
        {
            bootFwReceiveMode();
            /* If we return here, OTA was aborted/timed out — jump to app */
        }
    }

    jumpToApp();
}

/* ══════════════════════════════════════════════════════════════
 *  LED (GPIO 21)
 * ══════════════════════════════════════════════════════════════ */
static void ledInit(void)
{
    GPIO_setPinConfig(GPIO_21_GPIO21);
    GPIO_setPadConfig(21U, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(21U, GPIO_DIR_MODE_OUT);
    GPIO_writePin(21U, 1U);
}

static void ledSet(uint16_t on)
{
    GPIO_writePin(21U, on ? 0U : 1U);
}

/* ══════════════════════════════════════════════════════════════
 *  MCANA — BU-Board bus (needed for BU firmware OTA relay)
 * ══════════════════════════════════════════════════════════════ */
static void configureMCANA(void)
{
    MCAN_InitParams         initParams;
    MCAN_MsgRAMConfigParams ramCfg;
    MCAN_BitTimingParams    bitTimes;

    memset(&initParams, 0, sizeof(initParams));
    memset(&ramCfg,     0, sizeof(ramCfg));
    memset(&bitTimes,   0, sizeof(bitTimes));

    SysCtl_setMCANClk(CAN_BU_SYSCTL, SYSCTL_MCANCLK_DIV_5);
    GPIO_setPinConfig(CAN_BU_TX_PIN);
    GPIO_setPinConfig(CAN_BU_RX_PIN);

    initParams.fdMode    = 0x1U;
    initParams.brsEnable = 0x1U;

    ramCfg.txStartAddr   = BOOT_MCAN_TX_BUF_ADDR;
    ramCfg.txBufNum      = BOOT_MCAN_TX_BUF_NUM;
    ramCfg.txBufElemSize = MCAN_ELEM_SIZE_64BYTES;

    bitTimes.nomRatePrescalar  = 0x5U;
    bitTimes.nomTimeSeg1       = 0x6U;
    bitTimes.nomTimeSeg2       = 0x1U;
    bitTimes.nomSynchJumpWidth = 0x1U;
    bitTimes.dataRatePrescalar  = 0x0U;
    bitTimes.dataTimeSeg1       = 0xAU;
    bitTimes.dataTimeSeg2       = 0x2U;
    bitTimes.dataSynchJumpWidth = 0x2U;

    while (FALSE == MCAN_isMemInitDone(CAN_BU_BASE)) { }
    MCAN_setOpMode(CAN_BU_BASE, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(CAN_BU_BASE)) { }
    MCAN_init(CAN_BU_BASE, &initParams);
    MCAN_setBitTime(CAN_BU_BASE, &bitTimes);
    MCAN_msgRAMConfig(CAN_BU_BASE, &ramCfg);
    MCAN_setOpMode(CAN_BU_BASE, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(CAN_BU_BASE)) { }
}

static void canSendMsg(uint16_t canId, const uint8_t *payload, uint16_t len)
{
    MCAN_TxBufElement txMsg;
    volatile uint32_t timeout;
    uint16_t i;

    memset(&txMsg, 0, sizeof(txMsg));
    txMsg.id  = ((uint32_t)canId) << 18U;
    txMsg.dlc = 8U;
    txMsg.brs = 0x1U;
    txMsg.fdf = 0x1U;
    txMsg.efc = 1U;
    txMsg.mm  = 0xB0U;
    for (i = 0U; i < 8U && i < len; i++)
        txMsg.data[i] = payload[i];

    /* All S-Board boot manager communication goes to M-Board via MCANB */
    MCAN_writeMsgRam(CAN_MBOARD_BASE, MCAN_MEM_TYPE_BUF, 0U, &txMsg);
    MCAN_txBufAddReq(CAN_MBOARD_BASE, 0U);
    for (timeout = 0U; timeout < 2000000U; timeout++)
        if (MCAN_getTxBufReqPend(CAN_MBOARD_BASE) == 0U) break;
}

static void canSendHello(void)
{
    uint8_t p[8] = {'B','O','O','T','M','G','R',0x01};
    configureMCANA();   /* Init MCANA for BU-Board OTA relay */
    configureMCANB();   /* Init MCANB for M-Board communication */
    canSendMsg(CAN_ID_HELLO, p, 8U);
}

static void canSendDebug(uint16_t canId, uint8_t b0, uint8_t b1,
                          uint32_t val1, uint32_t val2)
{
    uint8_t p[8];
    p[0] = b0; p[1] = b1;
    p[2] = (uint8_t)(val1);        p[3] = (uint8_t)(val1 >> 8);
    p[4] = (uint8_t)(val1 >> 16);  p[5] = (uint8_t)(val1 >> 24);
    p[6] = (uint8_t)(val2);        p[7] = (uint8_t)(val2 >> 8);
    canSendMsg(canId, p, 8U);
}

/* ══════════════════════════════════════════════════════════════
 *  MCANB — M-Board bus for heartbeat
 * ══════════════════════════════════════════════════════════════ */
static void configureMCANB(void)
{
    MCAN_InitParams         initParams;
    MCAN_MsgRAMConfigParams ramCfg;
    MCAN_BitTimingParams    bitTimes;

    memset(&initParams, 0, sizeof(initParams));
    memset(&ramCfg,     0, sizeof(ramCfg));
    memset(&bitTimes,   0, sizeof(bitTimes));

    SysCtl_setMCANClk(CAN_MBOARD_SYSCTL, SYSCTL_MCANCLK_DIV_5);
    GPIO_setPinConfig(CAN_MBOARD_TX_PIN);
    GPIO_setPinConfig(CAN_MBOARD_RX_PIN);

    initParams.fdMode    = 0x1U;
    initParams.brsEnable = 0x1U;

    ramCfg.txStartAddr   = BOOT_MCAN_TX_BUF_ADDR;
    ramCfg.txBufNum      = BOOT_MCAN_TX_BUF_NUM;
    ramCfg.txBufElemSize = MCAN_ELEM_SIZE_64BYTES;

    bitTimes.nomRatePrescalar  = 0x5U;
    bitTimes.nomTimeSeg1       = 0x6U;
    bitTimes.nomTimeSeg2       = 0x1U;
    bitTimes.nomSynchJumpWidth = 0x1U;
    bitTimes.dataRatePrescalar  = 0x0U;
    bitTimes.dataTimeSeg1       = 0xAU;
    bitTimes.dataTimeSeg2       = 0x2U;
    bitTimes.dataSynchJumpWidth = 0x2U;

    while (FALSE == MCAN_isMemInitDone(CAN_MBOARD_BASE)) { }
    MCAN_setOpMode(CAN_MBOARD_BASE, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(CAN_MBOARD_BASE)) { }
    MCAN_init(CAN_MBOARD_BASE, &initParams);
    MCAN_setBitTime(CAN_MBOARD_BASE, &bitTimes);
    MCAN_msgRAMConfig(CAN_MBOARD_BASE, &ramCfg);
    MCAN_setOpMode(CAN_MBOARD_BASE, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(CAN_MBOARD_BASE)) { }
}

/* ══════════════════════════════════════════════════════════════
 *  HEARTBEAT — 0x7FF on MCANB (M-Board bus) = "I am in boot mode"
 * ══════════════════════════════════════════════════════════════ */
static void sendBootHeartbeat(void)
{
    MCAN_TxBufElement txMsg;
    volatile uint32_t timeout;

    memset(&txMsg, 0, sizeof(txMsg));
    txMsg.id  = ((uint32_t)HEARTBEAT_BOOT_CAN_ID) << 18U;
    txMsg.dlc = 8U;
    txMsg.brs = 0x1U;
    txMsg.fdf = 0x1U;
    txMsg.efc = 1U;
    txMsg.mm  = 0xB0U;
    txMsg.data[0] = 'B';
    txMsg.data[1] = 'O';
    txMsg.data[2] = 'O';
    txMsg.data[3] = 'T';

    MCAN_writeMsgRam(CAN_MBOARD_BASE, MCAN_MEM_TYPE_BUF, 0U, &txMsg);
    MCAN_txBufAddReq(CAN_MBOARD_BASE, 0U);
    for (timeout = 0U; timeout < 2000000U; timeout++)
        if (MCAN_getTxBufReqPend(CAN_MBOARD_BASE) == 0U) break;
}

/* ══════════════════════════════════════════════════════════════
 *  DELAY
 * ══════════════════════════════════════════════════════════════ */
static void delayCycles(uint32_t cycles)
{
    volatile uint32_t i;
    for (i = 0U; i < cycles; i++) { __asm(" NOP"); }
}

/* ══════════════════════════════════════════════════════════════
 *  JUMP TO APPLICATION
 * ══════════════════════════════════════════════════════════════ */
static void jumpToApp(void)
{
    ((void (*)(void))APP_ENTRY_ADDR)();
}

/* ══════════════════════════════════════════════════════════════
 *  RAM-RESIDENT FLASH FUNCTIONS
 *
 *  ALL functions that touch the flash state machine MUST run
 *  from RAM. The flash module is inaccessible during erase/program,
 *  so code executing from flash would stall the CPU.
 * ══════════════════════════════════════════════════════════════ */

/* Clear any pending FSM error flags — matches app-side ClearFSMStatus() */
#pragma CODE_SECTION(clearFSM, ".TI.ramfunc")
static void clearFSM(void)
{
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) { }

    if (Fapi_getFsmStatus() != 0U)
    {
        Fapi_issueAsyncCommand(Fapi_ClearStatus);
        while (Fapi_getFsmStatus() != 0U) { }
    }
}

#pragma CODE_SECTION(eraseSector, ".TI.ramfunc")
static void eraseSector(uint32_t sectorAddr)
{
    clearFSM();

    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC_UNLOCK_A);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC_UNLOCK_B);

    Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                      (uint32_t *)sectorAddr);

    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) { }
}

#pragma CODE_SECTION(programEightWords, ".TI.ramfunc")
static void programEightWords(uint32_t destAddr, const uint16_t *src)
{
    clearFSM();

    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC_UNLOCK_A);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC_UNLOCK_B);

    Fapi_issueProgrammingCommand((uint32_t *)destAddr,
                                 (uint16_t *)src, 8U,
                                 0, 0, Fapi_AutoEccGeneration);

    while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy) { }
}

#pragma CODE_SECTION(copyBank2ToBank0, ".TI.ramfunc")
static void copyBank2ToBank0(uint32_t imageSize)
{
    uint32_t sectorAddr, srcAddr, dstAddr, wordsRemaining;
    uint16_t i;

    /* Erase application area */
    for (i = 0U; i < APP_SECTOR_COUNT; i++)
    {
        sectorAddr = BANK0_APP_START + ((uint32_t)i * SECTOR_SIZE_WORDS);
        eraseSector(sectorAddr);
    }

    /* Program 8 words at a time */
    srcAddr        = BANK2_START;
    dstAddr        = BANK0_APP_START;
    wordsRemaining = (imageSize + 1U) / 2U;

    while (wordsRemaining >= 8U)
    {
        programEightWords(dstAddr, (const uint16_t *)srcAddr);
        srcAddr        += 8U;
        dstAddr        += 8U;
        wordsRemaining -= 8U;
    }

    /* Trailing words padded with 0xFFFF */
    if (wordsRemaining > 0U)
    {
        uint16_t padBuf[8];
        for (i = 0U; i < 8U; i++)
            padBuf[i] = (i < wordsRemaining) ?
                         *((volatile uint16_t *)(srcAddr + i)) : 0xFFFFU;
        programEightWords(dstAddr, padBuf);
    }
}

#pragma CODE_SECTION(clearBootFlag, ".TI.ramfunc")
static void clearBootFlag(void)
{
    eraseSector(BANK3_FLAG_ADDR);
}

/* ══════════════════════════════════════════════════════════════
 *  OTA CAN ISR — MUST execute from RAM
 *
 *  Handles RX FIFO 0 (data frames, ID 6) and dedicated
 *  RX buffer 0 (command frames, ID 7) on MCANB.
 * ══════════════════════════════════════════════════════════════ */
#pragma CODE_SECTION(bootMcanISR, ".TI.ramfunc")
__interrupt void bootMcanISR(void)
{
    uint32_t intrStatus = MCAN_getIntrStatus(CAN_MBOARD_BASE);
    MCAN_clearIntrStatus(CAN_MBOARD_BASE, intrStatus);
    MCAN_clearInterrupt(CAN_MBOARD_BASE, 0x2U);

    /* RX FIFO 0 — data frames (ID 6) */
    if (intrStatus & MCAN_IR_RF0N_MASK)
    {
        MCAN_RxFIFOStatus fifoSt;
        fifoSt.num = MCAN_RX_FIFO_NUM_0;
        MCAN_getRxFIFOStatus(CAN_MBOARD_BASE, &fifoSt);
        while (fifoSt.fillLvl > 0)
        {
            MCAN_RxBufElement rxElem;
            MCAN_readMsgRam(CAN_MBOARD_BASE, MCAN_MEM_TYPE_FIFO, 0U,
                            MCAN_RX_FIFO_NUM_0, &rxElem);

            /* Copy payload to ring buffer if in RECEIVING state */
            if (bootRxState == BOOT_RX_RECEIVING)
            {
                uint16_t nextHead = (bootRxHead + 1U) & BOOT_RX_RING_MASK;
                if (nextHead != bootRxTail)  /* not full */
                {
                    uint16_t i;
                    for (i = 0; i < 64; i++)
                        bootRxRing[bootRxHead].payload[i] = rxElem.data[i];
                    bootRxHead = nextHead;
                }
            }

            MCAN_writeRxFIFOAck(CAN_MBOARD_BASE, MCAN_RX_FIFO_NUM_0, fifoSt.getIdx);
            MCAN_getRxFIFOStatus(CAN_MBOARD_BASE, &fifoSt);
        }
    }

    /* Dedicated RX buffer — command frames (ID 7) */
    if (intrStatus & MCAN_IR_DRX_MASK)
    {
        MCAN_RxNewDataStatus newData;
        MCAN_getNewDataStatus(CAN_MBOARD_BASE, &newData);
        if (newData.statusLow & (1UL << 0U))  /* buffer 0 */
        {
            MCAN_RxBufElement rxElem;
            MCAN_readMsgRam(CAN_MBOARD_BASE, MCAN_MEM_TYPE_BUF, 0U, 0U, &rxElem);

            /* S-Board has no board addressing — accept all commands */
            bootRxPendingCmd = rxElem.data[0];
            {
                uint16_t i;
                for (i = 0; i < 64; i++)
                    bootRxCmdData[i] = rxElem.data[i];
            }
        }
        MCAN_clearNewDataStatus(CAN_MBOARD_BASE, &newData);
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

/* ══════════════════════════════════════════════════════════════
 *  OTA CAN RX Configuration — reconfigure MCANB with RX FIFO
 *  + filters for OTA CAN IDs (6 = data, 7 = command)
 * ══════════════════════════════════════════════════════════════ */
static void configureMCANB_RX(void)
{
    MCAN_InitParams initParams;
    MCAN_MsgRAMConfigParams ramCfg;
    MCAN_StdMsgIDFilterElement filt;
    MCAN_BitTimingParams bitTimes;

    memset(&initParams, 0, sizeof(initParams));
    memset(&ramCfg, 0, sizeof(ramCfg));
    memset(&filt, 0, sizeof(filt));
    memset(&bitTimes, 0, sizeof(bitTimes));

    MCAN_setOpMode(CAN_MBOARD_BASE, MCAN_OPERATION_MODE_SW_INIT);
    while (MCAN_OPERATION_MODE_SW_INIT != MCAN_getOpMode(CAN_MBOARD_BASE)) {}

    initParams.fdMode = 0x1U;
    initParams.brsEnable = 0x1U;

    /* Message RAM: 3 filters, FIFO0 (16 entries for data), 1 RX buf (cmd), 2 TX bufs */
    ramCfg.flssa = 0x0U;
    ramCfg.lss = 3U;
    ramCfg.rxFIFO0startAddr = 3U * 4U;  /* after 3 filter elements */
    ramCfg.rxFIFO0size = 16U;
    ramCfg.rxFIFO0OpMode = 1U;  /* overwrite */
    ramCfg.rxFIFO0ElemSize = MCAN_ELEM_SIZE_64BYTES;
    ramCfg.rxBufStartAddr = ramCfg.rxFIFO0startAddr + (18U * 4U * 16U);
    ramCfg.rxBufElemSize = MCAN_ELEM_SIZE_64BYTES;
    ramCfg.txStartAddr = ramCfg.rxBufStartAddr + (18U * 4U * 1U);
    ramCfg.txBufNum = 2U;
    ramCfg.txBufElemSize = MCAN_ELEM_SIZE_64BYTES;

    bitTimes.nomRatePrescalar = 0x5U;
    bitTimes.nomTimeSeg1 = 0x6U;
    bitTimes.nomTimeSeg2 = 0x1U;
    bitTimes.nomSynchJumpWidth = 0x1U;
    bitTimes.dataRatePrescalar = 0x0U;
    bitTimes.dataTimeSeg1 = 0xAU;
    bitTimes.dataTimeSeg2 = 0x2U;
    bitTimes.dataSynchJumpWidth = 0x2U;

    MCAN_init(CAN_MBOARD_BASE, &initParams);
    MCAN_setBitTime(CAN_MBOARD_BASE, &bitTimes);
    MCAN_msgRAMConfig(CAN_MBOARD_BASE, &ramCfg);

    /* Filter 0: ID 6 (data) -> FIFO 0 */
    filt.sfec = MCAN_STDFILTEC_FIFO0;
    filt.sft = MCAN_STDFILT_CLASSIC;
    filt.sfid1 = SB_FW_DATA_CAN_ID;
    filt.sfid2 = 0x7FFU;
    MCAN_addStdMsgIDFilter(CAN_MBOARD_BASE, 0U, &filt);

    /* Filter 1: ID 7 (cmd) -> Dedicated RX Buffer 0 */
    filt.sfec = MCAN_STDFILTEC_RXBUFF;
    filt.sft = MCAN_STDFILT_CLASSIC;
    filt.sfid1 = SB_FW_CMD_CAN_ID;
    filt.sfid2 = 0x000U;  /* buffer index 0 */
    MCAN_addStdMsgIDFilter(CAN_MBOARD_BASE, 1U, &filt);

    /* Filter 2: reject all others */
    filt.sfec = MCAN_STDFILTEC_REJECT;
    filt.sft = MCAN_STDFILT_CLASSIC;
    filt.sfid1 = 0x000U;
    filt.sfid2 = 0x000U;
    MCAN_addStdMsgIDFilter(CAN_MBOARD_BASE, 2U, &filt);

    MCAN_setOpMode(CAN_MBOARD_BASE, MCAN_OPERATION_MODE_NORMAL);
    while (MCAN_OPERATION_MODE_NORMAL != MCAN_getOpMode(CAN_MBOARD_BASE)) {}

    /* Enable interrupts */
    MCAN_enableIntr(CAN_MBOARD_BASE, MCAN_INTR_MASK_ALL, 1U);
    MCAN_selectIntrLine(CAN_MBOARD_BASE, MCAN_INTR_MASK_ALL, MCAN_INTR_LINE_NUM_1);
    MCAN_enableIntrLine(CAN_MBOARD_BASE, MCAN_INTR_LINE_NUM_1, 1U);

    Interrupt_register(CAN_MBOARD_INT_LINE1, &bootMcanISR);
    Interrupt_enable(CAN_MBOARD_INT_LINE1);
}

/* ══════════════════════════════════════════════════════════════
 *  OTA CAN Response — send response on CAN ID 8 via MCANB
 * ══════════════════════════════════════════════════════════════ */
static void bootFwSendResponse(uint8_t respCode, uint16_t seq)
{
    MCAN_TxBufElement txMsg;
    volatile uint32_t timeout;

    memset(&txMsg, 0, sizeof(txMsg));
    txMsg.id = ((uint32_t)SB_FW_RESP_CAN_ID) << 18U;
    txMsg.dlc = 15U;
    txMsg.brs = 0x1U;
    txMsg.fdf = 0x1U;
    txMsg.efc = 1U;
    txMsg.mm = 0xCFU;
    txMsg.data[0] = respCode;
    txMsg.data[1] = 0x00U;  /* S-Board has no board ID — always 0 */
    txMsg.data[2] = (uint8_t)(seq & 0xFFU);
    txMsg.data[3] = (uint8_t)((seq >> 8) & 0xFFU);
    txMsg.data[4] = (uint8_t)bootRxState;
    txMsg.data[5] = (uint8_t)(bootRxBytesReceived & 0xFFU);
    txMsg.data[6] = (uint8_t)((bootRxBytesReceived >> 8) & 0xFFU);
    txMsg.data[7] = (uint8_t)((bootRxBytesReceived >> 16) & 0xFFU);

    MCAN_writeMsgRam(CAN_MBOARD_BASE, MCAN_MEM_TYPE_BUF, 0U, &txMsg);
    MCAN_txBufAddReq(CAN_MBOARD_BASE, 0U);
    for (timeout = 0U; timeout < 2000000U; timeout++)
        if (MCAN_getTxBufReqPend(CAN_MBOARD_BASE) == 0U) break;
}

/* ══════════════════════════════════════════════════════════════
 *  RAM-RESIDENT OTA FLASH FUNCTIONS
 * ══════════════════════════════════════════════════════════════ */

/* Write 64 bytes (one CAN-FD payload) to Bank 2 staging area */
#pragma CODE_SECTION(writePayloadToBank2, ".TI.ramfunc")
static Fapi_StatusType writePayloadToBank2(const uint8_t *payload)
{
    uint16_t chunk;
    for (chunk = 0; chunk < 4U; chunk++)
    {
        uint16_t wordBuf[8];
        uint16_t i;
        for (i = 0; i < 8U; i++)
        {
            uint16_t byteIdx = (chunk * 16U) + (i * 2U);
            wordBuf[i] = ((uint16_t)payload[byteIdx]) |
                          ((uint16_t)payload[byteIdx + 1U] << 8);
        }

        Fapi_StatusType st;
        clearFSM();
        Fapi_setupBankSectorEnable(
            FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC_UNLOCK_A);
        Fapi_setupBankSectorEnable(
            FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC_UNLOCK_B);
        st = Fapi_issueProgrammingCommand((uint32_t *)bootRxWriteAddr,
                                           wordBuf, 8U, 0, 0,
                                           Fapi_AutoEccGeneration);
        if (st != Fapi_Status_Success) return st;
        while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy) {}
        bootRxWriteAddr += 8U;
    }
    return Fapi_Status_Success;
}

/* Clear the OTA receive-mode flag (Bank 3 sector 1) */
#pragma CODE_SECTION(eraseOtaFlag, ".TI.ramfunc")
static void eraseOtaFlag(void)
{
    clearFSM();
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC_UNLOCK_A);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC_UNLOCK_B);
    Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                      (uint32_t *)BANK3_OTA_FLAG_ADDR);
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) {}
}

/* Write copy-flag to Bank 3 sector 0 after successful OTA receive */
#pragma CODE_SECTION(writeBootFlagForOTA, ".TI.ramfunc")
static void writeBootFlagForOTA(void)
{
    uint16_t flagData[8];
    flagData[0] = FLAG_UPDATE_PENDING;  /* 0xA5A5 */
    flagData[1] = FLAG_CRC_VALID;       /* 0x5A5A */
    flagData[2] = (uint16_t)(bootRxImageSize & 0xFFFFU);
    flagData[3] = (uint16_t)((bootRxImageSize >> 16) & 0xFFFFU);
    flagData[4] = (uint16_t)(bootRxImageCRC & 0xFFFFU);
    flagData[5] = (uint16_t)((bootRxImageCRC >> 16) & 0xFFFFU);
    flagData[6] = 0xFFFFU;
    flagData[7] = 0xFFFFU;

    /* Erase Bank 3 sector 0 */
    clearFSM();
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC_UNLOCK_A);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC_UNLOCK_B);
    Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                      (uint32_t *)BANK3_FLAG_ADDR);
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady) {}

    /* Program flag */
    clearFSM();
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTA, SEC_UNLOCK_A);
    Fapi_setupBankSectorEnable(
        FLASH_WRAPPER_PROGRAM_BASE + FLASH_O_CMDWEPROTB, SEC_UNLOCK_B);
    Fapi_issueProgrammingCommand((uint32_t *)BANK3_FLAG_ADDR,
                                  flagData, 8U, 0, 0,
                                  Fapi_AutoEccGeneration);
    while (Fapi_checkFsmForReady() == Fapi_Status_FsmBusy) {}
}

/* Erase all 128 sectors of Bank 2 (staging area) */
static void eraseStagingBank(void)
{
    uint16_t i;
    EALLOW;
    for (i = 0; i < BANK2_SECTOR_COUNT; i++)
    {
        uint32_t addr = BANK2_START + ((uint32_t)i * SECTOR_SIZE_WORDS);
        eraseSector(addr);
    }
    EDIS;
}

/* ══════════════════════════════════════════════════════════════
 *  OTA FW RECEIVE MODE — main receive function
 *
 *  Called from main() when the OTA receive-mode flag is set.
 *  Blocks until firmware is received + activated, or aborted/
 *  timed out.
 * ══════════════════════════════════════════════════════════════ */
static void bootFwReceiveMode(void)
{
    /* Init Flash API for Bank 2 writes */
    EALLOW;
    Fapi_initializeAPI(FlashTech_CPU0_BASE_ADDRESS,
                       DEVICE_SYSCLK_FREQ / 1000000U);
    Fapi_setActiveFlashBank(Fapi_FlashBank0);
    EDIS;

    /* Clear OTA flag so we don't loop back here on next boot */
    EALLOW;
    eraseOtaFlag();
    EDIS;

    /* Reconfigure MCANB with RX filters for OTA */
    configureMCANB_RX();
    EINT;

    /* Init state */
    bootRxState = BOOT_RX_IDLE;
    bootRxHead = 0;
    bootRxTail = 0;
    bootRxPendingCmd = 0;
    bootRxBytesReceived = 0;
    bootRxBurstCount = 0;
    bootRxSeq = 0;
    bootRxWriteAddr = BANK2_START;

    ledSet(1U);  /* LED on = in OTA mode */
    canSendDebug(0x29U, 0x01U, 0x00U, 0U, 0U);  /* Debug: entered OTA mode */

    /* Simple timeout using busy-wait counter */
    volatile uint32_t idleCounter = 0;
    uint32_t idleLimit = 150000000UL * 30U;  /* ~30 seconds at 150MHz */

    while (bootRxState != BOOT_RX_DONE && bootRxState != BOOT_RX_ABORT)
    {
        /* Process pending command */
        if (bootRxPendingCmd != 0)
        {
            uint8_t cmd = bootRxPendingCmd;
            bootRxPendingCmd = 0;
            idleCounter = 0;  /* Reset timeout on any command */

            switch (cmd)
            {
                case SB_CMD_FW_START:
                    bootRxState = BOOT_RX_PREPARING;
                    eraseStagingBank();
                    bootRxWriteAddr = BANK2_START;
                    bootRxBytesReceived = 0;
                    bootRxBurstCount = 0;
                    bootRxSeq = 0;
                    bootRxState = BOOT_RX_WAITING_HEADER;
                    bootFwSendResponse(SB_RESP_FW_ACK, 0);
                    break;

                case SB_CMD_FW_HEADER:
                    bootRxImageSize = ((uint32_t)bootRxCmdData[4]) |
                                      ((uint32_t)bootRxCmdData[5] << 8) |
                                      ((uint32_t)bootRxCmdData[6] << 16) |
                                      ((uint32_t)bootRxCmdData[7] << 24);
                    bootRxImageCRC  = ((uint32_t)bootRxCmdData[8]) |
                                      ((uint32_t)bootRxCmdData[9] << 8) |
                                      ((uint32_t)bootRxCmdData[10] << 16) |
                                      ((uint32_t)bootRxCmdData[11] << 24);
                    bootRxState = BOOT_RX_RECEIVING;
                    bootFwSendResponse(SB_RESP_FW_ACK, 0);
                    break;

                case SB_CMD_FW_COMPLETE:
                {
                    /* Flush any remaining burst ACK */
                    if (bootRxBurstCount > 0)
                    {
                        bootFwSendResponse(SB_RESP_FW_ACK, bootRxSeq);
                        bootRxBurstCount = 0;
                    }
                    bootRxState = BOOT_RX_VERIFYING;
                    uint32_t computed = computeCRC32(BANK2_START, bootRxImageSize);
                    if (computed == bootRxImageCRC)
                    {
                        bootFwSendResponse(SB_RESP_FW_CRC_PASS, bootRxSeq);
                        delayCycles(DEVICE_SYSCLK_FREQ / 100U);
                        /* Write copy-flag then reset — boot manager will copy Bank2->Bank0 */
                        EALLOW;
                        writeBootFlagForOTA();
                        EDIS;
                        SysCtl_resetDevice();
                        /* unreachable */
                    }
                    else
                    {
                        bootFwSendResponse(SB_RESP_FW_CRC_FAIL, bootRxSeq);
                        bootRxState = BOOT_RX_ABORT;
                    }
                    break;
                }

                default:
                    break;
            }
        }

        /* Drain ring buffer to Bank 2 flash */
        if (bootRxState == BOOT_RX_RECEIVING)
        {
            while (bootRxTail != bootRxHead)
            {
                idleCounter = 0;

                EALLOW;
                writePayloadToBank2(bootRxRing[bootRxTail].payload);
                EDIS;

                bootRxTail = (bootRxTail + 1U) & BOOT_RX_RING_MASK;
                bootRxBytesReceived += 64U;
                bootRxSeq++;
                bootRxBurstCount++;

                if (bootRxBurstCount >= SB_FW_BURST_SIZE)
                {
                    bootFwSendResponse(SB_RESP_FW_ACK, bootRxSeq);
                    bootRxBurstCount = 0;
                }
            }

            /* Final burst ACK if all bytes received */
            if (bootRxBurstCount > 0 && bootRxBytesReceived >= bootRxImageSize)
            {
                bootFwSendResponse(SB_RESP_FW_ACK, bootRxSeq);
                bootRxBurstCount = 0;
            }
        }

        /* Timeout */
        idleCounter++;
        if (idleCounter >= idleLimit)
        {
            bootRxState = BOOT_RX_ABORT;
        }
    }

    ledSet(0U);
    DINT;
}

/* ══════════════════════════════════════════════════════════════
 *  CRC32
 * ══════════════════════════════════════════════════════════════ */
static uint32_t computeCRC32(uint32_t startAddr, uint32_t numBytes)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t numWords = (numBytes + 1U) / 2U;
    volatile uint16_t *wordPtr = (volatile uint16_t *)startAddr;
    uint32_t i;

    for (i = 0U; i < numWords; i++)
    {
        uint16_t word = wordPtr[i];
        crc = (crc >> 8) ^ crc32Table[(crc ^ (word & 0xFFU)) & 0xFFU];
        crc = (crc >> 8) ^ crc32Table[(crc ^ (word >> 8))     & 0xFFU];
    }
    return crc ^ 0xFFFFFFFFU;
}
