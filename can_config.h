/**
 * @file can_config.h
 * @brief Configurable CAN bus mapping for S-Board Boot Manager
 *
 * The boot manager uses a single CAN bus to send debug/status messages
 * during the boot process. Change these definitions to use a different
 * MCAN module or GPIO pins.
 *
 * Current mapping (production S-Board):
 *   MCANA (GPIO 1 TX, GPIO 0 RX) --> BU-Board bus (debug output)
 */

#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

/* ═══════════════════════════════════════════════════════════════
 *  Boot CAN bus — uses the BU-Board bus for debug messages
 * ═══════════════════════════════════════════════════════════════ */
#define CAN_BU_BASE             MCANA_DRIVER_BASE
#define CAN_BU_MSG_RAM_BASE     MCANA_MSG_RAM_BASE
#define CAN_BU_TX_PIN           GPIO_1_MCANA_TX
#define CAN_BU_RX_PIN           GPIO_0_MCANA_RX
#define CAN_BU_SYSCTL           SYSCTL_MCANA
#define CAN_BU_INT_LINE1        INT_MCANA_1

#endif /* CAN_CONFIG_H */
