/* ========================================
 *
 * Copyright Hamish Johnson, 2026
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF
 * Quantum Internet Systems Lab (QISL),
 * Department of Physics, SFU, Canada.
 *
 * Author: Hamish Johnson (2026)
 *
 * ========================================
 *
 * Generic EEPROM slot store for PSoC 5LP.
 * Wraps the EEPROM_CONFIG component (PSoC Creator EEPROM v3.0).
 *
 * The PSoC 5LP EEPROM write API works in 16-byte rows.
 * Each slot occupies exactly one row (16 bytes).
 * Slot data must fit within PSOC_EEPROM_ROW_DATA_SIZE bytes.
 *
 * Layout per row:
 *   [magic_lo:1][magic_hi:1][len:1][data:13]
 *   Total = 16 bytes = one EEPROM row.
 *
 * Row assignments are fixed per slot ID — defined below.
 * Reading is direct memory access via CYDEV_EE_BASE.
 * Writing calls EEPROM_CONFIG_UpdateTemperature() then
 * EEPROM_CONFIG_Write(rowData, rowNumber).
 *
 * Adding a new slot:
 *   1. Define PSOC_EEPROM_SLOT_* ID and ROW below.
 *   2. Call psoc_eeprom_write/read with that ID and your struct.
 *   3. Struct must be <= PSOC_EEPROM_ROW_DATA_SIZE (13) bytes.
 */

#ifndef PSOC_EEPROM_H
#define PSOC_EEPROM_H

#include <project.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Row geometry
 * PSoC 5LP EEPROM row = 16 bytes.
 * We use 2 bytes for magic, 1 byte for data length,
 * leaving 13 bytes for slot data.
 * ============================================================ */
#define PSOC_EEPROM_ROW_SIZE        (16u)   /* bytes per EEPROM row     */
#define PSOC_EEPROM_ROW_MAGIC_LO    (0u)    /* offset of magic byte lo  */
#define PSOC_EEPROM_ROW_LEN         (1u)    /* offset of data length    */
#define PSOC_EEPROM_ROW_DATA        (2u)    /* offset of data start     */
#define PSOC_EEPROM_ROW_DATA_SIZE   (14u)   /* max bytes of data        */

#define PSOC_EEPROM_MAGIC_LO        (0xA5u)
#define PSOC_EEPROM_MAGIC_HI        (0x5Au)

/* ============================================================
 * Slot IDs and their fixed row assignments.
 * Each slot ID maps to one EEPROM row number.
 * Add new slots here — each needs a unique ID and row number.
 * Row numbers start at 0. Max rows = CYDEV_EE_SIZE / 16.
 * ============================================================ */
#define PSOC_EEPROM_SLOT_JOY_CAL    (0x01u)  /* joy_cal_t       row 0 */
#define PSOC_EEPROM_ROW_JOY_CAL     (0u)

#define PSOC_EEPROM_SLOT_JOY_SENSE  (0x02u)  /* uint8 sense     row 1 */
#define PSOC_EEPROM_ROW_JOY_SENSE   (1u)

/* Add more slots:
 * #define PSOC_EEPROM_SLOT_MY_CFG  (0x03u)
 * #define PSOC_EEPROM_ROW_MY_CFG   (2u)
 */

/* ============================================================
 * Public API
 * ============================================================ */

/* Call once at startup before any read or write.
 * Starts the EEPROM component. */
void psoc_eeprom_init(void);

/* Write data to a slot.
 * len must be <= PSOC_EEPROM_ROW_DATA_SIZE (13 bytes).
 * Calls UpdateTemperature before writing.
 * Returns 1 on success, 0 on failure. */
uint8 psoc_eeprom_write(uint8 slot_id, const void *data, uint8 len);

/* Read data from a slot into out.
 * len must match the size used when writing.
 * Returns 1 on success, 0 if slot blank, corrupt, or size mismatch. */
uint8 psoc_eeprom_read(uint8 slot_id, void *out, uint8 len);

/* Erase a single slot by overwriting its row with zeros. */
void psoc_eeprom_erase_slot(uint8 slot_id);

/* Erase all known slots. */
void psoc_eeprom_erase_all(void);

#ifdef __cplusplus
}
#endif

#endif /* PSOC_EEPROM_H */

/* [] END OF FILE */