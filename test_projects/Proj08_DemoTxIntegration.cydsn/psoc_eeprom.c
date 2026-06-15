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
 * Generic EEPROM slot store implementation for PSoC 5LP.
 * Uses EEPROM_CONFIG component API:
 *   EEPROM_CONFIG_Start()
 *   EEPROM_CONFIG_UpdateTemperature()
 *   EEPROM_CONFIG_Write(const uint8 *rowData, uint8 rowNumber)
 *   EEPROM_CONFIG_WriteByte(uint8 dataByte, uint16 address)
 * Reading is direct via (uint8 *)(CYDEV_EE_BASE + byte_offset).
 */

#include "psoc_eeprom.h"
#include <string.h>

/* ============================================================
 * slot_id_to_row
 * Maps a slot ID to its fixed EEPROM row number.
 * Returns 0xFF if the slot ID is unknown.
 * Add a case here whenever a new slot is defined in the header.
 * ============================================================ */
static uint8 slot_id_to_row(uint8 slot_id)
{
    switch (slot_id) {
        case PSOC_EEPROM_SLOT_JOY_CAL:   return PSOC_EEPROM_ROW_JOY_CAL;
        case PSOC_EEPROM_SLOT_JOY_SENSE: return PSOC_EEPROM_ROW_JOY_SENSE;
        /* Add cases here for new slots. */
        default: return 0xFFu;
    }
}

/* ============================================================
 * read_row_byte
 * Read one byte from EEPROM by row + byte offset within the row.
 * ============================================================ */
static uint8 read_row_byte(uint8 row, uint8 byte_offset)
{
    uint16 abs_addr = (uint16)((uint16)row * PSOC_EEPROM_ROW_SIZE
                               + byte_offset);
    return *(volatile uint8 *)(CYDEV_EE_BASE + abs_addr);
}

/* ============================================================
 * read_row_data
 * Copy len bytes of data from an EEPROM row starting at
 * PSOC_EEPROM_ROW_DATA offset into dst.
 * ============================================================ */
static void read_row_data(uint8 row, void *dst, uint8 len)
{
    uint16 abs_addr = (uint16)((uint16)row * PSOC_EEPROM_ROW_SIZE
                               + PSOC_EEPROM_ROW_DATA);
    memcpy(dst, (const void *)(CYDEV_EE_BASE + abs_addr), len);
}

/* ============================================================
 * psoc_eeprom_init
 * ============================================================ */
void psoc_eeprom_init(void)
{
    EEPROM_CONFIG_Start();
}

/* ============================================================
 * psoc_eeprom_write
 * Builds a 16-byte row buffer and writes it to EEPROM.
 * ============================================================ */
uint8 psoc_eeprom_write(uint8 slot_id, const void *data, uint8 len)
{
    if (!data) return 0u;
    if (len == 0u || len > PSOC_EEPROM_ROW_DATA_SIZE) return 0u;

    uint8 row = slot_id_to_row(slot_id);
    if (row == 0xFFu) return 0u;

    /* Build a full 16-byte row. Zero-pad unused data bytes. */
    uint8 row_buf[PSOC_EEPROM_ROW_SIZE];
    memset(row_buf, 0, sizeof(row_buf));
    row_buf[PSOC_EEPROM_ROW_MAGIC_LO] = PSOC_EEPROM_MAGIC_LO;
    row_buf[PSOC_EEPROM_ROW_LEN]      = len;
    memcpy(&row_buf[PSOC_EEPROM_ROW_DATA], data, len);

    /* Update temperature before write as per datasheet. */
    EEPROM_CONFIG_UpdateTemperature();

    /* Write the full row. EEPROM_CONFIG_Write takes row number. */
    cystatus r = EEPROM_CONFIG_Write(row_buf, row);
    return (r == CYRET_SUCCESS) ? 1u : 0u;
}

/* ============================================================
 * psoc_eeprom_read
 * ============================================================ */
uint8 psoc_eeprom_read(uint8 slot_id, void *out, uint8 len)
{
    if (!out || len == 0u) return 0u;

    uint8 row = slot_id_to_row(slot_id);
    if (row == 0xFFu) return 0u;

    /* Check magic bytes. */
    if (read_row_byte(row, PSOC_EEPROM_ROW_MAGIC_LO) != PSOC_EEPROM_MAGIC_LO)
        return 0u;

    /* Check stored length matches requested length. */
    uint8 stored_len = read_row_byte(row, PSOC_EEPROM_ROW_LEN);
    if (stored_len != len) return 0u;

    /* Copy data out. */
    read_row_data(row, out, len);
    return 1u;
}

/* ============================================================
 * psoc_eeprom_erase_slot
 * Overwrites the slot's row with zeros, invalidating the magic.
 * ============================================================ */
void psoc_eeprom_erase_slot(uint8 slot_id)
{
    uint8 row = slot_id_to_row(slot_id);
    if (row == 0xFFu) return;

    uint8 zeros[PSOC_EEPROM_ROW_SIZE];
    memset(zeros, 0, sizeof(zeros));
    EEPROM_CONFIG_UpdateTemperature();
    (void)EEPROM_CONFIG_Write(zeros, row);
}

/* ============================================================
 * psoc_eeprom_erase_all
 * Erases all known slots.
 * ============================================================ */
void psoc_eeprom_erase_all(void)
{
    psoc_eeprom_erase_slot(PSOC_EEPROM_SLOT_JOY_CAL);
    psoc_eeprom_erase_slot(PSOC_EEPROM_SLOT_JOY_SENSE);
    /* Add erase calls here for new slots. */
}

/* [] END OF FILE */