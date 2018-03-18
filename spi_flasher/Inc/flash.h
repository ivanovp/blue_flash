/**
 * @file        flash.h
 * @brief       Internal API of flash
 * @author      Copyright (C) Peter Ivanov, 2017
 *
 * Created:     2017-06-11 09:10:19
 * Last modify: 2017-06-16 11:53:05 ivanovp {Time-stamp}
 * Licence:     GPL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _INCLUDE_FLASH_H_
#define _INCLUDE_FLASH_H_

#include <stdint.h>
#include "flash_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    FLASH_SUCCESS = 0,
    FLASH_ERROR_GENERAL = 1,
    FLASH_ERROR_NOT_INITIALIZED = 2,
    FLASH_ERROR_FLASH_INIT = 12,
    FLASH_ERROR_FLASH_WRITE = 13,
    FLASH_ERROR_FLASH_READ = 14,
    FLASH_ERROR_FLASH_ERASE = 15,
    FLASH_ERROR_FLASH_TIMEOUT = 16,
    FLASH_ERROR_FLASH_GENERAL = 17,
} flash_status_t;

typedef uint32_t flash_address_t;

/** Number of blocks used by the file system */
#define FLASH_BLOCK_NUM_FS     (FLASH_BLOCK_NUM_ALL - FLASH_BLOCK_RESERVED_NUM)
/** Size of a block in bytes */
#define FLASH_BLOCK_SIZE_BYTE  (FLASH_PAGE_SIZE_BYTE * FLASH_PAGE_PER_BLOCK)
/** Size of the whole flash memory in bytes */
#define FLASH_SIZE_BYTE_ALL    (FLASH_BLOCK_SIZE_BYTE * FLASH_BLOCK_NUM_ALL)
/** Size of the file system in bytes */
#define FLASH_SIZE_BYTE_FS     (FLASH_BLOCK_SIZE_BYTE * FLASH_BLOCK_NUM_FS)
/** Number of all flash pages */
#define FLASH_PAGE_NUM_ALL     (FLASH_BLOCK_NUM_ALL * FLASH_PAGE_PER_BLOCK)
/** Number of flash pages used by the file system */
#define FLASH_PAGE_NUM_FS      (FLASH_BLOCK_NUM_FS * FLASH_PAGE_PER_BLOCK)

#define DFU_FLASH_DESCR_SIZE    128

extern uint8_t dfu_flash_descr[DFU_FLASH_DESCR_SIZE];

/**
 * @brief flash_init Initialize flash driver.
 *
 * @return FLASH_SUCCESS if flash memory is identified and initialized.
 */
flash_status_t flash_init(void);

/**
 * @brief flash_delete De-initialize flash driver.
 *
 * @return FLASH_SUCCESS if successfully de-initialized.
 */
flash_status_t flash_delete(void);

/**
 * @brief flash_read Read from flash memory.
 *
 * @param[in] a_address       Address in flash.
 * @param[out] a_buf          Buffer to fill.
 * @param[in] a_buf_size      Size of buffer.
 * @return FLASH_SUCCESS if read successfully finished.
 */
flash_status_t flash_read(flash_address_t a_address, void * const a_buf, size_t a_buf_size);

/**
 * @brief flash_write Write to flash memory.
 *
 * @param[in] a_address       Address in flash.
 * @param[in] a_buf           Buffer to write.
 * @param[in] a_buf_size      Size of buffer.
 * @return FLASH_SUCCESS if write successfully finished.
 */
flash_status_t flash_write(flash_address_t a_address, const void * const a_buf, size_t a_buf_size);

/**
 * @brief flash_erase Erase a block.
 *
 * @param[in] a_address       Address in flash.
 *
 * @return FLASH_SUCCESS if block was erased successfully.
 */
flash_status_t flash_erase(flash_address_t a_address);

/**
 * @brief flash_print_stat Called by the terminal to print information
 * about flash memory.
 */
void flash_print_stat(void);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_FLASH_H_ */
