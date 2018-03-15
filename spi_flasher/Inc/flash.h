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
    PIFS_SUCCESS = 0,
    PIFS_ERROR_GENERAL = 1,
    PIFS_ERROR_NOT_INITIALIZED = 2,
    PIFS_ERROR_FLASH_INIT = 12,
    PIFS_ERROR_FLASH_WRITE = 13,
    PIFS_ERROR_FLASH_READ = 14,
    PIFS_ERROR_FLASH_ERASE = 15,
    PIFS_ERROR_FLASH_TIMEOUT = 16,
    PIFS_ERROR_FLASH_GENERAL = 17,
} pifs_status_t;

#if (PIFS_FLASH_BLOCK_NUM_ALL < 255)
typedef uint8_t pifs_block_address_t;
#define PIFS_BLOCK_ADDRESS_INVALID   (UINT8_MAX - 1u)
#define PIFS_BLOCK_ADDRESS_ERASED    (UINT8_MAX)
#elif (PIFS_FLASH_BLOCK_NUM_ALL < 65535)
typedef uint16_t pifs_block_address_t;
#define PIFS_BLOCK_ADDRESS_INVALID   (UINT16_MAX - 1u)
#define PIFS_BLOCK_ADDRESS_ERASED    (UINT16_MAX)
#elif (PIFS_FLASH_BLOCK_NUM_ALL < 4294967295l)
typedef uint32_t pifs_block_address_t;
#define PIFS_BLOCK_ADDRESS_INVALID   (UINT32_MAX - 1u)
#define PIFS_BLOCK_ADDRESS_ERASED    (UINT32_MAX)
#else
#error PIFS_FLASH_BLOCK_NUM_ALL is too big!
#endif

#if (PIFS_FLASH_PAGE_PER_BLOCK < 255)
typedef uint8_t pifs_page_address_t;
#define PIFS_PAGE_ADDRESS_INVALID   (UINT8_MAX - 1u)
#define PIFS_PAGE_ADDRESS_ERASED    (UINT8_MAX)
#elif (PIFS_FLASH_PAGE_PER_BLOCK < 65535)
typedef uint16_t pifs_page_address_t;
#define PIFS_PAGE_ADDRESS_INVALID   (UINT16_MAX - 1u)
#define PIFS_PAGE_ADDRESS_ERASED    (UINT16_MAX)
#elif (PIFS_FLASH_PAGE_PER_BLOCK < 4294967295l)
typedef uint32_t pifs_page_address_t;
#define PIFS_PAGE_ADDRESS_INVALID   (UINT32_MAX - 1u)
#define PIFS_PAGE_ADDRESS_ERASED    (UINT32_MAX)
#else
#error PIFS_FLASH_PAGE_PER_BLOCK is too big!
#endif

#if (PIFS_FLASH_PAGE_SIZE_BYTE < 255 && PIFS_LOGICAL_PAGE_SIZE_BYTE < 255)
typedef uint8_t pifs_page_offset_t;
#define PIFS_PAGE_OFFSET_INVALID   (UINT8_MAX - 1u)
#define PIFS_PAGE_OFFSET_ERASED    (UINT8_MAX)
#elif (PIFS_FLASH_PAGE_SIZE_BYTE < 65535 && PIFS_LOGICAL_PAGE_SIZE_BYTE < 65535)
typedef uint16_t pifs_page_offset_t;
#define PIFS_PAGE_OFFSET_INVALID   (UINT16_MAX - 1u)
#define PIFS_PAGE_OFFSET_ERASED    (UINT16_MAX)
#elif (PIFS_FLASH_PAGE_SIZE_BYTE < 4294967295l && PIFS_LOGICAL_PAGE_SIZE_BYTE < 4294967295l)
typedef uint32_t pifs_page_offset_t;
#define PIFS_PAGE_OFFSET_INVALID   (UINT32_MAX - 1u)
#define PIFS_PAGE_OFFSET_ERASED    (UINT32_MAX)
#else
#error PIFS_FLASH_PAGE_SIZE_BYTE or PIFS_LOGICAL_PAGE_SIZE_BYTE is too big!
#endif

/** Number of blocks used by the file system */
#define PIFS_FLASH_BLOCK_NUM_FS     (PIFS_FLASH_BLOCK_NUM_ALL - PIFS_FLASH_BLOCK_RESERVED_NUM)
/** Size of a block in bytes */
#define PIFS_FLASH_BLOCK_SIZE_BYTE  (PIFS_FLASH_PAGE_SIZE_BYTE * PIFS_FLASH_PAGE_PER_BLOCK)
/** Size of the whole flash memory in bytes */
#define PIFS_FLASH_SIZE_BYTE_ALL    (PIFS_FLASH_BLOCK_SIZE_BYTE * PIFS_FLASH_BLOCK_NUM_ALL)
/** Size of the file system in bytes */
#define PIFS_FLASH_SIZE_BYTE_FS     (PIFS_FLASH_BLOCK_SIZE_BYTE * PIFS_FLASH_BLOCK_NUM_FS)
/** Number of all flash pages */
#define PIFS_FLASH_PAGE_NUM_ALL     (PIFS_FLASH_BLOCK_NUM_ALL * PIFS_FLASH_PAGE_PER_BLOCK)
/** Number of flash pages used by the file system */
#define PIFS_FLASH_PAGE_NUM_FS      (PIFS_FLASH_BLOCK_NUM_FS * PIFS_FLASH_PAGE_PER_BLOCK)

/**
 * @brief pifs_flash_init Initialize flash driver.
 *
 * @return PIFS_SUCCESS if flash memory is identified and initialized.
 */
pifs_status_t pifs_flash_init(void);

/**
 * @brief pifs_flash_delete De-initialize flash driver.
 *
 * @return PIFS_SUCCESS if successfully de-initialized.
 */
pifs_status_t pifs_flash_delete(void);

/**
 * @brief pifs_flash_read Read from flash memory.
 *
 * @param[in] a_block_address Address of block.
 * @param[in] a_page_address  Address of the page in block.
 * @param[in] a_page_offset   Offset in page.
 * @param[out] a_buf          Buffer to fill.
 * @param[in] a_buf_size      Size of buffer.
 * @return PIFS_SUCCESS if read successfully finished.
 */
pifs_status_t pifs_flash_read(pifs_block_address_t a_block_address, pifs_page_address_t a_page_address, pifs_page_offset_t a_page_offset, void * const a_buf, size_t a_buf_size);

/**
 * @brief pifs_flash_write Write to flash memory.
 *
 * @param[in] a_block_address Address of block.
 * @param[in] a_page_address  Address of the page in block.
 * @param[in] a_page_offset   Offset in page.
 * @param[in] a_buf           Buffer to write.
 * @param[in] a_buf_size      Size of buffer.
 * @return PIFS_SUCCESS if write successfully finished.
 */
pifs_status_t pifs_flash_write(pifs_block_address_t a_block_address, pifs_page_address_t a_page_address, pifs_page_address_t a_page_offset, const void * const a_buf, size_t a_buf_size);

/**
 * @brief pifs_flash_erase Erase a block.
 *
 * @param[in] a_block_address Block to erase.
 *
 * @return PIFS_SUCCESS if block was erased successfully.
 */
pifs_status_t pifs_flash_erase(pifs_block_address_t a_block_address);

/**
 * @brief pifs_flash_print_stat Called by the terminal to print information
 * about flash memory.
 */
void pifs_flash_print_stat(void);

#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_FLASH_H_ */
