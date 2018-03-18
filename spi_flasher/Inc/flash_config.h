/**
 * @file        flash_config.h
 * @brief       Flash configuration
 * @author      Copyright (C) Peter Ivanov, 2017
 *
 * Created:     2017-06-11 09:10:19
 * Last modify: 2017-08-19 07:18:50 ivanovp {Time-stamp}
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
#ifndef _INCLUDE_FLASH_CONFIG_H_
#define _INCLUDE_FLASH_CONFIG_H_

#define FLASH_TYPE_M25P40           0
#define FLASH_TYPE_M25P80           1
#define FLASH_TYPE_N25Q128A         2
#define FLASH_TYPE_S25FL127S_64K    3   /**< 64 KiB sector mode */
#define FLASH_TYPE_W25Q16DV_64K     4   /**< 64 KiB sector mode */
#define FLASH_TYPE_W25Q32BV_64K     5   /**< 64 KiB sector mode */
#define FLASH_TYPE_W25Q256FV_64K    6   /**< 64 KiB sector mode */

/** Type of flash memory */
#define FLASH_TYPE                  FLASH_TYPE_W25Q16DV_64K

#if FLASH_TYPE == FLASH_TYPE_M25P40
/* Geometry of ST M25P40 */
#define FLASH_BLOCK_NUM_ALL            8u      /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       0u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           256u    /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#elif FLASH_TYPE == FLASH_TYPE_M25P80
/* Geometry of ST M25P40 */
#define FLASH_BLOCK_NUM_ALL            16u     /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       0u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           256u    /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#elif FLASH_TYPE == FLASH_TYPE_N25Q128A
/* Geometry of Micron N25Q128A */
#define FLASH_BLOCK_NUM_ALL            4096u   /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       4u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           16u     /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#elif FLASH_TYPE == FLASH_TYPE_S25FL127S_64K
/* Geometry of Cypress S25FL127S */
#define FLASH_BLOCK_NUM_ALL            256u    /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       1u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           256u    /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#elif FLASH_TYPE == FLASH_TYPE_W25Q16DV_64K
/* Geometry of Winbond W25Q16DV */
#define FLASH_BLOCK_NUM_ALL            32u     /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       0u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           256u    /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#elif FLASH_TYPE == FLASH_TYPE_W25Q32BV_64K
/* Geometry of Winbond W25Q32BV */
#define FLASH_BLOCK_NUM_ALL            64u     /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       0u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           256u    /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#elif FLASH_TYPE == FLASH_TYPE_W25Q256FV_64K
/* Geometry of Winbond W25Q256FV */
#define FLASH_BLOCK_NUM_ALL            512u    /**< Number of blocks in flash memory */
#define FLASH_BLOCK_RESERVED_NUM       0u      /**< Index of first block to use by the file system */
#define FLASH_PAGE_PER_BLOCK           256u    /**< Number of pages in a block */
#define FLASH_PAGE_SIZE_BYTE           256u    /**< Size of a page in bytes */
#define FLASH_PAGE_SIZE_SPARE          0u      /**< Number of spare bytes in a page */
#define FLASH_4BYTE_ADDRESS            1
#endif

#define FLASH_ERASED_BYTE_VALUE        0xFFu
#define FLASH_PROGRAMMED_BYTE_VALUE    (FLASH_ERASED_BYTE_VALUE ^ 0xFFu)

#endif /* _INCLUDE_FLASH_CONFIG_H_ */
