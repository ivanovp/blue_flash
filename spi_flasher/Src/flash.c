/**
 * @file        flash.c
 * @brief       NOR flash driver for STM32
 * @author      Copyright (C) Peter Ivanov, 2017
 *
 * Created:     2017-06-11 09:10:19
 * Last modify: 2017-11-30 12:44:28 ivanovp {Time-stamp}
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
#include <stdio.h>
#include <string.h>

#define FLASH_DEBUG_LEVEL      5

#include "flash.h"
#include "flash_debug.h"
#include "uart.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifndef FLASH_4BYTE_ADDRESS
#define FLASH_4BYTE_ADDRESS	0
#endif

#ifndef FLASH_ENABLE_DMA
#define FLASH_ENABLE_DMA            0   /**< DMA is not working on STM32F1xx */
#endif
#define FLASH_TIMEOUT_MS            1000
#define FLASH_WRITE_TIMEOUT_MS      5000
#define FLASH_ERASE_TIMEOUT_MS      5000
#define FLASH_CHIP_ERASE_TIMEOUT_MS 400000
#define FLASH_DMA_TIMEOUT_MS        5

#define FLASH_DEFAULT_ERASE_COMMAND 0xD8
#define FLASH_DEFAULT_BLOCK_SIZE    65536

#define SET_CS_LOW()    do { \
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); \
	} while (0);
#define SET_CS_HIGH()   do { \
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); \
	} while (0);
#define STATUS_REG_WIP              0x01    /**< Write in progress bit in status register */

#define MS_TO_TICK(ms)              (ms * 1000 / osKernelSysTickFrequency)
#define FLASH_TIMEOUT_TICK          MS_TO_TICK(FLASH_TIMEOUT_MS)
#define FLASH_WRITE_TIMEOUT_TICK    MS_TO_TICK(FLASH_WRITE_TIMEOUT_MS)
#define FLASH_ERASE_TIMEOUT_TICK    MS_TO_TICK(FLASH_ERASE_TIMEOUT_MS)
#define FLASH_CHIP_ERASE_TIMEOUT_TICK MS_TO_TICK(FLASH_CHIP_ERASE_TIMEOUT_MS)

#define SFDP_SIGNATURE      0x50444653   /* SFDP */
#define SFDP_VERSION_MINOR  6
#define SFDP_VERSION_MAJOR  1
#define SFDP_MAX_NPH        6

/* Over 128 MBit 4-byte addressing is needed! */
#define SIZE_128MBIT        (16 * 1024 * 1024)

#define ENTER_4BYTE_WITH_B7()               (enter_4byte_addressing_cfg & (1u << 0))
#define ENTER_4BYTE_WITH_06_B7()            (enter_4byte_addressing_cfg & (1u << 1))
#define EXTENDED_ADDRESS_REG_31_24_BITS()   (enter_4byte_addressing_cfg & (1u << 2))
#define EXTENDED_ADDRESS_REG_30_24_BITS()   (enter_4byte_addressing_cfg & (1u << 3))
#define ALWAYS_OPERATES_IN_4BYTE_MODE()     (enter_4byte_addressing_cfg & (1u << 6))

typedef struct __attribute__((packed))
{
    uint8_t                 id;
    uint8_t                 version_minor;
    uint8_t                 version_major;
    uint8_t                 length_dw;     /* Length in double words */
    uint8_t                 table_ptr[3];
    uint8_t                 id_msb;        /* when SFDP version >= 1.5 */
} sfdp_parameter_header_t;

typedef struct __attribute__((packed))
{
    uint32_t                signature; /* =SFDP_SIGNATURE */
    uint8_t                 version_minor;
    uint8_t                 version_major;
    uint8_t                 nph;       /* Number of parameter header, 0 means one parameter header exists! */
    uint8_t                 unused;    /* Unused field, =0xFF */
    sfdp_parameter_header_t parameter_header[SFDP_MAX_NPH];
} sfdp_header_t;

typedef struct __attribute__((packed))
{
    uint8_t sector_size;
    uint8_t sector_erase_opcode;
} sector_t;

/* Default value of DFU flash descriptor
 *
 *                               .-- '@' marks start of DFuSe flash descriptor
 *                               |.-- human readable name
 *                               ||                   .-- base address
 *                               ||                   |          .-- number of sectors (erasable units)
 *                               ||                   |          |   .-- sector size in specified unit (kilobytes)
 *                               ||                   |          |   | .-- sector size unit (K=kilobytes)
 *                               ||                   |          |   | |.-- sector type "g": Readable, Erasable, Writable
 *                               ||                   |          |   | ||
 *                               vv                   v          v   v vv             */
#define DFU_FLASH_DESCR_DEFAULT "@SPI Flash not found/0x00000000/32*064Kg"
/* 1 digit for the sector type as follows:
 * - a (0x41): Readable
 * - b (0x42): Erasable
 * - c (0x43): Readable and Erasable
 * - d (0x44): Writable
 * - e (0x45): Readable and Writable
 * - f (0x46): Erasable and Writable
 * - g (0x47): Readable, Erasable and Writable
 */
uint8_t dfu_flash_descr[DFU_FLASH_DESCR_SIZE] = DFU_FLASH_DESCR_DEFAULT;

extern SPI_HandleTypeDef hspi1;
static SPI_HandleTypeDef *spi = &hspi1;
static bool_t flash_initialized = FALSE;
static const uint8_t cmd_dummy[1]            = { 0xFF };
static const uint8_t cmd_power_up[4]         = { 0xAB, 0, 0, 0 };
static const uint8_t cmd_power_down[1]       = { 0xB9 };
static const uint8_t cmd_read_id[1]          = { 0x9F };
static const uint8_t cmd_read_status_reg[1]  = { 0x05 };
static const uint8_t cmd_enter_4byte_mode[1] = { 0xB7 };
static uint8_t cmd_erase[5]                  = { FLASH_DEFAULT_ERASE_COMMAND };
static uint8_t cmd_read_data[5]              = { 0x03 };
static uint8_t cmd_page_program[5]           = { 0x02 };
static const uint8_t cmd_write_enable[1]     = { 0x06 };
static uint8_t cmd_read_sfdp[5]              = { 0x5A, 0, 0, 0, 0 }; /* 24-bit address + 8 dummy bits */
static const uint8_t cmd_chip_erase[1]       = { 0xC7 };
static uint8_t answer[3];
static sector_t sector_types[4];
static uint8_t enter_4byte_addressing_cfg = 0;
#if FLASH_ENABLE_DMA
static osSemaphoreId dma_finished;
osSemaphoreDef(dma_finished);
#endif
#if FLASH_TYPE != FLASH_TYPE_AUTO_DETECT
static uint32_t flash_block_num_all = FLASH_BLOCK_NUM_ALL;
static uint32_t flash_block_size_byte = FLASH_BLOCK_SIZE_BYTE;
static uint32_t flash_density_bytes = FLASH_SIZE_BYTE_ALL;
#if FLASH_4BYTE_ADDRESS
static uint8_t flash_address_bytes = 4;
#else
static uint8_t flash_address_bytes = 3;
#endif
#else
static uint32_t flash_block_num_all = 0;
static uint32_t flash_block_size_byte = FLASH_DEFAULT_BLOCK_SIZE;
static uint32_t flash_density_bytes = 0;
static uint8_t flash_address_bytes = 3;
#endif

flash_status_t flash_read_parameter_header(sfdp_parameter_header_t * a_parameter_header)
{
    flash_status_t    ret = FLASH_ERROR_GENERAL;
    uint32_t          dword;
    HAL_StatusTypeDef stat;
    uint8_t           addr_bytes;
    uint32_t          max_sector_size_byte = 0;
    uint32_t          sector_size_byte = 0;
    uint8_t           sector_erase_opcode = 0;
    uint8_t           i;

    FLASH_INFO2_MSG("ID MSB: 0x%02X\r\n", a_parameter_header->id_msb);
    FLASH_INFO2_MSG("ID LSB: 0x%02X\r\n", a_parameter_header->id);
    if (a_parameter_header->id_msb == 0xFF)
    {
        if (a_parameter_header->id == 0)
        {
            /* 0xFF00: Basic SPI protocol */
            FLASH_INFO2_MSG("Type: Basic SPI protocol\r\n");
            FLASH_INFO2_MSG("Version: %i.%i\r\n", a_parameter_header->version_major, a_parameter_header->version_minor);
            FLASH_INFO2_MSG("Length: %i DW\r\n", a_parameter_header->length_dw);
            FLASH_INFO2_MSG("Table pointer: 0x%02X%02X%02X\r\n",
                            a_parameter_header->table_ptr[2],
                    a_parameter_header->table_ptr[1],
                    a_parameter_header->table_ptr[0]);
            /* Set address to read from parameter table */
            cmd_read_sfdp[1] = a_parameter_header->table_ptr[2];
            cmd_read_sfdp[2] = a_parameter_header->table_ptr[1];
            cmd_read_sfdp[3] = a_parameter_header->table_ptr[0];
            SET_CS_LOW();
            if (HAL_SPI_Transmit(spi, cmd_read_sfdp, sizeof(cmd_read_sfdp), FLASH_TIMEOUT_TICK) == HAL_OK)
            {
                /* 1st DWORD */
                stat = HAL_SPI_Receive(spi, (uint8_t*)&dword, sizeof(dword), FLASH_TIMEOUT_TICK);
                if (stat == HAL_OK)
                {
                    addr_bytes = (dword >> 17) & 0x3;
                    if (addr_bytes == 0)
                    {
                        FLASH_INFO2_MSG("3-byte only addressing\r\n");
                        flash_address_bytes = 3;
                    }
                    else if (addr_bytes == 1)
                    {
                        FLASH_INFO2_MSG("3- or 4-byte addressing\r\n");
                        flash_address_bytes = 3;
                    }
                    else if (addr_bytes == 2)
                    {
                        FLASH_INFO2_MSG("4-byte only addressing\r\n");
                        flash_address_bytes = 4;
                    }
                    else
                    {
                        FLASH_ERROR_MSG("Erroneous addressing mode!\r\n");
                    }
                }
                /* 2nd DWORD, density */
                stat = HAL_SPI_Receive(spi, (uint8_t*)&dword, sizeof(dword), FLASH_TIMEOUT_TICK);
                if (stat == HAL_OK)
                {
                    if (dword & (1u << 31))
                    {
                        /* density is given in 2^N bits form */
                        dword &= ~(1u << 31);
                        flash_density_bytes = 1u << (dword - 3);
                    }
                    else
                    {
                        flash_density_bytes = (dword + 1) >> 3;
                    }
                    if (flash_density_bytes > SIZE_128MBIT)
                    {
                        /* Over 128 MBit, so four address bytes needed */
                        flash_address_bytes = 4;
                    }
                    /* Calculate default block number in case of no sector types field */
                    flash_block_num_all = flash_density_bytes / flash_block_size_byte;
                    FLASH_INFO2_MSG("Density: %i bytes\r\n", flash_density_bytes);
                }
                if (a_parameter_header->length_dw >= 7)
                {
                    /* 3rd..7th DWORDS, fast read parameters, omitting */
                    for (i = 0; i < 5 && stat == HAL_OK; i++)
                    {
                        stat = HAL_SPI_Receive(spi, (uint8_t*)&dword, sizeof(dword), FLASH_TIMEOUT_TICK);
                    }
                }
                if (a_parameter_header->length_dw >= 9 && stat == HAL_OK)
                {
                    /* 8th..9th DWORD, sector types */
                    stat = HAL_SPI_Receive(spi, (uint8_t*)&sector_types, sizeof(sector_types), FLASH_TIMEOUT_TICK);
                    for (i = 0; i < sizeof(sector_types) / sizeof(sector_types[0]); i++)
                    {
                        if (sector_types[i].sector_size > 0)
                        {
                            sector_size_byte = (1u << sector_types[i].sector_size);
                            FLASH_INFO2_MSG("Sector size: %i bytes, erase opcode: 0x%02X\r\n",
                                            sector_size_byte,
                                            sector_types[i].sector_erase_opcode);
                            if ((sector_size_byte > max_sector_size_byte)
                                    && (sector_size_byte <= 65536))
                            {
                                max_sector_size_byte = sector_size_byte;
                                sector_erase_opcode = sector_types[i].sector_erase_opcode;
                                ret = FLASH_SUCCESS;
                            }
                        }
                    }
                    if (ret == FLASH_SUCCESS)
                    {
                        flash_block_size_byte = max_sector_size_byte;
                        flash_block_num_all = flash_density_bytes / flash_block_size_byte;
                        cmd_erase[0] = sector_erase_opcode;
                        FLASH_INFO2_MSG("Selected sector size: %i bytes, erase opcode: 0x%02X\r\n",
                                        max_sector_size_byte,
                                        sector_erase_opcode);
                    }
                    else
                    {
                        FLASH_INFO2_MSG("No sector types found!\r\n");
                    }
                }
                if (a_parameter_header->length_dw >= 15
                        && a_parameter_header->version_major == 1 && a_parameter_header->version_minor >= 5)
                {
                    /* 10th..15th DWORDS, omitting */
                    /* TODO 14th DWORD: Status Register Polling Device Busy */
                    /* 16th DWORD, Enter 4-byte addressing */
                    for (i = 0; i < 7 && stat == HAL_OK; i++)
                    {
                        stat = HAL_SPI_Receive(spi, (uint8_t*)&dword, sizeof(dword), FLASH_TIMEOUT_TICK);
                    }
                    if (stat == HAL_OK)
                    {
                        enter_4byte_addressing_cfg = dword >> 24;
                        FLASH_INFO2_MSG("Enter 4-byte addressing: 0x%02X\r\n", enter_4byte_addressing_cfg);
                        if (ENTER_4BYTE_WITH_B7())
                        {
                            FLASH_INFO2_MSG("Enter 4-byte with command 0xB7\r\n");
                        }
                        if (ENTER_4BYTE_WITH_06_B7())
                        {
                            FLASH_INFO2_MSG("Enter 4-byte with command 0x06 and 0xB7\r\n");
                        }
                        if (EXTENDED_ADDRESS_REG_31_24_BITS())
                        {
                            FLASH_INFO2_MSG("8-bit volatile extended address register used to define A[31:24] bits\r\n");
                        }
                        if (EXTENDED_ADDRESS_REG_30_24_BITS())
                        {
                            FLASH_INFO2_MSG("8-bit volatile extended address register used to define A[30:24] bits\r\n");
                        }
                        if (ALWAYS_OPERATES_IN_4BYTE_MODE())
                        {
                            FLASH_INFO2_MSG("Always operates in 4-byte address mode\r\n");
                        }
                    }
                }
                SET_CS_HIGH();
            }
        }
        else if (a_parameter_header->id == 0x84)
        {
            /* 0xFF84: 4-byte Address Instruction Table */
            FLASH_INFO2_MSG("Type: 4-byte address instruction table\r\n");
            FLASH_INFO2_MSG("Version: %i.%i\r\n", a_parameter_header->version_major, a_parameter_header->version_minor);
            FLASH_INFO2_MSG("Length: %i DW\r\n", a_parameter_header->length_dw);
            FLASH_INFO2_MSG("Table pointer: 0x%02X%02X%02X\r\n",
                            a_parameter_header->table_ptr[2],
                    a_parameter_header->table_ptr[1],
                    a_parameter_header->table_ptr[0]);
            /* Set address to read from parameter table */
            cmd_read_sfdp[1] = a_parameter_header->table_ptr[2];
            cmd_read_sfdp[2] = a_parameter_header->table_ptr[1];
            cmd_read_sfdp[3] = a_parameter_header->table_ptr[0];
            SET_CS_LOW();
            if (HAL_SPI_Transmit(spi, cmd_read_sfdp, sizeof(cmd_read_sfdp), FLASH_TIMEOUT_TICK) == HAL_OK)
            {
                /* 1st DWORD */
                stat = HAL_SPI_Receive(spi, (uint8_t*)&dword, sizeof(dword), FLASH_TIMEOUT_TICK);
                if (stat == HAL_OK)
                {
                    FLASH_INFO2_MSG("1st DWORD: 0x%08X\r\n", dword);
                }
                if (stat == HAL_OK)
                {
                    /* 2nd DWORD */
                    stat = HAL_SPI_Receive(spi, (uint8_t*)&dword, sizeof(dword), FLASH_TIMEOUT_TICK);
                }
                if (stat == HAL_OK)
                {
                    FLASH_INFO2_MSG("2nd DWORD: 0x%08X\r\n", dword);
                }
            }
        }
        else
        {
            FLASH_ERROR_MSG("Unknown parameter header ID LSB: 0x%02X!\r\n", a_parameter_header->id);
            ret = FLASH_SUCCESS;
        }
    }
    else
    {
        FLASH_ERROR_MSG("Unknown parameter header ID MSB: 0x%02X!\r\n", a_parameter_header->id_msb);
        ret = FLASH_SUCCESS;
    }
    FLASH_INFO2_MSG("\r\n");

    return ret;
}

flash_status_t flash_read_sfdp(void)
{
    flash_status_t ret = FLASH_ERROR_GENERAL;
    sfdp_header_t  sfdp_header;
    uint8_t        i;

    FLASH_INFO2_MSG("\r\n");
    SET_CS_LOW();
    cmd_read_sfdp[1] = 0;
    cmd_read_sfdp[2] = 0;
    cmd_read_sfdp[3] = 0;
    if (HAL_SPI_Transmit(spi, cmd_read_sfdp, sizeof(cmd_read_sfdp), FLASH_TIMEOUT_TICK) == HAL_OK)
    {
        if (HAL_SPI_Receive(spi, &sfdp_header, sizeof(sfdp_header), FLASH_TIMEOUT_TICK) == HAL_OK)
        {
            SET_CS_HIGH();
            FLASH_INFO2_MSG("SFDP header\r\n");
            FLASH_INFO2_MSG("===========\r\n");
            FLASH_INFO2_MSG("Signature: 0x%08X\r\n", sfdp_header.signature);
            if (sfdp_header.signature == SFDP_SIGNATURE)
            {
                FLASH_INFO2_MSG("Version: %i.%i\r\n", sfdp_header.version_major, sfdp_header.version_minor);
                if (sfdp_header.version_major == 1)
                {
                    /* Number of parameters headers: 0 -> 1 parameter header. */
                    FLASH_INFO2_MSG("Number of parameter headers: %i\r\n", sfdp_header.nph + 1);
                    for (i = 0; i <= sfdp_header.nph && i < SFDP_MAX_NPH; i++)
                    {
                        FLASH_INFO2_MSG("#%i parameter header\r\n", i);
                        FLASH_INFO2_MSG("--------------------\r\n");
                        ret = flash_read_parameter_header(&(sfdp_header.parameter_header[i]));
                    }
                }
                else
                {
                    FLASH_ERROR_MSG("Unknown SFDP header version!\r\n");
                }
            }
            else
            {
                FLASH_ERROR_MSG("Wrong SFDP signature!\r\n");
            }
        }
    }
    SET_CS_HIGH();
    FLASH_INFO2_MSG("\r\n");

    return ret;
}

#if FLASH_ENABLE_DMA
/**
 * @brief HAL_SPI_RxCpltCallback Called when DMA RX operation finished.
 *
 * @param[in] hspi SPI handler.
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    osSemaphoreRelease(dma_finished);
}

/**
 * @brief HAL_SPI_TxCpltCallback Called when DMA TX operation finished.
 *
 * @param[in] hspi SPI handler.
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    osSemaphoreRelease(dma_finished);
}
#endif

void flash_write_address(uint8_t * buf, uint32_t address)
{
    if (flash_address_bytes == 4)
    {
        buf[0] = address >> 24;
        buf[1] = address >> 16;
        buf[2] = address >> 8;
        buf[3] = address;
    }
    else
    {
        buf[0] = address >> 16;
        buf[1] = address >> 8;
        buf[2] = address;
    }
}

/**
 * @brief flash_write_enable Run write enable command. Necessary before
 * writing or erasing the flash memory.
 *
 * @return FLASH_SUCCESS if write enabled.
 */
flash_status_t flash_write_enable(void)
{
    flash_status_t ret = FLASH_ERROR_FLASH_GENERAL;

    SET_CS_LOW();
    if (HAL_SPI_Transmit(spi, cmd_write_enable, sizeof(cmd_write_enable), FLASH_TIMEOUT_TICK) == HAL_OK)
	{
        ret = FLASH_SUCCESS;
	}
	SET_CS_HIGH();

	return ret;
}

/**
 * @brief flash_wait Wait for end of write/erase.
 *
 * @return FLASH_SUCCESS if write or erase operation finished successfully.
 */
flash_status_t flash_wait(uint32_t a_timeout, bool_t a_print_dots)
{
    flash_status_t ret = FLASH_ERROR_FLASH_TIMEOUT;
    uint32_t       tick_start;
    uint32_t       delta_tick;
    uint32_t       delta_tick_prev;

    tick_start = osKernelSysTick();
    do
    {
        SET_CS_LOW();
        if (HAL_SPI_Transmit(spi, cmd_read_status_reg, sizeof(cmd_read_status_reg), FLASH_TIMEOUT_TICK) == HAL_OK)
        {
            if (HAL_SPI_Receive(spi, answer, 1, FLASH_TIMEOUT_TICK) == HAL_OK)
            {
                if (!(answer[0] & STATUS_REG_WIP))
                {
                    ret = FLASH_SUCCESS;
                }
            }
        }
        SET_CS_HIGH();
        delta_tick = osKernelSysTick() - tick_start;
        if (a_print_dots && delta_tick != delta_tick_prev && ((delta_tick % 1000) == 0))
        {
            UART_putc('.');
        }
        delta_tick_prev = delta_tick;
    } while (ret != FLASH_SUCCESS && delta_tick < a_timeout);

    return ret;
}

flash_status_t flash_cmd(uint8_t * a_cmd, uint32_t a_cmd_length)
{
    flash_status_t ret;

    SET_CS_LOW();
    if (HAL_SPI_Transmit(spi, a_cmd, a_cmd_length, FLASH_TIMEOUT_TICK) == HAL_OK)
    {
        ret = FLASH_SUCCESS;
    }
    else
    {
        ret = FLASH_ERROR_GENERAL;
    }
    SET_CS_HIGH();

    return ret;
}

/**
 * @brief flash_init Initialize flash driver.
 *
 * @return FLASH_SUCCESS if flash memory is identified and initialized.
 */
flash_status_t flash_init(void)
{
    flash_status_t ret;
    uint8_t        n;

    flash_initialized = FALSE;

#if FLASH_ENABLE_DMA
    ret = FLASH_ERROR_FLASH_GENERAL;
    dma_finished = osSemaphoreCreate(osSemaphore(dma_finished), 1);
    if (dma_finished)
    {
        osSemaphoreWait(dma_finished, osWaitForever);
    }
    if (dma_finished)
#endif
    {
        ret = FLASH_ERROR_FLASH_INIT;
        SET_CS_HIGH();
        /* Waste some time */
        if (HAL_SPI_Transmit(spi, cmd_dummy, sizeof(cmd_dummy), FLASH_TIMEOUT_TICK) == HAL_OK)
        {
        }
        ret = flash_cmd(cmd_power_up, sizeof(cmd_power_up));
        /* Waste some time */
        if (HAL_SPI_Transmit(spi, cmd_dummy, sizeof(cmd_dummy), FLASH_TIMEOUT_TICK) == HAL_OK)
        {
        }
        SET_CS_LOW();
        if (HAL_SPI_Transmit(spi, cmd_read_id, sizeof(cmd_read_id), FLASH_TIMEOUT_TICK) == HAL_OK)
        {
            if (HAL_SPI_Receive(spi, answer, 3, FLASH_TIMEOUT_TICK) == HAL_OK)
            {
#if FLASH_TYPE == FLASH_TYPE_AUTO_DETECT
                SET_CS_HIGH();
                ret = flash_read_sfdp();
                if (ret != FLASH_SUCCESS)
                {
                    FLASH_INFO2_MSG("Trying to calculate flash geometry\r\n");
                    /* SFDP was unsuccessful, trying to calculate memory capacity */
                    /* from ID's 3rd byte */
                    /* BCD to binary conversion */
                    n = (answer[2] >> 4) * 10;
                    n += (answer[2] & 0xF);
                    flash_density_bytes = 64u << n;
                    FLASH_INFO2_MSG("Density: %i bytes\r\n", flash_density_bytes);
                    if (flash_density_bytes > SIZE_128MBIT)
                    {
                        /* Over 128 MBit, so four address bytes needed */
                        flash_address_bytes = 4;
                        FLASH_INFO2_MSG("4-byte adressing\r\n");
                    }
                    else
                    {
                        flash_address_bytes = 3;
                        FLASH_INFO2_MSG("3-byte adressing\r\n");
                    }
                    flash_block_size_byte = FLASH_DEFAULT_BLOCK_SIZE;
                    flash_block_num_all = flash_density_bytes / flash_block_size_byte;
                    cmd_erase[0] = FLASH_DEFAULT_ERASE_COMMAND;
                    ret = FLASH_SUCCESS;
                    FLASH_INFO2_MSG("\r\n");
                }
                if (ret == FLASH_SUCCESS)
#elif FLASH_TYPE == FLASH_TYPE_M25P40
                if (answer[0] == 0x20 && answer[1] == 0x20 && answer[2] == 0x13)
#elif FLASH_TYPE == FLASH_TYPE_W25Q16DV_64K
                if (answer[0] == 0xEF && answer[1] == 0x40 && answer[2] == 0x15)
#elif FLASH_TYPE == FLASH_TYPE_W25Q32BV_64K
                if (answer[0] == 0xEF && answer[1] == 0x40 && answer[2] == 0x16)
#elif FLASH_TYPE == FLASH_TYPE_W25Q256FV_4K || FLASH_TYPE == FLASH_TYPE_W25Q256FV_64K
                if (answer[0] == 0xEF && answer[1] == 0x40 && answer[2] == 0x19)
#elif FLASH_TYPE == FLASH_TYPE_S25FL127S_64K || FLASH_TYPE == FLASH_TYPE_S25FL127S_256K
                if (answer[0] == 0x01 && answer[1] == 0x20 && answer[2] == 0x18)
#endif
                {
#if FLASH_TYPE != FLASH_TYPE_AUTO_DETECT
                    flash_block_num_all = FLASH_BLOCK_NUM_ALL;
#endif
                    snprintf(dfu_flash_descr, sizeof(dfu_flash_descr),
                             "@SPI Flash (ID 0x%02X%02X%02X, Size: %i bytes)/0x00000000/%i*%03iKg",
                             answer[0], answer[1], answer[2],
                            flash_density_bytes,
                            flash_block_num_all,
                            (flash_block_size_byte >> 10)
                            );
                    flash_initialized = TRUE;
                    ret = FLASH_SUCCESS;
                }
                else
                {
                    strncpy(dfu_flash_descr, DFU_FLASH_DESCR_DEFAULT, sizeof(dfu_flash_descr));
                    FLASH_ERROR_MSG("Cannot identify flash: 0x%02X 0x%02X 0x%02X\r\n",
                                    answer[0], answer[1], answer[2]);
                }
            }
        }
        SET_CS_HIGH();

        if (ret == FLASH_SUCCESS && flash_address_bytes == 4)
        {
            if (ENTER_4BYTE_WITH_06_B7())
            {
                ret = flash_cmd(cmd_write_enable, sizeof(cmd_write_enable));
            }
            if (ret == FLASH_SUCCESS)
            {
                ret = flash_cmd(cmd_enter_4byte_mode, sizeof(cmd_enter_4byte_mode));
            }
        }
    }

    return ret;
}

/**
 * @brief flash_delete De-initialize flash driver.
 *
 * @return FLASH_SUCCESS if successfully de-initialized.
 */
flash_status_t flash_delete(void)
{
    flash_status_t ret = FLASH_ERROR_FLASH_GENERAL;

#if FLASH_ENABLE_DMA
    if (osSemaphoreDelete(dma_finished) == osOK)
#endif
    {
        ret = flash_cmd(cmd_power_down, sizeof(cmd_power_down));
        flash_initialized = FALSE;
    }

    return ret;
}

/**
 * @brief flash_read Read from flash memory.
 *
 * @param[in] a_address       Address in flash.
 * @param[out] a_buf          Buffer to fill.
 * @param[in] a_buf_size      Size of buffer.
 * @return FLASH_SUCCESS if read successfully finished.
 */
flash_status_t flash_read(flash_address_t a_address, void * const a_buf, size_t a_buf_size)
{
    flash_status_t ret = FLASH_ERROR_FLASH_INIT;

    if (flash_initialized)
    {
        ret = FLASH_ERROR_FLASH_READ;
        if ((a_address + a_buf_size) <= flash_density_bytes
#if FLASH_BLOCK_RESERVED_NUM > 0
                && a_address >= (FLASH_BLOCK_RESERVED_NUM * flash_block_size_byte)
#endif
                )
        {
            SET_CS_LOW();
            flash_write_address(&cmd_read_data[1], a_address);
            if (HAL_SPI_Transmit(spi, cmd_read_data, flash_address_bytes + 1, FLASH_TIMEOUT_TICK) == HAL_OK)
            {
#if FLASH_ENABLE_DMA
                if (HAL_SPI_Receive_DMA(spi, a_buf, a_buf_size) == HAL_OK)
                {
                    if (osSemaphoreWait(dma_finished, FLASH_DMA_TIMEOUT_MS) == osOK)
                    {
                        ret = FLASH_SUCCESS;
                    }
                }
#else
                if (HAL_SPI_Receive(spi, a_buf, a_buf_size, FLASH_TIMEOUT_TICK) == HAL_OK)
                {
                    ret = FLASH_SUCCESS;
                }
#endif
            }
            SET_CS_HIGH();
        }
        else
        {
            FLASH_ERROR_MSG("Trying to read from invalid flash address! 0x%X\r\n",
                            a_address);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

/**
 * @brief flash_write Write to flash memory.
 *
 * @param[in] a_address       Address in flash.
 * @param[in] a_buf           Buffer to write.
 * @param[in] a_buf_size      Size of buffer.
 * @return FLASH_SUCCESS if write successfully finished.
 */
flash_status_t flash_write(flash_address_t a_address, const void * const a_buf, size_t a_buf_size)
{
    flash_status_t ret = FLASH_ERROR_FLASH_INIT;
    
    if (flash_initialized)
    {
        ret = FLASH_ERROR_FLASH_WRITE;
        if ((a_address + a_buf_size) <= flash_density_bytes
#if FLASH_BLOCK_RESERVED_NUM > 0
                && a_address >= (FLASH_BLOCK_RESERVED_NUM * flash_block_size_byte)
#endif
                )
        {
            ret = flash_write_enable();
            if (ret == FLASH_SUCCESS)
            {
                ret = FLASH_ERROR_FLASH_WRITE;
                SET_CS_LOW();
                flash_write_address(&cmd_page_program[1], a_address);
                if (HAL_SPI_Transmit(spi, cmd_page_program, flash_address_bytes + 1, FLASH_TIMEOUT_TICK) == HAL_OK)
                {
#if FLASH_ENABLE_DMA
                    if (HAL_SPI_Transmit_DMA(spi, a_buf, a_buf_size) == HAL_OK)
                    {
                        if (osSemaphoreWait(dma_finished, FLASH_DMA_TIMEOUT_MS) == osOK)
                        {
                            ret = FLASH_SUCCESS;
                        }
                    }
#else
                    if (HAL_SPI_Transmit(spi, a_buf, a_buf_size, FLASH_TIMEOUT_TICK) == HAL_OK)
                    {
                        ret = FLASH_SUCCESS;
                    }
#endif
                }
                SET_CS_HIGH();
                if (ret == FLASH_SUCCESS)
                {
                    ret = flash_wait(FLASH_WRITE_TIMEOUT_TICK, FALSE);
                }
            }
        }
        else
        {
            FLASH_ERROR_MSG("Trying to write to invalid flash address! 0x%X\r\n",
                            a_address);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

/**
 * @brief flash_erase Erase a block.
 *
 * @param[in] a_address       Address in flash.
 *
 * @return FLASH_SUCCESS if block was erased successfully.
 */
flash_status_t flash_erase(flash_address_t a_address)
{
    flash_status_t ret = FLASH_ERROR_FLASH_INIT;
    
    if (flash_initialized)
    {
        ret = FLASH_ERROR_FLASH_ERASE;
        if ((a_address + flash_block_size_byte) <= flash_density_bytes
#if FLASH_BLOCK_RESERVED_NUM > 0
                && a_address >= (FLASH_BLOCK_RESERVED_NUM * flash_block_size_byte)
#endif
            )
        {
            ret = flash_write_enable();
            if (ret == FLASH_SUCCESS)
            {
                SET_CS_LOW();
                flash_write_address(&cmd_erase[1], a_address);
                if (HAL_SPI_Transmit(spi, cmd_erase, flash_address_bytes + 1, FLASH_TIMEOUT_TICK) == HAL_OK)
                {
                    ret = FLASH_SUCCESS;
                }
                else
                {
                    ret = FLASH_ERROR_FLASH_ERASE;
                }
                SET_CS_HIGH();
                if (ret == FLASH_SUCCESS)
                {
                    ret = flash_wait(FLASH_ERASE_TIMEOUT_TICK, FALSE);
                }
            }
        }
        else
        {
            FLASH_ERROR_MSG("Trying to erase invalid flash address! 0x%X\r\n",
                            a_address);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

/**
 * @brief flash_erase Erase all blocks.
 *
 * @return FLASH_SUCCESS if chip was erased successfully.
 */
flash_status_t flash_erase_all(bool_t a_print_dots)
{
    flash_status_t ret = FLASH_ERROR_FLASH_INIT;

    if (flash_initialized)
    {
        ret = flash_write_enable();
        if (ret == FLASH_SUCCESS)
        {
            ret = flash_cmd(cmd_chip_erase, sizeof(cmd_chip_erase));
        }
        if (ret == FLASH_SUCCESS)
        {
            ret = flash_wait(FLASH_CHIP_ERASE_TIMEOUT_TICK, a_print_dots);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

/**
 * @brief flash_print_stat Called by the terminal to print information
 * about flash memory.
 */
void flash_print_stat(void)
{
}
