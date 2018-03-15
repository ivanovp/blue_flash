/**
 * @file        flash_drv/stm32_spi_dma/flash.c
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

//#include "api_pifs.h"
#include "flash.h"
#include "pifs_debug.h"
#include "uart.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifndef PIFS_FLASH_4BYTE_ADDRESS
#define PIFS_FLASH_4BYTE_ADDRESS	0
#endif

#define FLASH_ENABLE_DEBUG          1
#ifndef FLASH_ENABLE_DMA
#define FLASH_ENABLE_DMA            0
#endif
#define FLASH_TIMEOUT_MS            1000
#define FLASH_WRITE_TIMEOUT_MS      5000
#define FLASH_DMA_TIMEOUT_MS        5

#define SET_CS_LOW()    do { \
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); \
	} while (0);
#define SET_CS_HIGH()   do { \
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); \
	} while (0);
#if PIFS_FLASH_4BYTE_ADDRESS
#define WRITE_ADDRESS(buf, block_address, page_address, page_offset) do { \
        (buf)[0] = (block_address) >> 8; \
		(buf)[1] = (block_address); \
		(buf)[2] = (page_address); \
		(buf)[3] = (page_offset); \
	} while (0);
#else
#define WRITE_ADDRESS(buf, block_address, page_address, page_offset) do { \
        (buf)[0] = (block_address); \
        (buf)[1] = (page_address); \
        (buf)[2] = (page_offset); \
    } while (0);
#endif
#define STATUS_REG_WIP              0x01    /**< Write in progress bit in status register */

#define MS_TO_TICK(ms)              (ms * 1000 / osKernelSysTickFrequency)
#define FLASH_TIMEOUT_TICK          MS_TO_TICK(FLASH_TIMEOUT_MS)
#define FLASH_WRITE_TIMEOUT_TICK    MS_TO_TICK(FLASH_WRITE_TIMEOUT_MS)

#if FLASH_ENABLE_DEBUG
#define FLASH_ERROR_MSG(...)    do { \
        UART_printf("%s:%i ERROR: ", __FUNCTION__, __LINE__); \
        UART_printf(__VA_ARGS__); \
    } while (0);
#else
#define FLASH_ERROR_MSG(...)
#endif

extern SPI_HandleTypeDef hspi1;

static SPI_HandleTypeDef *spi = &hspi1;
static bool_t flash_initialized = FALSE;
static const uint8_t cmd_read_id[1]          = { 0x9F };
static const uint8_t cmd_read_status_reg[1]  = { 0x05 };
#if PIFS_FLASH_4BYTE_ADDRESS
static const uint8_t cmd_enter_4byte_mode[1] = { 0xB7 };
static uint8_t cmd_erase[5]                  = { 0xD8 };
static uint8_t cmd_read_data[5]              = { 0x13 };
static uint8_t cmd_page_program[5]           = { 0x02 };
#else
static uint8_t cmd_erase[4]                  = { 0xD8 };
static uint8_t cmd_read_data[4]              = { 0x03 };
static uint8_t cmd_page_program[4]           = { 0x02 };
#endif
static const uint8_t cmd_write_enable[1]     = { 0x06 };
static uint8_t answer[3];
#if FLASH_ENABLE_DMA
static osSemaphoreId dma_finished;
osSemaphoreDef(dma_finished);
#endif

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

/**
 * @brief pifs_flash_write_enable Run write enable command. Necessary before
 * writing or erasing the flash memory.
 *
 * @return PIFS_SUCCESS if write enabled.
 */
pifs_status_t pifs_flash_write_enable(void)
{
    pifs_status_t ret = PIFS_ERROR_FLASH_GENERAL;

    SET_CS_LOW();
    if (HAL_SPI_Transmit(spi, cmd_write_enable, sizeof(cmd_write_enable), FLASH_TIMEOUT_TICK) == HAL_OK)
	{
		ret = PIFS_SUCCESS;
	}
	SET_CS_HIGH();

	return ret;
}

/**
 * @brief pifs_flash_wait Wait for end of write/erase.
 *
 * @return PIFS_SUCCESS if write or erase operation finished successfully.
 */
pifs_status_t pifs_flash_wait(void)
{
    pifs_status_t ret = PIFS_ERROR_FLASH_TIMEOUT;
    uint32_t      tick_start;

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
                    ret = PIFS_SUCCESS;
                }
            }
        }
        SET_CS_HIGH();
    } while (ret != PIFS_SUCCESS && (osKernelSysTick() - tick_start) < FLASH_WRITE_TIMEOUT_TICK);

    return ret;
}

/**
 * @brief pifs_flash_init Initialize flash driver.
 *
 * @return PIFS_SUCCESS if flash memory is identified and initialized.
 */
pifs_status_t pifs_flash_init(void)
{
    pifs_status_t ret;

    flash_initialized = FALSE;

#if FLASH_ENABLE_DMA
    ret = PIFS_ERROR_FLASH_GENERAL;
    dma_finished = osSemaphoreCreate(osSemaphore(dma_finished), 1);
    if (dma_finished)
    {
        osSemaphoreWait(dma_finished, osWaitForever);
    }
    if (dma_finished)
#endif
    {
        ret = PIFS_ERROR_FLASH_INIT;
        SET_CS_LOW();
        if (HAL_SPI_Transmit(spi, cmd_read_id, sizeof(cmd_read_id), FLASH_TIMEOUT_TICK) == HAL_OK)
        {
            if (HAL_SPI_Receive(spi, answer, 3, FLASH_TIMEOUT_TICK) == HAL_OK)
            {
#if FLASH_TYPE == FLASH_TYPE_M25P40
                if (answer[0] == 0x20 && answer[1] == 0x20 && answer[2] == 0x13)
#elif FLASH_TYPE == FLASH_TYPE_W25Q16DV_4K || FLASH_TYPE == FLASH_TYPE_W25Q16DV_32K || FLASH_TYPE == FLASH_TYPE_W25Q16DV_64K
                if (answer[0] == 0xEF && answer[1] == 0x40 && answer[2] == 0x15)
#elif FLASH_TYPE == FLASH_TYPE_W25Q32BV_4K || FLASH_TYPE == FLASH_TYPE_W25Q32BV_32K || FLASH_TYPE == FLASH_TYPE_W25Q32BV_64K
                if (answer[0] == 0xEF && answer[1] == 0x40 && answer[2] == 0x16)
#elif FLASH_TYPE == FLASH_TYPE_W25Q256FV_4K || FLASH_TYPE == FLASH_TYPE_W25Q256FV_64K
                if (answer[0] == 0xEF && answer[1] == 0x40 && answer[2] == 0x19)
#elif FLASH_TYPE == FLASH_TYPE_S25FL127S_64K || FLASH_TYPE == FLASH_TYPE_S25FL127S_256K
                if (answer[0] == 0x01 && answer[1] == 0x20 && answer[2] == 0x18)
#endif
                {
                    flash_initialized = TRUE;
                    ret = PIFS_SUCCESS;
                }
                else
                {
                    FLASH_ERROR_MSG("Cannot identify flash: 0x%02X 0x%02X 0x%02X\r\n",
                                    answer[0], answer[1], answer[2]);
                }
            }
        }
        SET_CS_HIGH();

#if PIFS_FLASH_4BYTE_ADDRESS
        if (ret == PIFS_SUCCESS)
        {
            //osDelay(1);
            SET_CS_LOW();
            if (HAL_SPI_Transmit(spi, cmd_enter_4byte_mode, sizeof(cmd_enter_4byte_mode), FLASH_TIMEOUT) == HAL_OK)
            {
                ret = PIFS_SUCCESS;
            }
            else
            {
                ret = PIFS_ERROR_FLASH_INIT;
            }
            SET_CS_HIGH();
        }
#endif
    }

    return ret;
}

/**
 * @brief pifs_flash_delete De-initialize flash driver.
 *
 * @return PIFS_SUCCESS if successfully de-initialized.
 */
pifs_status_t pifs_flash_delete(void)
{
    pifs_status_t ret = PIFS_ERROR_FLASH_GENERAL;

#if FLASH_ENABLE_DMA
    if (osSemaphoreDelete(dma_finished) == osOK)
#endif
    {
        flash_initialized = FALSE;
        ret = PIFS_SUCCESS;
    }

    return ret;
}

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
pifs_status_t pifs_flash_read(pifs_block_address_t a_block_address, pifs_page_address_t a_page_address, pifs_page_offset_t a_page_offset, void * const a_buf, size_t a_buf_size)
{
    pifs_status_t ret = PIFS_ERROR_FLASH_INIT;
    long int offset = a_block_address * PIFS_FLASH_BLOCK_SIZE_BYTE 
        + a_page_address * PIFS_FLASH_PAGE_SIZE_BYTE
        + a_page_offset;

    if (flash_initialized)
    {
        ret = PIFS_ERROR_FLASH_READ;
        if ((offset + a_buf_size) <= PIFS_FLASH_SIZE_BYTE_ALL
#if PIFS_FLASH_BLOCK_RESERVED_NUM > 0
                && offset >= (PIFS_FLASH_BLOCK_RESERVED_NUM * PIFS_FLASH_BLOCK_SIZE_BYTE)
#endif
                )
        {
            SET_CS_LOW();
            WRITE_ADDRESS(&cmd_read_data[1], a_block_address, a_page_address, a_page_offset);
            if (HAL_SPI_Transmit(spi, cmd_read_data, sizeof(cmd_read_data), FLASH_TIMEOUT_TICK) == HAL_OK)
            {
#if FLASH_ENABLE_DMA
                if (HAL_SPI_Receive_DMA(spi, a_buf, a_buf_size) == HAL_OK)
                {
                    if (osSemaphoreWait(dma_finished, FLASH_DMA_TIMEOUT_MS) == osOK)
                    {
                        ret = PIFS_SUCCESS;
                    }
                }
#else
                if (HAL_SPI_Receive(spi, a_buf, a_buf_size, FLASH_TIMEOUT_TICK) == HAL_OK)
                {
                    ret = PIFS_SUCCESS;
                }
#endif
            }
            SET_CS_HIGH();
        }
        else
        {
            FLASH_ERROR_MSG("Trying to read from invalid flash address! BA%i/PA%i/OFS%i\r\n",
                            a_block_address, a_page_address, a_page_offset);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

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
pifs_status_t pifs_flash_write(pifs_block_address_t a_block_address, pifs_page_address_t a_page_address, pifs_page_address_t a_page_offset, const void * const a_buf, size_t a_buf_size)
{
    pifs_status_t ret = PIFS_ERROR_FLASH_INIT;
    long int offset = a_block_address * PIFS_FLASH_BLOCK_SIZE_BYTE 
        + a_page_address * PIFS_FLASH_PAGE_SIZE_BYTE
        + a_page_offset;
    
    if (flash_initialized)
    {
        ret = PIFS_ERROR_FLASH_WRITE;
        if ((offset + a_buf_size) <= PIFS_FLASH_SIZE_BYTE_ALL
#if PIFS_FLASH_BLOCK_RESERVED_NUM > 0
                && offset >= (PIFS_FLASH_BLOCK_RESERVED_NUM * PIFS_FLASH_BLOCK_SIZE_BYTE)
#endif
                )
        {
            ret = pifs_flash_write_enable();
            if (ret == PIFS_SUCCESS)
            {
                SET_CS_LOW();
                WRITE_ADDRESS(&cmd_page_program[1], a_block_address, a_page_address, a_page_offset);
                if (HAL_SPI_Transmit(spi, cmd_page_program, sizeof(cmd_page_program), FLASH_TIMEOUT_TICK) == HAL_OK)
                {
#if FLASH_ENABLE_DMA
                    if (HAL_SPI_Transmit_DMA(spi, a_buf, a_buf_size) == HAL_OK)
                    {
                        if (osSemaphoreWait(dma_finished, FLASH_DMA_TIMEOUT_MS) == osOK)
                        {
                            ret = PIFS_SUCCESS;
                        }
                    }
#else
                    if (HAL_SPI_Transmit(spi, a_buf, a_buf_size, FLASH_TIMEOUT_TICK) == HAL_OK)
                    {
                        ret = PIFS_SUCCESS;
                    }
#endif
                }
                SET_CS_HIGH();
                if (ret == PIFS_SUCCESS)
                {
                    ret = pifs_flash_wait();
                }
            }
        }
        else
        {
            FLASH_ERROR_MSG("Trying to write to invalid flash address! BA%i/PA%i/OFS%i\r\n",
                            a_block_address, a_page_address, a_page_offset);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

/**
 * @brief pifs_flash_erase Erase a block.
 *
 * @param[in] a_block_address Block to erase.
 *
 * @return PIFS_SUCCESS if block was erased successfully.
 */
pifs_status_t pifs_flash_erase(pifs_block_address_t a_block_address)
{
    pifs_status_t ret = PIFS_ERROR_FLASH_INIT;
    long int offset = a_block_address * PIFS_FLASH_BLOCK_SIZE_BYTE;
    
    if (flash_initialized)
    {
        ret = PIFS_ERROR_FLASH_ERASE;
        if ((offset + PIFS_FLASH_BLOCK_SIZE_BYTE) <= PIFS_FLASH_SIZE_BYTE_ALL
#if PIFS_FLASH_BLOCK_RESERVED_NUM > 0
                && offset >= (PIFS_FLASH_BLOCK_RESERVED_NUM * PIFS_FLASH_BLOCK_SIZE_BYTE)
#endif
            )
        {
            ret = pifs_flash_write_enable();
            if (ret == PIFS_SUCCESS)
            {
                SET_CS_LOW();
                WRITE_ADDRESS(&cmd_erase[1], a_block_address, 0, 0);
                if (HAL_SPI_Transmit(spi, cmd_erase, sizeof(cmd_erase), FLASH_TIMEOUT_TICK) == HAL_OK)
                {
                    ret = PIFS_SUCCESS;
                }
                SET_CS_HIGH();
                if (ret == PIFS_SUCCESS)
                {
                    ret = pifs_flash_wait();
                }
            }
        }
        else
        {
            FLASH_ERROR_MSG("Trying to erase invalid flash address! BA%i\r\n",
                            a_block_address);
        }
    }
    else
    {
        FLASH_ERROR_MSG("Not initialized!\r\n");
    }

    return ret;
}

/**
 * @brief pifs_flash_print_stat Called by the terminal to print information
 * about flash memory.
 */
void pifs_flash_print_stat(void)
{
}
