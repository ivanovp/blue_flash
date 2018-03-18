/**
  ******************************************************************************
  * @file           : usbd_dfu_if.c
  * @brief          : Usb device for Download Firmware Update.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_dfu_if.h"

/* USER CODE BEGIN INCLUDE */
#include "common.h"
#include "flash.h"
#include "uart.h"
#include "cmsis_os.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

bool_t flash_is_initialized = FALSE;
uint8_t page_buf[USBD_DFU_XFER_SIZE];
extern osSemaphoreId creset_sem;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_DFU
  * @brief Usb DFU device module.
  * @{
  */

/** @defgroup USBD_DFU_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_DFU_Private_Defines
  * @brief Private defines.
  * @{
  */

#define FLASH_DESC_STR      "@SPI Flash/0x00000000/32*064Kg"

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Use variable for flash description! */
#undef FLASH_DESC_STR
#define FLASH_DESC_STR      dfu_flash_descr
#define ENABLE_DFU_DEBUG    0

#if ENABLE_DFU_DEBUG
#define DFU_DEBUG_MSG(...)    do { \
        UART_printf(__VA_ARGS__); \
    } while (0);
#else
#define DFU_DEBUG_MSG(...)
#endif

#define DFU_INFO_MSG(...)    do { \
        UART_printf(__VA_ARGS__); \
    } while (0);
#define DFU_ERROR_MSG(...)    do { \
        UART_printf("%s:%i ERROR: ", __FUNCTION__, __LINE__); \
        UART_printf(__VA_ARGS__); \
    } while (0);
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_DFU_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_DFU_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_DFU_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_DFU_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static uint16_t MEM_If_Init_FS(void);
static uint16_t MEM_If_Erase_FS(uint32_t Add);
static uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_DeInit_FS(void);
static uint16_t MEM_If_GetStatus_FS(uint32_t Add, uint8_t Cmd, uint8_t *buffer);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN USBD_DFU_MediaTypeDef USBD_DFU_fops_FS __ALIGN_END =
{
   (uint8_t*)FLASH_DESC_STR,
    MEM_If_Init_FS,
    MEM_If_DeInit_FS,
    MEM_If_Erase_FS,
    MEM_If_Write_FS,
    MEM_If_Read_FS,
    MEM_If_GetStatus_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Memory initialization routine.
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Init_FS(void)
{
  /* USER CODE BEGIN 0 */
    return USBD_OK;
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes Memory
  * @retval USBD_OK if operation is successful, MAL_FAIL else
  */
uint16_t MEM_If_DeInit_FS(void)
{
  /* USER CODE BEGIN 1 */
    uint16_t ret = USBD_FAIL;

    ret = dfu_flash_deinit();

    return ret;
  /* USER CODE END 1 */
}

/**
  * @brief  Erase sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Erase_FS(uint32_t Add)
{
  /* USER CODE BEGIN 2 */
	uint16_t ret = USBD_FAIL;
    flash_block_address_t ba;
    flash_status_t fl_ret;

    ret = dfu_flash_init();
    if (ret == USBD_OK)
    {
        osSemaphoreRelease(creset_sem);
        SET_CRESET_ON();

        ba = Add / FLASH_BLOCK_SIZE_BYTE;
        fl_ret = flash_erase(ba);
        if (fl_ret == FLASH_SUCCESS)
        {
            DFU_DEBUG_MSG("Erased block %i\r\n", ba);
            ret = USBD_OK;
        }
        else
        {
            DFU_ERROR_MSG("Cannot erase block! Status: %i\r\n", fl_ret);
        }
    }

	return ret;
  /* USER CODE END 2 */
}

/**
  * @brief  Memory write routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* USER CODE BEGIN 3 */
	uint16_t ret = USBD_FAIL;
    flash_block_address_t ba;
    flash_page_address_t  pa;
    flash_page_offset_t   ofs;
	uintptr_t addr = (uintptr_t) dest;
    flash_status_t fl_ret;

    ret = dfu_flash_init();
    if (ret == USBD_OK)
    {
        osSemaphoreRelease(creset_sem);
        SET_CRESET_ON();

        ba = addr / FLASH_BLOCK_SIZE_BYTE;
        pa = (addr % FLASH_BLOCK_SIZE_BYTE) / FLASH_PAGE_SIZE_BYTE;
        ofs = (addr % FLASH_BLOCK_SIZE_BYTE) % FLASH_PAGE_SIZE_BYTE;
        fl_ret = flash_write(ba, pa, ofs, src, Len);
        if (fl_ret == FLASH_SUCCESS)
        {
            DFU_DEBUG_MSG("Written block %i, page %i\r\n", ba, pa);
            ret = USBD_OK;
        }
        else
        {
            DFU_ERROR_MSG("Cannot write! Status: %i\r\n", fl_ret);
        }
    }

	return ret;
  /* USER CODE END 3 */
}

/**
  * @brief  Memory read routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* Return a valid address to avoid HardFault */
  /* USER CODE BEGIN 4 */
	uint16_t             ret;
    flash_block_address_t ba;
    flash_page_address_t  pa;
    flash_page_offset_t   ofs;
    uintptr_t            addr = (uintptr_t) src;
    flash_status_t        fl_ret;

    ret = dfu_flash_init();
    if (ret == USBD_OK)
    {
        osSemaphoreRelease(creset_sem);
        SET_CRESET_ON();

        ba = addr / FLASH_BLOCK_SIZE_BYTE;
        pa = (addr % FLASH_BLOCK_SIZE_BYTE) / FLASH_PAGE_SIZE_BYTE;
        ofs = (addr % FLASH_BLOCK_SIZE_BYTE) % FLASH_PAGE_SIZE_BYTE;
        fl_ret = flash_read(ba, pa, ofs, page_buf, Len);
        if (fl_ret == FLASH_SUCCESS)
        {
            DFU_DEBUG_MSG("Read block %i, page %i\r\n", ba, pa);
        }
        else
        {
            DFU_ERROR_MSG("Cannot read! Status: %i\r\n", fl_ret);
        }
    }

	return page_buf;
  /* USER CODE END 4 */
}

/**
  * @brief  Get status routine
  * @param  Add: Address to be read from
  * @param  Cmd: Number of data to be read (in bytes)
  * @param  buffer: used for returning the time necessary for a program or an erase operation
  * @retval USBD_OK if operation is successful
  */
uint16_t MEM_If_GetStatus_FS(uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
  /* USER CODE BEGIN 5 */
    uint16_t ret = USBD_FAIL;

    DFU_DEBUG_MSG("Get status, add: 0x%X, cmd: %i, buf: 0x%X\r\n", Add, Cmd, buffer);

    ret = dfu_flash_init();
    if (ret == USBD_OK)
    {
        osSemaphoreRelease(creset_sem);
        SET_CRESET_ON();

        switch (Cmd)
        {
            case DFU_MEDIA_PROGRAM:

                break;

            case DFU_MEDIA_ERASE:
            default:

                break;
        }
    }

    return ret;
  /* USER CODE END 5 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint16_t dfu_flash_init(void)
{
    uint16_t ret = USBD_FAIL;
    flash_status_t fl_ret;
    GPIO_InitTypeDef GPIO_InitStruct;

    if (!flash_is_initialized)
    {
        SET_CRESET_ON();

        if (HAL_SPI_Init(&hspi1) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }

        GPIO_InitStruct.Pin = SPI_CS_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

        fl_ret = flash_init();
        if (fl_ret == FLASH_SUCCESS)
        {
            DFU_INFO_MSG("Flash initialized\r\n");
            DFU_INFO_MSG("DFU flash descriptor: %s\r\n", dfu_flash_descr);
            ret = USBD_OK;
            flash_is_initialized = TRUE;
        }
        else
        {
            DFU_ERROR_MSG("Cannot initialize flash! Status: %i\r\n", fl_ret);
            SET_CRESET_OFF();
        }
    }
    else
    {
        ret = USBD_OK;
    }

    return ret;
}

void dfu_release_flash_interface(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    DFU_INFO_MSG("Release CRESET\r\n");
    SET_CRESET_OFF();
    osDelay(100);
    HAL_SPI_DeInit(&hspi1);
    DFU_INFO_MSG("SPI deinitialized\r\n");
    GPIO_InitStruct.Pin = SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    DFU_INFO_MSG("GPIO pins released\r\n");

    DFU_INFO_MSG("Set CRESET\r\n");
    SET_CRESET_ON();
    osDelay(50);
    DFU_INFO_MSG("Release CRESET\r\n");
    SET_CRESET_OFF();
}

uint16_t dfu_flash_deinit(void)
{
    uint16_t ret = USBD_FAIL;
    flash_status_t fl_ret;

    if (flash_is_initialized)
    {
        fl_ret = flash_delete();
        if (fl_ret == FLASH_SUCCESS)
        {
            DFU_INFO_MSG("Flash de-initialized\r\n");

            dfu_release_flash_interface();

            ret = USBD_OK;
            flash_is_initialized = FALSE;
        }
        else
        {
            DFU_ERROR_MSG("Cannot de-initialize flash! Status: %i\r\n", fl_ret);
        }
    }

    return ret;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
