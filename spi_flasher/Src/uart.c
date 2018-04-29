/**
 * @file        uart.c
 * @brief       UART printf, putc functions
 * @author      Copyright (C) Peter Ivanov, 2017
 *
 * Created      2017-06-13 11:48:53
 * Last modify: 2017-06-16 19:07:42 ivanovp {Time-stamp}
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
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "common.h"

extern UART_HandleTypeDef huart1;

extern osMutexId printf_mutex;
static char printf_buf[128];

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief UART_putc Send a character to the UART.
 * @param ch Character to print.
 */
void UART_putc(const char ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*) &ch, sizeof(ch), HAL_MAX_DELAY);
}

/**
 * @brief Print string to the UART. It works like the usual printf.
 * Use when OS is not running!
 * @param fmt Format string.
 */
void UART_printf_(const char * fmt, ...)
{
    static va_list valist;

    va_start(valist, fmt);
    vsnprintf(printf_buf, sizeof(printf_buf), fmt, valist);
    va_end(valist);
    HAL_UART_Transmit(&huart1, (uint8_t*) printf_buf, strlen(printf_buf),
            HAL_MAX_DELAY);
}

/**
 * @brief Print string to the UART. It works like the usual printf.
 * Use when OS is running (thread safe).
 * @param fmt Format string.
 */
void UART_printf(const char * fmt, ...)
{
    static va_list valist;

    if (printf_mutex && osMutexWait(printf_mutex, osWaitForever) == osOK)
    {
        va_start(valist, fmt);
        vsnprintf(printf_buf, sizeof(printf_buf), fmt, valist);
        va_end(valist);
        HAL_UART_Transmit(&huart1, (uint8_t*) printf_buf, strlen(printf_buf),
                HAL_MAX_DELAY);
        if (printf_mutex)
        {
            osMutexRelease(printf_mutex);
        }
    }
}

/**
 * @brief UART_getchar Read a character from UART. Blocking!
 * @return Character from UART or 0 if unsuccessful.
 */
uint8_t UART_getchar(void)
{
    uint8_t ch;

    if (HAL_UART_Receive(&huart1, &ch, sizeof(ch), HAL_MAX_DELAY) == HAL_OK)
    {
        return ch;
    }

    return 0;
}

/**
 * @brief UART_getchar Read a character from UART. Non-blocking!
 * @return Character from UART or 0 if unsuccessful.
 */
uint8_t UART_getcharNB(void)
{
    uint8_t ch;

    if (HAL_UART_Receive(&huart1, &ch, sizeof(ch), 0) == HAL_OK)
    {
        return ch;
    }

    return 0;
}

size_t UART_getLine(uint8_t * a_buf, size_t a_buf_size)
{
    size_t idx = 0;
    uint8_t ch = 0;

    do
    {
        while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
        {
            osDelay(1);
        }
        if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
        {
            (void)HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
            a_buf[idx] = ch;
            idx++;
        }
    } while (ch != ASCII_LF && idx < a_buf_size);

    a_buf[idx] = 0;

    return idx;
}
int _write(int file, char *buf, int len)
{
    if (printf_mutex)
    {
        /* Inside OS context */
        if (osMutexWait(printf_mutex, osWaitForever) == osOK)
        {
            HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
            if (printf_mutex)
            {
                osMutexRelease(printf_mutex);
            }
        }
    }
    else
    {
        /* Outside OS context */
        HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
    }
    return len;
}

int _read(int file, char *buf, int len)
{
    int cntr = 0;
    if (printf_mutex)
    {
        /* Inside OS context */
        if (osMutexWait(printf_mutex, osWaitForever) == osOK)
        {
            HAL_UART_Receive(&huart1, buf, 1, HAL_MAX_DELAY);
            HAL_UART_Transmit(&huart1, buf, 1, HAL_MAX_DELAY); /* Echo */
            buf++;
            cntr++;
            while (HAL_UART_Receive(&huart1, buf, 1, 0) == HAL_OK && cntr < len)
            {
                HAL_UART_Transmit(&huart1, buf, 1, HAL_MAX_DELAY); /* Echo */
                buf++;
                cntr++;
            }
            if (printf_mutex)
            {
                osMutexRelease(printf_mutex);
            }
        }
    }
    else
    {
        /* Outside OS context */
        while (HAL_UART_Receive(&huart1, buf, 1, 0) == HAL_OK && cntr < len)
        {
            HAL_UART_Transmit(&huart1, buf, 1, HAL_MAX_DELAY); /* Echo */
            buf++;
            cntr++;
        }
    }
    return cntr;
}
