/**
 * @file        uart.h
 * @brief       Prototype of UART functions
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
#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <string.h>

extern void UART_putc(const char ch);
extern void UART_printf_(const char * fmt, ...);
extern void UART_printf(const char * fmt, ...);
extern uint8_t UART_getchar(void);
extern size_t UART_getLine(uint8_t * a_buf, size_t a_buf_size);

/* Fallback functions for exception handlers */
extern void _putchar(char c);
extern void _print (char * s);
extern void _printHex32 (uint32_t aNum);
extern void _printHex16 (uint16_t aNum);
extern void _printHex8 (uint8_t aNum);

#endif /* __UART_H */
