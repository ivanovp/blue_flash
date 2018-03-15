/**
 * @file        common.h
 * @brief       Common definitions
 * @author      Copyright (C) Peter Ivanov, 2011, 2012, 2017
 *
 * Created      2011-01-19 11:48:53
 * Last modify: 2017-06-27 19:18:04 ivanovp {Time-stamp}
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

#ifndef _INCLUDE_COMMON_H_
#define _INCLUDE_COMMON_H_

#ifndef bool_t
typedef unsigned char bool_t;
#endif

#ifndef FALSE
#define FALSE           (0)
#endif
#ifndef TRUE
#define TRUE            (1)
#endif

#define TOSTR(s)    XSTR(s)
#define XSTR(s)     #s

#define ASCII_CR            (13)
#define ASCII_LF            (10)
#define ASCII_SPACE         (' ')
#define ASCII_BACKSPACE     (8)
#define ASCII_TAB           ('\t')
#define ASCII_QUOTE         ('"')
#define ASCII_APOSTROPHE    ('\'')

#define ENABLE_WATCHDOG     0

#define SET_CRESET_ON()     HAL_GPIO_WritePin(CRESET_GPIO_Port, CRESET_Pin, GPIO_PIN_RESET);
#define SET_CRESET_OFF()    HAL_GPIO_WritePin(CRESET_GPIO_Port, CRESET_Pin, GPIO_PIN_SET);

#endif /* _INCLUDE_COMMON_H_ */
