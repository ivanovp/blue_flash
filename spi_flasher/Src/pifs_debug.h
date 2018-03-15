/**
 * @file        pifs_debug.h
 * @brief       Common definitions
 * @author      Copyright (C) Peter Ivanov, 2017
 *
 * Created:     2017-06-11 09:10:19
 * Last modify: 2017-06-16 13:12:44 ivanovp {Time-stamp}
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
#ifndef _INCLUDE_PIFS_DEBUG_H_
#define _INCLUDE_PIFS_DEBUG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "common.h"

#if ENABLE_SW_TRAP == 0
#define SOFTWARE_BREAKPOINT()
#else
#if defined(__i386__) || defined(__x86_64__)
#define SOFTWARE_BREAKPOINT() do    \
    {                               \
        __asm__ volatile("int $3"); \
    } while (0)
#else
#define SOFTWARE_BREAKPOINT() __builtin_trap()
#endif
#endif

#define PIFS_ASSERT(expression) do {                                       \
        if (!((expression)))                                               \
        {                                                                  \
            printf("ERROR: Assertion '%s' failed. File: %s, line: %i\r\n", \
                    TOSTR(expression),                                     \
                    __FILE__, __LINE__                                     \
                   );                                                      \
            pifs_delete();                                                 \
            SOFTWARE_BREAKPOINT();                                         \
            exit(-1);                                                      \
        }                                                                  \
    } while (0);

#define PIFS_PRINT_MSG(...) printf(__VA_ARGS__)

#if (PIFS_DEBUG_LEVEL >= 1)
#define PIFS_ERROR_MSG(...)    do { \
        printf("%s:%i ERROR: ", __FUNCTION__, __LINE__);        \
        printf(__VA_ARGS__);                                    \
        fflush(stdout);                                         \
    } while (0);
#else
#define PIFS_ERROR_MSG(...)
#endif

#if (PIFS_DEBUG_LEVEL >= 1)
#define PIFS_FATAL_ERROR_MSG(...)    do {                       \
        printf("%s:%i FATAL_ERROR: ", __FUNCTION__, __LINE__);  \
        printf( __VA_ARGS__);                                   \
        pifs_delete();                                          \
        fflush(stdout);                                         \
        SOFTWARE_BREAKPOINT();                                  \
        exit(-1);                                               \
    } while (0);
#else
#define PIFS_FATAL_ERROR_MSG(...)
#endif

#if (PIFS_DEBUG_LEVEL >= 2)
#define PIFS_WARNING_MSG(...)    do { \
        printf("%s:%i WARNING: ", __FUNCTION__, __LINE__);      \
        printf(__VA_ARGS__);                                    \
        fflush(stdout);                                         \
    } while (0);
#else
#define PIFS_WARNING_MSG(...)
#endif

#if (PIFS_DEBUG_LEVEL >= 3)
#define PIFS_NOTICE_MSG(...)    do {                            \
        printf("%s ", __FUNCTION__);                            \
        printf(__VA_ARGS__);                                    \
        fflush(stdout);                                         \
    } while (0);
#else
#define PIFS_NOTICE_MSG(...)
#endif

#if (PIFS_DEBUG_LEVEL >= 4)
#define PIFS_INFO_MSG(...)    do {                              \
        printf("%s ", __FUNCTION__);                            \
        printf(__VA_ARGS__);                                    \
        fflush(stdout);                                         \
    } while (0);
#else
#define PIFS_INFO_MSG(...)
#endif

#if (PIFS_DEBUG_LEVEL >= 5)
#define PIFS_DEBUG_MSG(...)    do {                             \
        printf("%s ", __FUNCTION__);                            \
        printf(__VA_ARGS__);                                    \
        fflush(stdout);                                         \
    } while (0);
#else
#define PIFS_DEBUG_MSG(...)
#endif

#endif /* _INCLUDE_PIFS_DEBUG_H_ */
