/**
 * @file        isr_handlers.c
 * @brief       Hard fault exception handler
 * @author      Copyright (C) Peter Ivanov, 2017
 *
 * Created      2017-06-13 11:48:53
 * Last modify: 2018-03-15 18:32:48 ivanovp {Time-stamp}
 * Licence:     GPL
 */
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

/* 1: Use assembly HardFault handler. This does not work without debugger!
 * 0: Use C HardFault handler.
 */
#define HARDFAULT_HANDLER_ASM_STUB   0

static const char  hexChars[] = "0123456789ABCDEF";

#if 0
extern void UART_putc(const char ch);
#define _putchar(c)  UART_putc(c)
#else
void _putchar(char c)
{
    USART1->DR = c;
    while ((USART1->SR & UART_FLAG_TXE) == 0u)
    {
    }
}
#endif

void _print (char * s)
{
    while (*s)
    {
        _putchar((uint8_t)*s++);
    }
}

void _printHex32 (uint32_t aNum)
{
    int  i = 8;

    while (i-- > 0)
    {
        _putchar(hexChars[(aNum >> 28) & 0xF]);
        aNum <<= 4;
    }
}

void _printHex16 (uint16_t aNum)
{
    int  i = 4;

    while (i-- > 0)
    {
        _putchar(hexChars[(aNum >> 12) & 0xF]);
        aNum <<= 4;
    }
}

void _printHex8 (uint8_t aNum)
{
    _putchar(hexChars[(aNum >> 4) & 0xF]);
    _putchar(hexChars[aNum & 0xF]);
}

/**
 * @brief Hard Fault handler in C.
 * Idea comes from Joseph Yiu, minor edits by FVH.
 * Called from HardFault_Handler in file xxx.s
 * UFSR, BFSR, MMFSR, HFSR register parser
 * by Peter Ivanov.
 */
//__attribute__((naked)) /* This disturbs the debugger! */
#if HARDFAULT_HANDLER_ASM_STUB
void hard_fault_handler_c (uint32_t * args)
#else
void HardFault_Handler(uint32_t * args)
#endif
{
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_psr;

    uint32_t ccr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t afsr;
    uint32_t dfsr;
    uint16_t ufsr;
    uint8_t  mmfsr;
    uint8_t  bfsr;

#if HARDFAULT_HANDLER_ASM_STUB == 0
    asm("TST LR, #4\r\n"
        "ITE EQ            /* if-then-else */\r\n"
        "MRSEQ R0, MSP     /* if TST LR, #4 is true*/\r\n"
        "MRSNE R0, PSP     /* otherwise */\r\n");
#endif

    stacked_r0 = ((uint32_t) args[0]);
    stacked_r1 = ((uint32_t) args[1]);
    stacked_r2 = ((uint32_t) args[2]);
    stacked_r3 = ((uint32_t) args[3]);
    stacked_r12 = ((uint32_t) args[4]);
    stacked_lr = ((uint32_t) args[5]);
    stacked_pc = ((uint32_t) args[6]);
    stacked_psr = ((uint32_t) args[7]);

    ccr = *((volatile uint32_t *)0xE000ED14);
    cfsr = *((volatile uint32_t *)0xE000ED28);
    hfsr = *((volatile uint32_t *)0xE000ED2C);
    mmfar = *((volatile uint32_t *)0xE000ED34);
    bfar = *((volatile uint32_t *)0xE000ED38);
    afsr = *((volatile uint32_t *)0xE000ED3C);
    dfsr = *((volatile uint32_t *)0xE000ED30);
    ufsr = cfsr >> 16;
    mmfsr = cfsr & 0xFF;
    bfsr = (cfsr >> 8) & 0xFF;

    _print("\r\n\r\nHardFault exception!\r\n");
    _print("\r\nR0:     0x");
    _printHex32(stacked_r0);
    _print("\r\nR1:     0x");
    _printHex32(stacked_r1);
    _print("\r\nR2:     0x");
    _printHex32(stacked_r2);
    _print("\r\nR3:     0x");
    _printHex32(stacked_r3);
    _print("\r\nR12:    0x");
    _printHex32(stacked_r12);
    _print("\r\nR14/LR: 0x");
    _printHex32(stacked_lr);
    _print("\r\nR15/PC: 0x");
    _printHex32(stacked_pc);
    _print("\r\nPSR:    0x");
    _printHex32(stacked_psr);

    _print("\r\nCCR:    0x");
    _printHex32(ccr);

    _print("\r\nCFSR:   0x");
    _printHex32(cfsr);

    _print("\r\n-UFSR:  0x");
    _printHex16(ufsr);
    if (ufsr & (1u << 9))
    {
        _print(" DIVBYZERO");
    }
    if (ufsr & (1u << 8))
    {
        _print(" UNALIGNED");
    }
    if (ufsr & (1u << 3))
    {
        _print(" NOCP");
    }
    if (ufsr & (1u << 2))
    {
        _print(" INVPC");
    }
    if (ufsr & (1u << 1))
    {
        _print(" INVSTATE");
    }
    if (ufsr & (1u << 0))
    {
        _print(" UNDEFINSTR");
    }
    _print("\r\n-BFSR:  0x");
    _printHex8(bfsr);
    if (bfsr & (1u << 7))
    {
        _print(" BFARVALID");
    }
    if (bfsr & (1u << 4))
    {
        _print(" STKERR");
    }
    if (bfsr & (1u << 3))
    {
        _print(" UNSTKERR");
    }
    if (bfsr & (1u << 2))
    {
        _print(" IMPRECISERR");
    }
    if (bfsr & (1u << 1))
    {
        _print(" PRECISERR");
    }
    if (bfsr & (1u << 0))
    {
        _print(" IBUSERR");
    }
    _print("\r\n-MMFSR: 0x");
    _printHex8(mmfsr);
    if (mmfsr & (1u << 7))
    {
        _print(" MMARVALID");
    }
    if (mmfsr & (1u << 4))
    {
        _print(" MSTKERR");
    }
    if (mmfsr & (1u << 3))
    {
        _print(" MUNSTKERR");
    }
    if (mmfsr & (1u << 1))
    {
        _print(" DACCVIOL");
    }
    if (mmfsr & (1u << 0))
    {
        _print(" IACCVIOL");
    }

    _print("\r\nHFSR:   0x");
    _printHex32(hfsr);
    if (hfsr & (1u << 30))
    {
        _print(" FORCED");
    }
    if (hfsr & (1u << 1))
    {
        _print(" VECTTBL");
    }

    _print("\r\nMMFAR:  0x");
    _printHex32(mmfar);

    _print("\r\nBFAR:   0x");
    _printHex32(bfar);

    _print("\r\nAFSR:   0x");
    _printHex32(afsr);

    _print("\r\nDFSR:   0x");
    _printHex32(dfsr);

    _print("\r\nSHCSR:  0x");
    _printHex32(SCB->SHCSR);

    _print("\r\n");

    asm ("BKPT #01");
    while (1);
}
