/**
 * Nvic.c
 *
 * Created on: 2026-04-12
 * Author    : AbdallahDarwish
 */

#include "Nvic.h"

typedef struct {
    volatile uint32 NVIC_ISER[8];
    uint32 _r[24];
    volatile uint32 NVIC_ICER[8];
} NvicType;

#define NVIC          ((NvicType*)0xE000E100)


void Nvic_EnableIrq(uint8 IrqNumber) {
    NVIC->NVIC_ISER[IrqNumber / 32] |= (0x01 << (IrqNumber % 32));
}


void Nvic_DisableIrq(uint8 IrqNumber) {
    NVIC->NVIC_ICER[IrqNumber / 32] |= (0x01 << (IrqNumber % 32));
}


void Nvic_SetPriority(uint8 IrqNumber, uint8 Priority) {
    /* IPR registers are byte-accessible.
     * STM32F4 uses 4 bits of priority in the upper nibble [7:4]. */
    volatile uint8 *ipr = (volatile uint8 *)(0xE000E400UL + IrqNumber);
    *ipr = (Priority << 4);
}


void Nvic_SetPriorityGrouping(uint8 PriorityGroup) {
    /* SCB->AIRCR at 0xE000ED0C.  Must write VECTKEY = 0x05FA. */
    volatile uint32 *aircr = (volatile uint32 *)0xE000ED0CUL;
    uint32 val = *aircr;
    val &= ~(0xFFFF0700UL);                         /* clear key + PRIGROUP */
    val |= (0x05FA0000UL) | ((uint32)(PriorityGroup & 0x07) << 8);
    *aircr = val;
}
