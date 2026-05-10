/**
 * Nvic.h
 *
 * Created on: 2026-04-12
 * Author    : AbdallahDarwish
 */

#ifndef NVIC_H
#define NVIC_H
#include "Std_Types.h"


void Nvic_EnableIrq(uint8 IrqNumber);

void Nvic_DisableIrq(uint8 IrqNumber);

/**
 * @brief  Set the priority of an IRQ (0 = highest).
 * @param  IrqNumber  IRQ number (0-239)
 * @param  Priority   Priority level (0-15 for 4-bit priority)
 */
void Nvic_SetPriority(uint8 IrqNumber, uint8 Priority);

/**
 * @brief  Configure the priority grouping (PRIGROUP field in SCB->AIRCR).
 * @param  PriorityGroup  0-7 (e.g. 3 = 4 bits preemption, 0 bits sub)
 */
void Nvic_SetPriorityGrouping(uint8 PriorityGroup);

#endif //NVIC_H
