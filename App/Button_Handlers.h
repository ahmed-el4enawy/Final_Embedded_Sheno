/**
 * Button_Handlers.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  EXTI callback wiring for cabin buttons, hallway buttons,
 *  emergency stop, and floor sensors.
 *
 *  [FIX #7] All buttons are now interrupt-driven via EXTI.
 *  Cabin buttons moved to PB12-PB15 (EXTI12-15) to avoid
 *  the EXTI mux conflict with floor sensors (PC0-3 on EXTI0-3).
 */

#ifndef BUTTON_HANDLERS_H
#define BUTTON_HANDLERS_H

#include "Elevator_FSM.h"

/**
 * @brief  Configure all GPIO pins and EXTI interrupts for buttons
 *         and floor sensors.  Must be called after RCC and SYSCFG
 *         clocks are enabled.
 * @param  ctx  Pointer to the local elevator context
 */
void Buttons_Init(ElevatorContext *ctx);

#endif /* BUTTON_HANDLERS_H */
