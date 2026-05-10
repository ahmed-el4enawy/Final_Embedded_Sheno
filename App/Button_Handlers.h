/**
 * Button_Handlers.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  EXTI callback wiring for cabin buttons, hallway buttons,
 *  emergency stop, and floor sensors.
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
