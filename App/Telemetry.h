/**
 * Telemetry.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  500 ms non-blocking UART status reports.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "Std_Types.h"
#include "Elevator_FSM.h"

/**
 * @brief  Initialise telemetry (starts 500 ms periodic timer).
 */
void Telemetry_Init(void);

/**
 * @brief  Called from main loop.  If the 500 ms flag is set,
 *         formats and transmits the current status.
 * @param  elevA       Local elevator context
 * @param  elevB       Remote elevator context (may be NULL on slave)
 * @param  commOk      TRUE if SPI link is healthy
 * @param  hallCalls   Pending hallway calls bitmask
 * @return TRUE if a telemetry report was actually transmitted.
 */
boolean Telemetry_Update(const ElevatorContext *elevA,
                      const ElevatorContext *elevB,
                      boolean commOk,
                      uint8 hallCalls);

#endif /* TELEMETRY_H */
