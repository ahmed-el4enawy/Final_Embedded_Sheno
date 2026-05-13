/**
 * Telemetry.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Non-blocking UART status reports with structured log tags.
 *
 *  [REDESIGN] Atomic state snapshots, structured tags, no duplicate logs.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "Std_Types.h"
#include "Elevator_FSM.h"

/**
 * @brief  Initialise telemetry (starts periodic timer).
 */
void Telemetry_Init(void);

/**
 * @brief  Called from main loop.  If the periodic flag is set,
 *         formats and transmits the current status.
 * @param  elevA       Local elevator context
 * @param  elevB       Remote elevator context (may be NULL on slave)
 * @param  commOk      TRUE if SPI link is healthy
 * @param  hallCalls   Pending hallway calls bitmask
 * @param  spiErrors   Cumulative checksum error count
 * @return TRUE if a telemetry report was actually transmitted.
 */
boolean Telemetry_Update(const ElevatorContext *elevA,
                      const ElevatorContext *elevB,
                      boolean commOk,
                      uint8 hallCalls,
                      uint32 spiErrors);

#endif /* TELEMETRY_H */
