/**
 * Dispatcher.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Master-only Task Allocation Algorithm.
 *  Implements directional-optimization scoring to decide which
 *  elevator (A or B) should handle each hallway call.
 *
 *  [REDESIGN] Atomic score+assign, idempotent re-runs, no duplicate execution.
 */

#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "Std_Types.h"
#include "Elevator_FSM.h"

/* Assignment result */
#define ASSIGN_ELEV_A    0U
#define ASSIGN_ELEV_B    1U
#define ASSIGN_NONE      0xFFU

/**
 * @brief  Initialise the dispatcher (clears pending hall calls).
 */
void Dispatcher_Init(void);

/**
 * @brief  Register a new hallway call.
 * @param  floor      The floor (1-4)
 * @param  direction  DIR_UP or DIR_DOWN
 */
void Dispatcher_RegisterHallCall(uint8 floor, uint8 direction);

/**
 * @brief  Run the dispatcher algorithm.
 *         Evaluates all pending (unassigned) hall calls and assigns
 *         each to Elevator A or B based on the scoring rules.
 * @param  elevA  Pointer to Elevator A context (local on master)
 * @param  elevB  Pointer to Elevator B context (remote, from SPI)
 * @param  commFault  TRUE if SPI link is down
 */
void Dispatcher_Run(ElevatorContext *elevA, ElevatorContext *elevB,
                    boolean commFault);

/**
 * @brief  Get the bitmask of hall calls assigned to Elevator A.
 */
uint8 Dispatcher_GetAssignedA(void);

/**
 * @brief  Get the bitmask of hall calls assigned to Elevator B.
 */
uint8 Dispatcher_GetAssignedB(void);

/**
 * @brief  Get pending (unassigned) hall calls bitmask.
 */
uint8 Dispatcher_GetPendingHallCalls(void);

/**
 * @brief  Get B's assigned calls as a FLOOR bitmask (bit0=F1..bit3=F4).
 *         Used by BuildLocalFrame to send floor-level assignments via SPI.
 */
uint8 Dispatcher_GetAssignedBFloorMask(void);

#endif /* DISPATCHER_H */
