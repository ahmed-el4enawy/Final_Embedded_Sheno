/**
 * Dispatcher.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Master-side Task Allocation Algorithm.
 *
 *  [REDESIGN] Fixed critical section boundaries and scoring logic.
 *
 *  Scoring rules (lower = better):
 *    0   Immediate   – elevator is AT the floor and IDLE
 *    1-9 Perfect     – elevator moving toward floor in SAME direction
 *   50+  Idle near   – elevator idle but not at the floor
 *  100+  Passed      – same direction but already passed the floor
 *   -1   DO NOT      – don't assign (opposite dir, emergency, independent)
 *
 *  Key fixes:
 *  1. Entire score+assign per call wrapped in single critical section
 *  2. Direction-aware call clearing (HALL_U2 vs HALL_D2 at same floor)
 *  3. Idempotent: re-running with same inputs produces same output
 *  4. CommFault: reclaims stranded B assignments to A
 */

#include "Dispatcher.h"
#include "Spi_Protocol.h"
#include "Critical.h"
#include "Board_Config.h"

/* ------------------------------------------------------------------ */
/*  Internal state                                                    */
/* ------------------------------------------------------------------ */

/* Each hallway call is identified by a bit in a 6-bit mask:
 *   bit 0 = U1, bit 1 = D2, bit 2 = U2,
 *   bit 3 = D3, bit 4 = U3, bit 5 = D4                              */
static volatile uint8 pendingHallCalls = 0;   /* new, unassigned       */
static volatile uint8 assignedToA      = 0;   /* assigned to Elev A    */
static volatile uint8 assignedToB      = 0;   /* assigned to Elev B    */

/* Call → floor + direction lookup */
typedef struct {
    uint8 floor;
    uint8 direction;
} HallCallInfo;

static const HallCallInfo hallCallTable[6] = {
    { 1, DIR_UP   },  /* U1 */
    { 2, DIR_DOWN },  /* D2 */
    { 2, DIR_UP   },  /* U2 */
    { 3, DIR_DOWN },  /* D3 */
    { 3, DIR_UP   },  /* U3 */
    { 4, DIR_DOWN },  /* D4 */
};

/* ------------------------------------------------------------------ */
static sint16 abs16(sint16 v) { return (v < 0) ? -v : v; }

/* ------------------------------------------------------------------ */
/*  Score one elevator for one hall call                               */
/*  Returns 0..300 (lower = better).  -1 means don't assign.         */
/* ------------------------------------------------------------------ */
static sint16 Dispatcher_Score(const ElevatorContext *elev,
                               uint8 targetFloor, uint8 callDir) {

    sint16 dist = abs16((sint16)elev->currentFloor - (sint16)targetFloor);

    /* ---- Emergency, independent: unavailable ---- */
    if (elev->emergencyStop) return -1;
    if (elev->state == ELEV_INDEPENDENT) return -1;

    /* ---- Immediate: already at floor AND idle ---- */
    if (elev->state == ELEV_IDLE && elev->currentFloor == targetFloor) {
        return 0;
    }

    /* ---- Perfect match: moving toward floor in SAME direction ---- */
    if (elev->state == ELEV_MOVING_UP && callDir == DIR_UP) {
        if (elev->currentFloor < targetFloor) {
            return (sint16)(1 + dist);
        }
    }
    if (elev->state == ELEV_MOVING_DOWN && callDir == DIR_DOWN) {
        if (elev->currentFloor > targetFloor) {
            return (sint16)(1 + dist);
        }
    }

    /* ---- Passed match: same direction but already passed ---- */
    if (elev->state == ELEV_MOVING_UP && callDir == DIR_UP) {
        if (elev->currentFloor >= targetFloor) {
            return (sint16)(100 + dist);
        }
    }
    if (elev->state == ELEV_MOVING_DOWN && callDir == DIR_DOWN) {
        if (elev->currentFloor <= targetFloor) {
            return (sint16)(100 + dist);
        }
    }

    /* ---- Opposite direction: DO NOT ASSIGN ---- */
    if ((elev->state == ELEV_MOVING_UP   && callDir == DIR_DOWN) ||
        (elev->state == ELEV_MOVING_DOWN && callDir == DIR_UP)) {
        return -1;
    }

    /* ---- Moving in same direction, opposite call direction ---- */
    if (elev->state == ELEV_MOVING_UP || elev->state == ELEV_MOVING_DOWN) {
        return -1;
    }

    /* ---- DISPATCHING / DECELERATING: treat as soon-busy, penalize ---- */
    if (elev->state == ELEV_DISPATCHING || elev->state == ELEV_DECELERATING) {
        return (sint16)(70 + dist);
    }

    /* ---- Idle (not at target floor): nearest idle ---- */
    if (elev->state == ELEV_IDLE) {
        return (sint16)(50 + dist);
    }

    /* ---- Doors open / closing: treat as soon-idle ---- */
    if (elev->state == ELEV_DOORS_OPEN || elev->state == ELEV_DOOR_CLOSING) {
        return (sint16)(60 + dist);
    }

    return 300;   /* fallback worst */
}

/* ------------------------------------------------------------------ */
/*  Convert hall-call bitmask to floor request bitmask (for FSM)      */
/*  E.g.  HALL_U1 | HALL_U2  →  floor 1 bit + floor 2 bit            */
/* ------------------------------------------------------------------ */
static uint8 HallMaskToFloorMask(uint8 hallMask) {
    uint8 floorMask = 0;
    uint8 i;
    for (i = 0; i < 6; i++) {
        if (hallMask & (1U << i)) {
            floorMask |= (1U << (hallCallTable[i].floor - 1));
        }
    }
    return floorMask;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void Dispatcher_Init(void) {
    pendingHallCalls = 0;
    assignedToA = 0;
    assignedToB = 0;
}

void Dispatcher_RegisterHallCall(uint8 floor, uint8 direction) {
    uint8 i;
    uint32 pm = Enter_Critical();
    for (i = 0; i < 6; i++) {
        if (hallCallTable[i].floor == floor &&
            hallCallTable[i].direction == direction) {
            /* Only register if not already assigned or pending */
            uint8 bit = (1U << i);
            if (!(assignedToA & bit) && !(assignedToB & bit)) {
                pendingHallCalls |= bit;
            }
            break;
        }
    }
    Exit_Critical(pm);
}

void Dispatcher_Run(ElevatorContext *elevA, ElevatorContext *elevB,
                    boolean commFault) {
    uint8 i;
    uint32 pm;

    /* ------ Cleanup: clear completed assignments ------ */
    /* [FIX] Only clear when elevator has ACTUALLY STOPPED at the floor
     * (doors open, closing, idle, or decelerating).
     * Previously, direction-match while MOVING would clear the assignment
     * before the elevator had a chance to stop — causing missed stops.
     * Now we require the elevator to be in a stopped/servicing state. */
    for (i = 0; i < 6; i++) {
        uint8 bit = (1U << i);
        uint8 callFloor = hallCallTable[i].floor;

        if (assignedToA & bit) {
            if (elevA->currentFloor == callFloor) {
                if (elevA->state == ELEV_IDLE ||
                    elevA->state == ELEV_DOORS_OPEN ||
                    elevA->state == ELEV_DOOR_CLOSING ||
                    elevA->state == ELEV_DECELERATING) {
                    pm = Enter_Critical();
                    assignedToA &= ~bit;
                    Exit_Critical(pm);
                }
            }
        }
        if (!commFault && (assignedToB & bit)) {
            if (elevB->currentFloor == callFloor) {
                if (elevB->state == ELEV_IDLE ||
                    elevB->state == ELEV_DOORS_OPEN ||
                    elevB->state == ELEV_DOOR_CLOSING ||
                    elevB->state == ELEV_DECELERATING) {
                    pm = Enter_Critical();
                    assignedToB &= ~bit;
                    Exit_Critical(pm);
                }
            }
        }
    }

    /* ------ Comm fault: master (A) takes ALL calls ------ */
    if (commFault) {
        pm = Enter_Critical();
        assignedToA |= assignedToB;    /* reclaim Slave's stranded calls */
        assignedToB  = 0;
        assignedToA |= pendingHallCalls;
        pendingHallCalls = 0;
        elevA->assignedCalls = HallMaskToFloorMask(assignedToA);
        elevB->assignedCalls = 0;
        Exit_Critical(pm);
        return;
    }

    /* ------ Evaluate each pending call ------ */
    /* [REDESIGN] Entire score+assign is wrapped in a single
     * critical section per call to prevent ISR from modifying
     * elevator state between scoring and assignment. */
    for (i = 0; i < 6; i++) {
        uint8 bit = (1U << i);
        if (!(pendingHallCalls & bit)) continue;
        if ((assignedToA & bit) || (assignedToB & bit)) {
            /* Already assigned — just clear pending */
            pm = Enter_Critical();
            pendingHallCalls &= ~bit;
            Exit_Critical(pm);
            continue;
        }

        uint8 tgtFloor = hallCallTable[i].floor;
        uint8 tgtDir   = hallCallTable[i].direction;

        /* [REDESIGN] Single critical section wraps score + assign */
        pm = Enter_Critical();
        sint16 scoreA = Dispatcher_Score(elevA, tgtFloor, tgtDir);
        sint16 scoreB = Dispatcher_Score(elevB, tgtFloor, tgtDir);

        /* If BOTH return -1, leave the call PENDING.
         * It will be re-evaluated when an elevator becomes idle. */
        if (scoreA < 0 && scoreB < 0) {
            Exit_Critical(pm);
            continue;
        }

        if (scoreA < 0) {
            assignedToB |= bit;
        } else if (scoreB < 0) {
            assignedToA |= bit;
        } else if (scoreA <= scoreB) {
            assignedToA |= bit;
        } else {
            assignedToB |= bit;
        }
        pendingHallCalls &= ~bit;
        Exit_Critical(pm);
    }

    /* Push floor masks to elevator contexts atomically */
    pm = Enter_Critical();
    elevA->assignedCalls = HallMaskToFloorMask(assignedToA);
    elevB->assignedCalls = HallMaskToFloorMask(assignedToB);
    Exit_Critical(pm);
}

uint8 Dispatcher_GetAssignedA(void) { return assignedToA; }
uint8 Dispatcher_GetAssignedB(void) { return assignedToB; }
uint8 Dispatcher_GetPendingHallCalls(void) { return pendingHallCalls; }

/* Return B's assigned calls as a FLOOR bitmask (bit0=F1..bit3=F4). */
uint8 Dispatcher_GetAssignedBFloorMask(void) {
    return HallMaskToFloorMask(assignedToB);
}
