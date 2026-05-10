/**
 * Dispatcher.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Master-side Task Allocation Algorithm.
 *
 *  Scoring rules (lower = better):
 *    0   Immediate   – elevator is AT the floor and IDLE
 *    1-9 Perfect     – elevator moving toward floor in SAME direction
 *   50+  Idle near   – elevator idle but not at the floor
 *  100+  Passed      – same direction but already passed the floor
 *  200+  Opposite    – moving away from requested direction
 *  -1    DO NOT      – don't assign (opposite still has pending stops)
 *
 *  Forbidden: round-robin, nearest-idle-only.
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

    /* ---- Emergency or comm-fault: unavailable ---- */
    if (elev->emergencyStop) return -1;

    /* ---- Immediate: already at floor AND idle ---- */
    if (elev->state == ELEV_IDLE && elev->currentFloor == targetFloor) {
        return 0;
    }

    /* ---- Perfect match: moving toward floor in SAME direction ---- */
    if (elev->state == ELEV_MOVING_UP && callDir == DIR_UP) {
        if (elev->currentFloor < targetFloor) {
            return (sint16)(1 + dist);          /* best moving score */
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

    /* ---- Opposite direction: moving away ---- */
    if ((elev->state == ELEV_MOVING_UP   && callDir == DIR_DOWN) ||
        (elev->state == ELEV_MOVING_DOWN && callDir == DIR_UP)) {
        return (sint16)(200 + dist);
    }

    /* ---- Moving in same direction, opposite call direction ---- */
    if (elev->state == ELEV_MOVING_UP || elev->state == ELEV_MOVING_DOWN) {
        return (sint16)(150 + dist);
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
            pendingHallCalls |= (1U << i);
            break;
        }
    }
    Exit_Critical(pm);
}

void Dispatcher_Run(ElevatorContext *elevA, ElevatorContext *elevB,
                    boolean commFault) {
    uint8 i;
    uint32 pm;

    /* ------ Comm fault: master (A) takes ALL calls ------ */
    if (commFault) {
        pm = Enter_Critical();
        assignedToA |= pendingHallCalls;
        pendingHallCalls = 0;
        elevA->assignedCalls = HallMaskToFloorMask(assignedToA);
        Exit_Critical(pm);
        return;
    }

    /* ------ Evaluate each pending call ------ */
    for (i = 0; i < 6; i++) {
        uint8 bit = (1U << i);
        if (!(pendingHallCalls & bit)) continue;       /* not pending  */
        if ((assignedToA & bit) || (assignedToB & bit)) {
            /* Already assigned, skip (will be cleared when serviced) */
            pm = Enter_Critical();
            pendingHallCalls &= ~bit;
            Exit_Critical(pm);
            continue;
        }

        uint8 tgtFloor = hallCallTable[i].floor;
        uint8 tgtDir   = hallCallTable[i].direction;

        sint16 scoreA = Dispatcher_Score(elevA, tgtFloor, tgtDir);
        sint16 scoreB = Dispatcher_Score(elevB, tgtFloor, tgtDir);

        /* If both negative, leave pending for retry later */
        if (scoreA < 0 && scoreB < 0) continue;

        pm = Enter_Critical();
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

    /* Push floor masks to elevator contexts */
    pm = Enter_Critical();
    elevA->assignedCalls = HallMaskToFloorMask(assignedToA);
    elevB->assignedCalls = HallMaskToFloorMask(assignedToB);
    Exit_Critical(pm);
}

uint8 Dispatcher_GetAssignedA(void) { return assignedToA; }
uint8 Dispatcher_GetAssignedB(void) { return assignedToB; }
uint8 Dispatcher_GetPendingHallCalls(void) { return pendingHallCalls; }
