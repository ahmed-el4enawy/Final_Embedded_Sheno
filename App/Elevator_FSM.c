/**
 * Elevator_FSM.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Elevator Finite State Machine implementation.
 *  States: IDLE → MOVING_UP/DOWN → ARRIVING → DOORS_OPEN → DOOR_CLOSING → ...
 *  Emergency Stop can be entered from ANY state.
 */

#include "Elevator_FSM.h"
#include "Board_Config.h"
#include "Pwm.h"
#include "Critical.h"

/* ------------------------------------------------------------------ */
/*  Floor bitmask helpers                                             */
/* ------------------------------------------------------------------ */
#define FLOOR_BIT(f)   (1U << ((f) - 1))   /* floor 1-4 → bit 0-3 */

/* ------------------------------------------------------------------ */
/*  Forward declarations                                              */
/* ------------------------------------------------------------------ */
static void Elevator_SetMotorSpeed(uint8 dutyPercent);
static uint8 Elevator_FindNextTargetUp(const ElevatorContext *ctx);
static uint8 Elevator_FindNextTargetDown(const ElevatorContext *ctx);

/* ------------------------------------------------------------------ */
/*  Global elevator context (one per board)                           */
/* ------------------------------------------------------------------ */

void Elevator_Init(ElevatorContext *ctx) {
    ctx->state          = ELEV_IDLE;
    ctx->currentFloor   = 1;
    ctx->direction      = DIR_NONE;
    ctx->cabinRequests  = 0;
    ctx->assignedCalls  = 0;
    ctx->emergencyStop  = 0;
    ctx->doorOpen       = 0;
    ctx->doorTimerActive = 0;
    ctx->startDoorTimer  = 0;
    Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);
}

/* ------------------------------------------------------------------ */
/*  Public helpers                                                    */
/* ------------------------------------------------------------------ */

void Elevator_AddCabinRequest(ElevatorContext *ctx, uint8 floor) {
    if (floor >= 1 && floor <= NUM_FLOORS) {
        uint32 pm = Enter_Critical();
        ctx->cabinRequests |= FLOOR_BIT(floor);
        Exit_Critical(pm);
    }
}

void Elevator_AddAssignedCalls(ElevatorContext *ctx, uint8 mask) {
    uint32 pm = Enter_Critical();
    ctx->assignedCalls |= mask;
    Exit_Critical(pm);
}

void Elevator_FloorSensorTriggered(ElevatorContext *ctx, uint8 floor) {
    if (floor >= 1 && floor <= NUM_FLOORS) {
        ctx->currentFloor = floor;
    }
}

void Elevator_EmergencyStop(ElevatorContext *ctx) {
    ctx->emergencyStop = 1;
    ctx->state = ELEV_EMERGENCY_STOP;
    Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);
}

void Elevator_EmergencyClear(ElevatorContext *ctx) {
    ctx->emergencyStop = 0;
    ctx->state = ELEV_IDLE;
    ctx->direction = DIR_NONE;
}

void Elevator_DoorTimerExpired(ElevatorContext *ctx) {
    ctx->doorTimerActive = 0;
    if (ctx->state == ELEV_DOORS_OPEN) {
        ctx->state = ELEV_DOOR_CLOSING;
    }
}

boolean Elevator_ShouldStopAtFloor(const ElevatorContext *ctx, uint8 floor) {
    uint8 mask = FLOOR_BIT(floor);
    return ((ctx->cabinRequests & mask) || (ctx->assignedCalls & mask))
           ? TRUE : FALSE;
}

uint8 Elevator_GetPacketState(const ElevatorContext *ctx) {
    switch (ctx->state) {
        case ELEV_IDLE:            return PKT_STATE_IDLE;
        case ELEV_MOVING_UP:       return PKT_STATE_MOVING_UP;
        case ELEV_MOVING_DOWN:     return PKT_STATE_MOVING_DOWN;
        case ELEV_ARRIVING:        return (ctx->direction == DIR_UP)
                                          ? PKT_STATE_MOVING_UP
                                          : PKT_STATE_MOVING_DOWN;
        case ELEV_DOORS_OPEN:      return PKT_STATE_DOORS_OPEN;
        case ELEV_DOOR_CLOSING:    return PKT_STATE_DOOR_CLOSING;
        case ELEV_EMERGENCY_STOP:  return PKT_STATE_EMERGENCY;
        default:                   return PKT_STATE_IDLE;
    }
}

uint8 Elevator_GetPendingFloors(const ElevatorContext *ctx) {
    return ctx->cabinRequests | ctx->assignedCalls;
}

/* ------------------------------------------------------------------ */
/*  Motor speed via PWM                                               */
/* ------------------------------------------------------------------ */
static void Elevator_SetMotorSpeed(uint8 dutyPercent) {
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, dutyPercent);
}

/* ------------------------------------------------------------------ */
/*  Find next floor with a request above / below current              */
/* ------------------------------------------------------------------ */
static uint8 Elevator_FindNextTargetUp(const ElevatorContext *ctx) {
    uint8 pending = Elevator_GetPendingFloors(ctx);
    uint8 f;
    for (f = ctx->currentFloor + 1; f <= NUM_FLOORS; f++) {
        if (pending & FLOOR_BIT(f)) return f;
    }
    return 0; /* no target above */
}

static uint8 Elevator_FindNextTargetDown(const ElevatorContext *ctx) {
    uint8 pending = Elevator_GetPendingFloors(ctx);
    uint8 f;
    for (f = ctx->currentFloor; f >= 2; f--) {
        if (pending & FLOOR_BIT(f - 1)) return f - 1;
    }
    return 0; /* no target below */
}

/* ------------------------------------------------------------------ */
/*  Clear the request for the current floor                           */
/* ------------------------------------------------------------------ */
static void Elevator_ClearCurrentFloor(ElevatorContext *ctx) {
    uint32 pm = Enter_Critical();
    uint8 mask = ~FLOOR_BIT(ctx->currentFloor);
    ctx->cabinRequests &= mask;
    ctx->assignedCalls &= mask;
    Exit_Critical(pm);
}

/* ------------------------------------------------------------------ */
/*  FSM tick  — called from main loop                                 */
/* ------------------------------------------------------------------ */
void Elevator_Run(ElevatorContext *ctx) {

    /* Emergency has absolute priority */
    if (ctx->emergencyStop) {
        ctx->state = ELEV_EMERGENCY_STOP;
        Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);
        return;
    }

    switch (ctx->state) {

    /* ============================================================ */
    case ELEV_IDLE:
        Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);
        ctx->direction = DIR_NONE;

        /* Check if current floor is requested (already here) */
        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor)) {
            Elevator_ClearCurrentFloor(ctx);
            ctx->doorOpen = 1;
            ctx->state = ELEV_DOORS_OPEN;
            if (!ctx->doorTimerActive) {
                ctx->doorTimerActive = 1;
                if (ctx->startDoorTimer) ctx->startDoorTimer();
            }
            break;
        }

        /* Look for requests above */
        if (Elevator_FindNextTargetUp(ctx)) {
            ctx->direction = DIR_UP;
            ctx->state = ELEV_MOVING_UP;
            Elevator_SetMotorSpeed(MOTOR_DUTY_FULL);
            break;
        }
        /* Look for requests below */
        if (Elevator_FindNextTargetDown(ctx)) {
            ctx->direction = DIR_DOWN;
            ctx->state = ELEV_MOVING_DOWN;
            Elevator_SetMotorSpeed(MOTOR_DUTY_FULL);
            break;
        }
        break;

    /* ============================================================ */
    case ELEV_MOVING_UP:
        Elevator_SetMotorSpeed(MOTOR_DUTY_FULL);

        /* Floor sensor has updated currentFloor — check if we stop */
        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor)) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorSpeed(MOTOR_DUTY_SLOW);
        }
        /* If we've reached top floor, also stop */
        if (ctx->currentFloor >= NUM_FLOORS) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorSpeed(MOTOR_DUTY_SLOW);
        }
        break;

    /* ============================================================ */
    case ELEV_MOVING_DOWN:
        Elevator_SetMotorSpeed(MOTOR_DUTY_FULL);

        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor)) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorSpeed(MOTOR_DUTY_SLOW);
        }
        if (ctx->currentFloor <= 1) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorSpeed(MOTOR_DUTY_SLOW);
        }
        break;

    /* ============================================================ */
    case ELEV_ARRIVING:
        Elevator_SetMotorSpeed(MOTOR_DUTY_SLOW);
        /* Transition to doors open */
        Elevator_ClearCurrentFloor(ctx);
        ctx->doorOpen = 1;
        ctx->state = ELEV_DOORS_OPEN;
        if (!ctx->doorTimerActive) {
            ctx->doorTimerActive = 1;
            if (ctx->startDoorTimer) ctx->startDoorTimer();
        }
        break;

    /* ============================================================ */
    case ELEV_DOORS_OPEN:
        Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);
        /* Wait for door timer to expire (handled via Timer callback) */
        if (!ctx->doorTimerActive) {
            ctx->state = ELEV_DOOR_CLOSING;
        }
        break;

    /* ============================================================ */
    case ELEV_DOOR_CLOSING:
        ctx->doorOpen = 0;
        Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);

        /* Decide next action: continue in current direction or reverse */
        if (ctx->direction == DIR_UP && Elevator_FindNextTargetUp(ctx)) {
            ctx->state = ELEV_MOVING_UP;
        } else if (ctx->direction == DIR_DOWN && Elevator_FindNextTargetDown(ctx)) {
            ctx->state = ELEV_MOVING_DOWN;
        } else if (Elevator_FindNextTargetUp(ctx)) {
            ctx->direction = DIR_UP;
            ctx->state = ELEV_MOVING_UP;
        } else if (Elevator_FindNextTargetDown(ctx)) {
            ctx->direction = DIR_DOWN;
            ctx->state = ELEV_MOVING_DOWN;
        } else {
            ctx->state = ELEV_IDLE;
            ctx->direction = DIR_NONE;
        }
        break;

    /* ============================================================ */
    case ELEV_EMERGENCY_STOP:
        Elevator_SetMotorSpeed(MOTOR_DUTY_STOP);
        /* Stay here until emergency is cleared */
        break;

    default:
        ctx->state = ELEV_IDLE;
        break;
    }
}
