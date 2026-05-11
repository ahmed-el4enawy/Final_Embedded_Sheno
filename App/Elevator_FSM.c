/**
 * Elevator_FSM.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Elevator Finite State Machine implementation.
 *  States: IDLE → MOVING_UP/DOWN → ARRIVING → DOORS_OPEN → DOOR_CLOSING → ...
 *  Emergency Stop can be entered from ANY state.
 *
 *  [FIX #1] PWM Speed Ramping — non-blocking duty-cycle ramp using SysTick.
 *  [FIX #3] Independent mode — entered on SPI comm fault (slave).
 */

#include "Elevator_FSM.h"
#include "Board_Config.h"
#include "Pwm.h"
#include "Timer.h"
#include "Critical.h"

/* ------------------------------------------------------------------ */
/*  SysTick millisecond counter (defined in main.c, [FIX #1/#4])      */
/* ------------------------------------------------------------------ */
extern volatile uint32 sysTickMs;

/* ------------------------------------------------------------------ */
/*  Floor bitmask helpers                                             */
/* ------------------------------------------------------------------ */
#define FLOOR_BIT(f)   (1U << ((f) - 1))   /* floor 1-4 → bit 0-3 */

/* ------------------------------------------------------------------ */
/*  Forward declarations                                              */
/* ------------------------------------------------------------------ */
static void Elevator_SetMotorTarget(ElevatorContext *ctx, uint8 targetDuty);
static void Elevator_RampTick(ElevatorContext *ctx);
static uint8 Elevator_FindNextTargetUp(const ElevatorContext *ctx);
static uint8 Elevator_FindNextTargetDown(const ElevatorContext *ctx);

/* ------------------------------------------------------------------ */
/*  Global elevator context (one per board)                           */
/* ------------------------------------------------------------------ */

void Elevator_Init(ElevatorContext *ctx) {
    ctx->state          = ELEV_IDLE;
    ctx->prevState      = ELEV_IDLE;
    ctx->currentFloor   = 1;
    ctx->direction      = DIR_NONE;
    ctx->cabinRequests  = 0;
    ctx->assignedCalls  = 0;
    ctx->emergencyStop  = 0;
    ctx->doorOpen       = 0;
    ctx->doorTimerActive = 0;
    ctx->startDoorTimer  = 0;

    /* [FIX #1] Initialise PWM ramp context */
    ctx->ramp.currentDuty  = 0;
    ctx->ramp.targetDuty   = 0;
    ctx->ramp.lastStepTick = 0;
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, 0);
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
        uint32 pm = Enter_Critical();
        ctx->currentFloor = floor;
        Exit_Critical(pm);
    }
}

void Elevator_EmergencyStop(ElevatorContext *ctx) {
    /* [FIX — Race Condition: Emergency Routines]
     *
     * This function is called from EXTI ISR (emergency button press).
     * BuildLocalFrame() in the main loop reads ctx->state, ctx->emergencyStop,
     * and ctx->ramp asynchronously for SPI frame packing.
     *
     * Without a critical section, a torn read can occur:
     *   - Main loop reads ctx->state = ELEV_IDLE (stale)
     *   - ISR fires, sets ctx->state = ELEV_EMERGENCY_STOP
     *   - Main loop reads ctx->emergencyStop = 1 (fresh)
     *   - Frame sent with state=IDLE + emergency=1 → protocol violation
     *
     * The critical section ensures all fields are updated atomically. */
    uint32 pm = Enter_Critical();
    ctx->emergencyStop = 1;
    ctx->state = ELEV_EMERGENCY_STOP;
    /* [FIX #1] Instant stop — bypass ramp for safety */
    ctx->ramp.targetDuty  = MOTOR_DUTY_STOP;
    ctx->ramp.currentDuty = MOTOR_DUTY_STOP;
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_DUTY_STOP);
    Exit_Critical(pm);
}

void Elevator_EmergencyClear(ElevatorContext *ctx) {
    /* [FIX — Race Condition: Emergency Routines]
     * Same rationale as EmergencyStop — must atomically update state,
     * emergencyStop, and direction so BuildLocalFrame() cannot observe
     * a partially-cleared emergency (e.g., state=IDLE but emergencyStop=1). */
    uint32 pm = Enter_Critical();
    ctx->emergencyStop = 0;
    ctx->state = ELEV_IDLE;
    ctx->direction = DIR_NONE;
    Exit_Critical(pm);
}

void Elevator_DoorTimerExpired(ElevatorContext *ctx) {
    /* [FIX #5 — Split State Transition]
     *
     * CRITICAL: This function is called from a TIM5 ISR (DoorTimerCb)
     * which fires asynchronously.  The previous implementation also
     * mutated ctx->state here:
     *     if (ctx->state == ELEV_DOORS_OPEN)
     *         ctx->state = ELEV_DOOR_CLOSING;
     *
     * That created a dual-master state mutation.  The ISR could fire
     * between any two instructions in Elevator_Run(), including in the
     * middle of the ELEV_DOORS_OPEN case, corrupting the transition
     * logic.  The FSM's ELEV_DOORS_OPEN handler ALREADY performs:
     *     if (!ctx->doorTimerActive) ctx->state = ELEV_DOOR_CLOSING;
     *
     * Therefore the ISR must ONLY set the flag.  The synchronous FSM
     * is the sole owner of state transitions. */
    ctx->doorTimerActive = 0;
}

boolean Elevator_ShouldStopAtFloor(const ElevatorContext *ctx, uint8 floor) {
    uint8 mask = FLOOR_BIT(floor);
    /* [FIX #3] In INDEPENDENT mode, only respond to cabin requests */
    if (ctx->state == ELEV_INDEPENDENT) {
        return (ctx->cabinRequests & mask) ? TRUE : FALSE;
    }
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
        case ELEV_INDEPENDENT:     return PKT_STATE_EMERGENCY;  /* [FIX #3] report as emergency to master */
        default:                   return PKT_STATE_IDLE;
    }
}

uint8 Elevator_GetPendingFloors(const ElevatorContext *ctx) {
    /* [FIX #3] In independent mode, ignore assignedCalls */
    if (ctx->state == ELEV_INDEPENDENT) {
        return ctx->cabinRequests;
    }
    return ctx->cabinRequests | ctx->assignedCalls;
}

/* ------------------------------------------------------------------ */
/*  [FIX #3] Independent mode entry / exit                            */
/* ------------------------------------------------------------------ */

void Elevator_EnterIndependentMode(ElevatorContext *ctx) {
    uint32 pm = Enter_Critical();
    /* Discard all externally assigned calls */
    ctx->assignedCalls = 0;
    /* Stop the motor immediately for safety */
    ctx->ramp.targetDuty  = MOTOR_DUTY_STOP;
    ctx->ramp.currentDuty = MOTOR_DUTY_STOP;
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_DUTY_STOP);
    ctx->state = ELEV_INDEPENDENT;
    ctx->direction = DIR_NONE;
    ctx->doorOpen = 0;
    Exit_Critical(pm);
}

void Elevator_ExitIndependentMode(ElevatorContext *ctx) {
    uint32 pm = Enter_Critical();
    if (ctx->state == ELEV_INDEPENDENT) {
        ctx->state = ELEV_IDLE;
        ctx->direction = DIR_NONE;
    }
    Exit_Critical(pm);
}

/* ------------------------------------------------------------------ */
/*  [FIX #1] PWM speed ramping — non-blocking                        */
/*                                                                     */
/*  Instead of snapping the duty cycle, we set a TARGET and step       */
/*  toward it by RAMP_STEP_PERCENT every RAMP_STEP_INTERVAL_MS ms.     */
/*  The ramp tick is called from the main FSM loop (no extra timer).  */
/* ------------------------------------------------------------------ */

/**
 * @brief  Set the desired motor duty.  The actual PWM will ramp to it.
 */
static void Elevator_SetMotorTarget(ElevatorContext *ctx, uint8 targetDuty) {
    ctx->ramp.targetDuty = targetDuty;
}

/**
 * @brief  Non-blocking ramp tick.  Must be called every FSM iteration.
 *         Steps currentDuty toward targetDuty at RAMP_STEP_INTERVAL ms
 *         pace, RAMP_STEP_PERCENT per step.
 */
static void Elevator_RampTick(ElevatorContext *ctx) {
    if (ctx->ramp.currentDuty == ctx->ramp.targetDuty) {
        return;   /* already at target — nothing to do */
    }

    uint32 now = sysTickMs;
    uint32 elapsed = now - ctx->ramp.lastStepTick;
    if (elapsed < RAMP_STEP_INTERVAL_MS) {
        return;   /* not yet time for next step */
    }
    ctx->ramp.lastStepTick = now;

    if (ctx->ramp.currentDuty < ctx->ramp.targetDuty) {
        /* Ramping UP */
        uint8 remaining = ctx->ramp.targetDuty - ctx->ramp.currentDuty;
        if (remaining <= RAMP_STEP_PERCENT) {
            ctx->ramp.currentDuty = ctx->ramp.targetDuty;
        } else {
            ctx->ramp.currentDuty += RAMP_STEP_PERCENT;
        }
    } else {
        /* Ramping DOWN */
        uint8 remaining = ctx->ramp.currentDuty - ctx->ramp.targetDuty;
        if (remaining <= RAMP_STEP_PERCENT) {
            ctx->ramp.currentDuty = ctx->ramp.targetDuty;
        } else {
            ctx->ramp.currentDuty -= RAMP_STEP_PERCENT;
        }
    }

    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL,
                       ctx->ramp.currentDuty);
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
    /* Update prevState for transition telemetry before state logic runs */
    ctx->prevState = ctx->state;

    /* [FIX #1] Always run the non-blocking PWM ramp tick */
    Elevator_RampTick(ctx);

    /* Emergency has absolute priority */
    if (ctx->emergencyStop) {
        ctx->state = ELEV_EMERGENCY_STOP;
        /* Instant stop for safety — bypass ramp */
        ctx->ramp.targetDuty  = MOTOR_DUTY_STOP;
        ctx->ramp.currentDuty = MOTOR_DUTY_STOP;
        Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_DUTY_STOP);
        return;
    }

    switch (ctx->state) {

    /* ============================================================ */
    case ELEV_IDLE:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_STOP);       /* [FIX #1] ramp target */
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
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);   /* [FIX #1] ramp up */
            break;
        }
        /* Look for requests below */
        if (Elevator_FindNextTargetDown(ctx)) {
            ctx->direction = DIR_DOWN;
            ctx->state = ELEV_MOVING_DOWN;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);   /* [FIX #1] ramp up */
            break;
        }
        break;

    /* ============================================================ */
    case ELEV_MOVING_UP:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);       /* [FIX #1] maintain target */

        /* Floor sensor has updated currentFloor — check if we stop */
        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor)) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);   /* [FIX #1] ramp down to slow */
        }
        /* If we've reached top floor, also stop */
        if (ctx->currentFloor >= NUM_FLOORS) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);   /* [FIX #1] ramp down to slow */
        }
        break;

    /* ============================================================ */
    case ELEV_MOVING_DOWN:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);       /* [FIX #1] maintain target */

        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor)) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);   /* [FIX #1] ramp down to slow */
        }
        if (ctx->currentFloor <= 1) {
            ctx->state = ELEV_ARRIVING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);   /* [FIX #1] ramp down to slow */
        }
        break;

    /* ============================================================ */
    case ELEV_ARRIVING:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);       /* [FIX #1] keep slow target */
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
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_STOP);       /* [FIX #1] ramp to stop */
        /* Wait for door timer to expire (handled via Timer callback) */
        if (!ctx->doorTimerActive) {
            ctx->state = ELEV_DOOR_CLOSING;
        }
        break;

    /* ============================================================ */
    case ELEV_DOOR_CLOSING:
        ctx->doorOpen = 0;
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_STOP);       /* [FIX #1] */

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
        /* Instant stop — no ramp for safety */
        ctx->ramp.targetDuty  = MOTOR_DUTY_STOP;
        ctx->ramp.currentDuty = MOTOR_DUTY_STOP;
        Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_DUTY_STOP);
        /* Stay here until emergency is cleared */
        break;

    /* ============================================================ */
    /* [FIX #3] Independent mode — slave operates alone, cabin-only */
    case ELEV_INDEPENDENT:
        /* Motor is already stopped on entry.
         * In independent mode the elevator only services cabin requests.
         * If a cabin request exists, transition to IDLE to begin servicing.
         * External (assigned) calls are ignored. */
        if (ctx->cabinRequests != 0) {
            ctx->state = ELEV_IDLE;
        }
        break;

    default:
        ctx->state = ELEV_IDLE;
        break;
    }
}
