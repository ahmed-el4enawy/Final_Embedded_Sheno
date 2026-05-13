/**
 * Elevator_FSM.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Elevator Finite State Machine implementation.
 *
 *  [REDESIGN] Complete rewrite for deterministic, race-condition-free operation.
 *
 *  Key changes:
 *  1. Added DISPATCHING state — picks targetFloor and commits direction.
 *     targetFloor is locked until arrival. No mid-travel direction changes.
 *
 *  2. Added DECELERATING state — waits for motor ramp to reach SLOW
 *     before opening doors. Prevents fall-through from ARRIVING→DOORS_OPEN
 *     in a single FSM tick.
 *
 *  3. Event-based floor sensor — ISR sets floorSensorEvent flag,
 *     FSM consumes it. Adjacency-validated: only accepts ±1 floor transitions.
 *
 *  4. Exactly ONE state transition per Elevator_Run() call — no fall-through.
 *     prevState is recorded AFTER the switch, not before.
 *
 *  5. Emergency stop can be entered from ANY state (highest priority).
 *     ISR only sets emergencyStop flag; FSM forces the transition.
 */

#include "Elevator_FSM.h"
#include "Board_Config.h"
#include "Pwm.h"
#include "Timer.h"
#include "Critical.h"

/* ------------------------------------------------------------------ */
/*  SysTick millisecond counter (defined in main.c)                    */
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
static uint8 Elevator_PickNextTarget(const ElevatorContext *ctx);
static uint8 Elevator_FindNextTargetUp(const ElevatorContext *ctx);
static uint8 Elevator_FindNextTargetDown(const ElevatorContext *ctx);
static void Elevator_ClearCurrentFloor(ElevatorContext *ctx);
static void Elevator_ConsumeFloorSensor(ElevatorContext *ctx);
static void Elevator_ReevaluateTarget(ElevatorContext *ctx);

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */
void Elevator_Init(ElevatorContext *ctx) {
    ctx->state          = ELEV_IDLE;
    ctx->prevState      = ELEV_IDLE;
    ctx->currentFloor   = 1;
    ctx->targetFloor    = 0;
    ctx->direction      = DIR_NONE;
    ctx->cabinRequests  = 0;
    ctx->assignedCalls  = 0;
    ctx->emergencyStop  = 0;
    ctx->doorOpen       = 0;
    ctx->doorTimerActive = 0;
    ctx->floorSensorEvent = 0;
    ctx->sensorFloor    = 0;
    ctx->startDoorTimer  = 0;

    /* Initialise PWM ramp context */
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

/**
 * @brief  [REDESIGN] Event-based floor sensor notification.
 *         ISR-safe: only sets flag + floor value.
 *         Adjacency validation is performed when FSM consumes the event.
 */
void Elevator_FloorSensorTriggered(ElevatorContext *ctx, uint8 floor) {
    if (floor >= 1 && floor <= NUM_FLOORS) {
        ctx->sensorFloor = floor;
        ctx->floorSensorEvent = 1;  /* atomic uint8 write */
    }
}

void Elevator_EmergencyStop(ElevatorContext *ctx) {
    /* [Race-condition safe] Critical section ensures all fields
     * are updated atomically — BuildLocalFrame() cannot observe
     * a partially-updated context. */
    uint32 pm = Enter_Critical();
    ctx->emergencyStop = 1;
    ctx->state = ELEV_EMERGENCY_STOP;
    /* Instant stop — bypass ramp for safety */
    ctx->ramp.targetDuty  = MOTOR_DUTY_STOP;
    ctx->ramp.currentDuty = MOTOR_DUTY_STOP;
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_DUTY_STOP);
    Exit_Critical(pm);
}

void Elevator_EmergencyClear(ElevatorContext *ctx) {
    uint32 pm = Enter_Critical();
    ctx->emergencyStop = 0;
    ctx->state = ELEV_IDLE;
    ctx->direction = DIR_NONE;
    ctx->targetFloor = 0;
    Exit_Critical(pm);
}

/**
 * @brief  Door timer ISR callback.
 *         ONLY clears the flag — FSM is the sole owner of state transitions.
 */
void Elevator_DoorTimerExpired(ElevatorContext *ctx) {
    ctx->doorTimerActive = 0;
}

boolean Elevator_ShouldStopAtFloor(const ElevatorContext *ctx, uint8 floor) {
    uint8 mask = FLOOR_BIT(floor);
    if (ctx->state == ELEV_INDEPENDENT) {
        return (ctx->cabinRequests & mask) ? TRUE : FALSE;
    }
    return ((ctx->cabinRequests & mask) || (ctx->assignedCalls & mask))
           ? TRUE : FALSE;
}

uint8 Elevator_GetPacketState(const ElevatorContext *ctx) {
    switch (ctx->state) {
        case ELEV_IDLE:            return PKT_STATE_IDLE;
        case ELEV_DISPATCHING:     return PKT_STATE_IDLE;  /* externally looks idle */
        case ELEV_MOVING_UP:       return PKT_STATE_MOVING_UP;
        case ELEV_MOVING_DOWN:     return PKT_STATE_MOVING_DOWN;
        case ELEV_DECELERATING:    return (ctx->direction == DIR_UP)
                                          ? PKT_STATE_MOVING_UP
                                          : PKT_STATE_MOVING_DOWN;
        case ELEV_DOORS_OPEN:      return PKT_STATE_DOORS_OPEN;
        case ELEV_DOOR_CLOSING:    return PKT_STATE_DOOR_CLOSING;
        case ELEV_EMERGENCY_STOP:  return PKT_STATE_EMERGENCY;
        case ELEV_INDEPENDENT:     return PKT_STATE_EMERGENCY;
        default:                   return PKT_STATE_IDLE;
    }
}

uint8 Elevator_GetPendingFloors(const ElevatorContext *ctx) {
    if (ctx->state == ELEV_INDEPENDENT) {
        return ctx->cabinRequests;
    }
    return ctx->cabinRequests | ctx->assignedCalls;
}

/* ------------------------------------------------------------------ */
/*  Independent mode entry / exit                                      */
/* ------------------------------------------------------------------ */

void Elevator_EnterIndependentMode(ElevatorContext *ctx) {
    uint32 pm = Enter_Critical();
    ctx->assignedCalls = 0;
    ctx->targetFloor = 0;
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
        ctx->targetFloor = 0;
    }
    Exit_Critical(pm);
}

/* ------------------------------------------------------------------ */
/*  PWM speed ramping — non-blocking                                  */
/* ------------------------------------------------------------------ */

static void Elevator_SetMotorTarget(ElevatorContext *ctx, uint8 targetDuty) {
    ctx->ramp.targetDuty = targetDuty;
}

static void Elevator_RampTick(ElevatorContext *ctx) {
    if (ctx->ramp.currentDuty == ctx->ramp.targetDuty) {
        return;
    }

    uint32 now = sysTickMs;
    uint32 elapsed = now - ctx->ramp.lastStepTick;
    if (elapsed < RAMP_STEP_INTERVAL_MS) {
        return;
    }
    ctx->ramp.lastStepTick = now;

    if (ctx->ramp.currentDuty < ctx->ramp.targetDuty) {
        uint8 remaining = ctx->ramp.targetDuty - ctx->ramp.currentDuty;
        if (remaining <= RAMP_STEP_PERCENT) {
            ctx->ramp.currentDuty = ctx->ramp.targetDuty;
        } else {
            ctx->ramp.currentDuty += RAMP_STEP_PERCENT;
        }
    } else {
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
/*  [REDESIGN] SCAN-based target selection                            */
/*                                                                     */
/*  Phase 1: Continue in current direction (serve nearest ahead).     */
/*  Phase 2: Reverse direction (serve nearest behind).                */
/*  This is the classic elevator/SCAN algorithm.                      */
/* ------------------------------------------------------------------ */

static uint8 Elevator_FindNextTargetUp(const ElevatorContext *ctx) {
    uint8 pending = Elevator_GetPendingFloors(ctx);
    uint8 f;
    for (f = ctx->currentFloor + 1; f <= NUM_FLOORS; f++) {
        if (pending & FLOOR_BIT(f)) return f;
    }
    return 0;
}

static uint8 Elevator_FindNextTargetDown(const ElevatorContext *ctx) {
    uint8 pending = Elevator_GetPendingFloors(ctx);
    uint8 f;
    for (f = ctx->currentFloor; f >= 2; f--) {
        if (pending & FLOOR_BIT(f - 1)) return f - 1;
    }
    return 0;
}

/**
 * @brief  [REDESIGN] Pick the next target floor using SCAN algorithm.
 *         Continues in current direction first, then reverses.
 *         Returns 0 if no pending requests.
 */
static uint8 Elevator_PickNextTarget(const ElevatorContext *ctx) {
    uint8 pending = Elevator_GetPendingFloors(ctx);
    if (pending == 0) return 0;

    uint8 target;

    /* Phase 1: Continue in current direction */
    if (ctx->direction == DIR_UP || ctx->direction == DIR_NONE) {
        target = Elevator_FindNextTargetUp(ctx);
        if (target != 0) return target;
    }
    if (ctx->direction == DIR_DOWN || ctx->direction == DIR_NONE) {
        target = Elevator_FindNextTargetDown(ctx);
        if (target != 0) return target;
    }

    /* Phase 2: Reverse direction */
    if (ctx->direction == DIR_UP) {
        target = Elevator_FindNextTargetDown(ctx);
        if (target != 0) return target;
    }
    if (ctx->direction == DIR_DOWN) {
        target = Elevator_FindNextTargetUp(ctx);
        if (target != 0) return target;
    }

    /* Fallback: scan all floors (should not reach here) */
    {
        uint8 f;
        for (f = 1; f <= NUM_FLOORS; f++) {
            if (pending & FLOOR_BIT(f)) return f;
        }
    }

    return 0;
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
/*  [FIX] Re-evaluate targetFloor while moving to pick up en-route    */
/*  stops.  If a new request (hall call or cabin call) exists between  */
/*  the current floor and the committed target, update targetFloor     */
/*  to stop at the closer floor first.  The original target remains   */
/*  in cabinRequests/assignedCalls and will be picked up after the    */
/*  intermediate stop via DISPATCHING.                                */
/* ------------------------------------------------------------------ */
static void Elevator_ReevaluateTarget(ElevatorContext *ctx) {
    uint8 pending = Elevator_GetPendingFloors(ctx);
    if (pending == 0) return;

    if (ctx->direction == DIR_UP) {
        /* Scan from one floor above current up to (but not past) target */
        uint8 f;
        for (f = ctx->currentFloor + 1; f < ctx->targetFloor; f++) {
            if (pending & FLOOR_BIT(f)) {
                ctx->targetFloor = f;  /* closer intermediate stop */
                return;
            }
        }
    } else if (ctx->direction == DIR_DOWN) {
        /* Scan from one floor below current down to (but not past) target */
        uint8 f;
        for (f = ctx->currentFloor - 1; f > ctx->targetFloor; f--) {
            if (pending & FLOOR_BIT(f)) {
                ctx->targetFloor = f;  /* closer intermediate stop */
                return;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  [REDESIGN] Consume floor sensor event with adjacency validation   */
/*                                                                     */
/*  Only accepts floor transitions that are ±1 from currentFloor      */
/*  AND consistent with the current travel direction.                  */
/*  Rejects noise-induced jumps (e.g. F1→F3).                         */
/* ------------------------------------------------------------------ */
static void Elevator_ConsumeFloorSensor(ElevatorContext *ctx) {
    if (!ctx->floorSensorEvent) return;

    uint8 newFloor = ctx->sensorFloor;
    uint32 pm = Enter_Critical();
    ctx->floorSensorEvent = 0;
    Exit_Critical(pm);

    /* Adjacency check: only accept ±1 or same floor */
    sint8 delta = (sint8)newFloor - (sint8)ctx->currentFloor;

    if (delta == 0) return;  /* already here */

    /* Reject jumps larger than 1 floor */
    if (delta != 1 && delta != -1) return;

    /* Direction consistency: going up should not report going down */
    if (ctx->direction == DIR_UP   && delta < 0) return;
    if (ctx->direction == DIR_DOWN && delta > 0) return;

    /* Valid transition — update currentFloor */
    ctx->currentFloor = newFloor;
}

/* ------------------------------------------------------------------ */
/*  [REDESIGN] FSM tick — called from main loop                       */
/*                                                                     */
/*  Guarantees:                                                        */
/*  - Exactly ONE state transition per call (no fall-through)         */
/*  - prevState recorded AFTER switch (fires telemetry once)          */
/*  - Emergency has absolute priority                                 */
/*  - targetFloor is committed in DISPATCHING, locked until arrival   */
/* ------------------------------------------------------------------ */
void Elevator_Run(ElevatorContext *ctx) {
    /* Record entry state for transition detection */
    ElevatorState entryState = ctx->state;

    /* Always run the non-blocking PWM ramp tick */
    Elevator_RampTick(ctx);

    /* Emergency has absolute priority — force from any state */
    if (ctx->emergencyStop) {
        ctx->state = ELEV_EMERGENCY_STOP;
        /* Instant stop for safety — bypass ramp */
        ctx->ramp.targetDuty  = MOTOR_DUTY_STOP;
        ctx->ramp.currentDuty = MOTOR_DUTY_STOP;
        Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_DUTY_STOP);
        goto record_transition;
    }

    switch (ctx->state) {

    /* ============================================================ */
    case ELEV_IDLE:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_STOP);
        ctx->direction = DIR_NONE;
        ctx->targetFloor = 0;

        /* Check if current floor is requested (already here) */
        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor)) {
            Elevator_ClearCurrentFloor(ctx);
            ctx->doorOpen = 1;
            ctx->state = ELEV_DOORS_OPEN;
            if (!ctx->doorTimerActive) {
                ctx->doorTimerActive = 1;
                if (ctx->startDoorTimer) ctx->startDoorTimer();
            }
            break;  /* ONE transition */
        }

        /* Any pending requests? → go to DISPATCHING */
        if (Elevator_GetPendingFloors(ctx) != 0) {
            ctx->state = ELEV_DISPATCHING;
        }
        break;

    /* ============================================================ */
    /*  [REDESIGN] DISPATCHING — pick target and commit direction    */
    /* ============================================================ */
    case ELEV_DISPATCHING: {
        uint8 target = Elevator_PickNextTarget(ctx);
        if (target == 0) {
            /* No requests left — back to idle */
            ctx->state = ELEV_IDLE;
            break;
        }

        /* Commit target — locked until arrival */
        ctx->targetFloor = target;

        if (target > ctx->currentFloor) {
            ctx->direction = DIR_UP;
            ctx->state = ELEV_MOVING_UP;
        } else if (target < ctx->currentFloor) {
            ctx->direction = DIR_DOWN;
            ctx->state = ELEV_MOVING_DOWN;
        } else {
            /* Already at target (race: request arrived between checks) */
            Elevator_ClearCurrentFloor(ctx);
            ctx->doorOpen = 1;
            ctx->state = ELEV_DOORS_OPEN;
            if (!ctx->doorTimerActive) {
                ctx->doorTimerActive = 1;
                if (ctx->startDoorTimer) ctx->startDoorTimer();
            }
        }
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);
        break;
    }

    /* ============================================================ */
    case ELEV_MOVING_UP:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);

        /* Consume floor sensor event (set by ISR, adjacency-validated) */
        Elevator_ConsumeFloorSensor(ctx);

        /* [FIX] Re-evaluate target: pick up newly-assigned en-route stops */
        Elevator_ReevaluateTarget(ctx);

        /* Check if we should stop at current floor (target or en-route) */
        if (ctx->currentFloor == ctx->targetFloor) {
            ctx->state = ELEV_DECELERATING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
            break;
        }

        /* Also check for en-route stops (cabin request at current floor) */
        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor) &&
            ctx->currentFloor != ctx->targetFloor) {
            /* Stop en-route but keep targetFloor for later */
            ctx->state = ELEV_DECELERATING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
            break;
        }

        /* Boundary guard: top floor */
        if (ctx->currentFloor >= NUM_FLOORS) {
            ctx->state = ELEV_DECELERATING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
        }
        break;

    /* ============================================================ */
    case ELEV_MOVING_DOWN:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_FULL);

        /* Consume floor sensor event */
        Elevator_ConsumeFloorSensor(ctx);

        /* [FIX] Re-evaluate target: pick up newly-assigned en-route stops */
        Elevator_ReevaluateTarget(ctx);

        if (ctx->currentFloor == ctx->targetFloor) {
            ctx->state = ELEV_DECELERATING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
            break;
        }

        if (Elevator_ShouldStopAtFloor(ctx, ctx->currentFloor) &&
            ctx->currentFloor != ctx->targetFloor) {
            ctx->state = ELEV_DECELERATING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
            break;
        }

        /* Boundary guard: bottom floor */
        if (ctx->currentFloor <= 1) {
            ctx->state = ELEV_DECELERATING;
            Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
        }
        break;

    /* ============================================================ */
    /*  [REDESIGN] DECELERATING — wait for ramp to reach SLOW       */
    /*  Prevents old bug: ARRIVING instantly fell through to        */
    /*  DOORS_OPEN in the same FSM tick.                            */
    /* ============================================================ */
    case ELEV_DECELERATING:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_SLOW);
        /* Wait until motor has actually slowed down */
        if (ctx->ramp.currentDuty <= MOTOR_DUTY_SLOW) {
            Elevator_ClearCurrentFloor(ctx);
            ctx->doorOpen = 1;
            ctx->state = ELEV_DOORS_OPEN;
            if (!ctx->doorTimerActive) {
                ctx->doorTimerActive = 1;
                if (ctx->startDoorTimer) ctx->startDoorTimer();
            }
        }
        break;

    /* ============================================================ */
    case ELEV_DOORS_OPEN:
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_STOP);
        /* Wait for door timer to expire (ISR clears doorTimerActive) */
        if (!ctx->doorTimerActive) {
            ctx->state = ELEV_DOOR_CLOSING;
        }
        break;

    /* ============================================================ */
    case ELEV_DOOR_CLOSING:
        ctx->doorOpen = 0;
        Elevator_SetMotorTarget(ctx, MOTOR_DUTY_STOP);

        /* [REDESIGN] Go through DISPATCHING to properly pick next target.
         * This ensures SCAN direction continuity and targetFloor commitment. */
        if (Elevator_GetPendingFloors(ctx) != 0) {
            ctx->state = ELEV_DISPATCHING;
        } else {
            ctx->state = ELEV_IDLE;
            ctx->direction = DIR_NONE;
            ctx->targetFloor = 0;
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
    /* Independent mode — slave operates alone, cabin-only */
    case ELEV_INDEPENDENT:
        /* Motor is already stopped on entry.
         * If a cabin request exists, transition to IDLE to begin servicing. */
        if (ctx->cabinRequests != 0) {
            ctx->state = ELEV_IDLE;
        }
        break;

    default:
        ctx->state = ELEV_IDLE;
        break;
    }

record_transition:
    /* [REDESIGN] Transition telemetry — record AFTER the switch.
     * prevState is set only when an actual transition occurred.
     * This guarantees each transition is logged exactly once. */
    if (ctx->state != entryState) {
        ctx->prevState = entryState;
    }
}
