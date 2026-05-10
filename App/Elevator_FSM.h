/**
 * Elevator_FSM.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Finite State Machine for a single elevator serving 4 floors.
 */

#ifndef ELEVATOR_FSM_H
#define ELEVATOR_FSM_H

#include "Std_Types.h"
#include "Spi_Protocol.h"

/* ============================================================ */
/*  FSM states                                                   */
/* ============================================================ */
typedef enum {
    ELEV_IDLE,
    ELEV_MOVING_UP,
    ELEV_MOVING_DOWN,
    ELEV_ARRIVING,          /* Decelerating at target floor */
    ELEV_DOORS_OPEN,
    ELEV_DOOR_CLOSING,
    ELEV_EMERGENCY_STOP
} ElevatorState;

/* ============================================================ */
/*  Elevator run-time context  (all shared fields are volatile) */
/* ============================================================ */
typedef void (*DoorTimerStartFunc)(void);

typedef struct {
    volatile ElevatorState  state;
    volatile uint8          currentFloor;    /* 1 – 4                           */
    volatile uint8          direction;       /* DIR_UP, DIR_DOWN, DIR_NONE      */
    volatile uint8          cabinRequests;   /* Bitmask: bit0=F1 .. bit3=F4     */
    volatile uint8          assignedCalls;   /* Hall calls assigned by master   */
    volatile uint8          emergencyStop;   /* 1 = emergency active            */
    volatile uint8          doorOpen;        /* 1 = doors are open              */
    volatile uint8          doorTimerActive; /* 1 = waiting for door close      */
    DoorTimerStartFunc      startDoorTimer;  /* function to start door timer    */
} ElevatorContext;

/* ============================================================ */
/*  API                                                          */
/* ============================================================ */

/**
 * @brief  Initialise the elevator context to IDLE at floor 1.
 */
void Elevator_Init(ElevatorContext *ctx);

/**
 * @brief  Run one iteration of the FSM.  Call from the main loop.
 */
void Elevator_Run(ElevatorContext *ctx);

/**
 * @brief  Add a cabin (internal) request for a floor.
 * @param  floor  1 – 4
 */
void Elevator_AddCabinRequest(ElevatorContext *ctx, uint8 floor);

/**
 * @brief  Add a hall (external) call assigned by the dispatcher.
 * @param  mask  Bitmask of assigned calls
 */
void Elevator_AddAssignedCalls(ElevatorContext *ctx, uint8 mask);

/**
 * @brief  Notify the FSM that a floor sensor triggered.
 * @param  floor  The floor the elevator has reached (1-4)
 */
void Elevator_FloorSensorTriggered(ElevatorContext *ctx, uint8 floor);

/**
 * @brief  Trigger emergency stop (highest priority).
 */
void Elevator_EmergencyStop(ElevatorContext *ctx);

/**
 * @brief  Clear emergency and return to IDLE.
 */
void Elevator_EmergencyClear(ElevatorContext *ctx);

/**
 * @brief  Callback for door-close timer expiry.
 */
void Elevator_DoorTimerExpired(ElevatorContext *ctx);

/**
 * @brief  Check whether the elevator should stop at a given floor.
 * @return TRUE if the floor is requested (cabin or assigned)
 */
boolean Elevator_ShouldStopAtFloor(const ElevatorContext *ctx, uint8 floor);

/**
 * @brief  Convert FSM state to a PKT_STATE_* value for SPI protocol.
 */
uint8 Elevator_GetPacketState(const ElevatorContext *ctx);

/**
 * @brief  Get combined pending request mask (cabin | assigned).
 */
uint8 Elevator_GetPendingFloors(const ElevatorContext *ctx);

#endif /* ELEVATOR_FSM_H */
