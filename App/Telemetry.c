/**
 * Telemetry.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Non-blocking UART telemetry using a periodic timer flag.
 *
 *  [REDESIGN] Telemetry fixes:
 *  1. Atomic state snapshot under critical section — prevents torn reads
 *     that caused duplicate/inconsistent log lines.
 *  2. Structured log tags: [TRANS] for state transitions, [TEL] for periodic.
 *  3. Transition log fires exactly once per state change (using prevState
 *     which is now set AFTER the FSM switch, not before).
 *  4. Static buffer for DMA compatibility (DMA reads asynchronously).
 */

#include "Telemetry.h"
#include "Usart.h"
#include "Timer.h"
#include "Dma.h"
#include "Board_Config.h"
#include "Spi_Protocol.h"
#include "Critical.h"

/* ------------------------------------------------------------------ */
/*  Private state                                                     */
/* ------------------------------------------------------------------ */
static volatile uint8 telemetryReady = 0;

/* Static buffer so DMA can read it asynchronously after we return. */
static char telemetryLine[160];

/* Tracking last reported state for transition telemetry */
static ElevatorState lastStateA = ELEV_IDLE;
static ElevatorState lastStateB = ELEV_IDLE;

static void Telemetry_TimerCallback(void) {
    telemetryReady = 1;
    /* Re-arm periodic timer */
    Timer_DelayMsAsync(TELEMETRY_TIMER, TELEMETRY_PERIOD_MS, Telemetry_TimerCallback);
}

/* ------------------------------------------------------------------ */
/*  Helpers for building the status string                            */
/* ------------------------------------------------------------------ */
static const char *StateStr(uint8 s) {
    switch (s) {
        case ELEV_IDLE:           return "IDLE";
        case ELEV_DISPATCHING:    return "DISP";
        case ELEV_MOVING_UP:      return "MOV_UP";
        case ELEV_MOVING_DOWN:    return "MOV_DN";
        case ELEV_DECELERATING:   return "DECEL";
        case ELEV_DOORS_OPEN:     return "DOOR_O";
        case ELEV_DOOR_CLOSING:   return "DOOR_C";
        case ELEV_EMERGENCY_STOP: return "EMERG!";
        case ELEV_INDEPENDENT:    return "INDEP";
        default:                  return "???";
    }
}

static char digitChar(uint8 v) {
    return (char)('0' + (v % 10));
}

static void appendFloor(char *buf, uint32 *pos, uint8 floor) {
    buf[*pos] = digitChar(floor);
    (*pos)++;
}

static void appendUint(char *buf, uint32 *pos, uint32 val) {
    if (val == 0) {
        buf[(*pos)++] = '0';
        return;
    }
    char tmp[10];
    sint8 i = 0;
    while (val > 0) {
        tmp[i++] = digitChar(val % 10);
        val /= 10;
    }
    while (i > 0) {
        buf[(*pos)++] = tmp[--i];
    }
}

static void appendStr(char *buf, uint32 *pos, const char *s) {
    while (*s) {
        buf[*pos] = *s++;
        (*pos)++;
    }
}



/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void Telemetry_Init(void) {
    telemetryReady = 0;
    lastStateA = ELEV_IDLE;
    lastStateB = ELEV_IDLE;
    /* Start first period */
    Timer_DelayMsAsync(TELEMETRY_TIMER, TELEMETRY_PERIOD_MS,
                       Telemetry_TimerCallback);
}

boolean Telemetry_Update(const ElevatorContext *elevA,
                      const ElevatorContext *elevB,
                      boolean commOk,
                      uint8 hallCalls,
                      uint32 spiErrors) {
    if (!telemetryReady) return FALSE;
    telemetryReady = 0;

    /* [REDESIGN] Atomic snapshot of all state under critical section.
     * Prevents torn reads that caused duplicate/inconsistent logs. */
    ElevatorState snapStateA;
    uint8 snapFloorA;
    uint8 snapTargetA;
    ElevatorState snapStateB = ELEV_IDLE;
    uint8 snapFloorB = 0;

    {
        uint32 pm = Enter_Critical();
        snapStateA  = elevA->state;
        snapFloorA  = elevA->currentFloor;
        snapTargetA = elevA->targetFloor;
        if (elevB) {
            snapStateB  = elevB->state;
            snapFloorB  = elevB->currentFloor;
        }
        Exit_Critical(pm);
    }

    uint32 p = 0;

    /* [REDESIGN] Transition log — fires exactly once per state change.
     * Uses [TRANS] tag for structured log parsing. */
    if (snapStateA != lastStateA) {
        appendStr(telemetryLine, &p, "[TRANS] ");
#if IS_MASTER_BOARD
        appendStr(telemetryLine, &p, "A:");
#else
        appendStr(telemetryLine, &p, "B:");
#endif
        appendStr(telemetryLine, &p, StateStr((uint8)lastStateA));
        appendStr(telemetryLine, &p, "->");
        appendStr(telemetryLine, &p, StateStr((uint8)snapStateA));
        appendStr(telemetryLine, &p, " F");
        appendFloor(telemetryLine, &p, snapFloorA);
        appendStr(telemetryLine, &p, " T");
        appendFloor(telemetryLine, &p, snapTargetA);
        appendStr(telemetryLine, &p, "\r\n");
        lastStateA = snapStateA;
    }

    if (elevB && snapStateB != lastStateB) {
        appendStr(telemetryLine, &p, "[TRANS] B:");
        appendStr(telemetryLine, &p, StateStr((uint8)lastStateB));
        appendStr(telemetryLine, &p, "->");
        appendStr(telemetryLine, &p, StateStr((uint8)snapStateB));
        appendStr(telemetryLine, &p, " F");
        appendFloor(telemetryLine, &p, snapFloorB);
        appendStr(telemetryLine, &p, "\r\n");
        lastStateB = snapStateB;
    }

    /* [REDESIGN] Periodic status report — [TEL] tag */
#if IS_MASTER_BOARD
    appendStr(telemetryLine, &p, "[TEL] A:");
#else
    appendStr(telemetryLine, &p, "[TEL] B:");
#endif
    appendStr(telemetryLine, &p, StateStr((uint8)snapStateA));
    appendStr(telemetryLine, &p, " F");
    appendFloor(telemetryLine, &p, snapFloorA);
    appendStr(telemetryLine, &p, " T");
    appendFloor(telemetryLine, &p, snapTargetA);

    if (elevB) {
        appendStr(telemetryLine, &p, " | B:");
        appendStr(telemetryLine, &p, StateStr((uint8)snapStateB));
        appendStr(telemetryLine, &p, " F");
        appendFloor(telemetryLine, &p, snapFloorB);
    }

    appendStr(telemetryLine, &p, " | SPI:");
    appendStr(telemetryLine, &p, commOk ? "OK" : "FAULT");
    appendStr(telemetryLine, &p, "(E:");
    appendUint(telemetryLine, &p, spiErrors);
    appendStr(telemetryLine, &p, ")");

    appendStr(telemetryLine, &p, " H:0x");
    {
        uint8 hi = (hallCalls >> 4) & 0x0F;
        uint8 lo = hallCalls & 0x0F;
        telemetryLine[p++] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
        telemetryLine[p++] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);
    }

    appendStr(telemetryLine, &p, "\r\n");
    telemetryLine[p] = '\0';

    /* Transmit complete line atomically */
    Usart1_TransmitString(telemetryLine);
    return TRUE;
}
