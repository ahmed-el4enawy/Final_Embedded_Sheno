/**
 * Telemetry.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  500 ms non-blocking UART telemetry using a periodic timer flag.
 *  UART polling for debug output is explicitly allowed by the spec.
 */

#include "Telemetry.h"
#include "Usart.h"
#include "Timer.h"
#include "Board_Config.h"
#include "Spi_Protocol.h"

/* ------------------------------------------------------------------ */
/*  Private state                                                     */
/* ------------------------------------------------------------------ */
static volatile uint8 telemetryReady = 0;

/* Timer callback sets the flag (runs in ISR context) */
static void Telemetry_TimerCallback(void) {
    telemetryReady = 1;
    /* Re-arm periodic timer */
    Timer_DelayMsAsync(TELEMETRY_TIMER, TELEMETRY_PERIOD_MS,
                       Telemetry_TimerCallback);
}

/* ------------------------------------------------------------------ */
/*  Helpers for building the status string                            */
/* ------------------------------------------------------------------ */
static const char *StateStr(uint8 s) {
    switch (s) {
        case ELEV_IDLE:           return "IDLE";
        case ELEV_MOVING_UP:     return "MOV_UP";
        case ELEV_MOVING_DOWN:   return "MOV_DN";
        case ELEV_ARRIVING:      return "ARRIVE";
        case ELEV_DOORS_OPEN:    return "DOOR_O";
        case ELEV_DOOR_CLOSING:  return "DOOR_C";
        case ELEV_EMERGENCY_STOP:return "EMERG!";
        case ELEV_INDEPENDENT:   return "INDEP";   /* [FIX #3] */
        default:                 return "???";
    }
}

static char digitChar(uint8 v) {
    return (char)('0' + (v % 10));
}

/* Simple uint-to-string for small numbers (0-9 floor) */
static void appendFloor(char *buf, uint32 *pos, uint8 floor) {
    buf[*pos] = digitChar(floor);
    (*pos)++;
}

/* Copy a const string into buf */
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
    /* Start first 500 ms period */
    Timer_DelayMsAsync(TELEMETRY_TIMER, TELEMETRY_PERIOD_MS,
                       Telemetry_TimerCallback);
}

boolean Telemetry_Update(const ElevatorContext *elevA,
                      const ElevatorContext *elevB,
                      boolean commOk,
                      uint8 hallCalls) {
    if (!telemetryReady) return FALSE;
    telemetryReady = 0;

    char line[128];
    uint32 p = 0;

    appendStr(line, &p, "[TEL] A:");
    appendStr(line, &p, StateStr((uint8)elevA->state));
    appendStr(line, &p, " F");
    appendFloor(line, &p, elevA->currentFloor);

    if (elevB) {
        appendStr(line, &p, " | B:");
        appendStr(line, &p, StateStr((uint8)elevB->state));
        appendStr(line, &p, " F");
        appendFloor(line, &p, elevB->currentFloor);
    }

    appendStr(line, &p, " | SPI:");
    appendStr(line, &p, commOk ? "OK" : "FAULT");

    appendStr(line, &p, " | Hall:0x");
    /* Hex nibble for hall calls */
    uint8 hi = (hallCalls >> 4) & 0x0F;
    uint8 lo = hallCalls & 0x0F;
    line[p++] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
    line[p++] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);

    appendStr(line, &p, "\r\n");
    line[p] = '\0';

    /* UART polling transmit (allowed by spec for debug) */
    Usart1_TransmitString(line);
    return TRUE;
}
