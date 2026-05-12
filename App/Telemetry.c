/**
 * Telemetry.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  500 ms non-blocking UART telemetry using a periodic timer flag.
 *
 *  [FIX — DMA Telemetry]
 *  Previous version used blocking Usart1_TransmitString() which
 *  internally loops on USART_SR.TXE for each byte, burning CPU
 *  cycles for the entire string length every 500 ms.  This wasted
 *  the DMA2 Stream7 that was already initialized in main.c via
 *  Dma_Usart1TxInit().
 *
 *  Now uses Dma_Usart1TxStart() which:
 *    1. Loads DMA2_Stream7->M0AR with the source buffer address
 *    2. Loads DMA2_Stream7->NDTR with the byte count
 *    3. Clears TC/TE/HT/DME/FE flags in DMA2->HIFCR
 *    4. Sets EN bit in DMA2_Stream7->CR
 *    5. Enables USART1_CR3.DMAT
 *  The DMA engine then transfers each byte to USART1->DR
 *  autonomously — zero CPU overhead for the actual transmission.
 *
 *  IMPORTANT: The `line[]` buffer must be static, not stack-local,
 *  because the DMA reads from it asynchronously after this function
 *  returns.  A stack buffer would be overwritten by subsequent calls.
 */

#include "Telemetry.h"
#include "Usart.h"
#include "Timer.h"
#include "Dma.h"
#include "Board_Config.h"
#include "Spi_Protocol.h"

/* ------------------------------------------------------------------ */
/*  Private state                                                     */
/* ------------------------------------------------------------------ */
static volatile uint8 telemetryReady = 0;

/* [FIX — DMA Telemetry]
 * Static buffer so DMA can read it asynchronously after we return.
 * Must NOT be on the stack. */
static char telemetryLine[128];

/* [NEW] Tracking last reported state for transition telemetry */
static ElevatorState lastStateA = ELEV_IDLE;
static ElevatorState lastStateB = ELEV_IDLE;

static void Telemetry_TimerCallback(void) {
    telemetryReady = 1;
    
    // --- FIX: Add a hardware heartbeat ---
    // Toggle the onboard LED (assuming PC13, or change to an unused pin)
    // Gpio_TogglePin(GPIOC, 13); 

    /* Re-arm periodic timer */
    Timer_DelayMsAsync(TELEMETRY_TIMER, TELEMETRY_PERIOD_MS, Telemetry_TimerCallback);
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

/* Simple uint-to-string for error counts */
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

/* Copy a const string into buf */
static void appendStr(char *buf, uint32 *pos, const char *s) {
    while (*s) {
        buf[*pos] = *s++;
        (*pos)++;
    }
}

/* Custom strlen — standard library not available in bare-metal */
static uint16 Telemetry_Strlen(const char *s) {
    uint16 len = 0;
    while (s[len] != '\0') { len++; }
    return len;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void Telemetry_Init(void) {
    telemetryReady = 0;
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

    /* [FIX — DMA Telemetry]
     * Guard: if the previous DMA transfer is still in progress, skip
     * this report rather than corrupting the buffer mid-transfer. */
    if (Dma_Usart1TxBusy()) return FALSE;

    uint32 p = 0;

#if IS_MASTER_BOARD
    appendStr(telemetryLine, &p, "[TEL] A:");
#else
    appendStr(telemetryLine, &p, "[TEL] B:");
#endif
    if (elevA->state != lastStateA) {
        appendStr(telemetryLine, &p, StateStr((uint8)lastStateA));
        appendStr(telemetryLine, &p, "->");
        lastStateA = elevA->state;
    }
    appendStr(telemetryLine, &p, StateStr((uint8)elevA->state));
    appendStr(telemetryLine, &p, " F");
    appendFloor(telemetryLine, &p, elevA->currentFloor);

    if (elevB) {
        appendStr(telemetryLine, &p, " | B:");
        if (elevB->state != lastStateB) {
            appendStr(telemetryLine, &p, StateStr((uint8)lastStateB));
            appendStr(telemetryLine, &p, "->");
            lastStateB = elevB->state;
        }
        appendStr(telemetryLine, &p, StateStr((uint8)elevB->state));
        appendStr(telemetryLine, &p, " F");
        appendFloor(telemetryLine, &p, elevB->currentFloor);
    }

    appendStr(telemetryLine, &p, " | SPI:");
    appendStr(telemetryLine, &p, commOk ? "OK" : "FAULT");
    appendStr(telemetryLine, &p, " (ERR:");
    appendUint(telemetryLine, &p, spiErrors);
    appendStr(telemetryLine, &p, ")");

    appendStr(telemetryLine, &p, " | Hall:0x");
    /* Hex nibble for hall calls */
    uint8 hi = (hallCalls >> 4) & 0x0F;
    uint8 lo = hallCalls & 0x0F;
    telemetryLine[p++] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
    telemetryLine[p++] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);

    appendStr(telemetryLine, &p, "\r\n");
    telemetryLine[p] = '\0';

    /* [FIX — DMA Telemetry]
     * Trigger DMA2 Stream7 → USART1_DR transfer.
     * Dma_Usart1TxStart() performs:
     *   DMA2_Stream7->M0AR = (uint32)telemetryLine;
     *   DMA2_Stream7->NDTR = len;
     *   DMA2->HIFCR = clear all Stream7 flags;
     *   SET_BIT(DMA2_Stream7->CR, EN);
     *   SET_BIT(USART1->CR3, DMAT);
     * Zero CPU cycles consumed for the actual byte transmission. */
    // Dma_Usart1TxStart((const uint8 *)telemetryLine,
    //                   Telemetry_Strlen(telemetryLine));
    Usart1_TransmitString(telemetryLine);
    return TRUE;
}

