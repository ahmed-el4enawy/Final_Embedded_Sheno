/**
 * Button_Handlers.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Sets up EXTI callbacks for all buttons and floor sensors.
 *  All handlers set volatile flags — no heavy work in ISR.
 *
 *  [FIX #5] Non-blocking software debounce (50 ms) using SysTick
 *           timestamp.  Prevents MCU stalls from high-frequency
 *           button spamming.
 */

#include "Button_Handlers.h"
#include "Board_Config.h"
#include "Gpio.h"
#include "Exti.h"
#include "Nvic.h"
#include "Spi_Protocol.h"

#if IS_MASTER_BOARD
#include "Dispatcher.h"
#endif

/* ------------------------------------------------------------------ */
/*  SysTick millisecond counter (defined in main.c, [FIX #4])         */
/* ------------------------------------------------------------------ */
extern volatile uint32 sysTickMs;

/* ------------------------------------------------------------------ */
/*  [FIX #5] Debounce configuration                                  */
/* ------------------------------------------------------------------ */
#define DEBOUNCE_MS   50U   /* ignore EXTI edges within 50 ms */

/* ------------------------------------------------------------------ */
/*  The elevator context pointer is stored so ISR callbacks can use it */
/* ------------------------------------------------------------------ */
static ElevatorContext *localCtx = 0;

/* ------------------------------------------------------------------ */
/*  [FIX #5] Per-button last-trigger timestamps for debounce          */
/*  Using a single array indexed by a logical button ID.              */
/*  IDs:  0-3 = cabin F1-F4,  4 = emergency,                         */
/*        5-10 = hall U1,D2,U2,D3,U3,D4  (master only)               */
/* ------------------------------------------------------------------ */
#define BTN_ID_CABIN_F1    0U
#define BTN_ID_CABIN_F2    1U
#define BTN_ID_CABIN_F3    2U
#define BTN_ID_CABIN_F4    3U
#define BTN_ID_EMERGENCY   4U
#define BTN_ID_HALL_U1     5U
#define BTN_ID_HALL_D2     6U
#define BTN_ID_HALL_U2     7U
#define BTN_ID_HALL_D3     8U
#define BTN_ID_HALL_U3     9U
#define BTN_ID_HALL_D4    10U
#define BTN_ID_COUNT      11U

static volatile uint32 lastPressTick[BTN_ID_COUNT] = {0};

/**
 * @brief  [FIX #5] Non-blocking debounce check.
 *         Returns TRUE if enough time has passed since the last accepted
 *         press for this button ID — i.e. the press is valid.
 *         Returns FALSE if the press should be discarded (bounce).
 */
static boolean Debounce_Check(uint8 btnId) {
    uint32 now = sysTickMs;
    uint32 elapsed = now - lastPressTick[btnId];
    if (elapsed < DEBOUNCE_MS) {
        return FALSE;   /* too soon — discard */
    }
    lastPressTick[btnId] = now;
    return TRUE;
}

/* ------------------------------------------------------------------ */
/*  Cabin button callbacks (both boards)                              */
/*  [FIX #5] Each callback checks debounce before acting.            */
/* ------------------------------------------------------------------ */
static void CabinBtn_F1(void) {
    if (Debounce_Check(BTN_ID_CABIN_F1))
        Elevator_AddCabinRequest(localCtx, 1);
}
static void CabinBtn_F2(void) {
    if (Debounce_Check(BTN_ID_CABIN_F2))
        Elevator_AddCabinRequest(localCtx, 2);
}
static void CabinBtn_F3(void) {
    if (Debounce_Check(BTN_ID_CABIN_F3))
        Elevator_AddCabinRequest(localCtx, 3);
}
static void CabinBtn_F4(void) {
    if (Debounce_Check(BTN_ID_CABIN_F4))
        Elevator_AddCabinRequest(localCtx, 4);
}

/* ------------------------------------------------------------------ */
/*  Emergency stop callback (both boards)                             */
/*  [FIX #5] Debounce applied — but emergency is still highest NVIC  */
/*  priority so the ISR is still entered immediately.                 */
/* ------------------------------------------------------------------ */
static void EmergencyBtn(void) {
    if (Debounce_Check(BTN_ID_EMERGENCY))
        Elevator_EmergencyStop(localCtx);
}

/* ------------------------------------------------------------------ */
/*  Floor sensor callbacks (both boards)                              */
/*  Note: Floor sensors are NOT debounced — they are hardware         */
/*  position sensors, not user-operated buttons.                      */
/* ------------------------------------------------------------------ */
static void FloorSensor_F1(void) { Elevator_FloorSensorTriggered(localCtx, 1); }
static void FloorSensor_F2(void) { Elevator_FloorSensorTriggered(localCtx, 2); }
static void FloorSensor_F3(void) { Elevator_FloorSensorTriggered(localCtx, 3); }
static void FloorSensor_F4(void) { Elevator_FloorSensorTriggered(localCtx, 4); }

/* ------------------------------------------------------------------ */
/*  Hallway button callbacks (Master only)                            */
/*  [FIX #5] Debounced.                                              */
/* ------------------------------------------------------------------ */
#if IS_MASTER_BOARD
static void HallBtn_U1(void) {
    if (Debounce_Check(BTN_ID_HALL_U1))
        Dispatcher_RegisterHallCall(1, DIR_UP);
}
static void HallBtn_D2(void) {
    if (Debounce_Check(BTN_ID_HALL_D2))
        Dispatcher_RegisterHallCall(2, DIR_DOWN);
}
static void HallBtn_U2(void) {
    if (Debounce_Check(BTN_ID_HALL_U2))
        Dispatcher_RegisterHallCall(2, DIR_UP);
}
static void HallBtn_D3(void) {
    if (Debounce_Check(BTN_ID_HALL_D3))
        Dispatcher_RegisterHallCall(3, DIR_DOWN);
}
static void HallBtn_U3(void) {
    if (Debounce_Check(BTN_ID_HALL_U3))
        Dispatcher_RegisterHallCall(3, DIR_UP);
}
static void HallBtn_D4(void) {
    if (Debounce_Check(BTN_ID_HALL_D4))
        Dispatcher_RegisterHallCall(4, DIR_DOWN);
}
#endif

/* ------------------------------------------------------------------ */
/*  NVIC priority helper (direct register write)                      */
/* ------------------------------------------------------------------ */
#define NVIC_IPR_BASE   0xE000E400UL
static void SetIrqPriority(uint8 irqNum, uint8 priority) {
    /* IPR registers are byte-accessible.  Priority is in bits [7:4]. */
    volatile uint8 *ipr = (volatile uint8 *)(NVIC_IPR_BASE + irqNum);
    *ipr = (priority << 4);
}

/* ------------------------------------------------------------------ */
/*  Init all buttons + sensors                                        */
/* ------------------------------------------------------------------ */
void Buttons_Init(ElevatorContext *ctx) {
    localCtx = ctx;

    /* [FIX #5] Zero all debounce timestamps */
    {
        uint8 i;
        for (i = 0; i < BTN_ID_COUNT; i++) {
            lastPressTick[i] = 0;
        }
    }

    /* ---- Cabin floor buttons (PA0-PA3, pull-up, falling edge) ---- */
    Gpio_Init(CABIN_BTN_PORT, CABIN_BTN_PIN_F1, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(CABIN_BTN_PORT, CABIN_BTN_PIN_F2, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(CABIN_BTN_PORT, CABIN_BTN_PIN_F3, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(CABIN_BTN_PORT, CABIN_BTN_PIN_F4, GPIO_INPUT, GPIO_PULL_UP);

    Exti_Init(CABIN_BTN_PIN_F1, CABIN_BTN_EXTI_PORT, EXTI_EDGE_FALLING, CabinBtn_F1);
    Exti_Init(CABIN_BTN_PIN_F2, CABIN_BTN_EXTI_PORT, EXTI_EDGE_FALLING, CabinBtn_F2);
    Exti_Init(CABIN_BTN_PIN_F3, CABIN_BTN_EXTI_PORT, EXTI_EDGE_FALLING, CabinBtn_F3);
    Exti_Init(CABIN_BTN_PIN_F4, CABIN_BTN_EXTI_PORT, EXTI_EDGE_FALLING, CabinBtn_F4);

    Exti_Enable(CABIN_BTN_PIN_F1);
    Exti_Enable(CABIN_BTN_PIN_F2);
    Exti_Enable(CABIN_BTN_PIN_F3);
    Exti_Enable(CABIN_BTN_PIN_F4);

    /* ---- Emergency stop (PB10, pull-up, falling edge) ---- */
    Gpio_Init(EMERG_BTN_PORT, EMERG_BTN_PIN, GPIO_INPUT, GPIO_PULL_UP);
    Exti_Init(EMERG_BTN_PIN, EMERG_BTN_EXTI_PORT, EXTI_EDGE_FALLING, EmergencyBtn);
    Exti_Enable(EMERG_BTN_PIN);

    /* Give emergency the HIGHEST NVIC priority (0 = highest) */
    /* EXTI10 shares EXTI15_10_IRQHandler, NVIC IRQ = 40 */
    SetIrqPriority(40, 0);

    /* Lower priority for cabin buttons (priority 2) */
    SetIrqPriority(6,  2);   /* EXTI0 */
    SetIrqPriority(7,  2);   /* EXTI1 */
    SetIrqPriority(8,  2);   /* EXTI2 */
    SetIrqPriority(9,  2);   /* EXTI3 */

    /* ---- Floor sensors (PC11-PC14, pull-down, rising edge) ---- */
    Gpio_Init(FLOOR_SENS_PORT, FLOOR_SENS_PIN_F1, GPIO_INPUT, GPIO_PULL_DOWN);
    Gpio_Init(FLOOR_SENS_PORT, FLOOR_SENS_PIN_F2, GPIO_INPUT, GPIO_PULL_DOWN);
    Gpio_Init(FLOOR_SENS_PORT, FLOOR_SENS_PIN_F3, GPIO_INPUT, GPIO_PULL_DOWN);
    Gpio_Init(FLOOR_SENS_PORT, FLOOR_SENS_PIN_F4, GPIO_INPUT, GPIO_PULL_DOWN);

    Exti_Init(FLOOR_SENS_PIN_F1, FLOOR_SENS_EXTI_PORT, EXTI_EDGE_RISING, FloorSensor_F1);
    Exti_Init(FLOOR_SENS_PIN_F2, FLOOR_SENS_EXTI_PORT, EXTI_EDGE_RISING, FloorSensor_F2);
    Exti_Init(FLOOR_SENS_PIN_F3, FLOOR_SENS_EXTI_PORT, EXTI_EDGE_RISING, FloorSensor_F3);
    Exti_Init(FLOOR_SENS_PIN_F4, FLOOR_SENS_EXTI_PORT, EXTI_EDGE_RISING, FloorSensor_F4);

    Exti_Enable(FLOOR_SENS_PIN_F1);
    Exti_Enable(FLOOR_SENS_PIN_F2);
    Exti_Enable(FLOOR_SENS_PIN_F3);
    Exti_Enable(FLOOR_SENS_PIN_F4);

    /* Floor sensor NVIC priority = 1 (high, but below emergency) */
    /* EXTI11-14 all share NVIC IRQ 40 (EXTI15_10), same as emergency.
     * The callbacks differentiate which line fired.  Emergency callback
     * itself sets the flag immediately, so it effectively preempts. */

#if IS_MASTER_BOARD
    /* ---- Hallway buttons (PB4-PB9, pull-up, falling edge) ---- */
    Gpio_Init(HALL_BTN_PORT, HALL_BTN_PIN_U1, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(HALL_BTN_PORT, HALL_BTN_PIN_D2, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(HALL_BTN_PORT, HALL_BTN_PIN_U2, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(HALL_BTN_PORT, HALL_BTN_PIN_D3, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(HALL_BTN_PORT, HALL_BTN_PIN_U3, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(HALL_BTN_PORT, HALL_BTN_PIN_D4, GPIO_INPUT, GPIO_PULL_UP);

    Exti_Init(HALL_BTN_PIN_U1, HALL_BTN_EXTI_PORT, EXTI_EDGE_FALLING, HallBtn_U1);
    Exti_Init(HALL_BTN_PIN_D2, HALL_BTN_EXTI_PORT, EXTI_EDGE_FALLING, HallBtn_D2);
    Exti_Init(HALL_BTN_PIN_U2, HALL_BTN_EXTI_PORT, EXTI_EDGE_FALLING, HallBtn_U2);
    Exti_Init(HALL_BTN_PIN_D3, HALL_BTN_EXTI_PORT, EXTI_EDGE_FALLING, HallBtn_D3);
    Exti_Init(HALL_BTN_PIN_U3, HALL_BTN_EXTI_PORT, EXTI_EDGE_FALLING, HallBtn_U3);
    Exti_Init(HALL_BTN_PIN_D4, HALL_BTN_EXTI_PORT, EXTI_EDGE_FALLING, HallBtn_D4);

    Exti_Enable(HALL_BTN_PIN_U1);
    Exti_Enable(HALL_BTN_PIN_D2);
    Exti_Enable(HALL_BTN_PIN_U2);
    Exti_Enable(HALL_BTN_PIN_D3);
    Exti_Enable(HALL_BTN_PIN_U3);
    Exti_Enable(HALL_BTN_PIN_D4);

    /* Hall button NVIC priority = 3 */
    SetIrqPriority(10, 3);   /* EXTI4 */
    SetIrqPriority(23, 3);   /* EXTI9_5 */
#endif
}
