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
/* DEBOUNCE_MS is now defined in Board_Config.h */

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

    /* ---- Cabin floor buttons (pull-up, falling edge) ---- */
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

    /* ---- Emergency stop (pull-up, falling edge) ---- */
    Gpio_Init(EMERG_BTN_PORT, EMERG_BTN_PIN, GPIO_INPUT, GPIO_PULL_UP);
    Exti_Init(EMERG_BTN_PIN, EMERG_BTN_EXTI_PORT, EXTI_EDGE_FALLING, EmergencyBtn);
    Exti_Enable(EMERG_BTN_PIN);

    /* Give emergency the HIGHEST NVIC priority */
    SetIrqPriority(IRQ_EXTI15_10, PRIO_EMERGENCY);

    /* Lower priority for cabin buttons */
    SetIrqPriority(IRQ_EXTI0, PRIO_CABIN_BTN);
    SetIrqPriority(IRQ_EXTI1, PRIO_CABIN_BTN);
    SetIrqPriority(IRQ_EXTI2, PRIO_CABIN_BTN);
    SetIrqPriority(IRQ_EXTI3, PRIO_CABIN_BTN);

    /* ---- Floor sensors (pull-down, rising edge) ---- */
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

    /* Floor sensor NVIC priority (PC0-PC3 are on EXTI0-3) */
    /* Note: PA0-PA3 (Cabin) and PC0-PC3 (Floor) share EXTI0-3. 
     * STM32 EXTI can only map one port per line. 
     * Schematic check: PA0-PA3 and PC0-PC3 are assigned. 
     * Both share PRIO_CABIN_BTN (2) priority on the same NVIC vectors. */
    SetIrqPriority(IRQ_EXTI0, PRIO_CABIN_BTN);
    SetIrqPriority(IRQ_EXTI1, PRIO_CABIN_BTN);
    SetIrqPriority(IRQ_EXTI2, PRIO_CABIN_BTN);
    SetIrqPriority(IRQ_EXTI3, PRIO_CABIN_BTN);

#if IS_MASTER_BOARD
    /* ---- Hallway buttons (pull-up, falling edge) ---- */
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

    /* Hall button NVIC priority */
    SetIrqPriority(IRQ_EXTI4,   PRIO_HALL_BTN);
    SetIrqPriority(IRQ_EXTI9_5, PRIO_HALL_BTN);
#endif
}
