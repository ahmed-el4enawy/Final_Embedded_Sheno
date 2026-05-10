/**
 * Button_Handlers.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Sets up EXTI callbacks for all buttons and floor sensors.
 *  All handlers set volatile flags — no heavy work in ISR.
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
/*  The elevator context pointer is stored so ISR callbacks can use it */
/* ------------------------------------------------------------------ */
static ElevatorContext *localCtx = 0;

/* ------------------------------------------------------------------ */
/*  Cabin button callbacks (both boards)                              */
/* ------------------------------------------------------------------ */
static void CabinBtn_F1(void) { Elevator_AddCabinRequest(localCtx, 1); }
static void CabinBtn_F2(void) { Elevator_AddCabinRequest(localCtx, 2); }
static void CabinBtn_F3(void) { Elevator_AddCabinRequest(localCtx, 3); }
static void CabinBtn_F4(void) { Elevator_AddCabinRequest(localCtx, 4); }

/* ------------------------------------------------------------------ */
/*  Emergency stop callback (both boards)                             */
/* ------------------------------------------------------------------ */
static void EmergencyBtn(void) { Elevator_EmergencyStop(localCtx); }

/* ------------------------------------------------------------------ */
/*  Floor sensor callbacks (both boards)                              */
/* ------------------------------------------------------------------ */
static void FloorSensor_F1(void) { Elevator_FloorSensorTriggered(localCtx, 1); }
static void FloorSensor_F2(void) { Elevator_FloorSensorTriggered(localCtx, 2); }
static void FloorSensor_F3(void) { Elevator_FloorSensorTriggered(localCtx, 3); }
static void FloorSensor_F4(void) { Elevator_FloorSensorTriggered(localCtx, 4); }

/* ------------------------------------------------------------------ */
/*  Hallway button callbacks (Master only)                            */
/* ------------------------------------------------------------------ */
#if IS_MASTER_BOARD
static void HallBtn_U1(void) { Dispatcher_RegisterHallCall(1, DIR_UP);   }
static void HallBtn_D2(void) { Dispatcher_RegisterHallCall(2, DIR_DOWN); }
static void HallBtn_U2(void) { Dispatcher_RegisterHallCall(2, DIR_UP);   }
static void HallBtn_D3(void) { Dispatcher_RegisterHallCall(3, DIR_DOWN); }
static void HallBtn_U3(void) { Dispatcher_RegisterHallCall(3, DIR_UP);   }
static void HallBtn_D4(void) { Dispatcher_RegisterHallCall(4, DIR_DOWN); }
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
