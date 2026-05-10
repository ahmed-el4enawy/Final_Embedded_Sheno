/**
 * main.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Unified entry point for both Board A (Master) and Board B (Slave).
 *  Set IS_MASTER_BOARD in Board_Config.h to select the role.
 *
 *  Board A (Master): Elevator A FSM + Dispatcher + SPI Master + Hallway Buttons
 *  Board B (Slave) : Elevator B FSM + SPI Slave (receives hall calls from master)
 *
 *  [FIX #1] PWM speed ramping uses sysTickMs for non-blocking timestamps.
 *  [FIX #3] Slave enters ELEV_INDEPENDENT on SPI comm fault.
 *  [FIX #4] SysTick 1 ms counter replaces telemetry-gated timeout on slave.
 *  [FIX #5] Button debounce also uses sysTickMs (see Button_Handlers.c).
 */

#include "Std_Types.h"
#include "Board_Config.h"
#include "Critical.h"

/* Drivers */
#include "Rcc.h"
#include "Gpio.h"
#include "Exti.h"
#include "Nvic.h"
#include "Timer.h"
#include "Pwm.h"
#include "Usart.h"
#include "Spi.h"
#include "Dma.h"

/* Application */
#include "Spi_Protocol.h"
#include "Elevator_FSM.h"
#include "Button_Handlers.h"
#include "Telemetry.h"

#if IS_MASTER_BOARD
#include "Dispatcher.h"
#endif

/* ================================================================== */
/*  [FIX #1/#4/#5] Global 1 ms SysTick counter                        */
/*  Used by:                                                           */
/*   - PWM ramp (Elevator_FSM.c, [FIX #1])                           */
/*   - Slave SPI timeout detection (this file, [FIX #4])              */
/*   - Button debounce (Button_Handlers.c, [FIX #5])                  */
/* ================================================================== */
volatile uint32 sysTickMs = 0;

/* ================================================================== */
/*  Global state                                                       */
/* ================================================================== */

/* Local elevator context (Elevator A on Master, Elevator B on Slave) */
static ElevatorContext localElev;

/* Remote elevator context (received via SPI) */
static ElevatorContext remoteElev;

/* SPI frame buffers */
static uint8 spiTxBuf[SPI_FRAME_LEN];
static uint8 spiRxBuf[SPI_FRAME_LEN];

/* SPI synchronization flags */
static volatile uint8 spiExchangeFlag = 0;   /* set by timer ISR      */
static volatile uint8 spiRxReady      = 0;   /* set when valid frame  */
static volatile uint8 spiCommFault    = 0;   /* set on timeout        */
static volatile uint8 spiSequence     = 0;
static volatile uint8 spiLastRxSeq    = 0xFF;

/* [FIX #4] SPI timeout tracked via sysTickMs — NOT telemetry period */
static volatile uint32 spiLastGoodRxTick = 0;

/* Door timer callback (shared between master/slave) */
/* Forward declaration — defined after localElev */
static void DoorTimerCb(void);

/* Wrapper that the FSM calls when it needs a door timer */
static void StartDoorTimer(void) {
    Timer_DelayMsAsync(DOOR_TIMER, DOOR_OPEN_TIME_MS, DoorTimerCb);
}

static void DoorTimerCb(void) {
    Elevator_DoorTimerExpired(&localElev);
}

/* ================================================================== */
/*  SPI exchange helpers                                               */
/* ================================================================== */

/**
 * @brief Build a TX frame from the local elevator state.
 */
static void BuildLocalFrame(void) {
    SpiFrameData fd;
    fd.state         = Elevator_GetPacketState(&localElev);
    fd.currentFloor  = localElev.currentFloor;
    fd.direction     = localElev.direction;
    fd.cabinRequests = localElev.cabinRequests;
    fd.assignedCalls = localElev.assignedCalls;
    fd.flags         = 0;
    if (localElev.emergencyStop) fd.flags |= FLAG_EMERGENCY;
    if (spiCommFault)            fd.flags |= FLAG_COMM_FAULT;
    if (localElev.doorOpen)      fd.flags |= FLAG_DOOR_OPEN;

#if IS_MASTER_BOARD
    fd.hallCalls = Dispatcher_GetAssignedB();   /* hall calls FOR slave */
#else
    fd.hallCalls = 0;
#endif

    fd.sequence = spiSequence++;
    SpiProto_Pack(&fd, spiTxBuf);
}

/**
 * @brief Parse an RX frame and update the remote elevator context.
 * @return TRUE on valid frame.
 */
static boolean ParseRemoteFrame(void) {
    SpiFrameData fd;
    if (!SpiProto_Unpack(spiRxBuf, &fd)) {
        return FALSE;
    }

    /* Freshness check */
    if (fd.sequence == spiLastRxSeq) {
        return FALSE;   /* duplicate */
    }
    spiLastRxSeq = fd.sequence;

    uint32 pm = Enter_Critical();
    remoteElev.currentFloor  = fd.currentFloor;
    remoteElev.direction     = fd.direction;
    remoteElev.cabinRequests = fd.cabinRequests;
    remoteElev.assignedCalls = fd.assignedCalls;
    remoteElev.emergencyStop = (fd.flags & FLAG_EMERGENCY) ? 1 : 0;
    remoteElev.doorOpen      = (fd.flags & FLAG_DOOR_OPEN) ? 1 : 0;

    /* Map packet state back to FSM state */
    switch (fd.state) {
        case PKT_STATE_IDLE:         remoteElev.state = ELEV_IDLE;          break;
        case PKT_STATE_MOVING_UP:    remoteElev.state = ELEV_MOVING_UP;    break;
        case PKT_STATE_MOVING_DOWN:  remoteElev.state = ELEV_MOVING_DOWN;  break;
        case PKT_STATE_DOORS_OPEN:   remoteElev.state = ELEV_DOORS_OPEN;   break;
        case PKT_STATE_EMERGENCY:    remoteElev.state = ELEV_EMERGENCY_STOP; break;
        case PKT_STATE_DOOR_CLOSING: remoteElev.state = ELEV_DOOR_CLOSING; break;
        default:                     remoteElev.state = ELEV_IDLE;          break;
    }

#if !IS_MASTER_BOARD
    /* Slave: apply hall calls from master as assigned calls */
    localElev.assignedCalls = fd.hallCalls;
#endif

    Exit_Critical(pm);
    return TRUE;
}

/* ================================================================== */
/*  Timer callbacks                                                    */
/* ================================================================== */

#if IS_MASTER_BOARD
/**
 * 50 ms periodic SPI exchange timer (Master only).
 * Sets a flag — actual SPI work happens in main loop.
 */
static void SpiExchangeTimerCb(void) {
    spiExchangeFlag = 1;
    /* Re-arm */
    Timer_DelayMsAsync(SPI_EXCHANGE_TIMER, SPI_EXCHANGE_PERIOD_MS,
                       SpiExchangeTimerCb);
}
#endif

/* ================================================================== */
/*  Slave SPI callback (from ISR)                                      */
/* ================================================================== */

#if !IS_MASTER_BOARD
static void SlaveRxCallback(uint8 *buf, uint8 len) {
    (void)len;
    spiRxReady = 1;
    /* [FIX #4] Record the SysTick ms of last successful SPI reception */
    spiLastGoodRxTick = sysTickMs;
}
#endif

/* ================================================================== */
/*  [FIX #1/#4/#5] SysTick initialisation and ISR                      */
/*                                                                      */
/*  Configures SysTick for a 1 ms interrupt.  This provides a global   */
/*  free-running millisecond counter used by:                           */
/*   - PWM speed ramping (non-blocking duty step timing)               */
/*   - Slave SPI timeout detection (accurate to 1 ms)                  */
/*   - EXTI button debounce (50 ms timestamp checks)                   */
/*                                                                      */
/*  SysTick registers (Cortex-M4 — always available, no RCC needed):  */
/*   SYST_CSR   @ 0xE000E010  (Control and Status)                     */
/*   SYST_RVR   @ 0xE000E014  (Reload Value)                           */
/*   SYST_CVR   @ 0xE000E018  (Current Value)                          */
/* ================================================================== */

#define SYSTICK_CSR   (*(volatile uint32 *)0xE000E010UL)
#define SYSTICK_RVR   (*(volatile uint32 *)0xE000E014UL)
#define SYSTICK_CVR   (*(volatile uint32 *)0xE000E018UL)

/* Bit definitions for SYST_CSR */
#define SYSTICK_CSR_ENABLE      (1UL << 0)
#define SYSTICK_CSR_TICKINT     (1UL << 1)
#define SYSTICK_CSR_CLKSOURCE   (1UL << 2)   /* 1 = processor clock */

/* HSI = 16 MHz by default on STM32F4 (no PLL assumed) */
#define SYSTICK_CLOCK_HZ   16000000UL

static void SysTick_Init(void) {
    SYSTICK_RVR = (SYSTICK_CLOCK_HZ / 1000UL) - 1UL;   /* 1 ms reload */
    SYSTICK_CVR = 0;                                     /* clear current */
    SYSTICK_CSR = SYSTICK_CSR_CLKSOURCE
                | SYSTICK_CSR_TICKINT
                | SYSTICK_CSR_ENABLE;
}

/**
 * @brief  SysTick ISR — increments the global millisecond counter.
 *         The symbol SysTick_Handler is the standard Cortex-M vector name.
 */
void SysTick_Handler(void) {
    sysTickMs++;
}

/* ================================================================== */
/*  Peripheral initialisation                                          */
/* ================================================================== */

static void System_Init(void) {
    /* --- Clocks --- */
    Rcc_Init();

    /* Set NVIC priority grouping: 4 bits preemption, 0 bits sub */
    Nvic_SetPriorityGrouping(3);

    Rcc_Enable(RCC_GPIOA);
    Rcc_Enable(RCC_GPIOB);
    Rcc_Enable(RCC_GPIOC);
    Rcc_Enable(RCC_SYSCFG);
    Rcc_Enable(RCC_TIM2);
    Rcc_Enable(RCC_TIM3);
    Rcc_Enable(RCC_TIM4);
    Rcc_Enable(RCC_TIM5);
    Rcc_Enable(RCC_USART1);
    Rcc_Enable(RCC_SPI1);
    Rcc_Enable(RCC_DMA2);

    /* --- [FIX #1/#4/#5] SysTick — 1 ms global tick --- */
    SysTick_Init();

    /* --- USART1 --- */
    Usart1_Init();

    /* --- PWM motor LED (TIM3 CH3 on PB0, AF2) --- */
    Gpio_Init(MOTOR_LED_PORT, MOTOR_LED_PIN, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(MOTOR_LED_PORT, MOTOR_LED_PIN, MOTOR_LED_AF);
    Pwm_Init(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_PWM_PSC, MOTOR_PWM_ARR);
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, 0);
    Pwm_Start(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL);

    /* --- SPI1 pins (PA5=SCK, PA6=MISO, PA7=MOSI) --- */
    Gpio_Init(SPI_PORT, SPI_SCK_PIN,  GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(SPI_PORT, SPI_MISO_PIN, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(SPI_PORT, SPI_MOSI_PIN, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(SPI_PORT, SPI_SCK_PIN,  SPI_AF);
    Gpio_SetAF(SPI_PORT, SPI_MISO_PIN, SPI_AF);
    Gpio_SetAF(SPI_PORT, SPI_MOSI_PIN, SPI_AF);

#if IS_MASTER_BOARD
    Spi_Init(SPI_1, SPI_MODE_MASTER, SPI_BAUD_DIV64);
#else
    Spi_Init(SPI_1, SPI_MODE_SLAVE, 0);
#endif

    /* --- Elevator FSM --- */
    Elevator_Init(&localElev);
    Elevator_Init(&remoteElev);
    localElev.startDoorTimer = StartDoorTimer;
    /* remoteElev doesn't need a door timer — it's just a mirror */

    /* --- Buttons & sensors (EXTI) --- */
    Buttons_Init(&localElev);

#if IS_MASTER_BOARD
    /* --- Dispatcher --- */
    Dispatcher_Init();
#endif

    /* --- DMA for USART1 TX (bonus) --- */
    Dma_Usart1TxInit();

    /* --- Telemetry (500 ms periodic) --- */
    Telemetry_Init();

    /* [FIX #4] Initialise last-good-rx timestamp to now */
    spiLastGoodRxTick = sysTickMs;

#if IS_MASTER_BOARD
    /* --- 50 ms SPI exchange timer --- */
    Timer_DelayMsAsync(SPI_EXCHANGE_TIMER, SPI_EXCHANGE_PERIOD_MS,
                       SpiExchangeTimerCb);
#else
    /* Slave: pre-load TX and start async reception */
    BuildLocalFrame();
    Spi_SlavePreload(SPI_1, spiTxBuf, SPI_FRAME_LEN);
    Spi_SlaveStartAsync(SPI_1, SlaveRxCallback, spiRxBuf, SPI_FRAME_LEN);
#endif

    Usart1_TransmitString("\r\n=== Elevator System Started ===\r\n");
}

/* ================================================================== */
/*  MAIN                                                               */
/* ================================================================== */

int main(void) {

    System_Init();

    while (1) {

        /* -------- Run local elevator FSM -------- */
        /* NOTE: Emergency check is handled inside Elevator_Run() which
         * forces ELEV_EMERGENCY_STOP and instant-stops the motor via
         * the ramp context.  No duplicate handling needed here. */
        Elevator_Run(&localElev);

#if IS_MASTER_BOARD
        /* -------- Master: 50 ms SPI exchange -------- */
        if (spiExchangeFlag) {
            spiExchangeFlag = 0;

            /* Build TX frame with latest local state */
            BuildLocalFrame();

            /* Full-duplex transfer */
            uint32 pm = Enter_Critical();
            Spi_CsLow();
            Spi_TransmitReceive(SPI_1, spiTxBuf, spiRxBuf, SPI_FRAME_LEN);
            Spi_CsHigh();
            Exit_Critical(pm);

            /* Parse remote frame */
            if (ParseRemoteFrame()) {
                spiCommFault = 0;
                spiLastGoodRxTick = sysTickMs;  /* [FIX #4] reset timeout */
            } else {
                /* [FIX #4] Check elapsed time since last valid frame */
                uint32 elapsed = sysTickMs - spiLastGoodRxTick;
                if (elapsed >= SPI_TIMEOUT_MS) {
                    spiCommFault = 1;
                }
            }

            /* Run dispatcher with latest data */
            Dispatcher_Run(&localElev, &remoteElev,
                           spiCommFault ? TRUE : FALSE);
        }
#else
        /* -------- Slave: check for new SPI data -------- */
        if (spiRxReady) {
            spiRxReady = 0;
            if (ParseRemoteFrame()) {
                spiCommFault = 0;
                spiLastGoodRxTick = sysTickMs;  /* [FIX #4] reset timeout */

                /* [FIX #3] If we were in independent mode and comm is back,
                 * exit independent mode and resume normal operation. */
                if (localElev.state == ELEV_INDEPENDENT) {
                    Elevator_ExitIndependentMode(&localElev);
                }
            }

            /* Refresh TX buffer for next exchange */
            BuildLocalFrame();
            Spi_SlavePreload(SPI_1, spiTxBuf, SPI_FRAME_LEN);
        }

        /* [FIX #4] Slave comm-fault: accurate 1 ms SysTick-based timeout.
         * Checks elapsed time since last valid SPI frame EVERY main-loop
         * iteration — no longer gated by the 500 ms telemetry period. */
        {
            uint32 elapsed = sysTickMs - spiLastGoodRxTick;
            if (elapsed >= SPI_TIMEOUT_MS) {
                if (!spiCommFault) {
                    spiCommFault = 1;
                    /* [FIX #3] Force slave into independent/emergency mode.
                     * Clears assigned calls, stops motor, ignores external
                     * commands until SPI link recovers.
                     *
                     * EnterIndependentMode is called ONCE on the rising edge
                     * of the fault.  The FSM may internally transition
                     * INDEPENDENT→IDLE to service cabin requests while the
                     * fault persists — that is acceptable (degraded cabin-
                     * only operation).  We do NOT re-enter independent mode
                     * on subsequent iterations to avoid ping-ponging. */
                    Elevator_EnterIndependentMode(&localElev);
                }
                /* While fault persists, ensure assignedCalls stay cleared
                 * so the FSM only services cabin requests. */
                localElev.assignedCalls = 0;
            } else {
                spiCommFault = 0;
            }
        }
#endif

        /* -------- Telemetry (500 ms UART report) -------- */
#if IS_MASTER_BOARD
        Telemetry_Update(&localElev, &remoteElev,
                         spiCommFault ? FALSE : TRUE,
                         Dispatcher_GetPendingHallCalls());
#else
        Telemetry_Update(&localElev, (ElevatorContext *)0,
                         spiCommFault ? FALSE : TRUE, 0);
#endif
    }

    return 0;
}
