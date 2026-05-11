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

/* [NEW] Checksum failure counter for telemetry */
volatile uint32 checksumErrorCount = 0;

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
    uint32 pm = Enter_Critical();
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
    Exit_Critical(pm);

    SpiProto_Pack(&fd, spiTxBuf);
}

/**
 * @brief Parse an RX frame and update the remote elevator context.
 * @return TRUE on valid frame.
 */
static boolean ParseRemoteFrame(void) {
    SpiFrameData fd;
    if (!SpiProto_Unpack(spiRxBuf, &fd)) {
        checksumErrorCount++;
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

/* ------------------------------------------------------------------ */
/*  [FIX — SysTick Core Clock Hardcoding]                              */
/*                                                                      */
/*  Previous code:  #define SYSTICK_CLOCK_HZ  16000000UL               */
/*  This is violently wrong if Rcc_Init() configures the PLL.          */
/*  On STM32F401 the PLL can push SYSCLK to 84 MHz; a 16 MHz math     */
/*  base would fire SysTick every 0.19 ms instead of 1.0 ms.           */
/*                                                                      */
/*  Solution: Read the RCC_CFGR.SWS[1:0] bits to determine the active  */
/*  system clock source, then compute the true frequency:               */
/*   SWS=00  → HSI  = 16 MHz                                          */
/*   SWS=01  → HSE  = HSE_VALUE (typically 8 or 25 MHz)               */
/*   SWS=10  → PLL  → decode PLLCFGR (M, N, P) to get VCO/P          */
/* ------------------------------------------------------------------ */

/* RCC register addresses (STM32F4xx) */
#define RCC_BASE_ADDR       0x40023800UL
#define RCC_CR_REG          (*(volatile uint32 *)(RCC_BASE_ADDR + 0x00UL))
#define RCC_PLLCFGR_REG     (*(volatile uint32 *)(RCC_BASE_ADDR + 0x04UL))
#define RCC_CFGR_REG        (*(volatile uint32 *)(RCC_BASE_ADDR + 0x08UL))

/* Default oscillator frequencies */
#define HSI_VALUE_HZ        16000000UL
#define HSE_VALUE_HZ         8000000UL   /* adjust to match actual crystal */

/**
 * @brief  Dynamically compute the current SYSCLK frequency from
 *         the RCC_CFGR SWS bits and (if PLL) the PLLCFGR register.
 * @return System clock frequency in Hz.
 */
static uint32 Rcc_GetSystemClock(void) {
    uint32 cfgr = RCC_CFGR_REG;
    uint8  sws  = (uint8)((cfgr >> 2) & 0x03U);   /* SWS[1:0] = bits [3:2] */

    switch (sws) {
    case 0U:   /* HSI oscillator */
        return HSI_VALUE_HZ;

    case 1U:   /* HSE oscillator */
        return HSE_VALUE_HZ;

    case 2U: { /* PLL */
        /* Decode PLLCFGR:
         *   PLLM[5:0]  = bits  [5:0]   → divider  2..63
         *   PLLN[8:0]  = bits [14:6]   → multiplier 50..432
         *   PLLP[1:0]  = bits [17:16]  → 0→2, 1→4, 2→6, 3→8
         *   PLLSRC     = bit  [22]     → 0=HSI, 1=HSE
         */
        uint32 pllcfgr = RCC_PLLCFGR_REG;
        uint32 pllm = pllcfgr & 0x3FUL;
        uint32 plln = (pllcfgr >> 6)  & 0x1FFUL;
        uint32 pllp = (((pllcfgr >> 16) & 0x03UL) + 1UL) * 2UL;  /* 2,4,6,8 */
        uint32 pllsrc = (pllcfgr >> 22) & 0x01UL;

        uint32 inputClk = pllsrc ? HSE_VALUE_HZ : HSI_VALUE_HZ;

        /* Guard against divide-by-zero (should never happen in practice) */
        if (pllm == 0) pllm = 1;

        /* SYSCLK = (inputClk / PLLM) * PLLN / PLLP */
        return (inputClk / pllm) * plln / pllp;
    }

    default:   /* Should not occur */
        return HSI_VALUE_HZ;
    }
}

static void SysTick_Init(void) {
    uint32 sysClkHz = Rcc_GetSystemClock();
    SYSTICK_RVR = (sysClkHz / 1000UL) - 1UL;   /* 1 ms reload */
    SYSTICK_CVR = 0;                             /* clear current value */
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
/*  [NEW] Independent Watchdog (IWDG)                                  */
/* ================================================================== */
#define IWDG_BASE            0x40003000UL
#define IWDG_KR              (*(volatile uint32 *)(IWDG_BASE + 0x00))
#define IWDG_PR              (*(volatile uint32 *)(IWDG_BASE + 0x04))
#define IWDG_RLR             (*(volatile uint32 *)(IWDG_BASE + 0x08))
#define IWDG_SR              (*(volatile uint32 *)(IWDG_BASE + 0x0C))

static void Iwdg_Init(void) {
    IWDG_KR  = 0x5555;      /* Enable register access */
    IWDG_PR  = 4;           /* Prescaler = 64 (32kHz / 64 = 500Hz) */
    IWDG_RLR = 2000;        /* Reload = 2000 (2000 / 500Hz = 4 seconds) */
    IWDG_KR  = 0xAAAA;      /* Refresh / Start */
    IWDG_KR  = 0xCCCC;      /* Start watchdog */
}

static void Iwdg_Refresh(void) {
    IWDG_KR = 0xAAAA;
}

/* ================================================================== */
/*  System Control Block (SCB) - for System Handler Priorities        */
/* ================================================================== */
#define SCB_SHPR3            (*(volatile uint32 *)0xE000ED20UL)

/* ================================================================== */
/*  Peripheral initialisation                                          */
/* ================================================================== */

static void System_Init(void) {
    /* --- Clocks --- */
    Rcc_Init();

    /* Set NVIC priority grouping: 4 bits preemption, 0 bits sub */
    Nvic_SetPriorityGrouping(3);

    /* Set SysTick priority to lowest (15) so it doesn't preempt EXTI/SPI */
    SCB_SHPR3 |= (15UL << 24);

    Rcc_Enable(CABIN_BTN_RCC);
    Rcc_Enable(EMERG_BTN_RCC);
    Rcc_Enable(FLOOR_SENS_RCC);
#if IS_MASTER_BOARD
    Rcc_Enable(HALL_BTN_RCC);
#endif
    Rcc_Enable(RCC_SYSCFG);
    Rcc_Enable(RCC_TIM2);
    Rcc_Enable(MOTOR_PWM_RCC);
    Rcc_Enable(TELEMETRY_TIMER == TIMER4 ? RCC_TIM4 : RCC_TIM2); /* mapping simplified */
    Rcc_Enable(DOOR_TIMER == TIMER5 ? RCC_TIM5 : RCC_TIM2);
    Rcc_Enable(UART_RCC_ID);
    Rcc_Enable(SPI_RCC_ID);
    Rcc_Enable(RCC_DMA2);

    /* --- [FIX #1/#4/#5] SysTick — 1 ms global tick --- */
    SysTick_Init();

    /* --- USART1 --- */
    Usart1_Init();

    /* --- PWM motor LED (e.g. TIM3 CH1 on PC6, AF2) --- */
    Gpio_Init(MOTOR_PWM_PORT, MOTOR_PWM_PIN, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(MOTOR_PWM_PORT, MOTOR_PWM_PIN, MOTOR_PWM_AF);
    Pwm_Init(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, MOTOR_PWM_PSC, MOTOR_PWM_ARR);
    Pwm_SetDutyPercent(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL, 0);
    Pwm_Start(MOTOR_PWM_TIMER, MOTOR_PWM_CHANNEL);

    /* --- SPI1 pins (SCK, MISO, MOSI) --- */
    Gpio_Init(SPI_GPIO_PORT, SPI_SCK_PIN,  GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(SPI_GPIO_PORT, SPI_MISO_PIN, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(SPI_GPIO_PORT, SPI_MOSI_PIN, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(SPI_GPIO_PORT, SPI_SCK_PIN,  SPI_AF);
    Gpio_SetAF(SPI_GPIO_PORT, SPI_MISO_PIN, SPI_AF);
    Gpio_SetAF(SPI_GPIO_PORT, SPI_MOSI_PIN, SPI_AF);

#if IS_MASTER_BOARD
    Spi_Init(SPI_PERIPHERAL, SPI_MODE_MASTER, SPI_BAUD_DIVIDER);
#else
    Spi_Init(SPI_PERIPHERAL, SPI_MODE_SLAVE, 0);
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
    Spi_SlavePreload(SPI_PERIPHERAL, spiTxBuf, SPI_FRAME_LEN);
    Spi_SlaveStartAsync(SPI_PERIPHERAL, SlaveRxCallback, spiRxBuf, SPI_FRAME_LEN);
#endif

    /* [NEW] Independent Watchdog */
    Iwdg_Init();

    Usart1_TransmitString("\r\n=== Elevator System Started ===\r\n");
}

/* ================================================================== */
/*  [NON-BLOCKING MASTER] SPI completion callback                      */
/*                                                                      */
/*  Called from SPI1_IRQHandler when all 8 bytes have been exchanged.  */
/*  Sets spiRxReady so the main loop can parse the received frame on   */
/*  its next iteration — zero CPU time spent polling TXE/RXNE.         */
/* ================================================================== */
#if IS_MASTER_BOARD
static void MasterSpiDoneCb(uint8 *buf, uint8 len) {
    (void)buf;
    (void)len;
    spiRxReady = 1;
}
#endif

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
        /* -------- Master: 50 ms non-blocking SPI exchange -------- */
        /* [NON-BLOCKING MASTER]
         *
         * Previous code used blocking Spi_TransmitReceive() which polled
         * TXE/RXNE in a while loop.  If the slave was unresponsive, the
         * master CPU would hang indefinitely.
         *
         * New flow:
         *   1. Timer ISR sets spiExchangeFlag every 50 ms.
         *   2. Main loop builds the TX frame and calls
         *      Spi_MasterTransferAsync() which enables TXEIE/RXNEIE.
         *   3. SPI1 ISR feeds bytes from TxBuf, collects into RxBuf.
         *   4. On completion, ISR invokes MasterSpiDoneCb which sets
         *      spiRxReady = 1 and deasserts CS.
         *   5. Main loop sees spiRxReady on the NEXT iteration and
         *      parses the frame / runs the dispatcher.
         *
         * CPU is NEVER blocked polling SPI status registers. */

        /* Step 1: Initiate async transfer when timer fires */
        if (spiExchangeFlag && !Spi_MasterBusy()) {
            spiExchangeFlag = 0;
            BuildLocalFrame();
            Spi_MasterTransferAsync(SPI_PERIPHERAL, spiTxBuf, spiRxBuf,
                                    SPI_FRAME_LEN, MasterSpiDoneCb);
        }

        /* Step 2: Process completed transfer */
        if (spiRxReady) {
            spiRxReady = 0;

            /* Parse remote frame */
            if (ParseRemoteFrame()) {
                spiCommFault = 0;
                spiLastGoodRxTick = sysTickMs;
            } else {
                uint32 elapsed = sysTickMs - spiLastGoodRxTick;
                if (elapsed >= SPI_TIMEOUT_MS) {
                    if (!spiCommFault) {
                        spiLastRxSeq = 0xFF;
                    }
                    spiCommFault = 1;
                }
            }

            /* Run dispatcher with latest data */
            Dispatcher_Run(&localElev, &remoteElev,
                           spiCommFault ? TRUE : FALSE);
        }

        /* Step 3: Even without RX, check for timeout */
        if (!spiRxReady) {
            uint32 elapsed = sysTickMs - spiLastGoodRxTick;
            if (elapsed >= SPI_TIMEOUT_MS) {
                if (!spiCommFault) {
                    spiLastRxSeq = 0xFF;
                }
                spiCommFault = 1;
                /* Run dispatcher in fault mode */
                Dispatcher_Run(&localElev, &remoteElev, TRUE);
            }
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
            Spi_SlavePreload(SPI_PERIPHERAL, spiTxBuf, SPI_FRAME_LEN);
        }

        /* [FIX #4] Slave comm-fault: accurate 1 ms SysTick-based timeout.
         * Checks elapsed time since last valid SPI frame EVERY main-loop
         * iteration — no longer gated by the 500 ms telemetry period. */
        {
            uint32 elapsed = sysTickMs - spiLastGoodRxTick;
            if (elapsed >= SPI_TIMEOUT_MS) {
                if (!spiCommFault) {
                    spiCommFault = 1;

                    /* [FIX — Sequence Wrap-Around]
                     * Invalidate spiLastRxSeq so the first recovery frame
                     * post-fault is never dropped as a duplicate.  If the
                     * fault persists for exactly 256×50ms = 12.8s, the
                     * remote sequence byte wraps back to the stale value
                     * in spiLastRxSeq.  Setting 0xFF breaks the collision
                     * because ParseRemoteFrame() will accept any sequence
                     * that is != 0xFF. */
                    spiLastRxSeq = 0xFF;

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
                         Dispatcher_GetPendingHallCalls(),
                         checksumErrorCount);
#else
        Telemetry_Update(&localElev, (ElevatorContext *)0,
                         spiCommFault ? FALSE : TRUE, 0,
                         checksumErrorCount);
#endif

        /* -------- Refresh Watchdog -------- */
        Iwdg_Refresh();
    }

    return 0;
}
