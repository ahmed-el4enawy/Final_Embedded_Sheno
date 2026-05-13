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
 *  [REDESIGN] Main loop execution order:
 *  1. Run FSM (consumes events, produces state)
 *  2. Run Dispatcher (assigns hall calls)
 *  3. Build SPI frame AFTER FSM — sends fresh data
 *  4. SPI exchange
 *  5. Parse response
 *  6. Telemetry
 *
 *  [FIX #7] Cabin buttons moved to PB12-PB15 (EXTI12-15).
 *  All inputs are now interrupt-driven — no polling in main loop.
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
#include "Spi_Private.h"   /* SpiType for master state machine */
#include "Bit_Operations.h" /* READ_BIT for master state machine */
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
/*  Global 1 ms SysTick counter                                        */
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

/* SPI timeout tracked via sysTickMs */
static volatile uint32 spiLastGoodRxTick = 0;

/* Checksum failure counter for telemetry */
volatile uint32 checksumErrorCount = 0;

/* Door timer callback */
static void DoorTimerCb(void);

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
 *        [REDESIGN] Called AFTER Elevator_Run() so state is fresh.
 */
static void BuildLocalFrame(void) {
    SpiFrameData fd;
    uint32 pm = Enter_Critical();
    fd.state         = Elevator_GetPacketState(&localElev);
    fd.currentFloor  = localElev.currentFloor;
    fd.direction     = localElev.direction;
    fd.targetFloor   = localElev.targetFloor;
    fd.cabinRequests = localElev.cabinRequests;
    fd.assignedCalls = localElev.assignedCalls;
    fd.flags         = 0;
    if (localElev.emergencyStop) fd.flags |= FLAG_EMERGENCY;
    if (spiCommFault)            fd.flags |= FLAG_COMM_FAULT;
    if (localElev.doorOpen)      fd.flags |= FLAG_DOOR_OPEN;

#if IS_MASTER_BOARD
    fd.assignedCalls = Dispatcher_GetAssignedBFloorMask();  /* floor mask FOR slave FSM */
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

    /* Freshness check — reject exact duplicates */
    if (fd.sequence == spiLastRxSeq) {
        return FALSE;
    }

    /* [REDESIGN] Staleness check — reject if sequence wrapped too far back */
    {
        uint8 seqDelta = (uint8)(fd.sequence - spiLastRxSeq);
        if (seqDelta > 200) {
            /* Sequence went backwards by more than 55 — stale frame */
            return FALSE;
        }
    }

    spiLastRxSeq = fd.sequence;

    uint32 pm = Enter_Critical();
    remoteElev.currentFloor  = fd.currentFloor;
    remoteElev.targetFloor   = fd.targetFloor;
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
    localElev.assignedCalls = fd.assignedCalls;
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
    spiLastGoodRxTick = sysTickMs;
}
#endif

/* ================================================================== */
/*  SysTick initialisation and ISR                                     */
/* ================================================================== */

#define SYSTICK_CSR   (*(volatile uint32 *)0xE000E010UL)
#define SYSTICK_RVR   (*(volatile uint32 *)0xE000E014UL)
#define SYSTICK_CVR   (*(volatile uint32 *)0xE000E018UL)

#define SYSTICK_CSR_ENABLE      (1UL << 0)
#define SYSTICK_CSR_TICKINT     (1UL << 1)
#define SYSTICK_CSR_CLKSOURCE   (1UL << 2)

/* RCC register addresses (STM32F4xx) */
#define RCC_BASE_ADDR       0x40023800UL
#define RCC_CR_REG          (*(volatile uint32 *)(RCC_BASE_ADDR + 0x00UL))
#define RCC_PLLCFGR_REG     (*(volatile uint32 *)(RCC_BASE_ADDR + 0x04UL))
#define RCC_CFGR_REG        (*(volatile uint32 *)(RCC_BASE_ADDR + 0x08UL))

#define HSI_VALUE_HZ        16000000UL
#define HSE_VALUE_HZ         8000000UL

/**
 * @brief  Dynamically compute the current SYSCLK frequency.
 */
static uint32 Rcc_GetSystemClock(void) {
    uint32 cfgr = RCC_CFGR_REG;
    uint8  sws  = (uint8)((cfgr >> 2) & 0x03U);

    switch (sws) {
    case 0U:
        return HSI_VALUE_HZ;
    case 1U:
        return HSE_VALUE_HZ;
    case 2U: {
        uint32 pllcfgr = RCC_PLLCFGR_REG;
        uint32 pllm = pllcfgr & 0x3FUL;
        uint32 plln = (pllcfgr >> 6)  & 0x1FFUL;
        uint32 pllp = (((pllcfgr >> 16) & 0x03UL) + 1UL) * 2UL;
        uint32 pllsrc = (pllcfgr >> 22) & 0x01UL;
        uint32 inputClk = pllsrc ? HSE_VALUE_HZ : HSI_VALUE_HZ;
        if (pllm == 0) pllm = 1;
        return (inputClk / pllm) * plln / pllp;
    }
    default:
        return HSI_VALUE_HZ;
    }
}

static void SysTick_Init(void) {
    uint32 sysClkHz = Rcc_GetSystemClock();
    SYSTICK_RVR = (sysClkHz / 1000UL) - 1UL;
    SYSTICK_CVR = 0;
    SYSTICK_CSR = SYSTICK_CSR_CLKSOURCE
                | SYSTICK_CSR_TICKINT
                | SYSTICK_CSR_ENABLE;
}

void SysTick_Handler(void) {
    sysTickMs++;
}

/* ================================================================== */
/*  Independent Watchdog (IWDG)                                        */
/* ================================================================== */
#define IWDG_BASE            0x40003000UL
#define IWDG_KR              (*(volatile uint32 *)(IWDG_BASE + 0x00))
#define IWDG_PR              (*(volatile uint32 *)(IWDG_BASE + 0x04))
#define IWDG_RLR             (*(volatile uint32 *)(IWDG_BASE + 0x08))
#define IWDG_SR              (*(volatile uint32 *)(IWDG_BASE + 0x0C))

static void Iwdg_Init(void) {
    IWDG_KR  = 0x5555;
    IWDG_PR  = 4;
    IWDG_RLR = 2000;
    IWDG_KR  = 0xAAAA;
    IWDG_KR  = 0xCCCC;
}

static void Iwdg_Refresh(void) {
    IWDG_KR = 0xAAAA;
}

/* ================================================================== */
/*  System Control Block                                              */
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
    SCB_SHPR3 |= ((uint32)PRIO_SYSTICK << 24);

    /* Set peripheral priorities from Board_Config.h */
    Nvic_SetPriority(IRQ_SPI1,   PRIO_SPI);
    Nvic_SetPriority(IRQ_USART1, PRIO_USART);
    Nvic_SetPriority(28,         PRIO_TIMERS); /* TIM2 */
    Nvic_SetPriority(29,         PRIO_TIMERS); /* TIM3 */
    Nvic_SetPriority(30,         PRIO_TIMERS); /* TIM4 */
    Nvic_SetPriority(50,         PRIO_TIMERS); /* TIM5 */

    Rcc_Enable(CABIN_BTN_RCC);
    Rcc_Enable(EMERG_BTN_RCC);
    Rcc_Enable(FLOOR_SENS_RCC);
    Rcc_Enable(SPI_GPIO_RCC);      /* GPIOA for SPI1 pins (PA4-PA7) + UART (PA9-PA10) */
#if IS_MASTER_BOARD
    Rcc_Enable(HALL_BTN_RCC);
#endif
    Rcc_Enable(RCC_SYSCFG);
    Rcc_Enable(RCC_TIM2);
    Rcc_Enable(MOTOR_PWM_RCC);
    Rcc_Enable(TELEMETRY_TIMER == TIMER4 ? RCC_TIM4 : RCC_TIM2);
    Rcc_Enable(DOOR_TIMER == TIMER5 ? RCC_TIM5 : RCC_TIM2);
    Rcc_Enable(UART_RCC_ID);
    Rcc_Enable(SPI_RCC_ID);
    *((volatile uint32 *)0x40023830UL) |= (1UL << 22);

    /* --- SysTick — 1 ms global tick --- */
    SysTick_Init();

    /* --- USART1 --- */
    Usart1_Init();

    /* --- PWM motor LED (TIM3 CH1 on PC6, AF2) --- */
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

    /* --- Buttons & sensors (EXTI) --- */
    Buttons_Init(&localElev);

#if IS_MASTER_BOARD
    Dispatcher_Init();
#endif

    /* --- DMA for USART1 TX --- */
    Dma_Usart1TxInit();

    /* --- Telemetry --- */
    Telemetry_Init();

    /* Initialise last-good-rx timestamp */
    spiLastGoodRxTick = sysTickMs;

#if IS_MASTER_BOARD
    /* --- 50 ms SPI exchange timer --- */
    Timer_DelayMsAsync(SPI_EXCHANGE_TIMER, SPI_EXCHANGE_PERIOD_MS,
                       SpiExchangeTimerCb);
#else
    /* Slave: pre-load TX and start ISR-driven reception */
    BuildLocalFrame();
    Spi_SlavePreload(SPI_PERIPHERAL, spiTxBuf, SPI_FRAME_LEN);
    Spi_SlaveStartAsync(SPI_PERIPHERAL, SlaveRxCallback, spiRxBuf, SPI_FRAME_LEN);
#endif

    /* Independent Watchdog */
    Iwdg_Init();

    Usart1_TransmitString("\r\n=== Elevator System Started ===\r\n");
}

/* ================================================================== */
/*  MAIN                                                               */
/* ================================================================== */

int main(void) {

    System_Init();

    while (1) {

        /* ======================================================== */
        /*  STEP 1: RUN LOCAL ELEVATOR FSM                           */
        /*  Consumes floor sensor events, produces state transitions.*/
        /*  Must run BEFORE BuildLocalFrame so frame carries fresh   */
        /*  state data.                                              */
        /* ======================================================== */
        Elevator_Run(&localElev);

#if IS_MASTER_BOARD
        /* ======================================================== */
        /*  STEP 2: RUN DISPATCHER (Master only)                     */
        /*  Assigns pending hall calls to elevators A/B.             */
        /*  Runs AFTER FSM so elevator states are current.           */
        /* ======================================================== */
        Dispatcher_Run(&localElev, &remoteElev,
                       spiCommFault ? TRUE : FALSE);

        /* ======================================================== */
        /*  STEP 3: SPI EXCHANGE                                     */
        /*  BuildLocalFrame runs AFTER FSM+Dispatcher = fresh data. */
        /*  Non-blocking state machine sends one byte per iteration. */
        /* ======================================================== */
        {
            static uint8 spiSmState = 0;  /* 0=idle, 1=sending, 2=wait_bsy, 3=write_next */
            static uint8 spiSmIdx   = 0;
            SpiType *spiHw = (SpiType *)0x40013000UL; /* SPI1 */

            switch (spiSmState) {
            case 0: /* IDLE — wait for exchange trigger */
                if (spiExchangeFlag) {
                    spiExchangeFlag = 0;
                    /* [REDESIGN] BuildLocalFrame AFTER FSM ran */
                    BuildLocalFrame();
                    /* Clear OVR */
                    { volatile uint8 d = (uint8)spiHw->DR; d = (uint8)spiHw->SR; (void)d; }
                    Spi_CsLow();
                    spiSmIdx = 0;
                    spiHw->DR = spiTxBuf[0];
                    spiSmState = 1;
                }
                break;

            case 1: /* READ — check if current byte finished */
                if (READ_BIT(spiHw->SR, 0U)) { /* RXNE */
                    spiRxBuf[spiSmIdx] = (uint8)spiHw->DR;
                    spiSmIdx++;
                    if (spiSmIdx < SPI_FRAME_LEN) {
                        spiSmState = 3;
                    } else {
                        spiSmState = 2;
                    }
                }
                break;

            case 2: /* WAIT_BSY — deassert CS when bus is idle */
                if (!READ_BIT(spiHw->SR, 7U)) { /* BSY=0 */
                    Spi_CsHigh();
                    spiRxReady = 1;
                    spiSmState = 0;
                }
                break;

            case 3: /* WRITE_NEXT — load next byte (one iteration later) */
                spiHw->DR = spiTxBuf[spiSmIdx];
                spiSmState = 1;
                break;
            }
        }

        /* ======================================================== */
        /*  STEP 4: PROCESS SPI RESPONSE                             */
        /* ======================================================== */
        if (spiRxReady) {
            spiRxReady = 0;

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
        }

        /* Timeout check (no exchange received in time) */
        if (!spiRxReady) {
            uint32 elapsed = sysTickMs - spiLastGoodRxTick;
            if (elapsed >= SPI_TIMEOUT_MS) {
                if (!spiCommFault) {
                    spiLastRxSeq = 0xFF;
                }
                spiCommFault = 1;
                /* Re-run dispatcher in fault mode */
                Dispatcher_Run(&localElev, &remoteElev, TRUE);
            }
        }
#else
        /* ======================================================== */
        /*  SLAVE: ISR-driven SPI receive                            */
        /* ======================================================== */

        /* Process completed frame */
        if (spiRxReady) {
            spiRxReady = 0;
            if (ParseRemoteFrame()) {
                spiCommFault = 0;
                spiLastGoodRxTick = sysTickMs;

                if (localElev.state == ELEV_INDEPENDENT) {
                    Elevator_ExitIndependentMode(&localElev);
                }
            }

            /* [REDESIGN] Build frame AFTER FSM ran — fresh state */
            BuildLocalFrame();
            Spi_SlavePreload(SPI_PERIPHERAL, spiTxBuf, SPI_FRAME_LEN);
        }

        /* Slave comm-fault: SysTick-based timeout */
        {
            uint32 elapsed = sysTickMs - spiLastGoodRxTick;
            if (elapsed >= SPI_TIMEOUT_MS) {
                if (!spiCommFault) {
                    spiCommFault = 1;
                    spiLastRxSeq = 0xFF;
                    Elevator_EnterIndependentMode(&localElev);
                }
                /* While fault persists, keep assigned calls cleared */
                localElev.assignedCalls = 0;
            } else {
                spiCommFault = 0;
            }
        }
#endif

        /* ======================================================== */
        /*  STEP 5: TELEMETRY                                        */
        /* ======================================================== */
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

        /* ======================================================== */
        /*  STEP 6: WATCHDOG REFRESH                                 */
        /* ======================================================== */
        Iwdg_Refresh();
    }

    return 0;
}
