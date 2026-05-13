/**
 * Board_Config.h
 *
 *  Centralized hardware configuration for the Dual-Elevator Project.
 *  All pin assignments, peripheral selections, and timing constants
 *  are defined here to match the Proteus schematic exactly.
 *
 *  [PROTEUS MAPPING - BOARD A (MASTER)]
 *  - SPI1: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4
 *  - UART1: TX=PA9, RX=PA10
 *  - Cabin: PA0-PA3
 *  - Hall: PB4-PB9
 *  - Emergency: PB10
 *  - Floor: PC0-PC3
 *  - PWM: PC6 (TIM3 CH1)
 *
 *  [PROTEUS MAPPING - BOARD B (SLAVE)]
 *  - SPI1: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PB1
 *  - UART1: TX=PA9, RX=PA10
 *  - Cabin: PA0-PA3
 *  - Emergency: PB10
 *  - Floor: PC0-PC3
 *  - PWM: PC6 (TIM3 CH1)
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "Gpio.h"
#include "Rcc.h"
#include "Exti.h"

/* ========================================================================== */
/*  1. BOARD SELECTION                                                        */
/* ========================================================================== */
/* Set to 1 for Master (Board A), 0 for Slave (Board B) */
#ifndef IS_MASTER_BOARD
    #define IS_MASTER_BOARD  0
#endif

/* ========================================================================== */
/*  2. SPI CONFIGURATION (IPC Link)                                          */
/* ========================================================================== */
#define SPI_PERIPHERAL          SPI_1
#define SPI_RCC_ID              RCC_SPI1
#define SPI_GPIO_PORT           GPIO_A
#define SPI_GPIO_RCC            RCC_GPIOA
#define SPI_SCK_PIN             5U
#define SPI_MISO_PIN            6U
#define SPI_MOSI_PIN            7U
#define SPI_AF                  GPIO_AF5

/* Chip Select (CS/NSS)
 * SPI1_NSS = PA4 (AF5) on STM32F401.
 * Master: PA4 as GPIO output (manual CS toggle).
 * Slave:  PA4 as AF5 input  (hardware NSS — SPI peripheral
 *         auto-selects/deselects based on the pin level).
 *
 * [FIX — SPI Root Cause]
 * Previous config used PB1 for slave CS.  PB1 has NO SPI1 alternate-
 * function mapping on STM32F401, so the slave SPI peripheral never
 * saw the chip-select assertion.  Result: zero frames exchanged.
 */
#define SPI_CS_PORT             GPIO_A
#define SPI_CS_RCC              RCC_GPIOA
#define SPI_CS_PIN              4U

#define SPI_BAUD_DIVIDER        SPI_BAUD_DIV64

/* ========================================================================== */
/*  3. UART CONFIGURATION (Telemetry)                                        */
/* ========================================================================== */
#define UART_PERIPHERAL         USART_1
#define UART_RCC_ID             RCC_USART1
#define UART_GPIO_PORT          GPIO_A
#define UART_GPIO_RCC           RCC_GPIOA
#define UART_TX_PIN             9U
#define UART_RX_PIN             10U
#define UART_AF                 GPIO_AF7
#define UART_BAUD_RATE          115200

/* ========================================================================== */
/*  4. PWM CONFIGURATION (Motor Simulation LED)                               */
/* ========================================================================== */
/* PC6 -> TIM3 Channel 1, Alternate Function 2 */
#define MOTOR_PWM_TIMER         TIMER3
#define MOTOR_PWM_RCC           RCC_TIM3
#define MOTOR_PWM_PORT          GPIO_C
#define MOTOR_PWM_PORT_RCC      RCC_GPIOC
#define MOTOR_PWM_PIN           6U
#define MOTOR_PWM_CHANNEL       PWM_CHANNEL_1
#define MOTOR_PWM_AF            GPIO_AF2

/* 10 kHz PWM Calculation (16 MHz HSI) */
#define MOTOR_PWM_PSC           15U     /* 16 MHz / 16 = 1 MHz clock */
#define MOTOR_PWM_ARR           99U     /* 1 MHz / 100 = 10 kHz frequency */

/* Duty Cycle Constants */
#define MOTOR_DUTY_STOP         0U
#define MOTOR_DUTY_SLOW         20U
#define MOTOR_DUTY_FULL         100U

/* ========================================================================== */
/*  5. EXTI CONFIGURATION (Buttons & Sensors)                                 */
/* ========================================================================== */

/* Cabin Floor Buttons (PA0 - PA3) */
#define CABIN_BTN_PORT          GPIO_A
#define CABIN_BTN_RCC           RCC_GPIOA
#define CABIN_BTN_EXTI_PORT     EXTI_PORT_A
#define CABIN_BTN_PIN_F1        0U
#define CABIN_BTN_PIN_F2        1U
#define CABIN_BTN_PIN_F3        2U
#define CABIN_BTN_PIN_F4        3U

/* Emergency Stop Button (PB10) */
#define EMERG_BTN_PORT          GPIO_B
#define EMERG_BTN_RCC           RCC_GPIOB
#define EMERG_BTN_EXTI_PORT     EXTI_PORT_B
#define EMERG_BTN_PIN           10U

/* Floor Position Sensors (PC0 - PC3) */
#define FLOOR_SENS_PORT         GPIO_C
#define FLOOR_SENS_RCC          RCC_GPIOC
#define FLOOR_SENS_EXTI_PORT    EXTI_PORT_C
#define FLOOR_SENS_PIN_F1       0U
#define FLOOR_SENS_PIN_F2       1U
#define FLOOR_SENS_PIN_F3       2U
#define FLOOR_SENS_PIN_F4       3U

/* Hallway Call Buttons (PB4 - PB9, Master Only) */
#if IS_MASTER_BOARD
    #define HALL_BTN_PORT       GPIO_B
    #define HALL_BTN_RCC        RCC_GPIOB
    #define HALL_BTN_EXTI_PORT  EXTI_PORT_B
    #define HALL_BTN_PIN_U1     4U
    #define HALL_BTN_PIN_D2     5U
    #define HALL_BTN_PIN_U2     6U
    #define HALL_BTN_PIN_D3     7U
    #define HALL_BTN_PIN_U3     8U
    #define HALL_BTN_PIN_D4     9U
#endif

/* ========================================================================== */
/*  6. TIMING CONSTANTS                                                       */
/* ========================================================================== */
#define SPI_EXCHANGE_TIMER      TIMER2
#define SPI_EXCHANGE_PERIOD_MS  50U
#define SPI_TIMEOUT_MS          200U

#define TELEMETRY_TIMER         TIMER4
#define TELEMETRY_PERIOD_MS     100U

#define DOOR_TIMER              TIMER5
#define DOOR_OPEN_TIME_MS       500U

#define DEBOUNCE_MS             50U
#define RAMP_STEP_INTERVAL_MS   20U
#define RAMP_STEP_PERCENT       5U

/* ========================================================================== */
/*  7. IRQ PRIORITIES (0 = Highest)                                          */
/* ========================================================================== */
/* EXTI15_10 handles Emergency (PB10).
 * EXTI0-3 handles Cabin Buttons (PA0-3) AND Floor Sensors (PC0-3).
 * EXTI4, 9_5 handle Hall Buttons (PB4-9).
 */
#define PRIO_EMERGENCY          0U
#define PRIO_FLOOR_SENS         1U
#define PRIO_CABIN_BTN          2U
#define PRIO_HALL_BTN           3U
#define PRIO_SPI                4U
#define PRIO_USART              4U
#define PRIO_TIMERS             5U
#define PRIO_SYSTICK            15U

/* NVIC IRQ Numbers for STM32F401 */
#define IRQ_EXTI0               6U
#define IRQ_EXTI1               7U
#define IRQ_EXTI2               8U
#define IRQ_EXTI3               9U
#define IRQ_EXTI4               10U
#define IRQ_EXTI9_5             23U
#define IRQ_EXTI15_10           40U
#define IRQ_SPI1                35U
#define IRQ_USART1              37U

/* ========================================================================== */
/*  9. DMA CONFIGURATION (USART1 TX)                                         */
/* ========================================================================== */
#define UART_DMA_STREAM_ID      7U
#define UART_DMA_CHANNEL_ID     4U
#define UART_DMA_DR_ADDR        0x40011004UL  /* USART1->DR address */

/* ========================================================================== */
/*  10. HELPER MACROS                                                         */
/* ========================================================================== */
#define GPIO_PIN_MASK(pin)      (1UL << (pin))
#define NUM_FLOORS              4U

#endif /* BOARD_CONFIG_H */
