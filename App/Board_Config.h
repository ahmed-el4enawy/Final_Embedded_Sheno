/**
 * Board_Config.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Change IS_MASTER_BOARD to 0 when compiling for Board B (Slave).
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ============================================================ */
/*  >>> SET TO 1 FOR MASTER (Board A), 0 FOR SLAVE (Board B) << */
/* ============================================================ */
#define IS_MASTER_BOARD   1

/* ============================================================ */
/*  Number of floors                                            */
/* ============================================================ */
#define NUM_FLOORS        4

/* ============================================================ */
/*  Pin Mapping  (shared between Master and Slave)              */
/* ============================================================ */

/* --- Cabin floor buttons (internal requests) --- */
#define CABIN_BTN_PORT         GPIO_A
#define CABIN_BTN_EXTI_PORT    EXTI_PORT_A
#define CABIN_BTN_PIN_F1       0U
#define CABIN_BTN_PIN_F2       1U
#define CABIN_BTN_PIN_F3       2U
#define CABIN_BTN_PIN_F4       3U

/* --- Emergency stop button --- */
#define EMERG_BTN_PORT         GPIO_B
#define EMERG_BTN_EXTI_PORT    EXTI_PORT_B
#define EMERG_BTN_PIN          10U

/* --- Floor position sensors --- */
#define FLOOR_SENS_PORT        GPIO_C
#define FLOOR_SENS_EXTI_PORT   EXTI_PORT_C
#define FLOOR_SENS_PIN_F1      11U
#define FLOOR_SENS_PIN_F2      12U
#define FLOOR_SENS_PIN_F3      13U
#define FLOOR_SENS_PIN_F4      14U

/* --- PWM motor-simulation LED  (TIM3 CH3 on PB0, AF2) --- */
#define MOTOR_LED_PORT         GPIO_B
#define MOTOR_LED_PIN          0U
#define MOTOR_LED_AF           GPIO_AF2
#define MOTOR_PWM_TIMER        TIMER3
#define MOTOR_PWM_CHANNEL      PWM_CHANNEL_3
#define MOTOR_PWM_PSC          15U     /* 16 MHz / 16 = 1 MHz tick  */
#define MOTOR_PWM_ARR          99U     /* 1 MHz / 100 = 10 kHz PWM  */

/* --- SPI1 pins (IPC link) --- */
#define SPI_PORT               GPIO_A
#define SPI_SCK_PIN            5U
#define SPI_MISO_PIN           6U
#define SPI_MOSI_PIN           7U
#define SPI_CS_PORT            GPIO_A
#define SPI_CS_PIN             4U       /* Master drives CS manually */
#define SPI_AF                 GPIO_AF5

/* --- USART1 pins (telemetry to PC) --- */
#define UART_PORT              GPIO_A
#define UART_TX_PIN            9U
#define UART_RX_PIN            10U
#define UART_AF                GPIO_AF7

/* --- Hallway call buttons (Master only) --- */
#if IS_MASTER_BOARD
#define HALL_BTN_PORT          GPIO_B
#define HALL_BTN_EXTI_PORT     EXTI_PORT_B
#define HALL_BTN_PIN_U1        4U       /* Up   at floor 1 */
#define HALL_BTN_PIN_D2        5U       /* Down at floor 2 */
#define HALL_BTN_PIN_U2        6U       /* Up   at floor 2 */
#define HALL_BTN_PIN_D3        7U       /* Down at floor 3 */
#define HALL_BTN_PIN_U3        8U       /* Up   at floor 3 */
#define HALL_BTN_PIN_D4        9U       /* Down at floor 4 */
#endif

/* ============================================================ */
/*  Timer allocation                                            */
/* ============================================================ */
#define SPI_EXCHANGE_TIMER     TIMER2   /* 50 ms periodic   (Master) */
#define TELEMETRY_TIMER        TIMER4   /* 500 ms periodic           */
#define DOOR_TIMER             TIMER5   /* 3 s one-shot door close   */

/* ============================================================ */
/*  Timing constants (ms)                                       */
/* ============================================================ */
#define SPI_EXCHANGE_PERIOD_MS   50U
#define TELEMETRY_PERIOD_MS      500U
#define DOOR_OPEN_TIME_MS        3000U
#define SPI_TIMEOUT_MS           200U    /* comm-fault threshold     */

/* ============================================================ */
/*  PWM duty percentages for motor simulation                   */
/* ============================================================ */
#define MOTOR_DUTY_STOP          0U
#define MOTOR_DUTY_SLOW          20U
#define MOTOR_DUTY_FULL          100U

#endif /* BOARD_CONFIG_H */
