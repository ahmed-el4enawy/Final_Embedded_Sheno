# Register Map — STM32F401xE Peripheral Addresses

> **Submission Requirement (Section 6):** "Register Map: A table of all used peripherals and their base addresses."

## Core Peripherals

| Peripheral | Base Address | Bus | Used Registers | Purpose in Project |
|------------|-------------|-----|----------------|-------------------|
| **RCC** | `0x4002 3800` | AHB1 | `CR`, `CFGR`, `PLLCFGR`, `AHB1ENR`, `APB1ENR`, `APB2ENR` | Clock enable for all peripherals, PLL configuration |
| **GPIOA** | `0x4002 0000` | AHB1 | `MODER`, `OTYPER`, `PUPDR`, `IDR`, `ODR`, `AFRL`, `AFRH` | SPI1 (PA4–PA7), USART1 (PA9–PA10), Cabin buttons (PA0–PA3) |
| **GPIOB** | `0x4002 0400` | AHB1 | `MODER`, `OTYPER`, `PUPDR`, `IDR`, `ODR`, `AFRL` | PWM LED (PB0), Hall buttons (PB4–PB9), Emergency (PB10) |
| **GPIOC** | `0x4002 0800` | AHB1 | `MODER`, `PUPDR`, `IDR` | Floor sensors (PC11–PC14) |

## Interrupt Controllers

| Peripheral | Base Address | Bus | Used Registers | Purpose in Project |
|------------|-------------|-----|----------------|-------------------|
| **EXTI** | `0x4001 3C00` | APB2 | `IMR`, `RTSR`, `FTSR`, `PR` | Edge-triggered interrupts for all 11 buttons + 4 floor sensors |
| **SYSCFG** | `0x4001 3800` | APB2 | `EXTICR1`–`EXTICR4` | EXTI source port selection (PA/PB/PC → EXTI line) |
| **NVIC** | `0xE000 E100` | PPB | `ISER[0..7]`, `IPR[0..59]` | IRQ enable + priority for SPI1, USART1, TIM2–5, EXTI lines |
| **SysTick** | `0xE000 E010` | PPB | `CSR`, `RVR`, `CVR` | 1 ms global tick (PWM ramp, SPI timeout, debounce) |
| **SCB** | `0xE000 ED00` | PPB | `AIRCR` | Priority grouping (4 bits preemption, 0 sub-priority) |

## Timers

| Peripheral | Base Address | Bus | Used Registers | Purpose in Project |
|------------|-------------|-----|----------------|-------------------|
| **TIM2** | `0x4000 0000` | APB1 | `PSC`, `ARR`, `DIER`, `SR`, `CR1` | 50 ms periodic SPI exchange trigger (Master only) |
| **TIM3** | `0x4000 0400` | APB1 | `PSC`, `ARR`, `CCR3`, `CCMR2`, `CCER`, `CR1` | 10 kHz PWM motor-simulation LED (CH3, PB0, AF2) |
| **TIM4** | `0x4000 0800` | APB1 | `PSC`, `ARR`, `DIER`, `SR`, `CR1` | 500 ms periodic telemetry report trigger |
| **TIM5** | `0x4000 0C00` | APB1 | `PSC`, `ARR`, `DIER`, `SR`, `CR1` | 3 s one-shot door-open timer |

## Communication Peripherals

| Peripheral | Base Address | Bus | Used Registers | Purpose in Project |
|------------|-------------|-----|----------------|-------------------|
| **SPI1** | `0x4001 3000` | APB2 | `CR1`, `CR2`, `SR`, `DR` | Full-duplex IPC link (8-byte checksummed frames, 50 ms period) |
| **USART1** | `0x4001 1000` | APB2 | `BRR`, `CR1`, `CR3`, `SR`, `DR` | 9600-baud telemetry to PC (DMA-backed TX) |
| **DMA2** | `0x4002 6400` | AHB1 | `HISR`, `HIFCR`, Stream7: `CR`, `NDTR`, `PAR`, `M0AR`, `FCR` | Stream 7, Ch4 → USART1_TX (zero-CPU telemetry output) |

## NVIC Priority Assignment

| IRQ # | Vector Name | Priority | Source |
|-------|------------|----------|--------|
| 40 | `EXTI15_10_IRQHandler` | **0** (Highest) | Emergency stop (PB10) + Floor sensors (PC11–14) |
| 6 | `EXTI0_IRQHandler` | 2 | Cabin button F1 (PA0) |
| 7 | `EXTI1_IRQHandler` | 2 | Cabin button F2 (PA1) |
| 8 | `EXTI2_IRQHandler` | 2 | Cabin button F3 (PA2) |
| 9 | `EXTI3_IRQHandler` | 2 | Cabin button F4 (PA3) |
| 10 | `EXTI4_IRQHandler` | 3 | Hall button U1 (PB4) |
| 23 | `EXTI9_5_IRQHandler` | 3 | Hall buttons D2–D4 (PB5–PB9) |
| 35 | `SPI1_IRQHandler` | Default (4) | SPI1 RXNE/TXE (non-blocking Master + Slave) |
| 37 | `USART1_IRQHandler` | Default (4) | USART1 RXNE (ring buffer reception) |
