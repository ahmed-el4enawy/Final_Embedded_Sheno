# Interrupt Priority Mapping

This document details the NVIC priority configuration of the Dual-Elevator system. Following the safety-critical requirements of the project, **Emergency Stop** is assigned the highest possible priority to ensure it preempts all other system activities.

## NVIC Priority Table

The system uses 4-bit priority grouping (16 levels, 0 is highest).

| Interrupt Source | NVIC IRQ | Group Priority | Role | Rationale |
|------------------|----------|----------------|------|-----------|
| **Emergency Button** | EXTI15_10 (40) | **0** | Critical Stop | Must preempt motor movement and communication immediately. |
| **Floor Sensors** | EXTI0-3 / 15_10 | **1** | Safety/Position | Ensures floor arrivals are recorded even during heavy CPU load. |
| **Cabin Buttons** | EXTI0-3 (6-9) | **2** | User Input | High responsiveness for passengers inside the cabin. |
| **Hall Buttons** | EXTI4 / 9_5 | **3** | User Input | Responsiveness for hallway passengers. |
| **SPI1** | SPI1 (35) | **4** | Communication | Non-blocking IPC. Lower than sensors to avoid missing floor edges. |
| **USART1** | USART1 (37) | **4** | Telemetry | Status reporting. Shared with SPI to prevent UART from starving IPC. |
| **Timers** | TIM2-5 | **5** | Application Timing | FSM intervals, door timing, and telemetry gating. |
| **SysTick** | SCB SHPR3 | **15** | Global Timebase | Lowest priority. Increments 1ms counter but never blocks ISRs. |

## Preemption Logic

1.  **Safety First**: If the Emergency Button (Priority 0) is pressed while the elevator is processing a Floor Sensor arrival (Priority 1), the Emergency ISR will **preempt** the sensor logic, instantly cutting motor power via the register-level override.
2.  **Communication Isolation**: SPI and UART interrupts (Priority 4) are kept below physical sensors and buttons. This prevents a "babbling brook" failure on the communication bus from causing the elevator to miss a floor sensor or ignore an emergency press.
3.  **Non-Blocking Guarantee**: Since all interrupts are non-blocking (performing only flag updates or quick register writes), the latency for the lowest priority interrupt (SysTick) remains well within the 1ms jitter budget.

## EXTI Mapping Detail

| EXTI Line | Pin | Function |
|-----------|-----|----------|
| EXTI0     | PA0 | Cabin Floor 1 |
| EXTI1     | PA1 | Cabin Floor 2 |
| EXTI2     | PA2 | Cabin Floor 3 |
| EXTI3     | PA3 | Cabin Floor 4 |
| EXTI4     | PB4 | Hallway U1 |
| EXTI9_5   | PB5-9 | Hallway D2-D4 |
| EXTI15_10 | PB10 | **Emergency Stop** |
| EXTI15_10 | PC11-14 | Floor Sensors 1-4 |
