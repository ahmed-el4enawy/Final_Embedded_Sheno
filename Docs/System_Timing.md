# System Timing & Latency

This table summarizes the critical timing intervals and latency guarantees of the firmware.

| Operation | Interval / Latency | Implementation | Purpose |
|-----------|--------------------|----------------|---------|
| **Global Tick** | 1 ms | SysTick ISR | Timebase for all non-blocking logic. |
| **SPI Exchange** | 50 ms | TIM2 ISR (Master) | Synchronizes state between Board A and Board B. |
| **SPI Timeout** | 200 ms | SysTick Comparison | Detected after 4 missing frames (4 * 50ms). |
| **Button Debounce**| 50 ms | SysTick Timestamp | Software filtering of EXTI signal noise. |
| **PWM Ramp Step** | 20 ms | SysTick Timestamp | 5% duty change every step for smooth motor simulation. |
| **Telemetry** | 500 ms | TIM4 ISR / DMA | Periodic status reporting to UART terminal. |
| **Door Timer** | 3000 ms | TIM5 One-Shot | Duration the doors remain open at a floor. |
| **IWDG Timeout** | 4000 ms | Hardware Watchdog | Failsafe reset if the main loop hangs. |

## Timing Rationale

* **SPI 50ms**: Provides 20 exchanges per second, which feels "real-time" to a user while keeping bus utilization low.
* **Debounce 50ms**: Standard value for tactile switches to ignore contact bounce while remaining responsive to rapid human presses.
* **SPI Timeout 200ms**: Chosen to be 4x the exchange period to prevent "flickering" faults during minor electrical noise, while still reacting fast enough to clear the bus for independent mode.
* **Ramp 20ms**: Creates a 400ms full-scale ramp (0 to 100%), which is visually smooth on an LED without making the elevator feel sluggish.
