# Memory Footprint & Resource Usage

This document provides an estimate of the Flash and RAM consumption for the bare-metal elevator firmware.

## Estimated Usage (STM32F401xE)

| Resource | Capacity | Used (Estimated) | Usage % |
|----------|----------|------------------|---------|
| **Flash** | 512 KB | ~18 KB | ~3.5% |
| **SRAM** | 96 KB | ~4 KB | ~4.1% |

## RAM Breakdown

1. **Static Allocation (~1.5 KB)**:
   * Peripheral driver control blocks (DMA, SPI, USART).
   * Global FSM contexts (local + remote elevator states).
   * Telemetry buffers (DMA-aligned).
   * Ring buffers for USART RX.
2. **Stack (~2 KB)**:
   * Allocated for local variables within FSM and Dispatcher functions.
   * Interrupt service routine (ISR) context saving.
3. **Heap (0 KB)**:
   * **No dynamic memory allocation (`malloc`) is used.** This ensures deterministic behavior and prevents memory fragmentation/leaks, which is a critical requirement for safety-critical embedded systems.

## Optimization Techniques

* **Register-Level Drivers**: By avoiding the STM32 HAL (Hardware Abstraction Layer), the Flash footprint is reduced by 60-80%, and execution speed is significantly increased due to the absence of thick function-call overhead.
* **Non-Blocking Logic**: Avoiding `delay()` loops ensures that CPU cycles are never wasted. The system remains in a low-power "wait for interrupt" state effectively via the main loop's high-frequency polling.
* **Bit-Packed Protocol**: The SPI frame is packed into 8 bytes using bit-fields and nibbles, minimizing bus time and RAM storage for IPC data.
