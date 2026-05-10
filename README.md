# 🏗️ Collaborative Dual-Elevator System over SPI IPC

> **A real-time, bare-metal firmware project implementing two cooperative elevators across independent STM32F4 Cortex-M4 microcontrollers, communicating over a full-duplex SPI inter-processor link.**

---

## 👥 Team 13

| Name | Role |
|------|------|
| **Ahmed Salah Elshenawy** | Team Lead |
| **Abdullah Mohammed Khalifa Mansour** | Firmware Engineer |
| **Alhussien Ayman Hanafy** | Firmware Engineer |
| **Mohamed Elsayed Attallah** | Firmware Engineer |

---

## 📋 Project Objective

Design and implement a **collaborative dual-elevator control system** using two independent STM32F401 microcontrollers (Cortex-M4) that coordinate in real-time to efficiently service 4 floors. The system demonstrates advanced embedded concepts including:

- **Bare-metal firmware** — No RTOS, no HAL; all peripherals driven by custom register-level drivers
- **Finite State Machines (FSM)** — Non-blocking, event-driven elevator control
- **SPI Inter-Processor Communication** — Full-duplex, checksummed 8-byte frame protocol
- **Directional Task Allocation** — Intelligent dispatcher algorithm for optimal call assignment
- **Fault Tolerance** — Automatic failover to independent mode on communication failure
- **PWM Speed Ramping** — Smooth, non-blocking motor speed transitions

---

## 🏗️ Hardware Architecture

```
┌───────────────────────────────────┐     SPI1 (Full Duplex)     ┌───────────────────────────────────┐
│          BOARD A (Master)         │◄──────────────────────────►│          BOARD B (Slave)           │
│          STM32F401RE              │   MOSI / MISO / SCK / CS   │          STM32F401RE              │
│                                   │                             │                                   │
│  ┌─────────────────────────────┐  │                             │  ┌─────────────────────────────┐  │
│  │ Elevator A FSM              │  │                             │  │ Elevator B FSM              │  │
│  │ + Dispatcher (scoring alg)  │  │                             │  │ + Independent Mode fallback │  │
│  └─────────────────────────────┘  │                             │  └─────────────────────────────┘  │
│                                   │                             │                                   │
│  Inputs:                          │                             │  Inputs:                          │
│   • 4× Cabin Buttons   (PA0-PA3) │                             │   • 4× Cabin Buttons   (PA0-PA3) │
│   • 6× Hallway Buttons (PB4-PB9) │                             │   • 1× Emergency Stop  (PB10)    │
│   • 1× Emergency Stop  (PB10)    │                             │   • 4× Floor Sensors   (PC11-14) │
│   • 4× Floor Sensors   (PC11-14) │                             │                                   │
│                                   │                             │  Outputs:                         │
│  Outputs:                         │                             │   • PWM Motor LED      (PB0)     │
│   • PWM Motor LED      (PB0)     │                             │   • UART Telemetry     (PA9)     │
│   • UART Telemetry     (PA9)     │                             │                                   │
└───────────────────────────────────┘                             └───────────────────────────────────┘
```

### Peripheral Summary

| Peripheral | Function | Configuration |
|-----------|----------|---------------|
| **SPI1** | Master ↔ Slave IPC | Full-duplex, 250 kHz (DIV64), 8-byte frames at 50 ms intervals |
| **TIM3 CH3** | PWM motor simulation LED | 10 kHz PWM, duty ramps: 0% → 20% → 100% |
| **TIM2** | SPI exchange timer (Master) | 50 ms one-shot, re-armed in ISR |
| **TIM4** | UART telemetry timer | 500 ms periodic |
| **TIM5** | Door open timer | 3000 ms one-shot |
| **SysTick** | Global 1 ms tick counter | Used for PWM ramping, SPI timeout, button debounce |
| **USART1** | Debug telemetry to PC | 115200 baud, DMA-backed TX |
| **EXTI** | All buttons & floor sensors | Edge-triggered, NVIC priority-managed |

---

## 📡 SPI Inter-Processor Communication (IPC)

### Frame Format (8 bytes, full-duplex)

| Byte | Field | Content |
|------|-------|---------|
| 0 | Header | `0xA5` — magic byte for frame synchronisation |
| 1 | State \| Floor | Upper nibble: FSM state (4-bit), Lower nibble: current floor (1–4) |
| 2 | Direction \| Cabin | Upper nibble: direction (`DIR_UP`/`DOWN`/`NONE`), Lower nibble: cabin request bitmask |
| 3 | Assigned Calls | Bitmask of hall calls assigned to THIS elevator |
| 4 | Hall Calls | Master→Slave: new hall call assignments for Slave |
| 5 | Flags | Bit 0: Emergency, Bit 1: Comm Fault, Bit 2: Door Open |
| 6 | Sequence | Rolling counter (0–255) for duplicate/freshness detection |
| 7 | Checksum | XOR of bytes 0–6 |

### Protocol Features

- **Non-blocking**: Master uses a 50 ms timer flag; Slave uses ISR-driven async reception
- **Integrity**: XOR checksum + header validation rejects corrupted frames
- **Freshness**: Sequence counter discards duplicate/stale frames
- **Fault Detection**: SysTick-based elapsed-time check (1 ms resolution, 200 ms threshold)
- **Graceful Degradation**: Slave enters `ELEV_INDEPENDENT` mode on comm fault; Master assumes all calls

---

## 🧠 Task Allocation Algorithm

The Master runs a **directional-optimisation scoring algorithm** on every 50 ms SPI cycle:

| Condition | Score | Description |
|-----------|-------|-------------|
| **Immediate** | `0` | Elevator is IDLE and already at the requested floor |
| **Perfect Match** | `1 + distance` | Moving toward the floor in the same direction as the call |
| **Idle Near** | `50 + distance` | Elevator is IDLE but at a different floor |
| **Doors Open/Closing** | `60 + distance` | Soon-to-be-idle, treat as idle with penalty |
| **Passed** | `100 + distance` | Same direction but already passed the floor |
| **Opposite Direction** | `-1` (DO NOT ASSIGN) | Moving away — call stays pending until path completes |
| **Emergency/Independent** | `-1` (DO NOT ASSIGN) | Elevator unavailable |

> **Key rule**: If both elevators score `-1`, the call remains in `pendingHallCalls` and is re-evaluated every cycle until one elevator becomes available. Completed assignments are automatically cleared when the elevator's pending-floors mask no longer includes the target floor.

---

## ⚙️ Software Engineering Requirements

### Finite State Machine

```
                    ┌──────────────────────────────────────┐
                    │          ELEV_EMERGENCY_STOP          │ ◄── EmergencyStop() from ANY state
                    └──────────────────────────────────────┘
                                    │ EmergencyClear()
                                    ▼
┌──────────┐   request   ┌───────────────┐   floor sensor   ┌────────────┐
│ ELEV_IDLE │────────────►│ ELEV_MOVING_  │─────────────────►│ELEV_ARRIVING│
│           │◄───────┐    │  UP / DOWN    │                  │ (ramp slow) │
└──────────┘        │    └───────────────┘                  └─────┬──────┘
                    │                                              │
                    │    ┌───────────────┐   timer expired   ┌─────▼──────┐
                    └────│ELEV_DOOR_     │◄──────────────────│ELEV_DOORS_ │
                         │  CLOSING      │                   │   OPEN     │
                         └───────────────┘                   └────────────┘

                    ┌──────────────────────────────────────┐
                    │        ELEV_INDEPENDENT (Slave)       │ ◄── SPI comm fault
                    │   Cabin-only service, no ext. calls   │
                    └──────────────────────────────────────┘
```

### Non-Blocking Design

- **Zero blocking delays** in the main loop — all timing uses hardware timer callbacks or SysTick timestamps
- SPI exchange, telemetry, and door timers are all ISR-flag-driven
- PWM speed ramp uses non-blocking 20 ms timestamp checks (5% per step)
- Button debounce uses non-blocking 50 ms timestamp checks per button

### Concurrency Protection

| Mechanism | Usage |
|-----------|-------|
| `volatile` qualifier | All ISR-shared flags: `spiExchangeFlag`, `spiRxReady`, `spiCommFault`, `sysTickMs`, all `ElevatorContext` fields, debounce timestamps |
| `Enter_Critical()` / `Exit_Critical()` | PRIMASK-based critical sections protecting SPI buffer R/W, FSM context updates, dispatcher state, and floor-request bitmask modifications |
| NVIC Priority Grouping | 4 preemption bits: Emergency (0) > Floor sensors (1) > Cabin buttons (2) > Hall buttons (3) |

### Button Debounce

All user-operated buttons (cabin, hallway, emergency) implement a **50 ms non-blocking software debounce** inside their EXTI ISR callbacks. Floor sensors are not debounced (they are hardware position sensors, not user-operated).

---

## 📂 Repository Structure

```
Final_Embedded_Sheno/
├── src/
│   └── main.c                  # Entry point, SysTick, SPI exchange, main loop
├── App/
│   ├── Elevator_FSM.c/h        # FSM implementation + PWM ramping
│   ├── Dispatcher.c/h          # Task allocation algorithm (Master only)
│   ├── Button_Handlers.c/h     # EXTI callbacks with debounce
│   ├── Spi_Protocol.c/h        # 8-byte frame pack/unpack/checksum
│   ├── Telemetry.c/h           # 500 ms UART status reports
│   └── Board_Config.h          # Pin mapping, timer allocation, timing constants
├── Spi/                        # SPI register-level driver
├── Timer/                      # GP Timer driver (async + OC toggle)
├── Pwm/                        # PWM driver (duty cycle control)
├── Gpio/                       # GPIO driver
├── Exti/                       # EXTI driver (edge-triggered interrupts)
├── Nvic/                       # NVIC driver (priority management)
├── Rcc/                        # RCC driver (clock enables)
├── Usart/                      # USART driver
├── Dma/                        # DMA driver (USART1 TX)
├── RingBuffer/                 # Ring buffer utility
├── Lib/
│   ├── Std_Types.h             # uint8, uint16, uint32, boolean, etc.
│   ├── Critical.h              # Enter_Critical() / Exit_Critical()
│   └── Bit_Operations.h        # SET_BIT, CLEAR_BIT, READ_BIT macros
├── CMakeLists.txt              # Build configuration
└── README.md                   # This file
```

---

## 🔧 Build & Flash

### Prerequisites

- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- CMake ≥ 3.16
- OpenOCD or ST-Link utility for flashing

### Building

```bash
# Configure for Master (Board A)
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain.cmake ..
make

# For Slave (Board B): change IS_MASTER_BOARD to 0 in App/Board_Config.h
# then rebuild
```

### Flashing

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program build/Final_Embedded.elf verify reset exit"
```

---

## 📊 UART Telemetry Output

Connect a USB-UART adapter to PA9 (TX) at **115200 baud** to see real-time status:

```
=== Elevator System Started ===
[TEL] A:IDLE F1 | B:IDLE F1 | SPI:OK | Hall:0x00
[TEL] A:MOV_UP F1 | B:IDLE F1 | SPI:OK | Hall:0x01
[TEL] A:ARRIVE F3 | B:MOV_DN F4 | SPI:OK | Hall:0x00
[TEL] A:DOOR_O F3 | B:ARRIVE F2 | SPI:OK | Hall:0x00
```

---

## 📜 License

This project was developed as a final project for the Embedded Systems course. All rights reserved by Team 13.
