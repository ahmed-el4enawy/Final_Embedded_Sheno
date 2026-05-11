# Distributed Dual-Elevator Control System
[![Target](https://img.shields.io/badge/Target-STM32F401-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f401.html)
[![Language](https://img.shields.io/badge/Language-C-green.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Architecture](https://img.shields.io/badge/Architecture-Bare--Metal-red.svg)](#technical-highlights)
[![IPC](https://img.shields.io/badge/IPC-Full--Duplex%20SPI-orange.svg)](Docs/spi_frame.md)

> **Advanced real-time firmware implementing a collaborative, distributed elevator system across dual STM32F401 microcontrollers.**

---

## 👥 Engineering Team (Team 13)

| Name | Primary Responsibility |
|------|------------------------|
| **Ahmed Salah Elshenawy** | Systems Architecture & Lead |
| **Abdullah M. Khalifa** | Firmware & Driver Development |
| **Alhussien Ayman Hanafy** | Hardware Integration & FSM |
| **Mohamed Elsayed Attallah** | Telemetry & Validation |

---

## 🚀 Technical Highlights

This project serves as a comprehensive demonstration of professional-grade embedded systems engineering, featuring:

- **Register-Level Driver Suite**: 100% bare-metal implementation for SPI, I2C, USART (DMA), TIM, PWM, and EXTI. No HAL or Middleware utilized.
- **Asynchronous Non-Blocking Design**: Architecture is entirely event-driven. The main execution loop is guaranteed freeze-free through the use of non-blocking timing intervals and ISR-flag synchronization.
- **Inter-Processor Communication (IPC)**: A robust, full-duplex SPI protocol featuring XOR-checksum validation, rolling sequence counters for stale-frame rejection, and 1ms-resolution timeout detection.
- **Intelligent Dispatcher**: A directional-optimisation scoring algorithm that dynamically assigns hall calls based on elevator state, proximity, and movement vectors.
- **Industrial Safety Standards**: Features an Independent Hardware Watchdog (IWDG) and automatic "Independent Mode" failover for graceful degradation during communication loss.

---

## 🏗️ System Architecture

### Hardware Interconnect

```text
       BOARD A (MASTER)                           BOARD B (SLAVE)
   ┌───────────────────────┐                  ┌───────────────────────┐
   │    Elevator A FSM     │   SPI1 (Duplex)  │    Elevator B FSM     │
   │  + Dispatcher Logic   │<────────────────>│  + Failover Logic     │
   └──────────┬────────────┘   50ms Period    └──────────┬────────────┘
              │                                          │
      Inputs: Hall Calls                         Inputs: Cabin Calls
              Cabin Calls                                Sensors
              Sensors
```

### Pin Mapping (Proteus Logic)

| Subsystem | Signal | Pin | Peripheral |
|-----------|--------|-----|------------|
| **IPC Link** | MOSI/MISO/SCK | PA5-PA7 | SPI1 (DIV64) |
| **Motor Drive** | PWM Output | **PC6** | TIM3 CH1 (AF2) |
| **Sensors** | Floor Position | **PC0-PC3** | EXTI (Priority 1) |
| **Emergency** | Stop Trigger | **PB10** | EXTI (Priority 0) |
| **Telemetry** | Status TX | **PA9** | USART1 (9600, DMA) |
| **UI Inputs** | Cabin Buttons | **PA0-PA3** | Polled (10ms) |

---

## ⚙️ Core Engineering Details

### 1. PWM Motor Simulation
Motor speed ramping is simulated via LED intensity using a **10 kHz** PWM signal.
- **Math**: `f = HSI(16MHz) / [(PSC+1) * (ARR+1)]`
- **Config**: `PSC=15`, `ARR=99` → 1 MHz tick with 1% duty resolution.
- **Ramping**: Smooth 0% → 20% → 100% transitions executed via 20ms asynchronous steps.

### 2. Deterministic IPC Protocol
Communication occurs in fixed 8-byte frames every 50ms to ensure predictable CPU load.
- **Header**: `0xA5` sync byte.
- **Integrity**: Per-frame XOR checksum validation.
- **State**: Encapsulated FSM state, current floor, and directional intent.

### 3. Register Base Addresses (Memory Map)
| Block | Address | Block | Address |
|-------|---------|-------|---------|
| **RCC** | `0x40023800` | **USART1** | `0x40011000` |
| **GPIOA**| `0x40020000` | **DMA2** | `0x40026400` |
| **TIM3** | `0x40000400` | **NVIC** | `0xE000E100` |

---

## 📂 Repository Organization

```text
.
├── src/main.c              # System Entry & Task Scheduler
├── App/                    # Application Logic (FSM, Dispatcher, Protocol)
├── Docs/                   # Technical Specs (FSM, SPI, Timing)
├── Gpio, Spi, Timer...     # Register-Level Peripheral Drivers (Root)
└── CMakeLists.txt          # Professional Build Configuration
```

---

## 🔧 Build & Deployment

### Compilation
The project utilizes CMake for cross-compilation with the ARM GCC toolchain.
```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-gcc-toolchain.cmake ..
make
```

### Loading to Proteus
1. Build the project to generate `stm32-template.elf`.
2. In Proteus, double-click each STM32F401 chip and link the `.elf` file.
3. Configure the **Virtual Terminal** to **9600 Baud** to monitor telemetry.

---

## 📊 Compliance & Quality
- **FSM Implementation**: `App/Elevator_FSM.c`
- **SPI Definitions**: `Docs/spi_frame.md`
- **Timing Rationale**: `Docs/timing.md`
- **Atomic Access**: All shared contexts protected via `Critical.h` (PRIMASK).

---
*Developed for the Embedded Systems Final Evaluation. All rights reserved © Team 13.*
