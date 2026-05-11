# FSM State Transition Table

| Current State | Event | Next State | Action |
|---------------|-------|------------|--------|
| **IDLE** | `Elevator_AddCabinRequest` / `assignedCalls` | **IDLE** | Request bits set in mask |
| **IDLE** | `currentFloor` is requested | **DOORS_OPEN** | Stop motor, start 3s door timer |
| **IDLE** | Request above `currentFloor` | **MOVING_UP** | Ramp motor UP |
| **IDLE** | Request below `currentFloor` | **MOVING_DOWN** | Ramp motor UP |
| **MOVING_UP** | `floorReached` (target floor) | **ARRIVING** | Ramp motor to SLOW speed |
| **MOVING_UP** | Top floor reached | **ARRIVING** | Ramp motor to SLOW speed |
| **MOVING_DOWN** | `floorReached` (target floor) | **ARRIVING** | Ramp motor to SLOW speed |
| **MOVING_DOWN** | Bottom floor reached | **ARRIVING** | Ramp motor to SLOW speed |
| **ARRIVING** | At target floor | **DOORS_OPEN** | Stop motor, start 3s door timer, clear request |
| **DOORS_OPEN** | 3s timer expires | **DOOR_CLOSING** | Set doorOpen = 0 |
| **DOOR_CLOSING** | Requests in current dir | **MOVING_UP/DN** | Continue path, ramp motor |
| **DOOR_CLOSING** | No more requests | **IDLE** | Stay at floor |
| **ANY** | `EmergencyStop` (EXTI) | **EMERGENCY_STOP** | INSTANT stop (bypass ramp), set flag |
| **EMERGENCY_STOP** | `EmergencyClear` | **IDLE** | Clear flag, stay stopped |
| **IDLE** | SPI Comm Fault (Slave) | **INDEPENDENT** | Clear assignedCalls, report as Emergency |
| **INDEPENDENT** | `cabinRequest` added | **IDLE** | Service cabin-only requests |
| **INDEPENDENT** | SPI Link Restored | **IDLE** | Resume normal operation |

## Transition Logic Notes

1. **Non-Blocking Ramping**: The FSM does not block during motor speed changes. It sets a `targetDuty` and the `RampTick` (called every main loop) increments/decrements the PWM duty cycle until it matches the target.
2. **Atomic Transitions**: The FSM is the sole owner of state transitions. Asynchronous events (Timer, SPI RX) only set flags or update data; the FSM checks these synchronously in `main.c`'s `while(1)` loop.
3. **Emergency Preemption**: The `Elevator_Run` function checks the `emergencyStop` flag at the very beginning of its execution, ensuring that an emergency state overrides any other FSM logic immediately.
