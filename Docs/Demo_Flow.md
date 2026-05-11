# Live Demo Flow & Sequence

This document outlines the recommended sequence for demonstrating the system's features and robustness during a TA presentation.

## Stage 1: Basic Operation
1. **Normal Call**: Press a cabin button (F4) on Board A.
   * *Expectation*: Board A LED (PWM) ramps up, "MOV_UP" telemetry, ramps down at F4, "DOOR_O" telemetry.
2. **Hall Call**: Press a hallway button (U2) on Board A.
   * *Expectation*: Dispatcher assigns the call to the most optimal board. Verify the target elevator responds.

## Stage 2: Dual-Elevator Optimization
3. **Simultaneous Calls**: Press Hallway U1 and Hallway D4.
   * *Expectation*: Dispatcher assigns U1 to the closest elevator and D4 to the other. Both move simultaneously.
4. **Perfect Match**: While Board A is moving UP past Floor 2 to Floor 4, press Hallway U3.
   * *Expectation*: Dispatcher assigns U3 to Board A because it is already moving UP and hasn't passed Floor 3 yet.

## Stage 3: Safety & Robustness
5. **Emergency Stop**: Press the Emergency button during movement.
   * *Expectation*: Motor LED turns OFF instantly (bypassing the ramp). Telemetry shows "EMERG!". System ignores all requests.
6. **Button Spam**: Rapidly press several floor buttons.
   * *Expectation*: Software debounce (50ms) filters the spam. System remains responsive; no lockups or duplicate request corruption.

## Stage 4: Communication Failure & Recovery
7. **SPI Disconnect**: Disconnect the SPI cable (or reset the Master/Slave).
   * *Expectation*: 
     * **Master** detects "FAULT" and reclaims all pending hall calls.
     * **Slave** detects "FAULT" and enters **INDEPENDENT** mode (cabin-only service).
8. **SPI Reconnect**: Restore the SPI link.
   * *Expectation*: Both boards detect "OK" status. Slave exits independent mode and begins accepting dispatcher assignments again.

## Stage 5: System Verification
9. **Watchdog Check**: Simulate a code hang (if a debug command exists).
   * *Expectation*: IWDG resets the board after 4 seconds of inactivity.
10. **Telemetry Audit**: Observe the 500ms UART report for state accuracy and comm-fault flags.
