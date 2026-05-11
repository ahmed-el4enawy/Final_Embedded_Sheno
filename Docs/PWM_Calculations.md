# PWM Motor-Simulation LED — Frequency Calculations

> **Submission Requirement (Section 6):** "PWM Math: Calculations for PSC and ARR to achieve specific frequency."

## Objective

Generate a **10 kHz** PWM signal on **PB0** (TIM3 Channel 3) to simulate elevator motor speed via LED brightness (duty cycle).

## System Clock

```
Clock Source: HSI (High-Speed Internal oscillator)
HSI Frequency: 16 MHz
PLL: NOT configured (Rcc_Init() only enables HSI)
```

TIM3 is on the APB1 bus. With APB1 prescaler = 1 (default):

```
f_TIM3 = f_APB1 = f_SYSCLK = 16,000,000 Hz
```

## PWM Frequency Formula

```
                    f_TIM
f_PWM = ─────────────────────────
        (PSC + 1) × (ARR + 1)
```

## Calculation

**Target:** f_PWM = 10,000 Hz

```
Step 1: Calculate total divider needed

    (PSC + 1) × (ARR + 1) = f_TIM / f_PWM
                           = 16,000,000 / 10,000
                           = 1,600

Step 2: Factor 1,600 into PSC and ARR

    Option A: PSC = 15, ARR = 99  → 16 × 100 = 1,600  ✓  (1% resolution)
    Option B: PSC = 7,  ARR = 199 → 8  × 200 = 1,600  ✓  (0.5% resolution)
    Option C: PSC = 3,  ARR = 399 → 4  × 400 = 1,600  ✓  (0.25% resolution)

Step 3: Select optimal values

    ► PSC = 15, ARR = 99

    Reason: ARR = 99 gives exactly 100 steps (0–99),
    so each CCR3 increment = exactly 1% duty cycle.
    This aligns with RAMP_STEP_PERCENT = 5% (5 CCR steps per ramp tick).
```

## Verification

```
Timer tick frequency:
    f_tick = 16,000,000 / (15 + 1) = 1,000,000 Hz = 1 MHz  (1 µs per tick)

PWM period:
    T_PWM = (99 + 1) / 1,000,000 = 100 µs

PWM frequency:
    f_PWM = 1 / T_PWM = 1 / 0.0001 = 10,000 Hz = 10 kHz  ✓
```

## Duty Cycle Mapping

```
Duty Cycle = CCR3 / (ARR + 1) × 100%
```

| Motor State | CCR3 Value | Duty Cycle | `Board_Config.h` Macro |
|-------------|-----------|------------|----------------------|
| **Stopped** | 0 | 0% | `MOTOR_DUTY_STOP  = 0` |
| **Slow** (arriving) | 20 | 20% | `MOTOR_DUTY_SLOW  = 20` |
| **Full speed** | 100 | 100% | `MOTOR_DUTY_FULL  = 100` |

## Speed Ramping

The PWM ramp engine (`Elevator_RampTick()` in `Elevator_FSM.c`) adjusts the duty cycle non-blockingly:

```
Ramp step size:     5% per step   (RAMP_STEP_PERCENT = 5)
Ramp step interval: 20 ms         (RAMP_STEP_INTERVAL = 20)
Time to ramp 0→100%: 20 steps × 20 ms = 400 ms
Time to ramp 100→20%: 16 steps × 20 ms = 320 ms
```

## Configuration in Code

```c
/* Board_Config.h */
#define MOTOR_PWM_TIMER    TIMER3
#define MOTOR_PWM_CHANNEL  PWM_CHANNEL_3
#define MOTOR_PWM_PSC      15U    /* 16 MHz / 16 = 1 MHz tick   */
#define MOTOR_PWM_ARR      99U    /* 1 MHz / 100 = 10 kHz PWM   */
```

## Alternative: If Using 84 MHz PLL

```
(PSC + 1) × (ARR + 1) = 84,000,000 / 10,000 = 8,400

Option: PSC = 83, ARR = 99  → 84 × 100 = 8,400  ✓
Same 1% duty resolution, same ARR value.
Only PSC changes to compensate for higher clock.
```
