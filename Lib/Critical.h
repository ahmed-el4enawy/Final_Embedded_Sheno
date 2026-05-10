/**
 * Critical.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Provides Enter_Critical() / Exit_Critical() to protect shared
 *  resources (SPI buffers, FSM flags) from ISR/main-loop races.
 */

#ifndef CRITICAL_H
#define CRITICAL_H

#include "Std_Types.h"

/**
 * @brief Disable all configurable interrupts (PRIMASK = 1).
 *        Saves the previous PRIMASK so nesting is safe.
 * @return Previous PRIMASK value (0 = interrupts were enabled)
 */
static inline uint32 Enter_Critical(void) {
    uint32 primask;
    __asm volatile (
        "MRS %0, PRIMASK\n"
        "CPSID I\n"
        : "=r" (primask)
        :
        : "memory"
    );
    return primask;
}

/**
 * @brief Restore PRIMASK to a previously saved value.
 * @param primask  Value returned by Enter_Critical()
 */
static inline void Exit_Critical(uint32 primask) {
    __asm volatile (
        "MSR PRIMASK, %0\n"
        :
        : "r" (primask)
        : "memory"
    );
}

#endif /* CRITICAL_H */
