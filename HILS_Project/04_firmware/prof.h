/* prof.h - Stage 4 section 6 ARM Cortex-M4 DWT cycle-counter WCET profiler */
#ifndef PROF_H
#define PROF_H
#include <stdint.h>
#ifdef STM32F446xx
#include "stm32f4xx.h"
static inline void Prof_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t Prof_Now(void) { return DWT->CYCCNT; }
static inline uint32_t Prof_CyclesToUs(uint32_t cycles) { return cycles / (SystemCoreClock / 1000000u); }
#endif
#endif /* PROF_H */
