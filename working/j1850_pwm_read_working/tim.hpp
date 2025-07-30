#ifndef TIM_HPP
#define TIM_HPP

#include <Arduino.h>

// Timer definitions
#define TC_BASE      TC0
#define TC_CH        0 // Đổi tên từ TC_CHANNEL thành TC_CH để tránh xung đột
#define ID_TC        ID_TC0

// Initialize timer to count up every microsecond
static inline void initTimer() {
    // Disable write protection and enable peripheral clock
    PMC->PMC_WPMR = 0x504D43; // PMC_WPKEY to disable write protection
    PMC->PMC_PCER0 = (1u << ID_TC); // Enable TC0 clock

    // Disable timer and interrupts
    TC_BASE->TC_CHANNEL[TC_CH].TC_CCR = TC_CCR_CLKDIS;
    TC_BASE->TC_CHANNEL[TC_CH].TC_IDR = 0xFFFFFFFF;
    TC_BASE->TC_CHANNEL[TC_CH].TC_SR; // Clear status

    // Configure timer for 1 MHz counting (1us resolution with 42 MHz clock)
    TC_BASE->TC_CHANNEL[TC_CH].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | // MCK/2 = 42 MHz
                                        TC_CMR_WAVE |                // Waveform mode
                                        TC_CMR_WAVSEL_UP;            // Count up

    // Start timer
    TC_BASE->TC_CHANNEL[TC_CH].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

// Get current timer value in microseconds
static inline uint32_t getMicros() {
    return TC_BASE->TC_CHANNEL[TC_CH].TC_CV / 42; // 42 ticks = 1 us
}

// Reset timer to 0
static inline void resetTimer() {
    TC_BASE->TC_CHANNEL[TC_CH].TC_CCR = TC_CCR_SWTRG;
}

// Delay for specified number of microseconds
static inline void delayus(uint32_t us) {
    uint32_t target = TC_BASE->TC_CHANNEL[TC_CH].TC_CV + (us * 42);
    while (TC_BASE->TC_CHANNEL[TC_CH].TC_CV < target);
}

#endif // TIM_HPP