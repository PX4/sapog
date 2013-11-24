/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko (pavel.kirienko@gmail.com)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <ch.h>
#include <hal.h>
#include <assert.h>
#include <stm32f10x.h>
#include "pwm.h"
#include "adc.h"
#include "timer.h"
#include "common.h"

#define GLUE2_(A, B)     A##B
#define GLUE2(A, B)      GLUE2_(A, B)
#define GLUE3_(A, B, C)  A##B##C
#define GLUE3(A, B, C)   GLUE3_(A, B, C)

/**
 * Timer declaration
 */
#define TIMER_NUMBER   2

#define TIMX           GLUE2(TIM, TIMER_NUMBER)
#define TIMX_IRQn       GLUE3(TIM, TIMER_NUMBER, _IRQn)
#define TIMX_IRQHandler GLUE3(TIM, TIMER_NUMBER, _IRQHandler)

#if TIMER_NUMBER == 1 || (TIMER_NUMBER >= 8 && TIMER_NUMBER <= 11)
#  define TIMX_RCC_ENR          RCC->APB2ENR
#  define TIMX_RCC_RSTR         RCC->APB2RSTR
#  define TIMX_RCC_ENR_MASK     GLUE3(RCC_APB2ENR_TIM,  TIMER_NUMBER, EN)
#  define TIMX_RCC_RSTR_MASK    GLUE3(RCC_APB2RSTR_TIM, TIMER_NUMBER, RST)
#  define TIMX_INPUT_CLOCK      STM32_TIMCLK2
#else
#  define TIMX_RCC_ENR          RCC->APB1ENR
#  define TIMX_RCC_RSTR         RCC->APB1RSTR
#  define TIMX_RCC_ENR_MASK     GLUE3(RCC_APB1ENR_TIM,  TIMER_NUMBER, EN)
#  define TIMX_RCC_RSTR_MASK    GLUE3(RCC_APB1RSTR_TIM, TIMER_NUMBER, RST)
#  define TIMX_INPUT_CLOCK      STM32_TIMCLK1
#endif


/**
 * The timer frequency is a compromise between maximum delay and timer resolution
 * 2 MHz --> ~32ms max
 * 4 MHz --> ~16ms max
 * 5 MHz --> ~13ms max
 */
static const int MAX_FREQUENCY = 5000000;

static const uint64_t INT_1E9 = 1000000000ul;
static const uint32_t TICKS_PER_OVERFLOW = 0xFFFF + 1;

static uint32_t _nanosec_per_tick = 0;    // 0 means uninitialized
static uint64_t _raw_ticks = 0;


/*
 * I'd like to take a moment to speak about the timers.
 * Our STM32 is unlucky enough to not have enough timers to implement the firmware
 * logic properly, this is why we do two different tasks of different priorities
 * inside the same ISR:
 * 1. Managing callout interface for phase switching. This should be executed with
 *    nearly zero delay since it directly affects the primary function.
 * 2. Updating the counter for submicrosecond timestamping. This IRQ can be delayed
 *    for up to (nanosec_per_tick * 2^16) nanoseconds with no side effects, which
 *    effectively allows to drop it to the kernel IRQ priority.
 * Since there is only one ISR, we do both things at hard real time priority level.
 *
 * TODO either:
 * 1. Allocate another general purpose timer for timestamping.
 * 2. Use advanced timer (advanced timers have independent IRQ vectors for overflow
 *    event and compare match).
 */
__attribute__((optimize(3)))
CH_FAST_IRQ_HANDLER(TIMX_IRQHandler)
{
	TESTPAD_SET(GPIO_PORT_TEST_MTIM, GPIO_PIN_TEST_MTIM);

	// It is not necessary to check DIER because UIE is always enabled
	if (TIMX->SR & TIM_SR_UIF) {
		TIMX->SR = ~TIM_SR_UIF;
		_raw_ticks += TICKS_PER_OVERFLOW;
	}

	if ((TIMX->SR & TIM_SR_CC1IF) && (TIMX->DIER & TIM_DIER_CC1IE)) {
		TIMX->DIER &= ~TIM_DIER_CC1IE; // Disable this compare match
		TIMX->SR = ~TIM_SR_CC1IF;
		// Callback must be called when the IRQ has been ACKed, not other way
		const uint64_t timestamp = motor_timer_hnsec() - 5;
		motor_timer_callback(timestamp);
	}

	TESTPAD_CLEAR(GPIO_PORT_TEST_MTIM, GPIO_PIN_TEST_MTIM);
}

/*
max_freq = 10000000
for prescaler in xrange(int(TIMX_INPUT_CLOCK / float(max_freq) + 0.5), 65535 + 1):
    if TIMX_INPUT_CLOCK % prescaler:
            continue
    if int(1e9) % (TIMX_INPUT_CLOCK / prescaler):
            continue
    print prescaler, TIMX_INPUT_CLOCK / prescaler, int(1e9) / (TIMX_INPUT_CLOCK / prescaler)
 */
void motor_timer_init(void)
{
	chSysDisable();

	// Power-on and reset
	TIMX_RCC_ENR |= TIMX_RCC_ENR_MASK;
	TIMX_RCC_RSTR |=  TIMX_RCC_RSTR_MASK;
	TIMX_RCC_RSTR &= ~TIMX_RCC_RSTR_MASK;

	chSysEnable();

	// Find the optimal prescaler value
	uint32_t prescaler = (uint32_t)(TIMX_INPUT_CLOCK / ((float)MAX_FREQUENCY)); // Initial value
	if (prescaler < 1)
		prescaler = 1;

	for (;; prescaler++) {
		assert_always(prescaler < 0xFFFF);

		if (TIMX_INPUT_CLOCK % prescaler)
			continue;

		const uint32_t prescaled_clock = TIMX_INPUT_CLOCK / prescaler;
		if (INT_1E9 % prescaled_clock)
			continue;

		break; // Ok, current prescaler value can divide the timer frequency with no remainder
	}
	_nanosec_per_tick = INT_1E9 / (TIMX_INPUT_CLOCK / prescaler);
	assert_always(_nanosec_per_tick < 1000);      // Make sure it is sane

	lowsyslog("Motor: Timer resolution: %u nanosec\n", _nanosec_per_tick);

	// Enable IRQ
	nvicEnableVector(TIMX_IRQn,  MOTOR_IRQ_PRIORITY_MASK);

	// Start the timer
	TIMX->ARR = 0xFFFF;
	TIMX->PSC = (uint16_t)(prescaler - 1);
	TIMX->CR1 = TIM_CR1_URS;
	TIMX->SR = 0;
	TIMX->EGR = TIM_EGR_UG;     // Reload immediately
	TIMX->DIER = TIM_DIER_UIE;
	TIMX->CR1 = TIM_CR1_CEN;    // Start
}

uint64_t motor_timer_get_max_delay_hnsec(void)
{
	assert_always(_nanosec_per_tick > 0);   // Make sure the timer was initialized
	return (_nanosec_per_tick * TICKS_PER_OVERFLOW) / 100;
}

__attribute__((optimize(3)))
uint64_t motor_timer_hnsec(void)
{
	assert(_nanosec_per_tick > 0);  // Make sure the timer was initialized

	/*
	 * We don't want the timer IRQ to interrupt this sequence.
	 * If the priority of the timestamping IRQ were lower, we could
	 * raise BASEPRI to its level and let other higher-priority IRQs
	 * interrupt and even re-enter this function, it would be fine.
	 *
	 * Note that BASEPRI must be restored correctly upon the exit,
	 * despite the fact that ChibiOS simply does not have API for that.
	 *
	 * Also, we can't use port_disable()/port_enable() here, because
	 * port_enable() not only restores PRIMASK state but also resets
	 * the BASEPRI register to the default value (0 - all enabled),
	 * which actually may break some interrupted critical section
	 * inside the kernel.
	 */
	irq_primask_disable();

	volatile uint64_t ticks = _raw_ticks;
	volatile uint_fast16_t sample = TIMX->CNT;

	if (TIMX->SR & TIM_SR_UIF) {
		/*
		 * The timer has overflowed either before or after SR was checked.
		 * We need to sample it once more to be sure that the obtained
		 * counter value was wrapped over zero.
		 */
		sample = TIMX->CNT;
		/*
		 * The timer interrupt was set, but not handled yet.
		 * Thus we need to adjust the tick counter manually.
		 */
		ticks += TICKS_PER_OVERFLOW;
	}

	irq_primask_enable();

	return (ticks + sample) * _nanosec_per_tick / 100;
}

__attribute__((optimize(3)))
void motor_timer_set_relative(int delay_hnsec)
{
	delay_hnsec -= 1 * HNSEC_PER_USEC;
	if (delay_hnsec < 0)
		delay_hnsec = 0;

	assert(_nanosec_per_tick > 0);
	int delay_ticks = (delay_hnsec * 100) / _nanosec_per_tick;

	assert(delay_ticks <= 0xFFFF);
	if (delay_ticks > 0xFFFF)
		delay_ticks = 0xFFFF;

	/*
	 * Interrupts must be disabled completely because the following
	 * sequence requires strict timing.
	 * No port_*() functions shall be used here!
	 */
	irq_primask_disable();

	if (delay_hnsec > 2 * HNSEC_PER_USEC) {
		TIMX->CCR1 = TIMX->CNT + delay_ticks;
		TIMX->SR = ~TIM_SR_CC1IF;             // Acknowledge IRQ
		TIMX->DIER |= TIM_DIER_CC1IE;         // Enable this compare match
	} else {
		// Force the update event immediately because the delay is too small
		TIMX->DIER |= TIM_DIER_CC1IE;  // Either here or at the next statement IRQ will be generated
		TIMX->EGR = TIM_EGR_CC1G;
	}

	irq_primask_enable();
}

void motor_timer_cancel(void)
{
	TIMX->DIER &= ~TIM_DIER_CC1IE;
	TIMX->SR = ~TIM_SR_CC1IF;
}

void motor_timer_udelay(int usecs)
{
	usecs -= 1;
	const uint64_t deadline = motor_timer_hnsec() + usecs * HNSEC_PER_USEC;
	while (motor_timer_hnsec() < deadline) {
		/*
		 * Do some yak-shaving in order to reduce the frequency of calls to
		 * motor_timer_hnsec(), because each call disables interrupts for some time.
		 */
		for (volatile int i = 0; i < 8; i++) { }
	}
}
