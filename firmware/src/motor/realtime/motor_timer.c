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

#include "timer.h"
#include "pwm.h"
#include "adc.h"
#include "internal.h"
#include <ch.h>
#include <hal.h>
#include <sys.h>
#include <assert.h>
#include <stm32f10x.h>

// To prevent occasional use of a wrong timer
#undef TIM1
#undef TIM2
#undef TIM3
#undef TIM5
#undef TIM7
#undef TIM8

#define GLUE2_(A, B)     A##B
#define GLUE2(A, B)      GLUE2_(A, B)
#define GLUE3_(A, B, C)  A##B##C
#define GLUE3(A, B, C)   GLUE3_(A, B, C)

/**
 * Event callout timer declaration
 */
#define TIMEVT_NUM   4

#define TIMEVT            GLUE2(TIM, TIMEVT_NUM)
#define TIMEVT_IRQn       GLUE3(TIM, TIMEVT_NUM, _IRQn)
#define TIMEVT_IRQHandler GLUE3(TIM, TIMEVT_NUM, _IRQHandler)

#if TIMEVT_NUM == 1 || (TIMEVT_NUM >= 8 && TIMEVT_NUM <= 11)
#  error Nope
#else
#  define TIMEVT_RCC_ENR          RCC->APB1ENR
#  define TIMEVT_RCC_RSTR         RCC->APB1RSTR
#  define TIMEVT_RCC_ENR_MASK     GLUE3(RCC_APB1ENR_TIM,  TIMEVT_NUM, EN)
#  define TIMEVT_RCC_RSTR_MASK    GLUE3(RCC_APB1RSTR_TIM, TIMEVT_NUM, RST)
#  define TIMEVT_INPUT_CLOCK      STM32_TIMCLK1
#endif

/**
 * Timestamping timer declaration
 */
#define TIMSTP_NUM   6

#define TIMSTP            GLUE2(TIM, TIMSTP_NUM)
#define TIMSTP_IRQn       GLUE3(TIM, TIMSTP_NUM, _IRQn)
#define TIMSTP_IRQHandler GLUE3(TIM, TIMSTP_NUM, _IRQHandler)

#if TIMSTP_NUM == 1 || (TIMSTP_NUM >= 8 && TIMSTP_NUM <= 11)
#  error Nope
#else
#  define TIMSTP_RCC_ENR          RCC->APB1ENR
#  define TIMSTP_RCC_RSTR         RCC->APB1RSTR
#  define TIMSTP_RCC_ENR_MASK     GLUE3(RCC_APB1ENR_TIM,  TIMSTP_NUM, EN)
#  define TIMSTP_RCC_RSTR_MASK    GLUE3(RCC_APB1RSTR_TIM, TIMSTP_NUM, RST)
#  define TIMSTP_INPUT_CLOCK      STM32_TIMCLK1
#endif

/**
 * Sanity check
 */
#if TIMSTP_INPUT_CLOCK != TIMEVT_INPUT_CLOCK
# error "Invalid timer clocks"
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
static volatile uint64_t _raw_ticks = 0;

/**
 * Event timer ISR
 */
__attribute__((optimize(3)))
CH_FAST_IRQ_HANDLER(TIMEVT_IRQHandler)
{
	TESTPAD_SET(GPIO_PORT_TEST_MTIM, GPIO_PIN_TEST_MTIM);

	if ((TIMEVT->SR & TIM_SR_CC1IF) && (TIMEVT->DIER & TIM_DIER_CC1IE)) {
		TIMEVT->DIER &= ~TIM_DIER_CC1IE; // Disable this compare match
		TIMEVT->SR = ~TIM_SR_CC1IF;
		// Callback must be called when the IRQ has been ACKed, not other way
		const uint64_t timestamp = motor_timer_hnsec() - 2;
		motor_timer_callback(timestamp);
	}

	TESTPAD_CLEAR(GPIO_PORT_TEST_MTIM, GPIO_PIN_TEST_MTIM);
}

/**
 * Timestamping timer overflow ISR
 * TODO: There must be some way to lower the priority of this IRQ to prevent it from interfering with motor control.
 *       This should be implemented in the future, because in theory that should improve stability, reduce the
 *       torque ripple and increase efficiency. The problem with lowering the priority is that this ISR will be
 *       occasionally preempted by the motor control IRQs that will need the timestamping too, hence the race
 *       condition.
 */
__attribute__((optimize(3)))
CH_FAST_IRQ_HANDLER(TIMSTP_IRQHandler)
{
	assert(TIMSTP->SR & TIM_SR_UIF);
	TIMSTP->SR = ~TIM_SR_UIF;
	_raw_ticks += TICKS_PER_OVERFLOW;
}

/*
max_freq = 10000000
for prescaler in xrange(int(TIMEVT_INPUT_CLOCK / float(max_freq) + 0.5), 65535 + 1):
    if TIMEVT_INPUT_CLOCK % prescaler:
            continue
    if int(1e9) % (TIMEVT_INPUT_CLOCK / prescaler):
            continue
    print prescaler, TIMEVT_INPUT_CLOCK / prescaler, int(1e9) / (TIMEVT_INPUT_CLOCK / prescaler)
 */
void motor_timer_init(void)
{
	chSysDisable();

	// Power-on and reset
	TIMEVT_RCC_ENR |= TIMEVT_RCC_ENR_MASK;
	TIMEVT_RCC_RSTR |=  TIMEVT_RCC_RSTR_MASK;
	TIMEVT_RCC_RSTR &= ~TIMEVT_RCC_RSTR_MASK;

	TIMSTP_RCC_ENR |= TIMSTP_RCC_ENR_MASK;
	TIMSTP_RCC_RSTR |=  TIMSTP_RCC_RSTR_MASK;
	TIMSTP_RCC_RSTR &= ~TIMSTP_RCC_RSTR_MASK;

	chSysEnable();

	// Find the optimal prescaler value
	uint32_t prescaler = (uint32_t)(TIMEVT_INPUT_CLOCK / ((float)MAX_FREQUENCY)); // Initial value
	if (prescaler < 1)
		prescaler = 1;

	for (;; prescaler++) {
		assert_always(prescaler < 0xFFFF);

		if (TIMEVT_INPUT_CLOCK % prescaler) {
			continue;
		}
		const uint32_t prescaled_clock = TIMEVT_INPUT_CLOCK / prescaler;
		if (INT_1E9 % prescaled_clock) {
			continue;
		}
		break; // Ok, current prescaler value can divide the timer frequency with no remainder
	}
	_nanosec_per_tick = INT_1E9 / (TIMEVT_INPUT_CLOCK / prescaler);
	assert_always(_nanosec_per_tick < 1000);      // Make sure it is sane

	lowsyslog("Motor: Timer resolution: %u nanosec\n", (unsigned)_nanosec_per_tick);

	// Enable IRQ
	nvicEnableVector(TIMEVT_IRQn,  MOTOR_IRQ_PRIORITY_MASK);
	nvicEnableVector(TIMSTP_IRQn,  MOTOR_IRQ_PRIORITY_MASK);

	// Start the event timer
	TIMEVT->ARR = 0xFFFF;
	TIMEVT->PSC = (uint16_t)(prescaler - 1);
	TIMEVT->CR1 = TIM_CR1_URS;
	TIMEVT->SR  = 0;
	TIMEVT->EGR = TIM_EGR_UG;     // Reload immediately
	TIMEVT->CR1 = TIM_CR1_CEN;    // Start

	// Start the timestamping timer
	TIMSTP->ARR = 0xFFFF;
	TIMSTP->PSC = (uint16_t)(prescaler - 1);
	TIMSTP->CR1 = TIM_CR1_URS;
	TIMSTP->SR  = 0;
	TIMSTP->EGR = TIM_EGR_UG;     // Reload immediately
	TIMSTP->DIER = TIM_DIER_UIE;
	TIMSTP->CR1 = TIM_CR1_CEN;    // Start
}

uint64_t motor_timer_get_max_delay_hnsec(void)
{
	assert_always(_nanosec_per_tick > 0);   // Make sure the timer was initialized
	return (_nanosec_per_tick * TICKS_PER_OVERFLOW) / 100;
}

__attribute__((optimize(1)))          // To prevent the code reordering
uint64_t motor_timer_hnsec(void)
{
	assert(_nanosec_per_tick > 0);  // Make sure the timer was initialized

#if !NDEBUG
	static volatile uint64_t prev_output;
	irq_primask_disable();
	const volatile uint64_t prev_output_sample = prev_output;
	irq_primask_enable();
#endif

	volatile uint64_t ticks = 0;
	volatile uint_fast16_t sample = 0;

	while (1) {
		ticks = _raw_ticks;
		sample = TIMSTP->CNT;

		const volatile uint64_t ticks2 = _raw_ticks;

		if (ticks == ticks2) {
			if (TIMSTP->SR & TIM_SR_UIF) {
				sample = TIMSTP->CNT;
				ticks += TICKS_PER_OVERFLOW;
			}
			break;
		}
	}

	const uint64_t output = ((ticks + sample) * _nanosec_per_tick) / 100;

#if !NDEBUG
	irq_primask_disable();
	// Make sure the prev output was not modified from another context.
	if (prev_output_sample == prev_output) {
		assert(prev_output <= output);
		prev_output = output;
	}
	irq_primask_enable();
#endif
	return output;
}

__attribute__((optimize(3)))
void motor_timer_set_relative(int delay_hnsec)
{
	delay_hnsec -= 1 * HNSEC_PER_USEC;
	if (delay_hnsec < 0) {
		delay_hnsec = 0;
	}

	assert(_nanosec_per_tick > 0);
	int delay_ticks = (delay_hnsec * 100) / _nanosec_per_tick;

	if (delay_ticks > 0xFFFF) {
		assert(0);
		delay_ticks = 0xFFFF;
	}

	/*
	 * Interrupts must be disabled completely because the following
	 * sequence requires strict timing.
	 * No port_*() functions are allowed here!
	 */
	irq_primask_disable();

	if (delay_hnsec > HNSEC_PER_USEC) {
		TIMEVT->CCR1 = TIMEVT->CNT + delay_ticks;
		TIMEVT->SR = ~TIM_SR_CC1IF;             // Acknowledge IRQ
		TIMEVT->DIER |= TIM_DIER_CC1IE;         // Enable this compare match
	} else {
		// Force the update event immediately because the delay is too small
		TIMEVT->DIER |= TIM_DIER_CC1IE;  // Either here or at the next statement IRQ will be generated
		TIMEVT->EGR = TIM_EGR_CC1G;
	}

	irq_primask_enable();
}

__attribute__((optimize(3)))
void motor_timer_set_absolute(uint64_t timestamp_hnsec)
{
	const uint64_t current_timestamp = motor_timer_hnsec();
	if (timestamp_hnsec > current_timestamp) {
		motor_timer_set_relative(timestamp_hnsec - current_timestamp);
	} else {
		motor_timer_set_relative(0);
	}
}

void motor_timer_cancel(void)
{
	TIMEVT->DIER &= ~TIM_DIER_CC1IE;
	TIMEVT->SR = ~TIM_SR_CC1IF;
}

void motor_timer_hndelay(int hnsecs)
{
	static const int OVERHEAD_HNSEC = 1 * HNSEC_PER_USEC;
	if (hnsecs > OVERHEAD_HNSEC) {
		hnsecs -= OVERHEAD_HNSEC;
	} else {
		hnsecs = 0;
	}
	const uint64_t deadline = motor_timer_hnsec() + hnsecs;
	while (motor_timer_hnsec() < deadline) { }
}
