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

#pragma once

#include <stdbool.h>
#include <stdint.h>

__BEGIN_DECLS

struct motor_adc_sample
{
    uint64_t timestamp;
    int raw_phase_values[3];
};

#define MOTOR_ADC_RESOLUTION    12

/**
 * Hardcoded for STM32
 * One ADC sample at maximum speed takes 14 cycles; max ADC clock at 72 MHz input is 12 MHz, so one ADC sample is:
 *    (1 / 12M) * 14 = 1.17 usec
 */
#define MOTOR_ADC_SAMPLE_DURATION_NANOSEC       1170

/**
 * Full ADC sample of all channels takes:
 *    ceil(3 phases / 2 ADC) * one_sample_duration
 * Thus, optimal advance is one_sample_duration.
 */
#define MOTOR_ADC_SYNC_ADVANCE_NANOSEC  MOTOR_ADC_SAMPLE_DURATION_NANOSEC

void motor_adc_init(void);

void motor_adc_enable(bool enable);

struct motor_adc_sample motor_adc_get_last_sample(void);

/**
 * No OS API can be used from this callback!
 */
extern void motor_adc_sample_callback(const struct motor_adc_sample* sample);

__END_DECLS
