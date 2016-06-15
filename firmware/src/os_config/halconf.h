/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
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

#include "mcuconf.h"

#define HAL_USE_TM                  TRUE
#define HAL_USE_PAL                 TRUE
#define HAL_USE_ADC                 FALSE
#define HAL_USE_CAN                 FALSE
#define HAL_USE_DAC                 FALSE
#define HAL_USE_EXT                 FALSE
#define HAL_USE_GPT                 FALSE
#define HAL_USE_I2C                 FALSE
#define HAL_USE_I2S                 FALSE
#define HAL_USE_ICU                 TRUE
#define HAL_USE_MAC                 FALSE
#define HAL_USE_MMC_SPI             FALSE
#define HAL_USE_PWM                 FALSE
#define HAL_USE_RTC                 FALSE
#define HAL_USE_SDC                 FALSE
#define HAL_USE_SERIAL              TRUE
#define HAL_USE_SERIAL_USB          FALSE
#define HAL_USE_SPI                 FALSE
#define HAL_USE_UART                FALSE
#define HAL_USE_USB                 FALSE
#define HAL_USE_WDG                 FALSE

#define SERIAL_DEFAULT_BITRATE      115200
#define SERIAL_BUFFERS_SIZE         256

#include <zubax_chibios/sys/halconf_tail.h>
