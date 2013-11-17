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

#include <hal.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys.h>

#ifdef STM32F10X_XL
#  error "Add support for XL devices"
#endif

/*
 * The code below assumes that HSI oscillator is up and running,
 * otherwise the Flash controller (FPEC) may misbehave.
 * Any FPEC issues will be detected at run time during write/erase verification.
 */


#define RDP_KEY             0x00A5
#define FLASH_KEY1          0x45670123
#define FLASH_KEY2          0xCDEF89AB

#ifndef F_SIZE
#   define F_SIZE           (*((uint16_t*)0x1FFFF7E0))
#endif
#define FLASH_END           ((F_SIZE * 1024) + FLASH_BASE)

#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
#   define FLASH_PAGE_SIZE  0x800
#else
#   define FLASH_PAGE_SIZE  0x400
#endif

#define FLASH_PAGE_ADR      (FLASH_END - FLASH_PAGE_SIZE)
#define DATA_SIZE_MAX       FLASH_PAGE_SIZE


static void wait_ready(void)
{
	do {
		assert(!(FLASH->SR & FLASH_SR_PGERR));
		assert(!(FLASH->SR & FLASH_SR_WRPRTERR));
	} while (FLASH->SR & FLASH_SR_BSY);
	FLASH->SR |= FLASH_SR_EOP;
}

static void prologue(void)
{
	chSysLock();
	wait_ready();
	if (FLASH->CR & FLASH_CR_LOCK) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}
	FLASH->SR |= FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR; // Reset flags
	FLASH->CR = 0;
}

static void epilogue(void)
{
	FLASH->CR = FLASH_CR_LOCK;  // Reset the FPEC configuration and lock
	chSysUnlock();
}

int flash_storage_read(unsigned offset, void* data, unsigned len)
{
	if (!data || (offset + len) > DATA_SIZE_MAX) {
		assert(0);
		return -EINVAL;
	}

	/*
	 * Read directly, FPEC is not involved
	 */
	memcpy(data, (void*)(FLASH_PAGE_ADR + offset), len);
	return 0;
}

int flash_storage_write(unsigned offset, const void* data, unsigned len)
{
	if (!data || (offset + len) > DATA_SIZE_MAX) {
		assert(0);
		return -EINVAL;
	}

	/*
	 * Data alignment
	 */
	if (((size_t)data) % 2) {
		assert(0);
		return -EIO;
	}
	unsigned num_data_halfwords = len / 2;
	if (num_data_halfwords * 2 < len)
		num_data_halfwords += 1;

	/*
	 * Write
	 */
	prologue();

	FLASH->CR = FLASH_CR_PG;

	volatile uint16_t* flashptr16 = (uint16_t*)(FLASH_PAGE_ADR + offset);
	const uint16_t* ramptr16 = (const uint16_t*)data;
	for (unsigned i = 0; i < num_data_halfwords; i++) {
		*flashptr16++ = *ramptr16++;
		wait_ready();
	}

	wait_ready();
	FLASH->CR = 0;

	epilogue();

	/*
	 * Verify
	 */
	const int cmpres = memcmp(data, (void*)(FLASH_PAGE_ADR + offset), len);
	return cmpres ? -EIO : 0;
}

int flash_storage_erase(void)
{
	/*
	 * Erase
	 */
	prologue();

	FLASH->CR = FLASH_CR_PER;
	FLASH->AR = FLASH_PAGE_ADR;
	FLASH->CR |= FLASH_CR_STRT;

	wait_ready();
	FLASH->CR = 0;

	epilogue();

	/*
	 * Verify
	 */
	for (int i = 0; i < DATA_SIZE_MAX; i++) {
		uint8_t* ptr = (uint8_t*)(FLASH_PAGE_ADR + i);
		if (*ptr != 0xFF)
			return -EIO;
	}
	return 0;
}
