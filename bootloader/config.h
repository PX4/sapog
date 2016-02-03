/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/*
 * @file config.h
 *
 * bootloader definitions that configures the behavior and options
 * of the Boot loader
 * This file is relies on the parent folder's boot_config.h file and defines
 * different usages of the hardware for bootloading
 */


#pragma once

/************************************************************************************
 * Global platform definitions
 ************************************************************************************/

#define CONFIG_ARCH_CORTEXM3
#define CONFIG_ARCH_CHIP_STM32F105VB 1
#define CONFIG_USEC_PER_TICK 1000
#define CONFIG_IDLETHREAD_STACKSIZE 8192
#define CONFIG_STM32_NOEXT_VECTORS
#define CONFIG_STM32_STM32F10XX 1
#define CONFIG_STM32_CONNECTIVITYLINE 1


/************************************************************************************
 * Included Files
 ************************************************************************************/



#include <stm32.h>

#ifndef __ASSEMBLY__
#include <assert.h>
#include <string.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 24MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL source is HSE/1, PLL multipler is 3: PLL frequency is 24MHz (XTAL) x 3 = 72MHz */

/* PLL ouput is 72MHz */

#define STM32_PLL_PREDIV2       RCC_CFGR2_PREDIV2d3   /* 24MHz / 3 => 8MHz */
#define STM32_PLL_PLL2MUL       RCC_CFGR2_PLL2MULx9   /* 8MHz * 9  => 72MHz */
#define STM32_PLL_PREDIV1       RCC_CFGR2_PREDIV1d9   /* 72MHz / 9 => 8MHz */
#define STM32_PLL_PLLMUL        RCC_CFGR_PLLMUL_CLKx9 /* 8MHz * 9  => 72Mhz */
#define STM32_PLL_FREQUENCY     (72000000)

/* SYCLLK and HCLK are the PLL frequency */

#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY    /* same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

 /* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,15-17 are on APB2, others on APB1 */

#define STM32_TIM18_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_TIM27_FREQUENCY   STM32_HCLK_FREQUENCY

#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)

/* APB2 timer 1 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-4 will be twice PCLK1 (I presume the remaining will receive PCLK1) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Probes unused */
#define PROBE_INIT(mask)
#define PROBE(n,s)
#define PROBE_MARK(n)


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPT_PREFERRED_NODE_ID ANY_NODE_ID

//todo:wrap OPT_x in in ifdefs for command line definitions
#define OPT_TBOOT_MS            5000
#define OPT_NODE_STATUS_RATE_MS 800
#define OPT_NODE_INFO_RATE_MS   10
#define OPT_BL_NUMBER_TIMERS    7

#define OPT_RESTART_TIMEOUT_MS 20000u

/* Reserved for the Booloader */
#define OPT_BOOTLOADER_SIZE_IN_K (1024*16)

/* Reserved for the application out of the total
 * system flash minus the BOOTLOADER_SIZE_IN_K
 */
#define OPT_APPLICATION_RESERVER_IN_K 0

#define OPT_APPLICATION_IMAGE_OFFSET OPT_BOOTLOADER_SIZE_IN_K
#define OPT_APPLICATION_IMAGE_LENGTH (FLASH_SIZE-(OPT_BOOTLOADER_SIZE_IN_K+OPT_APPLICATION_RESERVER_IN_K))

#define HW_UAVCAN_NAME "io.px4.sapog"
#define HW_VERSION_MAJOR 1
#define HW_VERSION_MINOR 0

#define FLASH_BASE              STM32_FLASH_BASE
#define FLASH_NUMBER_PAGES      128
#define FLASH_PAGE_SIZE         STM32_FLASH_PAGESIZE
#define FLASH_SIZE              (FLASH_NUMBER_PAGES*FLASH_PAGE_SIZE)

#define PARAM_SIZE              (FLASH_PAGE_SIZE)

#define APPLICATION_LOAD_ADDRESS (FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET)
#define APPLICATION_SIZE (FLASH_SIZE-OPT_APPLICATION_IMAGE_OFFSET-PARAM_SIZE)
#define APPLICATION_LAST_8BIT_ADDRRESS  ((uint8_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint8_t)))
#define APPLICATION_LAST_32BIT_ADDRRESS ((uint32_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint32_t)))
#define APPLICATION_LAST_64BIT_ADDRRESS ((uint64_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint64_t)))


/* High-resolution timer
 */
#define HRT_TIMER               1       /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       1       /* use capture/compare channel */


#define GPIO_FET_A_L_LOW (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                          GPIO_PORTA | GPIO_PIN7)
#define GPIO_FET_B_L_LOW (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                          GPIO_PORTB | GPIO_PIN0)
#define GPIO_FET_C_L_LOW (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                          GPIO_PORTB | GPIO_PIN1)

#define GPIO_FET_A_H_LOW (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                          GPIO_PORTA | GPIO_PIN8)
#define GPIO_FET_B_H_LOW (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                          GPIO_PORTA | GPIO_PIN9)
#define GPIO_FET_C_H_LOW (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                          GPIO_PORTA | GPIO_PIN10)

#define GPIO_LED_R (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                    GPIO_PORTC | GPIO_PIN6)
#define GPIO_LED_G (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                    GPIO_PORTC | GPIO_PIN7)
#define GPIO_LED_B (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_OUTPUT_CLEAR | \
                    GPIO_PORTC | GPIO_PIN8)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

inline static void board_initialize(void)
{
    stm32_configgpio(GPIO_FET_A_H_LOW);
    stm32_configgpio(GPIO_FET_B_H_LOW);
    stm32_configgpio(GPIO_FET_C_H_LOW);

    stm32_configgpio(GPIO_FET_A_L_LOW);
    stm32_configgpio(GPIO_FET_B_L_LOW);
    stm32_configgpio(GPIO_FET_C_L_LOW);

    putreg32(getreg32(STM32_RCC_APB1ENR) | RCC_APB1ENR_CAN1EN, STM32_RCC_APB1ENR);

    stm32_configgpio(GPIO_CAN1_RX);
    stm32_configgpio(GPIO_CAN1_TX);

    putreg32(getreg32(STM32_RCC_APB1RSTR) | RCC_APB1RSTR_CAN1RST,
         STM32_RCC_APB1RSTR);
    putreg32(getreg32(STM32_RCC_APB1RSTR) & ~RCC_APB1RSTR_CAN1RST,
         STM32_RCC_APB1RSTR);

    stm32_configgpio(GPIO_LED_R);
    stm32_configgpio(GPIO_LED_G);
    stm32_configgpio(GPIO_LED_B);

    stm32_gpiowrite(GPIO_LED_R, true);
}

/************************************************************************************
 * Name: board_deinitialize
 *
 * Description:
 *   This function is called by the bootloader code prior to booting
 *   the application. Is should place the HW into an benign initialized state.
 *
 ************************************************************************************/

inline static void board_deinitialize(void)
{
}

/****************************************************************************
 * Name: board_get_product_name
 *
 * Description:
 *   Called to retrieve the product name. The returned value is a assumed
 *   to be written to a pascal style string that will be length prefixed
 *   and not null terminated
 *
 * Input Parameters:
 *    product_name - A pointer to a buffer to write the name.
 *    maxlen       - The maximum number of charter that can be written
 *
 * Returned Value:
 *   The length of characters written to the buffer.
 *
 ****************************************************************************/

inline static uint8_t board_get_product_name(uint8_t *product_name, size_t maxlen)
{
    assert(maxlen > sizeof(HW_UAVCAN_NAME)-1);
    memcpy(product_name, HW_UAVCAN_NAME, sizeof(HW_UAVCAN_NAME)-1);
    return sizeof(HW_UAVCAN_NAME)-1;
}

/****************************************************************************
 * Name: board_get_hardware_version
 *
 * Description:
 *   Called to retrieve the hardware version information.
 *
 * Input Parameters:
 *    major - A pointer to the major hardware version field.
 *    minor - A pointer to the minor hardware version field.
 *    unique_id - A pointer to the 16-byte unique ID field
 *    coa_length - A pointer to the 8-bit length field for the COA
 *    coa - A pointer to the COA field, with a maximum length of 255 bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

inline static void board_get_hardware_version(uint8_t *major, uint8_t *minor,
    uint8_t unique_id[16], uint8_t *coa_length, uint8_t *coa)
{
    *major = HW_VERSION_MAJOR;
    *minor = HW_VERSION_MINOR;
    *coa_length = 0u;

    memset(unique_id, 0, 16u);
    memcpy(unique_id, (void *)STM32_SYSMEM_UID, 12u);
}

/****************************************************************************
 * Name: board_indicate
 *
 * Description:
 *   Provides User feedback to indicate the state of the bootloader
 *   on board specific  hardware.
 *
 * Input Parameters:
 *    indication - A member of the uiindication_t
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

typedef enum {
    off,
    reset,
    autobaud_start,
    autobaud_end,
    allocation_start,
    allocation_end,
    fw_update_start,
    fw_update_erase_fail,
    fw_update_invalid_response,
    fw_update_timeout,
    fw_update_invalid_crc,
    jump_to_app,
} uiindication_t;

inline static void board_indicate(uiindication_t indication)
{
    stm32_gpiowrite(GPIO_LED_R, false);
    stm32_gpiowrite(GPIO_LED_G, false);
    stm32_gpiowrite(GPIO_LED_B, false);

    switch (indication) {
        case off:
        case reset:
            stm32_gpiowrite(GPIO_LED_R, true);
            break;
        case autobaud_start:
            stm32_gpiowrite(GPIO_LED_R, true);
            stm32_gpiowrite(GPIO_LED_G, true);
            break;
        case autobaud_end:
            stm32_gpiowrite(GPIO_LED_G, true);
            break;
        case allocation_start:
            stm32_gpiowrite(GPIO_LED_R, true);
            stm32_gpiowrite(GPIO_LED_B, true);
            break;
        case allocation_end:
            stm32_gpiowrite(GPIO_LED_B, true);
            break;
        case fw_update_start:
            stm32_gpiowrite(GPIO_LED_G, true);
            stm32_gpiowrite(GPIO_LED_B, true);
            break;
        case fw_update_erase_fail:
        case fw_update_invalid_response:
        case fw_update_timeout:
        case fw_update_invalid_crc:
            break;
        case jump_to_app:
        default:
            stm32_gpiowrite(GPIO_LED_R, true);
            stm32_gpiowrite(GPIO_LED_G, true);
            stm32_gpiowrite(GPIO_LED_B, true);
            break;
    }
}

/****************************************************************************
 * Name: board_should_wait_for_getnodeinfo
 *
 * Description:
 *   Returns 1 if the board should wait for a GetNodeInfo request before
 *   booting, or 0 if boot should proceed anyway.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    1 if the board should wait for GetNodeInfo, 0 otherwise
 *
 ****************************************************************************/

inline static uint8_t board_should_wait_for_getnodeinfo(void)
{
    return 0u;
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

