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

#define BOARD_NAME              "PX4ESC-compatible"

#define STM32_HSECLK            8000000
#define STM32_VDD               330

#define STM32F2XX

/*
 * FET driver pins
 */
#define GPIO_PORT_DRV_PWRGD     GPIOC
#define GPIO_PIN_DRV_PWRGD      13

#define GPIO_PORT_DRV_OCTW      GPIOC
#define GPIO_PIN_DRV_OCTW       14

#define GPIO_PORT_DRV_FAULT     GPIOC
#define GPIO_PIN_DRV_FAULT      15

#define GPIO_PORT_DRV_OC_ADJ    GPIOA
#define GPIO_PIN_DRV_OC_ADJ     4

#define GPIO_PORT_DRV_EN_GATE   GPIOA
#define GPIO_PIN_DRV_EN_GATE    5

#define GPIO_PORT_DRV_DC_CAL    GPIOA
#define GPIO_PIN_DRV_DC_CAL     6

#define GPIO_PORT_DRV_GAIN      GPIOB
#define GPIO_PIN_DRV_GAIN       2

/*
 * Testpoints
 */
// General testpoint
#define GPIO_PORT_TEST_A        GPIOD
#define GPIO_PIN_TEST_A         2
// ADC testpoint
#define GPIO_PORT_TEST_ADC      GPIOB
#define GPIO_PIN_TEST_ADC       3
// Motor timer testpoint
#define GPIO_PORT_TEST_MTIM     GPIOB
#define GPIO_PIN_TEST_MTIM      4
// Zero cross testpoint
#define GPIO_PORT_TEST_MZC      GPIOC
#define GPIO_PIN_TEST_MZC       12

/*
 * HW ID
 */
#define GPIO_PORT_HW_ID_0       GPIOB
#define GPIO_PIN_HW_ID_0        7

#define GPIO_PORT_HW_ID_1       GPIOC
#define GPIO_PIN_HW_ID_1        11

#define GPIO_PORT_HW_ID_2       GPIOC
#define GPIO_PIN_HW_ID_2        10

#define GPIO_PORT_HW_ID_3       GPIOA
#define GPIO_PIN_HW_ID_3        15

/*
 * Misc
 */
#define GPIO_PORT_NVMEM_CS      GPIOB
#define GPIO_PIN_NVMEM_CS       12

#define GPIO_PORT_SERIAL_RX     GPIOB
#define GPIO_PIN_SERIAL_RX      11

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0 << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1 << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2 << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3 << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0 << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1 << (n))
#define PIN_OSPEED_2M(n)            (0 << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1 << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2 << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3 << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0 << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1 << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2 << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/******************************************************************************
 * PORT A
 */
#define VAL_GPIOA_MODER     \
/* SENS A */                PIN_MODE_ANALOG(0)  |\
/* SENS B */                PIN_MODE_ANALOG(1)  |\
/* SENS C */                PIN_MODE_ANALOG(2)  |\
/* RC PWM */                PIN_MODE_ALTERNATE(3)  |\
/* DRV OC ADJ */            PIN_MODE_OUTPUT(4)  |\
/* DRV EN GATE */           PIN_MODE_OUTPUT(5)  |\
/* DRV DC CAL */            PIN_MODE_OUTPUT(6)  |\
/* TIM1 CH1N */             PIN_MODE_ALTERNATE(7)  |\
/* TIM1 CH1 */              PIN_MODE_ALTERNATE(8)  |\
/* TIM1 CH2 */              PIN_MODE_ALTERNATE(9)  |\
/* TIM1 CH3 */              PIN_MODE_ALTERNATE(10) |\
/* USB DM */                PIN_MODE_INPUT(11) |\
/* USB DP */                PIN_MODE_INPUT(12) |\
/* SWDIO */                 PIN_MODE_ALTERNATE(13) |\
/* SWDCLK */                PIN_MODE_ALTERNATE(14) |\
/* HW ID 3 */               PIN_MODE_INPUT(15)

#define VAL_GPIOA_OTYPER    0 /* ALL PUSHPULL */

#define VAL_GPIOA_OSPEEDR   \
/* SENS A */                PIN_OSPEED_2M(0)  |\
/* SENS B */                PIN_OSPEED_2M(1)  |\
/* SENS C */                PIN_OSPEED_2M(2)  |\
/* RC PWM */                PIN_OSPEED_2M(3)  |\
/* DRV OC ADJ */            PIN_OSPEED_2M(4)  |\
/* DRV EN GATE */           PIN_OSPEED_2M(5)  |\
/* DRV DC CAL */            PIN_OSPEED_2M(6)  |\
/* TIM1 CH1N */             PIN_OSPEED_25M(7)  |\
/* TIM1 CH1 */              PIN_OSPEED_25M(8)  |\
/* TIM1 CH2 */              PIN_OSPEED_25M(9)  |\
/* TIM1 CH3 */              PIN_OSPEED_25M(10) |\
/* USB DM */                PIN_OSPEED_25M(11) |\
/* USB DP */                PIN_OSPEED_25M(12) |\
/* SWDIO */                 PIN_OSPEED_25M(13) |\
/* SWDCLK */                PIN_OSPEED_25M(14) |\
/* HW ID 3 */               PIN_OSPEED_2M(15)

#define VAL_GPIOA_PUPDR     \
                            PIN_PUDR_PULLDOWN(0)  |\
                            PIN_PUDR_PULLDOWN(1)  |\
                            PIN_PUDR_PULLDOWN(2)  |\
                            PIN_PUDR_PULLDOWN(3)  |\
                            PIN_PUDR_PULLDOWN(4)  |\
                            PIN_PUDR_PULLDOWN(5)  |\
                            PIN_PUDR_PULLDOWN(6)  |\
                            PIN_PUDR_PULLDOWN(7)  |\
                            PIN_PUDR_PULLDOWN(8)  |\
                            PIN_PUDR_PULLDOWN(9)  |\
                            PIN_PUDR_PULLDOWN(10) |\
                            PIN_PUDR_PULLDOWN(11) |\
                            PIN_PUDR_PULLDOWN(12) |\
/* SWDIO */                 PIN_PUDR_PULLUP(13) |\
/* SWDCLK */                PIN_PUDR_PULLDOWN(14) |\
                            PIN_PUDR_PULLDOWN(15)

#define VAL_GPIOA_AFRL      \
                            PIN_AFIO_AF(0, 0)  |\
                            PIN_AFIO_AF(1, 0)  |\
                            PIN_AFIO_AF(2, 0)  |\
 /* RC PWM */               PIN_AFIO_AF(3, 2)  |\
                            PIN_AFIO_AF(4, 0)  |\
                            PIN_AFIO_AF(5, 0)  |\
                            PIN_AFIO_AF(6, 0)  |\
/* TIM1 CH1N */             PIN_AFIO_AF(7, 1)
#define VAL_GPIOA_AFRH      \
/* TIM1 CH1 */              PIN_AFIO_AF(8, 1)  |\
/* TIM1 CH2 */              PIN_AFIO_AF(9, 1)  |\
/* TIM1 CH3 */              PIN_AFIO_AF(10, 1) |\
                            PIN_AFIO_AF(11, 0) |\
                            PIN_AFIO_AF(12, 0) |\
/* SWDIO */                 PIN_AFIO_AF(13, 0) |\
/* SWDCLK */                PIN_AFIO_AF(14, 0) |\
                            PIN_AFIO_AF(15, 0)

#define VAL_GPIOA_ODR       \
                            (0 << 0)  |\
                            (0 << 1)  |\
                            (0 << 2)  |\
                            (0 << 3)  |\
/* DRV OC ADJ */            (1 << 4)  |\
                            (0 << 5)  |\
                            (0 << 6)  |\
                            (0 << 7)  |\
                            (0 << 8)  |\
                            (0 << 9)  |\
                            (0 << 10) |\
                            (0 << 11) |\
                            (0 << 12) |\
                            (0 << 13) |\
                            (0 << 14) |\
                            (0 << 15)

/******************************************************************************
 * PORT B
 */
#define VAL_GPIOB_MODER     \
/* TIM1 CH2N */             PIN_MODE_ALTERNATE(0)  |\
/* TIM1 CH3N */             PIN_MODE_ALTERNATE(1)  |\
/* DRV GAIN */              PIN_MODE_OUTPUT(2)  |\
/* TEST2 */                 PIN_MODE_OUTPUT(3)  |\
/* TEST3 */                 PIN_MODE_OUTPUT(4)  |\
/* CAN2 RX */               PIN_MODE_ALTERNATE(5)  |\
/* CAN2 TX */               PIN_MODE_ALTERNATE(6)  |\
/* HW ID 0 */               PIN_MODE_INPUT(7)  |\
/* CAN1 RX */               PIN_MODE_ALTERNATE(8)  |\
/* CAN1 TX */               PIN_MODE_ALTERNATE(9)  |\
/* UART TX */               PIN_MODE_ALTERNATE(10) |\
/* UART RX */               PIN_MODE_ALTERNATE(11) |\
/* NVMEM CS */              PIN_MODE_OUTPUT(12) |\
/* SPI SCK */               PIN_MODE_ALTERNATE(13) |\
/* SPI MISO */              PIN_MODE_ALTERNATE(14) |\
/* SPI MOSI */              PIN_MODE_ALTERNATE(15)

#define VAL_GPIOB_OTYPER    0 /* ALL PUSHPULL */

#define VAL_GPIOB_OSPEEDR   \
/* TIM1 CH2N */             PIN_OSPEED_25M(0)  |\
/* TIM1 CH3N */             PIN_OSPEED_25M(1)  |\
/* DRV GAIN */              PIN_OSPEED_2M(2)  |\
/* TEST2 */                 PIN_OSPEED_25M(3)  |\
/* TEST3 */                 PIN_OSPEED_25M(4)  |\
/* CAN2 RX */               PIN_OSPEED_25M(5)  |\
/* CAN2 TX */               PIN_OSPEED_25M(6)  |\
/* HW ID 0 */               PIN_OSPEED_2M(7)  |\
/* CAN1 RX */               PIN_OSPEED_25M(8)  |\
/* CAN1 TX */               PIN_OSPEED_25M(9)  |\
/* UART TX */               PIN_OSPEED_2M(10) |\
/* UART RX */               PIN_OSPEED_2M(11) |\
/* NVMEM CS */              PIN_OSPEED_2M(12) |\
/* SPI SCK */               PIN_OSPEED_25M(13) |\
/* SPI MISO */              PIN_OSPEED_25M(14) |\
/* SPI MOSI */              PIN_OSPEED_25M(15)

#define VAL_GPIOB_PUPDR     \
                            PIN_PUDR_PULLDOWN(0)  |\
                            PIN_PUDR_PULLDOWN(1)  |\
                            PIN_PUDR_PULLDOWN(2)  |\
                            PIN_PUDR_PULLDOWN(3)  |\
                            PIN_PUDR_PULLDOWN(4)  |\
                            PIN_PUDR_PULLDOWN(5)  |\
                            PIN_PUDR_PULLDOWN(6)  |\
                            PIN_PUDR_PULLDOWN(7)  |\
                            PIN_PUDR_PULLDOWN(8)  |\
                            PIN_PUDR_PULLDOWN(9)  |\
                            PIN_PUDR_PULLDOWN(10) |\
                            PIN_PUDR_PULLDOWN(11) |\
                            PIN_PUDR_PULLDOWN(12) |\
                            PIN_PUDR_PULLDOWN(13) |\
                            PIN_PUDR_PULLDOWN(14) |\
                            PIN_PUDR_PULLDOWN(15)

#define VAL_GPIOB_AFRL      \
/* TIM1 CH2N */             PIN_AFIO_AF(0, 1)  |\
/* TIM1 CH3N */             PIN_AFIO_AF(1, 1)  |\
/* DRV GAIN */              PIN_AFIO_AF(2, 0)  |\
/* TEST2 */                 PIN_AFIO_AF(3, 0)  |\
/* TEST3 */                 PIN_AFIO_AF(4, 0)  |\
/* CAN2 RX */               PIN_AFIO_AF(5, 9)  |\
/* CAN2 TX */               PIN_AFIO_AF(6, 9)  |\
/* HW ID 0 */               PIN_AFIO_AF(7, 0)
#define VAL_GPIOB_AFRH      \
/* CAN1 RX */               PIN_AFIO_AF(8, 9)  |\
/* CAN1 TX */               PIN_AFIO_AF(9, 9)  |\
/* UART TX */               PIN_AFIO_AF(10, 7) |\
/* UART RX */               PIN_AFIO_AF(11, 7) |\
/* NVMEM CS */              PIN_AFIO_AF(12, 0) |\
/* SPI SCK */               PIN_AFIO_AF(13, 5) |\
/* SPI MISO */              PIN_AFIO_AF(14, 5) |\
/* SPI MOSI */              PIN_AFIO_AF(15, 5)

#define VAL_GPIOB_ODR       \
                            (0 << 0)  |\
                            (0 << 1)  |\
                            (0 << 2)  |\
                            (0 << 3)  |\
                            (0 << 4)  |\
                            (0 << 5)  |\
                            (0 << 6)  |\
                            (0 << 7)  |\
                            (0 << 8)  |\
                            (0 << 9)  |\
                            (0 << 10) |\
                            (0 << 11) |\
/* NVMEM CS */              (1 << 12) |\
                            (0 << 13) |\
                            (0 << 14) |\
                            (0 << 15)

/******************************************************************************
 * PORT C
 */
#define VAL_GPIOC_MODER     \
/* TEMP SENS */             PIN_MODE_ANALOG(0)  |\
/* VBAT SENS */             PIN_MODE_ANALOG(1)  |\
/* CURRENT SENS 2 */        PIN_MODE_ANALOG(2)  |\
/* CURRENT SENS 1 */        PIN_MODE_ANALOG(3)  |\
/* not connected */         PIN_MODE_INPUT(4)  |\
/* not connected */         PIN_MODE_INPUT(5)  |\
/* RPM */                   PIN_MODE_INPUT(6)  |\
/* LED R */                 PIN_MODE_ALTERNATE(7)  |\
/* LED G */                 PIN_MODE_ALTERNATE(8)  |\
/* LED B */                 PIN_MODE_ALTERNATE(9)  |\
/* HW ID 2 */               PIN_MODE_INPUT(10) |\
/* HW ID 1 */               PIN_MODE_INPUT(11) |\
/* TEST4 */                 PIN_MODE_OUTPUT(12) |\
/* DRV PWRGD */             PIN_MODE_INPUT(13) |\
/* DRV OCTW */              PIN_MODE_INPUT(14) |\
/* DRV FAULT */             PIN_MODE_INPUT(15)

#define VAL_GPIOC_OTYPER    0 /* ALL PUSHPULL */

#define VAL_GPIOC_OSPEEDR   \
/* TEMP SENS */            PIN_OSPEED_2M(0)  |\
/* VBAT SENS */            PIN_OSPEED_2M(1)  |\
/* CURRENT SENS 2 */       PIN_OSPEED_2M(2)  |\
/* CURRENT SENS 1 */       PIN_OSPEED_2M(3)  |\
/* not connected */        PIN_OSPEED_2M(4)  |\
/* not connected */        PIN_OSPEED_2M(5)  |\
/* RPM */                  PIN_OSPEED_2M(6)  |\
/* LED R */                PIN_OSPEED_2M(7)  |\
/* LED G */                PIN_OSPEED_2M(8)  |\
/* LED B */                PIN_OSPEED_2M(9)  |\
/* HW ID 2 */              PIN_OSPEED_2M(10) |\
/* HW ID 1 */              PIN_OSPEED_2M(11) |\
/* TEST4 */                PIN_OSPEED_25M(12) |\
/* DRV PWRGD */            PIN_OSPEED_2M(13) |\
/* DRV OCTW */             PIN_OSPEED_2M(14) |\
/* DRV FAULT */            PIN_OSPEED_2M(15)

#define VAL_GPIOC_PUPDR     \
                            PIN_PUDR_PULLDOWN(0)  |\
                            PIN_PUDR_PULLDOWN(1)  |\
                            PIN_PUDR_PULLDOWN(2)  |\
                            PIN_PUDR_PULLDOWN(3)  |\
                            PIN_PUDR_PULLDOWN(4)  |\
                            PIN_PUDR_PULLDOWN(5)  |\
                            PIN_PUDR_PULLDOWN(6)  |\
                            PIN_PUDR_PULLDOWN(7)  |\
                            PIN_PUDR_PULLDOWN(8)  |\
                            PIN_PUDR_PULLDOWN(9)  |\
                            PIN_PUDR_PULLDOWN(10) |\
                            PIN_PUDR_PULLDOWN(11) |\
                            PIN_PUDR_PULLDOWN(12) |\
/* DRV PWRGD */             PIN_PUDR_PULLUP(13) |\
/* DRV OCTW */              PIN_PUDR_PULLUP(14) |\
/* DRV FAULT */             PIN_PUDR_PULLUP(15)

#define VAL_GPIOC_AFRL      \
/* TEMP SENS */             PIN_AFIO_AF(0, 0)  |\
/* VBAT SENS */             PIN_AFIO_AF(1, 0)  |\
/* CURRENT SENS 2 */        PIN_AFIO_AF(2, 0)  |\
/* CURRENT SENS 1 */        PIN_AFIO_AF(3, 0)  |\
/* not connected */         PIN_AFIO_AF(4, 0)  |\
/* not connected */         PIN_AFIO_AF(5, 0)  |\
/* RPM */                   PIN_AFIO_AF(6, 0)  |\
/* LED R */                 PIN_AFIO_AF(7, 2)
#define VAL_GPIOC_AFRH      \
/* LED G */                 PIN_AFIO_AF(8, 2)  |\
/* LED B */                 PIN_AFIO_AF(9, 2)  |\
/* HW ID 2 */               PIN_AFIO_AF(10, 0) |\
/* HW ID 1 */               PIN_AFIO_AF(11, 0) |\
/* TEST4 */                 PIN_AFIO_AF(12, 0) |\
/* DRV PWRGD */             PIN_AFIO_AF(13, 0) |\
/* DRV OCTW */              PIN_AFIO_AF(14, 0) |\
/* DRV FAULT */             PIN_AFIO_AF(15, 0)

#define VAL_GPIOC_ODR       \
                            (0 << 0)  |\
                            (0 << 1)  |\
                            (0 << 2)  |\
                            (0 << 3)  |\
                            (0 << 4)  |\
                            (0 << 5)  |\
                            (0 << 6)  |\
                            (0 << 7)  |\
                            (0 << 8)  |\
                            (0 << 9)  |\
                            (0 << 10) |\
                            (0 << 11) |\
                            (0 << 12) |\
                            (0 << 13) |\
                            (0 << 14) |\
                            (0 << 15)

/******************************************************************************
 * Nonexistent ports - default configuration
 */
#define VAL_GPIO_DEFAULT_MODER          0x00000000  // Input
#define VAL_GPIO_DEFAULT_OTYPER         0x00000000  // Push-pull
#define VAL_GPIO_DEFAULT_OSPEEDR        0x00000000  // Lowest speed
#define VAL_GPIO_DEFAULT_PUPDR          0xAAAAAAAA  // Pull-down
#define VAL_GPIO_DEFAULT_ODR            0x00000000  // Output low
#define VAL_GPIO_DEFAULT_AFRL           0x00000000  // AF0
#define VAL_GPIO_DEFAULT_AFRH           0x00000000


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

/// Called from ChibiOS init
void boardInit(void);

uint8_t board_get_hardware_revision(void);

#define BOARD_UNIQUE_ID_SIZE    12

void board_read_unique_id(uint8_t out_uid[BOARD_UNIQUE_ID_SIZE]);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */
