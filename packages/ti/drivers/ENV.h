/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       ENV.h
 *
 *  @brief      Environment definitions for different platforms
 *
 *  ============================================================================
 */

#ifndef ti_drivers_ENV__include
#define ti_drivers_ENV__include

#ifdef __cplusplus
extern "C" {
#endif

#if defined(MSP430WARE)
#include <msp430.h>
#include <gpio.h>
#include <wdt_a.h>
#include <sfr.h>
#include <ucs.h>
#include <usci_b_i2c.h>
#include <usci_b_spi.h>
#include <usci_a_uart.h>

/* GPIO definitions */
#define ENV_GPIO_INT_FALLING            GPIO_HIGH_TO_LOW_TRANSITION
#define ENV_GPIO_INT_RISING             GPIO_LOW_TO_HIGH_TRANSITION
#define ENV_GPIO_INT_BOTH_EDGES         GPIO_HIGH_TO_LOW_TRANSITION \
                                      | GPIO_LOW_TO_HIGH_TRANSITION
#define ENV_GPIO_INT_LOW                GPIO_INPUT_PIN_LOW
#define ENV_GPIO_INT_HIGH               GPIO_INPUT_PIN_HIGH

#define ENV_GPIO_CALLBACK_AMT           8

/* Watchdog definitions */
#define ENV_WATCHDOG_CLOCKSOURCE_SMCLK  WDT_A_CLOCKSOURCE_SMCLK
#define ENV_WATCHDOG_CLOCKSOURCE_ACLK   WDT_A_CLOCKSOURCE_ACLK
#define ENV_WATCHDOG_CLOCKSOURCE_VLOCLK WDT_A_CLOCKSOURCE_VLOCLK
#define ENV_WATCHDOG_CLOCKSOURCE_XCLK   WDT_A_CLOCKSOURCE_XCLK

#define ENV_WATCHDOG_CLOCKDIVIDER_2G    WDT_A_CLOCKDIVIDER_2G
#define ENV_WATCHDOG_CLOCKDIVIDER_128M  WDT_A_CLOCKDIVIDER_128M
#define ENV_WATCHDOG_CLOCKDIVIDER_8192K WDT_A_CLOCKDIVIDER_8192K
#define ENV_WATCHDOG_CLOCKDIVIDER_512K  WDT_A_CLOCKDIVIDER_512K
#define ENV_WATCHDOG_CLOCKDIVIDER_32K   WDT_A_CLOCKDIVIDER_32K
#define ENV_WATCHDOG_CLOCKDIVIDER_8192  WDT_A_CLOCKDIVIDER_8192
#define ENV_WATCHDOG_CLOCKDIVIDER_512   WDT_A_CLOCKDIVIDER_512
#define ENV_WATCHDOG_CLOCKDIVIDER_64    WDT_A_CLOCKDIVIDER_64

/* I2C definitions */
#define ENV_I2C_BITRATE_100000          USCI_B_I2C_SET_DATA_RATE_100KBPS >> 16
#define ENV_I2C_BITRATE_400000          USCI_B_I2C_SET_DATA_RATE_400KBPS >> 16

/* UART definitions */
#define ENV_UART_LEN_5                  0x00            /*!< NOT VALID */
#define ENV_UART_LEN_6                  0x00            /*!< NOT VALID */
#define ENV_UART_LEN_7                  0x00            /*!< NOT VALID */
#define ENV_UART_LEN_8                  0x08            /*!< Data length is 8 */

#define ENV_UART_STOP_ONE               USCI_A_UART_ONE_STOP_BIT
#define ENV_UART_STOP_TWO               USCI_A_UART_TWO_STOP_BITS

#define ENV_UART_PAR_NONE               USCI_A_UART_NO_PARITY
#define ENV_UART_PAR_EVEN               USCI_A_UART_EVEN_PARITY
#define ENV_UART_PAR_ODD                USCI_A_UART_ODD_PARITY
#define ENV_UART_PAR_ZERO               0
#define ENV_UART_PAR_ONE                1

/* SPI definitions */
#define ENV_SPI_MASTER                  0
#define ENV_SPI_SLAVE                   1
#define ENV_SPI_SLAVE_OD                2

#define ENV_SPI_POL0_PHA0               USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW  \
                                      | USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
#define ENV_SPI_POL0_PHA1               USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW  \
                                      | USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
#define ENV_SPI_POL1_PHA0               USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH \
                                      | USCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
#define ENV_SPI_POL1_PHA1               USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH \
                                      | USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
#define ENV_SPI_TI                     ~0                /*!< NOT VALID */
#define ENV_SPI_MW                     ~0                /*!< NOT VALID */

#elif (defined(TIVAWARE) || defined(MWARE) || defined(CCWARE))
#if defined(TIVAWARE)
#include <stdbool.h>

#if defined(gcc)
#include <stdint.h>
#endif

#endif

#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/uart.h>

#if (defined(TIVAWARE) || defined(MWARE))
#include <driverlib/ssi.h>
#include <driverlib/watchdog.h>
#else
#include <inc/hw_mcspi.h>
#include <driverlib/mcspi.h>
#include <driverlib/wdt.h>
#endif

/* GPIO peripheral */
#define ENV_GPIO_INT_FALLING            GPIO_FALLING_EDGE
#define ENV_GPIO_INT_RISING             GPIO_RISING_EDGE
#define ENV_GPIO_INT_BOTH_EDGES         GPIO_BOTH_EDGES
#define ENV_GPIO_INT_LOW                GPIO_LOW_LEVEL
#define ENV_GPIO_INT_HIGH               GPIO_HIGH_LEVEL

#define ENV_GPIO_CALLBACK_AMT           8

/* I2C definitions */
#define ENV_I2C_BITRATE_100000          FALSE
#define ENV_I2C_BITRATE_400000          TRUE

/* UART definitions */
#define ENV_UART_LEN_5                  UART_CONFIG_WLEN_5
#define ENV_UART_LEN_6                  UART_CONFIG_WLEN_6
#define ENV_UART_LEN_7                  UART_CONFIG_WLEN_7
#define ENV_UART_LEN_8                  UART_CONFIG_WLEN_8

#define ENV_UART_STOP_ONE               UART_CONFIG_STOP_ONE
#define ENV_UART_STOP_TWO               UART_CONFIG_STOP_TWO

#define ENV_UART_PAR_NONE               UART_CONFIG_PAR_NONE
#define ENV_UART_PAR_EVEN               UART_CONFIG_PAR_EVEN
#define ENV_UART_PAR_ODD                UART_CONFIG_PAR_ODD
#define ENV_UART_PAR_ZERO               UART_CONFIG_PAR_ZERO
#define ENV_UART_PAR_ONE                UART_CONFIG_PAR_ONE

#if (defined(TIVAWARE) || defined(MWARE))
/* SPI definitions */
#define ENV_SPI_MASTER                  SSI_MODE_MASTER
#define ENV_SPI_SLAVE                   SSI_MODE_SLAVE
#define ENV_SPI_SLAVE_OD                SSI_MODE_SLAVE_OD

#define ENV_SPI_POL0_PHA0               SSI_FRF_MOTO_MODE_0
#define ENV_SPI_POL0_PHA1               SSI_FRF_MOTO_MODE_1
#define ENV_SPI_POL1_PHA0               SSI_FRF_MOTO_MODE_2
#define ENV_SPI_POL1_PHA1               SSI_FRF_MOTO_MODE_3
#define ENV_SPI_TI                      SSI_FRF_TI
#define ENV_SPI_MW                      SSI_FRF_NMW

#else
/* SPI definitions */
#define ENV_SPI_MASTER                  MCSPI_MASTER_MODE
#define ENV_SPI_SLAVE                   MCSPI_SLAVE_MODE
#define ENV_SPI_SLAVE_OD                (0) /* Not supported on CC3101 */

// TODO Replace these values.
#define ENV_SPI_POL0_PHA0               (0)
#define ENV_SPI_POL0_PHA1               MCSPI_CH0CONF_PHA
#define ENV_SPI_POL1_PHA0               MCSPI_CH0CONF_POL
#define ENV_SPI_POL1_PHA1               MCSPI_CH0CONF_POL | MCSPI_CH0CONF_PHA
#define ENV_SPI_TI                      (0) /* Not supported on CC3101 */
#define ENV_SPI_MW                      (0) /* Not supported on CC3101 */
#endif

#elif defined(STARTERWARE)
#include <include/gpio_v2.h>
#include <include/hsi2c.h>
#include <include/mcspi.h>
#include <include/uart_irda_cir.h>

/* All peripherals */
#define ENV_MODULE_INPUT_CLK            (48000000)

/* GPIO peripheral */
#define ENV_GPIO_INT_FALLING            GPIO_INT_TYPE_FALL_EDGE
#define ENV_GPIO_INT_RISING             GPIO_INT_TYPE_RISE_EDGE
#define ENV_GPIO_INT_BOTH_EDGES         GPIO_INT_TYPE_BOTH_EDGE
#define ENV_GPIO_INT_LOW                GPIO_INT_TYPE_LEVEL_LOW
#define ENV_GPIO_INT_HIGH               GPIO_INT_TYPE_LEVEL_HIGH

#define ENV_GPIO_CALLBACK_AMT           32

/* UART peripheral */
#define ENV_UART_LEN_5                  UART_FRAME_WORD_LENGTH_5
#define ENV_UART_LEN_6                  UART_FRAME_WORD_LENGTH_6
#define ENV_UART_LEN_7                  UART_FRAME_WORD_LENGTH_7
#define ENV_UART_LEN_8                  UART_FRAME_WORD_LENGTH_8

#define ENV_UART_STOP_ONE               UART_FRAME_NUM_STB_1
#define ENV_UART_STOP_TWO               UART_FRAME_NUM_STB_1_5_2

#define ENV_UART_PAR_NONE               UART_PARITY_NONE
#define ENV_UART_PAR_EVEN               UART_EVEN_PARITY
#define ENV_UART_PAR_ODD                UART_ODD_PARITY
#define ENV_UART_PAR_ZERO               UART_PARITY_REPR_0
#define ENV_UART_PAR_ONE                UART_PARITY_REPR_1

/* I2C definitions */
#define ENV_I2C_BITRATE_100000          (100000)
#define ENV_I2C_BITRATE_400000          (400000)

/* SPI peripheral */
#define ENV_SPI_MASTER                  MCSPI_TX_RX_MODE
#define ENV_SPI_MASTER_ONLY             MCSPI_TX_ONLY_MODE
#define ENV_SPI_SLAVE                   MCSPI_RX_ONLY_MODE
#define ENV_SPI_SLAVE_OD                (0)

#define ENV_SPI_POL0_PHA0               MCSPI_CLK_MODE_0
#define ENV_SPI_POL0_PHA1               MCSPI_CLK_MODE_1
#define ENV_SPI_POL1_PHA0               MCSPI_CLK_MODE_2
#define ENV_SPI_POL1_PHA1               MCSPI_CLK_MODE_3
#define ENV_SPI_TI                      (0)
#define ENV_SPI_MW                      (0)

#else
#error This platform is not supported by TI-RTOS currently.

#endif

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_ENV__include */
