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
 *  @file       EK_LM4F120XL.h
 *
 *  @brief      EK_LM4F120XL Board Specific APIs
 *
 *  The EK_LM4F120XL header file should be included in an application as follows:
 *  @code
 *  #include <EK_LM4F120XL.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __EK_LM4F120XL_H
#define __EK_LM4F120XL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on EK_LM4F120XL are active high. */
#define EK_LM4F120XL_LED_OFF (0)
#define EK_LM4F120XL_LED_ON  (~0)

/* GPIO_Callbacks structure for GPIO interrupts */
extern const GPIO_Callbacks EK_LM4F120XL_gpioPortFCallbacks;

/*!
 *  @def    EK_LM4F120XL_GPIOName
 *  @brief  Enum of GPIO names on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_GPIOName {
    EK_LM4F120XL_LED_RED = 0,
    EK_LM4F120XL_LED_BLUE,
    EK_LM4F120XL_LED_GREEN,
    EK_LM4F120XL_SW1,
    EK_LM4F120XL_SW2,

    EK_LM4F120XL_GPIOCOUNT
} EK_LM4F120XL_GPIOName;

/*!
 *  @def    EK_LM4F120XL_I2CName
 *  @brief  Enum of I2C names on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_I2CName {
    EK_LM4F120XL_I2C0 = 0,
    EK_LM4F120XL_I2C3,

    EK_LM4F120XL_I2CCOUNT
} EK_LM4F120XL_I2CName;

/*!
 *  @def    EK_LM4F120XL_SDSPIName
 *  @brief  Enum of SDSPI names on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_SDSPIName {
    EK_LM4F120XL_SDSPI0 = 0,

    EK_LM4F120XL_SDSPICOUNT
} EK_LM4F120XL_SDSPIName;

/*!
 *  @def    EK_LM4F120XL_SPIName
 *  @brief  Enum of SPI names on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_SPIName {
    EK_LM4F120XL_SPI0 = 0,
    EK_LM4F120XL_SPI2,
    EK_LM4F120XL_SPI3,

    EK_LM4F120XL_SPICOUNT
} EK_LM4F120XL_SPIName;

/*!
 *  @def    EK_LM4F120XL_UARTName
 *  @brief  Enum of UARTs on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_UARTName {
    EK_LM4F120XL_UART0 = 0,

    EK_LM4F120XL_UARTCOUNT
} EK_LM4F120XL_UARTName;

/*!
 *  @def    EK_LM4F120XL_USBMode
 *  @brief  Enum of USB setup function on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_USBMode {
    EK_LM4F120XL_USBDEVICE,
    EK_LM4F120XL_USBHOST
} EK_LM4F120XL_USBMode;

/*!
 *  @def    EK_LM4F120XL_WatchdogName
 *  @brief  Enum of Watchdogs on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_WatchdogName {
    EK_LM4F120XL_WATCHDOG0 = 0,

    EK_LM4F120XL_WATCHDOGCOUNT
} EK_LM4F120XL_WatchdogName;

/*!
 *  @def    EK_LM4F120XL_WiFiName
 *  @brief  Enum of WiFi names on the EK_LM4F120XL dev board
 */
typedef enum EK_LM4F120XL_WiFiName {
    EK_LM4F120XL_WIFI = 0,

    EK_LM4F120XL_WIFICOUNT
} EK_LM4F120XL_WiFiName;

/*!
 *  @brief  Initialize board specific DMA settings
 *
 *  This function creates a hwi in case the DMA controller creates an error
 *  interrrupt, enables the DMA and supplies it with a uDMA control table.
 */
extern Void EK_LM4F120XL_initDMA(Void);

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Flash wait states based on the process
 *     - Disable clock source to watchdog module
 *     - Enable clock sources for peripherals
 */
extern Void EK_LM4F120XL_initGeneral(Void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern Void EK_LM4F120XL_initGPIO(Void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern Void EK_LM4F120XL_initI2C(Void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern Void EK_LM4F120XL_initSDSPI(Void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern Void EK_LM4F120XL_initSPI(Void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern Void EK_LM4F120XL_initUART(Void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
extern Void EK_LM4F120XL_initUSB(EK_LM4F120XL_USBMode usbMode);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern Void EK_LM4F120XL_initWatchdog(Void);

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 */
extern Void EK_LM4F120XL_initWiFi(Void);

#ifdef __cplusplus
}
#endif

#endif /* __EK_LM4F120XL_H */
