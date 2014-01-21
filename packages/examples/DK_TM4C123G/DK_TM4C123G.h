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
 *  @file       DK_TM4C123G.h
 *
 *  @brief      DK_TM4C123G Board Specific APIs
 *
 *  The DK_TM4C123G header file should be included in an application as follows:
 *  @code
 *  #include <DK_TM4C123G.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __DK_TM4C123G_H
#define __DK_TM4C123G_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on DK_TM4C123G are active high. */
#define DK_TM4C123G_LED_OFF (0)
#define DK_TM4C123G_LED_ON  (~0)

/* GPIO_Callbacks structure for GPIO interrupts */
extern const GPIO_Callbacks DK_TM4C123G_gpioPortMCallbacks;

/*!
 *  @def    DK_TM4C123G_GPIOName
 *  @brief  Enum of GPIO names on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_GPIOName {
    DK_TM4C123G_LED = 0,
    DK_TM4C123G_SW1,
    DK_TM4C123G_SW2,
    DK_TM4C123G_SW3,
    DK_TM4C123G_SW4,
    DK_TM4C123G_SW5,

    DK_TM4C123G_GPIOCOUNT
} DK_TM4C123G_GPIOName;

/*!
 *  @def    DK_TM4C123G_I2CName
 *  @brief  Enum of I2C names on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_I2CName {
    DK_TM4C123G_I2C0 = 0,
    DK_TM4C123G_I2C2,

    DK_TM4C123G_I2CCOUNT
} DK_TM4C123G_I2CName;

/*!
 *  @def    DK_TM4C123G_SDSPIName
 *  @brief  Enum of SDSPI names on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_SDSPIName {
    DK_TM4C123G_SDSPI0 = 0,

    DK_TM4C123G_SDSPICOUNT
} DK_TM4C123G_SDSPIName;

/*!
 *  @def    DK_TM4C123G_SPIName
 *  @brief  Enum of SPI names on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_SPIName {
    DK_TM4C123G_SPI0 = 0,
    DK_TM4C123G_SPI1,
    DK_TM4C123G_SPI2,
    DK_TM4C123G_SPI3,

    DK_TM4C123G_SPICOUNT
} DK_TM4C123G_SPIName;

/*!
 *  @def    DK_TM4C123G_UARTName
 *  @brief  Enum of UARTs on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_UARTName {
    DK_TM4C123G_UART0 = 0,

    DK_TM4C123G_UARTCOUNT
} DK_TM4C123G_UARTName;

/*!
 *  @def    DK_TM4C123G_USBMode
 *  @brief  Enum of USB setup function on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_USBMode {
    DK_TM4C123G_USBDEVICE,
    DK_TM4C123G_USBHOST
} DK_TM4C123G_USBMode;

/*!
 *  @def    DK_TM4C123G_USBMSCHFatFsName
 *  @brief  Enum of USBMSCHFatFs names on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_USBMSCHFatFsName {
    DK_TM4C123G_USBMSCHFatFs0 = 0,

    DK_TM4C123G_USBMSCHFatFsCOUNT
} DK_TM4C123G_USBMSCHFatFsName;

/*!
 *  @def    DK_TM4C123G_WatchdogName
 *  @brief  Enum of Watchdogs on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_WatchdogName {
    DK_TM4C123G_WATCHDOG0 = 0,

    DK_TM4C123G_WATCHDOGCOUNT
} DK_TM4C123G_WatchdogName;

/*!
 *  @def    DK_TM4C123G_WiFiName
 *  @brief  Enum of WiFi names on the DK_TM4C123G dev board
 */
typedef enum DK_TM4C123G_WiFiName {
    DK_TM4C123G_WIFI = 0,

    DK_TM4C123G_WIFICOUNT
} DK_TM4C123G_WiFiName;

/*!
 *  @brief  Initialize board specific DMA settings
 *
 *  This function creates a hwi in case the DMA controller creates an error
 *  interrrupt, enables the DMA and supplies it with a uDMA control table.
 */
extern Void DK_TM4C123G_initDMA(Void);

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern Void DK_TM4C123G_initGeneral(Void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern Void DK_TM4C123G_initGPIO(Void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern Void DK_TM4C123G_initI2C(Void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern Void DK_TM4C123G_initSDSPI(Void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern Void DK_TM4C123G_initSPI(Void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern Void DK_TM4C123G_initUART(Void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
extern Void DK_TM4C123G_initUSB(DK_TM4C123G_USBMode usbMode);

/*!
 *  @brief  Initialize board specific USBMSCHFatFs settings
 *
 *  This function initializes the board specific USBMSCHFatFs settings and then
 *  calls the USBMSCHFatFs_init API to initialize the USBMSCHFatFs module.
 *
 *  The USBMSCHFatFs peripherals controlled by the USBMSCHFatFs module are
 *  determined by the USBMSCHFatFs_config variable.
 */
extern Void DK_TM4C123G_initUSBMSCHFatFs(Void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern Void DK_TM4C123G_initWatchdog(Void);

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 */
extern Void DK_TM4C123G_initWiFi(Void);

#ifdef __cplusplus
}
#endif

#endif /* __DK_TM4C123G_H */
