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
 *  @file       TMDXDOCKH52C1.h
 *
 *  @brief      TMDXDOCKH52C1 Board Specific APIs
 *
 *  The TMDXDOCKH52C1 header file should be included in an application as follows:
 *  @code
 *  #include <TMDXDOCKH52C1.h>
 *  @endcode
 *
 *  ============================================================================
 */


#ifndef __TMDXDOCKH52C1_H
#define __TMDXDOCKH52C1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on TMDXDOCKH52C1 are active low. */
#define TMDXDOCKH52C1_LED_OFF (~0)
#define TMDXDOCKH52C1_LED_ON  (0)

/* GPIO_Callbacks structure for GPIO interrupts */
extern const GPIO_Callbacks TMDXDOCKH52C1_gpioPortBCallbacks;

/* I2C Address is represented as a 7-bit address */
#define TMDXDOCKH52C1_EEPROMADDRESS       0x50
#define TMDXDOCKH52C1_EEPROMPAGESIZE      64
#define TMDXDOCKH52C1_EEPROMADDRESSIZE    2
#define TMDXDOCKH52C1_EEPROMSTARTADDR     0

/*!
 *  @def    TMDXDOCKH52C1_EMACName
 *  @brief  Enum of EMAC names on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_EMACName {
    TMDXDOCKH52C1_EMAC0 = 0,

    TMDXDOCKH52C1_EMACCOUNT
} TMDXDOCKH52C1_EMACName;

/*!
 *  @def    TMDXDOCKH52C1_GPIOName
 *  @brief  Enum of GPIO names on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_GPIOName {
    TMDXDOCKH52C1_LD2 = 0,
    TMDXDOCKH52C1_LD3,
    TMDXDOCKH52C1_BUTTON,

    TMDXDOCKH52C1_GPIOCOUNT
} TMDXDOCKH52C1_GPIOName;

/*!
 *  @def    TMDXDOCKH52C1_I2CName
 *  @brief  Enum of I2C names on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_I2CName {
    TMDXDOCKH52C1_I2C0 = 0,
    TMDXDOCKH52C1_I2C1,

    TMDXDOCKH52C1_I2CCOUNT
} TMDXDOCKH52C1_I2CName;

/*!
 *  @def    TMDXDOCKH52C1_SDSPIName
 *  @brief  Enum of SDSPI names on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_SDSPIName {
    TMDXDOCKH52C1_SDSPI0 = 0,

    TMDXDOCKH52C1_SDSPICOUNT
} TMDXDOCKH52C1_SDSPIName;

/*!
 *  @def    TMDXDOCKH52C1_SPIName
 *  @brief  Enum of SPI names on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_SPIName {
	TMDXDOCKH52C1_SPI0 = 0,
	TMDXDOCKH52C1_SPI1,
	TMDXDOCKH52C1_SPI2,
	TMDXDOCKH52C1_SPI3,

    TMDXDOCKH52C1_SPICOUNT
} TMDXDOCKH52C1_SPIName;

/*!
 *  @def    TMDXDOCKH52C1_UARTName
 *  @brief  Enum of UARTs on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_UARTName {
    TMDXDOCKH52C1_UART0 = 0,

    TMDXDOCKH52C1_UARTCOUNT
} TMDXDOCKH52C1_UARTName;

/*!
 *  @def    TMDXDOCKH52C1_USBMode
 *  @brief  Enum of USB setup function on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_USBMode {
    TMDXDOCKH52C1_USBDEVICE,
    TMDXDOCKH52C1_USBHOST
} TMDXDOCKH52C1_USBMode;

/*!
 *  @def    TMDXDOCKH52C1_USBMSCHFatFsName
 *  @brief  Enum of USBMSCHFatFs names on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_USBMSCHFatFsName {
    TMDXDOCKH52C1_USBMSCHFatFs0 = 0,

    TMDXDOCKH52C1_USBMSCHFatFsCOUNT
} TMDXDOCKH52C1_USBMSCHFatFsName;

/*!
 *  @def    TMDXDOCKH52C1_WatchdogName
 *  @brief  Enum of Watchdogs on the TMDXDOCKH52C1 dev board
 */
typedef enum TMDXDOCKH52C1_WatchdogName {
    TMDXDOCKH52C1_WATCHDOG0 = 0,

    TMDXDOCKH52C1_WATCHDOGCOUNT
} TMDXDOCKH52C1_WatchdogName;

/*!
 *  @brief  Initialize board specific DMA settings
 *
 *  This function creates a hwi in case the DMA controller creates an error
 *  interrrupt, enables the DMA and supplies it with a uDMA control table.
 */
extern Void TMDXDOCKH52C1_initDMA(Void);

/*!
 *  @brief Initialize board specific EMAC settings
 *
 *  This function initializes the board specific EMAC settings and
 *  then calls the EMAC_init API to initialize the EMAC module.
 *
 *  The EMAC address is programmed as part of this call.
 *
 */
extern Void TMDXDOCKH52C1_initEMAC(Void);

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 *     - Disable clock source to watchdog module
 */
extern Void TMDXDOCKH52C1_initGeneral(Void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern Void TMDXDOCKH52C1_initGPIO(Void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern Void TMDXDOCKH52C1_initI2C(Void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern Void TMDXDOCKH52C1_initSDSPI(Void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern Void TMDXDOCKH52C1_initSPI(Void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern Void TMDXDOCKH52C1_initUART(Void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
extern Void TMDXDOCKH52C1_initUSB(TMDXDOCKH52C1_USBMode usbMode);

/*!
 *  @brief  Initialize board specific USBMSCHFatFs settings
 *
 *  This function initializes the board specific USBMSCHFatFs settings and then
 *  calls the USBMSCHFatFs_init API to initialize the USBMSCHFatFs module.
 *
 *  The USBMSCHFatFs peripherals controlled by the USBMSCHFatFs module are
 *  determined by the USBMSCHFatFs_config variable.
 */
extern Void TMDXDOCKH52C1_initUSBMSCHFatFs(Void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern Void TMDXDOCKH52C1_initWatchdog(Void);

#ifdef __cplusplus
}
#endif

#endif /* __TMDXDOCKH52C1_H */
