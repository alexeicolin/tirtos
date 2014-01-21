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

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "TMDXDOCKH52C1.h"

#define Board_initDMA               TMDXDOCKH52C1_initDMA
#define Board_initEMAC              TMDXDOCKH52C1_initEMAC
#define Board_initGeneral           TMDXDOCKH52C1_initGeneral
#define Board_initGPIO              TMDXDOCKH52C1_initGPIO
#define Board_initI2C               TMDXDOCKH52C1_initI2C
#define Board_initSDSPI             TMDXDOCKH52C1_initSDSPI
#define Board_initSPI               TMDXDOCKH52C1_initSPI
#define Board_initUART              TMDXDOCKH52C1_initUART
#define Board_initUSB               TMDXDOCKH52C1_initUSB
#define Board_initUSBMSCHFatFs      TMDXDOCKH52C1_initUSBMSCHFatFs
#define Board_initWatchdog          TMDXDOCKH52C1_initWatchdog

#define Board_LED_ON                TMDXDOCKH52C1_LED_ON
#define Board_LED_OFF               TMDXDOCKH52C1_LED_OFF
#define Board_LED0                  TMDXDOCKH52C1_LD2
#define Board_LED1                  TMDXDOCKH52C1_LD3
#define Board_LED2                  TMDXDOCKH52C1_LD3
#define Board_BUTTON0               TMDXDOCKH52C1_BUTTON
#define Board_BUTTON1               TMDXDOCKH52C1_BUTTON

#define Board_I2C0                  TMDXDOCKH52C1_I2C0
#define Board_I2C1                  TMDXDOCKH52C1_I2C1
#define Board_I2C_TMP               TMDXDOCKH52C1_I2C1
#define Board_I2C_EEPROM            TMDXDOCKH52C1_I2C0

#define Board_SDSPI0                TMDXDOCKH52C1_SDSPI0

#define Board_SPI0                  TMDXDOCKH52C1_SPI0
#define Board_SPI1                  TMDXDOCKH52C1_SPI1
#define Board_SPI_AT45DB
#define Board_SPI_AT45CS

#define Board_USBMSCHFatFs0         TMDXDOCKH52C1_USBMSCHFatFs0

#define Board_USBHOST               TMDXDOCKH52C1_USBHOST
#define Board_USBDEVICE             TMDXDOCKH52C1_USBDEVICE

#define Board_UART0                 TMDXDOCKH52C1_UART0

#define Board_WATCHDOG0             TMDXDOCKH52C1_WATCHDOG0

#define Board_gpioCallbacks0        TMDXDOCKH52C1_gpioPortBCallbacks
#define Board_gpioCallbacks1        TMDXDOCKH52C1_gpioPortBCallbacks

#define Board_EEPROMADDRESS         TMDXDOCKH52C1_EEPROMADDRESS
#define Board_EEPROMPAGESIZE        TMDXDOCKH52C1_EEPROMPAGESIZE
#define Board_EEPROMADDRESSIZE      TMDXDOCKH52C1_EEPROMADDRESSIZE
#define Board_EEPROMSTARTADDR       TMDXDOCKH52C1_EEPROMSTARTADDR

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
