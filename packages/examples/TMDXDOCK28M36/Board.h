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

#include "TMDXDOCK28M36.h"

#define Board_initDMA               TMDXDOCK28M36_initDMA
#define Board_initEMAC              TMDXDOCK28M36_initEMAC
#define Board_initGeneral           TMDXDOCK28M36_initGeneral
#define Board_initGPIO              TMDXDOCK28M36_initGPIO
#define Board_initI2C               TMDXDOCK28M36_initI2C
#define Board_initSDSPI             TMDXDOCK28M36_initSDSPI
#define Board_initSPI               TMDXDOCK28M36_initSPI
#define Board_initUART              TMDXDOCK28M36_initUART
#define Board_initUSB               TMDXDOCK28M36_initUSB
#define Board_initUSBMSCHFatFs      TMDXDOCK28M36_initUSBMSCHFatFs
#define Board_initWatchdog          TMDXDOCK28M36_initWatchdog

#define Board_LED_ON                TMDXDOCK28M36_LED_ON
#define Board_LED_OFF               TMDXDOCK28M36_LED_OFF
#define Board_LED0                  TMDXDOCK28M36_D1
#define Board_LED1                  TMDXDOCK28M36_D2
#define Board_LED2                  TMDXDOCK28M36_D2
#define Board_BUTTON0               TMDXDOCK28M36_BUTTON
#define Board_BUTTON1               TMDXDOCK28M36_BUTTON

#define Board_I2C0                  TMDXDOCK28M36_I2C0

#define Board_SDSPI0                TMDXDOCK28M36_SDSPI0

#define Board_SPI0					TMDXDOCK28M36_SPI0
#define Board_SPI1					TMDXDOCK28M36_SPI1
#define Board_SPI_AT45DB
#define Board_SPI_AT45CS

#define Board_USBMSCHFatFs0         TMDXDOCK28M36_USBMSCHFatFs0

#define Board_USBHOST               TMDXDOCK28M36_USBHOST
#define Board_USBDEVICE             TMDXDOCK28M36_USBDEVICE

#define Board_UART0                 TMDXDOCK28M36_UART0

#define Board_WATCHDOG0             TMDXDOCK28M36_WATCHDOG0

#define Board_gpioCallbacks0        TMDXDOCK28M36_gpioPortBCallbacks
#define Board_gpioCallbacks1        TMDXDOCK28M36_gpioPortBCallbacks

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
