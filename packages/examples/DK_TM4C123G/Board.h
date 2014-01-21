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

#include "DK_TM4C123G.h"

#define Board_initDMA               DK_TM4C123G_initDMA
#define Board_initGeneral           DK_TM4C123G_initGeneral
#define Board_initGPIO              DK_TM4C123G_initGPIO
#define Board_initI2C               DK_TM4C123G_initI2C
#define Board_initSDSPI             DK_TM4C123G_initSDSPI
#define Board_initSPI               DK_TM4C123G_initSPI
#define Board_initUART              DK_TM4C123G_initUART
#define Board_initUSB               DK_TM4C123G_initUSB
#define Board_initUSBMSCHFatFs      DK_TM4C123G_initUSBMSCHFatFs
#define Board_initWatchdog          DK_TM4C123G_initWatchdog
#define Board_initWiFi              DK_TM4C123G_initWiFi

#define Board_LED_ON                DK_TM4C123G_LED_ON
#define Board_LED_OFF               DK_TM4C123G_LED_OFF
#define Board_LED0                  DK_TM4C123G_LED
#define Board_LED1                  DK_TM4C123G_LED
#define Board_LED2                  DK_TM4C123G_LED
#define Board_BUTTON0               DK_TM4C123G_SW3
#define Board_BUTTON1               DK_TM4C123G_SW4

#define Board_I2C0                  DK_TM4C123G_I2C0
#define Board_I2C1                  DK_TM4C123G_I2C2

#define Board_SDSPI0                DK_TM4C123G_SDSPI0

#define Board_SPI0                  DK_TM4C123G_SPI1
#define Board_SPI1                  DK_TM4C123G_SPI3
#define Board_SPI_CC3000            DK_TM4C123G_SPI3
#define Board_SPI_AT45DB
#define Board_SPI_AT45CS

#define Board_USBMSCHFatFs0         DK_TM4C123G_USBMSCHFatFs0

#define Board_USBHOST               DK_TM4C123G_USBHOST
#define Board_USBDEVICE             DK_TM4C123G_USBDEVICE

#define Board_UART0                 DK_TM4C123G_UART0

#define Board_WATCHDOG0             DK_TM4C123G_WATCHDOG0

#define Board_WIFI                  DK_TM4C123G_WIFI

#define Board_gpioCallbacks0        DK_TM4C123G_gpioPortMCallbacks
#define Board_gpioCallbacks1        DK_TM4C123G_gpioPortMCallbacks

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
