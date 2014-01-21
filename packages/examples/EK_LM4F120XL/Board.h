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

#include "EK_LM4F120XL.h"

#define Board_initDMA               EK_LM4F120XL_initDMA
#define Board_initGeneral           EK_LM4F120XL_initGeneral
#define Board_initGPIO              EK_LM4F120XL_initGPIO
#define Board_initI2C				EK_LM4F120XL_initI2C
#define Board_initSDSPI             EK_LM4F120XL_initSDSPI
#define Board_initSPI               EK_LM4F120XL_initSPI
#define Board_initUART              EK_LM4F120XL_initUART
#define Board_initUSB               EK_LM4F120XL_initUSB
#define Board_initWatchdog          EK_LM4F120XL_initWatchdog
#define Board_initWiFi              EK_LM4F120XL_initWiFi

#define Board_LED_ON                EK_LM4F120XL_LED_ON
#define Board_LED_OFF               EK_LM4F120XL_LED_OFF
#define Board_LED0                  EK_LM4F120XL_LED_BLUE
#define Board_LED1                  EK_LM4F120XL_LED_GREEN
#define Board_LED2                  EK_LM4F120XL_LED_RED
#define Board_BUTTON0               EK_LM4F120XL_SW1
#define Board_BUTTON1               EK_LM4F120XL_SW2

#define Board_I2C0
#define Board_I2C1
#define Board_I2C_TMP               EK_LM4F120XL_I2C3
#define Board_I2C_NFC               EK_LM4F120XL_I2C3
#define Board_I2C_TPL0401           EK_LM4F120XL_I2C3

#define Board_SDSPI0                EK_LM4F120XL_SDSPI0

#define Board_SPI0                  EK_LM4F120XL_SPI0
#define Board_SPI1                  EK_LM4F120XL_SPI3
#define Board_SPI_CC3000            EK_LM4F120XL_SPI2
#define Board_SPI_AT45DB
#define Board_SPI_CS0

#define Board_USBDEVICE             EK_LM4F120XL_USBDEVICE

#define Board_UART0                 EK_LM4F120XL_UART0

#define Board_WATCHDOG0             EK_LM4F120XL_WATCHDOG0

#define Board_WIFI                  EK_LM4F120XL_WIFI

#define Board_gpioCallbacks0        EK_LM4F120XL_gpioPortFCallbacks
#define Board_gpioCallbacks1        EK_LM4F120XL_gpioPortFCallbacks

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
