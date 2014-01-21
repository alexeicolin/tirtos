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
 *  @file       UARTUSCIA.h
 *
 *  @brief      UART driver implementation for a USCIA peripheral
 *
 *  The UART header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/UART.h>
 *  #include <ti/drivers/uart/UARTUSCIA.h>
 *  @endcode
 *
 *  This UART driver implementation is designed to operate on a UCSI controller
 *  in UART mode. It uses the APIs for a USCIA controller.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_uart_UARTUSCIA__include
#define ti_drivers_uart_UARTUSCIA__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/drivers/UART.h>

#include <ti/sysbios/knl/Semaphore.h>

/* UARTUSCIA function table pointer */
extern const UART_FxnTable UARTUSCIA_fxnTable;

/*!
 *  @brief      UARTUSCIA Baudrate configuration
 *
 *  This structure is used to specifiy the usci controller's clock divider
 *  settings to achieve the desired baudrate given the indicated clock input
 *  frequency.
 *  Divider values can be determined by referring to the MSP430 baudrate
 *  calculator.
 *  http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 *
 *  A sample structure is shown below:
 *  @code
 *  const UARTUSCIA_BaudrateConfig uartUSCIABaudrates[] = {
 *   // Baudrate, input clock (Hz), Prescalar, UCBRFx, UCBRSx, Oversampling
 *      {115200,  8192000,          4,         7,      0,      1},
 *      {9600,    8192000,          53,        5,      0,      1},
 *      {9600,    32768,            3,         0,      3,      0},
 *  };
 *  @endcode
 */
typedef struct UARTUSCIA_BaudrateConfig {
    ULong   outputBaudrate; /*! Search criteria: desired baudrate */
    UInt32  inputClockFreq; /*! Search criteria: given this input clock frequency */

    UInt8   prescalar;      /*! Clock prescalar */
    UInt8   hwRegUCBRFx;    /*! UCBRFx lookup entry */
    UInt8   hwRegUCBRSx;    /*! UCBRSx lookup entry */
    UInt8   sampling;       /*! Oversampling mode (1: True; 0: False) */
} UARTUSCIA_BaudrateConfig;

/*!
 *  @brief      UARTUSCIA Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For MSP430Ware these definitions are found in:
 *      - inc/hw_memmap.h
 *      - usci_a_uart.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const UARTUSCIA_BaudrateConfig uartUSCIABaudrates[] = {
 *   // Baudrate, input clock (Hz), UCBRx, UCBRFx, UCBRSx, Oversampling
 *      {115200,  8192000,          4,     7,      0,      1},
 *      {9600,    8192000,          53,    5,      0,      1},
 *      {9600,    32768,            3,     0,      3,      0},
 *  };
 *
 *  const UARTUSCIA_HWAttrs uartUSCIAHWAttrs[] = {
 *      {
 *          USCI_A0_BASE,
 *          USCI_A_UART_CLOCKSOURCE_SMCLK,
 *          USCI_A_UART_LSB_FIRST,
 *          sizeof(uartUSCIABaudrates/UARTUSCIA_BaudrateConfig),
 *          uartUSCIABaudrates
 *      },
 *      {
 *          USCI_A1_BASE,
 *          USCI_A_UART_CLOCKSOURCE_SMCLK,
 *          USCI_A_UART_LSB_FIRST,
 *          sizeof(uartUSCIABaudrates/UARTUSCIA_BaudrateConfig),
 *          uartUSCIABaudrates
 *      },
 *  };
 *  @endcode
 */
typedef struct UARTUSCIA_HWAttrs {
    /*! USCI_A_UART Peripheral's base address */
    ULong   baseAddr;
    /*! USCI_A_UART Clock source */
    UChar   clockSource;
    /*!< USCI_A_UART Bit order */
    UInt    bitOrder;
    /*!< Number of UARTUSCIA_BaudrateConfig entries */
    UInt    numBaudrateEntries;
    /*!< Pointer to a table of possible UARTUSCIA_BaudrateConfig entries */
    UARTUSCIA_BaudrateConfig const *baudrateLUT;
} UARTUSCIA_HWAttrs;

/*!
 *  @brief      UARTUSCIA Object
 *
 *  Not intended to be used by the user.
 */
typedef struct UARTUSCIA_Object {
    /* UARTUSCIA control variables */
    Bool                 isOpen;           /* Status for open */
    UART_Mode            readMode;         /* Mode for all read calls */
    UART_Mode            writeMode;        /* Mode for all write calls */
    UInt32               readTimeout;      /* Timeout for read semaphore */
    UInt32               writeTimeout;     /* Timeout for write semaphore */
    UART_Callback        readCallback;     /* Pointer to read callback */
    UART_Callback        writeCallback;    /* Pointer to write callback */
    UART_ReturnMode      readReturnMode;   /* Receive return mode */
    UART_DataMode        readDataMode;     /* Type of data being read */
    UART_DataMode        writeDataMode;    /* Type of data being written */
    UART_Echo            readEcho;         /* Echo received data back */

    /* UARTUSCIA write variables */
    const Char          *writeBuf;         /* Buffer data pointer */
    UInt                 writeCount;       /* Number of Chars sent */
    UInt                 writeSize;        /* Chars remaining in buffer */
    Bool                 writeCR;          /* Write a return character */

    /* UARTUSCIA receive variables */
    Char                *readBuf;          /* Buffer data pointer */
    UInt                 readCount;        /* Number of Chars read */
    UInt                 readSize;         /* Chars remaining in buffer */

    /* UARTUSCIA SYS/BIOS objects */
    Semaphore_Struct     writeSem;         /* UARTUSCIA write semaphore*/
    Semaphore_Struct     readSem;          /* UARTUSCIA read semaphore */
} UARTUSCIA_Object, *UARTUSCIA_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_uart_UARTUSCIA__include */
