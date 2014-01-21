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
 *  @file       UARTTiva.h
 *
 *  @brief      UART driver implementation for a Tiva UART controller
 *
 *  The UART header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/UART.h>
 *  #include <ti/drivers/uart/UARTTiva.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef ti_drivers_uart_UARTTiva__include
#define ti_drivers_uart_UARTTiva__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/drivers/UART.h>
/*
 * The following allows this header file to be included in an application file
 * which also includes ti/sysbios/hal/Hwi.h.
 */
#define ti_sysbios_family_arm_m3_Hwi__nolocalnames
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

/* UART function table pointer */
extern const UART_FxnTable UARTTiva_fxnTable;

/*!
 *  @brief      UARTTiva Hardware attributes
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For TivaWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - inc/hw_ints.h
 *
 *  A sample structure is shown below:
 *  @code
 *  const UARTTiva_HWAttrs uartTivaHWAttrs[] = {
 *      {
 *          UART1_BASE,
 *          INT_UART1
 *      },
 *      {
 *          UART3_BASE,
 *          INT_UART3
 *      },
 *  };
 *  @endcode
 */
typedef struct UARTTiva_HWAttrs {
    /*! UART Peripheral's base address */
    ULong   baseAddr;
    /*! UART Peripheral's interrupt vector */
    Int     intNum;
} UARTTiva_HWAttrs;

/*!
 *  @brief      UARTTiva Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct UARTTiva_Object {
    /* UART control variables */
    Bool                 opened;           /* Has the obj been opened */
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

    /* UART write variables */
    const Char          *writeBuf;         /* Buffer data pointer */
    UInt                 writeCount;       /* Number of Chars sent */
    UInt                 writeSize;        /* Chars remaining in buffer */
    Bool                 writeCR;          /* Write a return character */

    /* UART receive variables */
    Char                *readBuf;          /* Buffer data pointer */
    UInt                 readCount;        /* Number of Chars read */
    UInt                 readSize;         /* Chars remaining in buffer */

    /* UART SYS/BIOS objects */
    ti_sysbios_family_arm_m3_Hwi_Struct hwi; /* Hwi object */
    Semaphore_Struct     writeSem;         /* UART write semaphore*/
    Semaphore_Struct     readSem;          /* UART read semaphore */
} UARTTiva_Object, *UARTTiva_Handle;

/* Do not interfere with the app if they include the family Hwi module */
#undef ti_sysbios_family_arm_m3_Hwi__nolocalnames

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_uart_UARTTiva__include */
