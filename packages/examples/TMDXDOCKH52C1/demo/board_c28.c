/*
 * Copyright (c) 2012, Texas Instruments Incorporated
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

/*
 *  ======== board_c28.c ========
 */

#include <xdc/std.h>

#include <F28M35x_Device.h>             /* F28M35x Headerfile Include File */
#include <F28M35x_Examples.h>           /* F28M35x Examples Include File */
#include <F28M35x_GlobalPrototypes.h>   /* F28M35x Global Prototypes */

/*
 *  ======== TMDXDOCKH52C1_c28_generalSetup ========
 */
Void TMDXDOCKH52C1_c28_generalSetup(Void)
{
    InitSysCtrl();
    InitGpio();
}

/*
 *  ======== TMDXDOCKH52C1_c28_LoggerIdle_uartSetup ========
 */
Void TMDXDOCKH52C1_c28_LoggerIdle_uartSetup(Void)
{
    /* Initialize GPIO pins for SCI */
    EALLOW;

    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;   /* Asynch input GPIO28 (SCIRXDA) */
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;    /* Configure GPIO28 for SCIRXDA */
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;    /* Configure GPIO29 for SCITXDA */

    EDIS;

    /* Enable TX and RX FIFO */
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;

    /*
     *  Initialize the SCI peripheral
     *
     *  SCICCR:
     *  Character length 8 bits
     *  1 stop bit
     *  Parity not enabled
     *  Idle-line mode protocol selected
     *
     *  SCICTL1:
     *  Enable RX and TX
     *  Leave SLEEP mode disabled
     *  Leave RX Err disabled
     *  Leave transmitter wakeup feature disabled
     *
     *  SCICTL2:
     *  Enable RXRDY/BRKDT interrupt
     *
     *  SCIHBAUD/SCILBAUD:
     *  115200 baud rate
     *
     *  Enable TX and RX FIFO
     */
    SciaRegs.SCICCR.bit.SCICHAR =(0x0008-1);
    SciaRegs.SCICCR.bit.STOPBITS = 0;
    SciaRegs.SCICCR.bit.PARITYENA = 0;
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;
    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
    SciaRegs.SCIHBAUD    =0x0000;
    SciaRegs.SCILBAUD    =0x000F;

    /* Release the SCI from Reset */
    SciaRegs.SCICTL1.all =0x0023;
}

/*
 *  ======== TMDXDOCKH52C1_c28_uartSendC28 ========
 */
Int TMDXDOCKH52C1_c28_uartSendC28(UChar *a, Int i)
{
    /* Wait for TX buffer be have space */
    while (SciaRegs.SCIFFTX.bit.TXFFST > 12) {
    }
    SciaRegs.SCITXBUF = *a++;
    SciaRegs.SCITXBUF = *a++;
    SciaRegs.SCITXBUF = *a++;
    SciaRegs.SCITXBUF = *a++;
    return 4;
}
