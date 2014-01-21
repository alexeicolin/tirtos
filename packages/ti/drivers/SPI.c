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
/*
 *  ======== SPI.c ========
 */

#include <ti/drivers/SPI.h>

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

/* Externs */
extern const SPI_Config SPI_config[];

/* Used to check status and initialization */
static Int SPI_count = -1;

/* Default SPI parameters structure */
const SPI_Params SPI_defaultParams = {
    SPI_MODE_BLOCKING,  /* transferMode */
    NULL,               /* transferCallbackFxn */
    SPI_MASTER,         /* mode */
    1000000,            /* bitRate */
    8,                  /* dataSize */
    SPI_POL0_PHA0       /* frameFormat */
};

/*
 *  ======== SPI_close ========
 */
Void SPI_close(SPI_Handle handle)
{
    Assert_isTrue((handle != NULL) && (SPI_count != -1), NULL);

    handle->fxnTablePtr->closeFxn(handle);
}

/*
 *  ======== SPI_init ========
 */
Void SPI_init(Void)
{
    if (SPI_count == -1) {
        /* Call each driver's init function */
        for (SPI_count = 0; SPI_config[SPI_count].fxnTablePtr != NULL; SPI_count++) {
            SPI_config[SPI_count].fxnTablePtr->initFxn((SPI_Handle)&(SPI_config[SPI_count]));
        }
    }
}

/*
 *  ======== SPI_open ========
 */
SPI_Handle SPI_open(UInt index, SPI_Params *params)
{
    SPI_Handle         handle;

    Assert_isTrue(index < SPI_count, NULL);

    /* Get handle for this driver instance */
    handle = (SPI_Handle)&(SPI_config[index]);

    return (handle->fxnTablePtr->openFxn(handle, params));
}

/*
 *  ======== SPI_Params_init ========
 */
Void SPI_Params_init(SPI_Params *params)
{
    Assert_isTrue(params != NULL, NULL);
    *params = SPI_defaultParams;
}

/*
 *  ======== SPI_serviceISR ========
 */
Void SPI_serviceISR(SPI_Handle handle)
{
    Assert_isTrue((handle != NULL), NULL);

    handle->fxnTablePtr->serviceISRFxn(handle);
}

/*
 *  ======== SPI_transfer ========
 */
Bool SPI_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    Assert_isTrue((handle != NULL) && (transaction != NULL), NULL);

    return (handle->fxnTablePtr->transferFxn(handle, transaction));
}
