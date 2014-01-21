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
 *  ======== I2C.c ========
 */

#include <ti/drivers/I2C.h>

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

/* Externs */
extern const I2C_Config I2C_config[];

/* Used to check status and initialization */
static Int I2C_count = -1;

/* Default I2C parameters structure */
const I2C_Params I2C_defaultParams = {
    I2C_MODE_BLOCKING,  /* transferMode */
    NULL,               /* transferCallbackFxn */
    I2C_100kHz          /* bitRate */
};

/*
 *  ======== I2C_close ========
 */
Void I2C_close(I2C_Handle handle)
{
    Assert_isTrue((handle != NULL) && (I2C_count != -1), NULL);

    handle->fxnTablePtr->closeFxn(handle);
}

/*
 *  ======== I2C_init ========
 */
Void I2C_init(Void)
{
    if (I2C_count == -1) {
        /* Call each driver's init function */
        for (I2C_count = 0; I2C_config[I2C_count].fxnTablePtr != NULL; I2C_count++) {
            I2C_config[I2C_count].fxnTablePtr->initFxn((I2C_Handle)&(I2C_config[I2C_count]));
        }
    }
}

/*
 *  ======== I2C_open ========
 */
I2C_Handle I2C_open(UInt index, I2C_Params *params)
{
    I2C_Handle         handle;

    Assert_isTrue(index < I2C_count, NULL);

    /* Get handle for this driver instance */
    handle = (I2C_Handle)&(I2C_config[index]);

    return (handle->fxnTablePtr->openFxn(handle, params));
}

/*
 *  ======== I2C_Params_init =======
 */
Void I2C_Params_init(I2C_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    *params = I2C_defaultParams;
}

/*
 *  ======== I2C_transfer ========
 */
Bool I2C_transfer(I2C_Handle handle, I2C_Transaction *transaction)
{
    Assert_isTrue((handle != NULL) && (transaction != NULL), NULL);

    return (handle->fxnTablePtr->transferFxn(handle, transaction));
}
