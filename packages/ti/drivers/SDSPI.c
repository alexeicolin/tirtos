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

#include <ti/drivers/SDSPI.h>

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>

/* FatFs */
#include <ti/sysbios/fatfs/ffconf.h>

/* Externs */
extern const SDSPI_Config SDSPI_config[];

/* Used to check status and initialization */
static Int SDSPI_count = -1;

/* Default SDSPI parameters structure */
const SDSPI_Params SDSPI_defaultParams = {
#if defined (MSP430WARE)
    2500000
#else
    12500000
#endif
};

/*
 *  ======== SDSPI_close ========
 */
Void SDSPI_close(SDSPI_Handle handle)
{
    Assert_isTrue((handle != NULL) && (SDSPI_count != -1), NULL);

    handle->fxnTablePtr->closeFxn(handle);
}

/*
 *  ======== SDSPI_init ========
 */
Void SDSPI_init(Void)
{
    if (SDSPI_count == -1) {
        /* Call each driver's init function */
        for (SDSPI_count = 0; SDSPI_config[SDSPI_count].fxnTablePtr != NULL; SDSPI_count++) {
            SDSPI_config[SDSPI_count].fxnTablePtr->initFxn((SDSPI_Handle)&(SDSPI_config[SDSPI_count]));
        }
    }
}

/*
 *  ======== SDSPI_open ========
 */
SDSPI_Handle SDSPI_open(UInt index, UInt drv, SDSPI_Params *params)
{
    SDSPI_Handle      handle;

    /* _VOLUMES is defined in <ti/sysbios/fatfs/ffconf.h> */
    Assert_isTrue((index < SDSPI_count) && (drv <= _VOLUMES), NULL);

    /* Get handle for this driver instance */
    handle = (SDSPI_Handle)&(SDSPI_config[index]);

    return (handle->fxnTablePtr->openFxn(handle, drv, params));
}

/*
 *  ======== SDSPI_Params_init ========
 *
 *  Defaults values are:
 *  @code
 *  bitRate             = 12500000 (Hz)
 *  @endcode
 *
 *  @param  params  Parameter structure to initialize
 */
Void SDSPI_Params_init(SDSPI_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    *params = SDSPI_defaultParams;
}
