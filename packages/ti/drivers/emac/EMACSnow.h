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
 *  @file       EMACSnow.h
 *
 *  @brief      EMACSnow Driver
 *
 *  The EMACSnow header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/emac/EMACSnow.h>
 *  @endcode
 *
 *  The EMACSnow driver produces log statements using the instrumented library. The
 *  following masks are used:
 *   - Diags_USER1 mask logs basic information
 *   - Diags_USER2 mask logs more detailed information
 *
 *  ============================================================================
 */

#ifndef ti_drivers_emac_EMACSnow__include
#define ti_drivers_emac_EMACSnow__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/sysbios/knl/Swi.h>

#include <ti/drivers/EMAC.h>

#define _INCLUDE_NIMU_CODE
#include <ti/ndk/inc/stkmain.h>

/*! @brief  EMACSnow function table */
extern const EMAC_FxnTable EMACSnow_fxnTable;

/*!
 *  @brief  EMACSnow Hardware attributes
 */
typedef struct EMACSnow_HWAttrs {
    ULong baseAddr; /*!< EMAC port */
    UInt8 intNum;    /*!< Interrupt Vector Id */
    UInt8 *macAddress;  /*!< Pointer to MAC address */
} EMACSnow_HWAttrs;

/*!
 *  @brief  EMACSnow Object
 *
 *  This structure should not be directly accessed. It is specified
 *  here to allow the application to supply the needed memory to the
 *  EMACSnow module.
 */
typedef struct EMACSnow_Object {
    Swi_Handle      swi;
	Hwi_Handle      hwi;
} EMACSnow_Object;

/*!
 *  @brief  This function initializes the EMACSnow driver
 *
 *  This function must be called before the NDK stack thread is started.
 *
 *  The EMACSnow_config structure must be present and initialized before this function
 *  is called.
 */
extern Void EMACSnow_init(UInt index);

/*!
 *  @brief  This function sets the EMACSnow driver
 *
 *  @brief  This function returns if the link is up
 *
 *  @return TRUE is the link is up. FALSE if it is down.
 */
extern Bool EMACSnow_isLinkUp(UInt index);

/*!
 *  @brief  This function is needed in the NIMUDeviceTable that is required
 *          by the NDK.
 */
extern Int EMACSnow_NIMUInit(STKEVENT_Handle hEvent);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_emac_EMACSnow__include */
