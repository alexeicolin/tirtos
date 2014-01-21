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
 *  ======== SPIMessageQTransport.c ========
 */

#include <xdc/std.h>
#include <string.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/ipc/MessageQ.h>
#include <ti/sdo/ipc/_MessageQ.h>
#include <ti/sdo/ipc/interfaces/IMessageQTransport.h>
#include <ti/drivers/SPI.h>

#include "package/internal/SPIMessageQTransport.xdc.h"

/* Different values used in the first word of each message */
#define STARTOFMSG   0x1243abfe
#define HANDSHAKEMSG 0x22334455

Void SPIMessageQTransport_swiFxn(UArg arg0, UArg arg1);
static Bits32 SPIMessageQTransport_checksum(Char *buffer, Bits32 msgSize);

/*
 *  ======== SPIMessageQTransport_callBack ========
 *  Called by SPI driver when transaction is completed.
 */
Void SPIMessageQTransport_callBack(SPI_Handle spiHandle,
                                   SPI_Transaction *transaction)
{
    MessageQ_Msg rxMsg;
    MessageQ_Msg txMsg;
    MessageQ_Msg copiedRxMsg;
    MessageQ_QueueId messageQ;
    Bits32 msgSize;
    Bits32 computedChecksum;
    Bits32 receivedChecksum;
    Queue_Handle txHandle;
    SPIMessageQTransport_Object *obj;

    /* SPI driver should never give an empty transaction */
    Assert_isTrue(transaction != NULL, NULL);

    obj = (SPIMessageQTransport_Object *)(transaction->arg);
    Assert_isTrue((obj != NULL), SPIMessageQTransport_A_nullObject);

    /*
     * If there was a txMsg and it was not static, free it since
     * it was sent
     */
    txMsg = (MessageQ_Msg)transaction->txBuf;
    if ((txMsg != NULL) &&
        (txMsg->heapId != ti_sdo_ipc_MessageQ_STATICMSG)) {

        /*
         *  If the message was a handshake response on the slave,
         *  set the handshake state to up.
         */
        if ((((Bits32 *)txMsg)[0] == HANDSHAKEMSG) &&
            (obj->master == FALSE)) {
            obj->handshake = SPIMessageQTransport_LinkStatus_UP;
            Log_print0(Diags_USER1, "SPIMessageQTransport: slave handshake done");
        }
        MessageQ_free(txMsg);
    }

    /*
     *  If there was a received msg, process it. The first field being zero
     *  indicates that there is no receive message. This is set by the
     *  SPI driver. On real messages the value should be STARTOFMSG or
     *  HANDSHAKEMSG.
     */
    rxMsg = (MessageQ_Msg)transaction->rxBuf;
    Assert_isTrue((rxMsg != NULL), NULL);
    if (((Char *)rxMsg)[0] != 0) {

        if ((((Bits32 *)rxMsg)[0] != STARTOFMSG) &&
            (((Bits32 *)rxMsg)[0] != HANDSHAKEMSG)) {
            /* Drop the message */
            obj->rxMsgDropped++;
            Log_error1("SPIMessageQTransport: Invalid msg flag 0x%x on \
                        incoming message",
                       (UArg)((Bits32 *)rxMsg)[0]);
            SPIMessageQTransport_module->errFxn(
                    SPIMessageQTransport_Reason_PHYSICALERR,
                    (IMessageQTransport_Handle)obj,
                    rxMsg, SPIMessageQTransport_Failure_BADMSG);
            obj->ready = TRUE;
            /* The slave needs to be posted. The master has a clock function */
            if (obj->master == FALSE) {
                Swi_post(obj->swi);
            }
            return;
        }

        msgSize = MessageQ_getMsgSize(rxMsg);

        /* Only test if enabled and the incoming packet has a checksum */
        if ((SPIMessageQTransport_checksumEnabled == TRUE) &&
            (((Bits32 *)rxMsg)[1] != 0)) {
            receivedChecksum = ((Bits32 *)rxMsg)[1];
            computedChecksum = SPIMessageQTransport_checksum((Char *)rxMsg,
                                                             msgSize);
            if (computedChecksum != receivedChecksum) {
                /* Drop the message */
                obj->rxMsgDropped++;
                Log_error2("SPIMessageQTransport: Checksum failed received = 0x%x, computed = 0x%x",
                           receivedChecksum, computedChecksum);
                SPIMessageQTransport_module->errFxn(
                    SPIMessageQTransport_Reason_PHYSICALERR,
                    (IMessageQTransport_Handle)obj,
                    rxMsg, SPIMessageQTransport_Failure_BADCHECKSUM);
            }
        }

        /*
         *  Copy into allocated message. The copy is done to allow
         *  the correct heapId to be used.
         */
        copiedRxMsg = MessageQ_alloc(rxMsg->heapId, msgSize);
        if (copiedRxMsg == NULL) {
            /* Drop the message */
            obj->rxMsgDropped++;
            Log_error1("SPIMessageQTransport: No messages available in heap %d",
                       rxMsg->heapId);
                SPIMessageQTransport_module->errFxn(
                    SPIMessageQTransport_Reason_FAILEDALLOC,
                    (IMessageQTransport_Handle)obj,
                    NULL, rxMsg->heapId);
            return;
        }

        /*
         *  Copy in the allocated message and either
         *     - put to the final destination if app msg
         *     - process handshake
         */
        memcpy(copiedRxMsg, rxMsg, msgSize);
        messageQ = MessageQ_getDstQueue(copiedRxMsg);

        if (((Bits32 *)copiedRxMsg)[0] == STARTOFMSG) {
            MessageQ_put(messageQ, copiedRxMsg);
        }
        else {
            /*
             *  If the slave send the handshake back. Handshake is complete
             *  when we reclaim the sent msg in the SPI callback.
             *  If the master the handshake is complete, so free the msg
             */
            if (obj->master == FALSE) {
                txHandle = SPIMessageQTransport_Instance_State_txQueue(obj);
                Queue_put(txHandle, (Queue_Elem *)copiedRxMsg);
            }
            else {
                obj->handshake = SPIMessageQTransport_LinkStatus_UP;
                Log_print0(Diags_USER1, "SPIMessageQTransport: master handshake done");
                MessageQ_free(copiedRxMsg);
            }
        }
    }

    obj->ready = TRUE;

    /*
     *  The master side is driven by the periodic clock. The slave is not,
     *  so we need to post the Swi.
     */
    if (obj->master == FALSE) {
        Log_print0(Diags_USER2, "SPIMessageQTransport: Posting swi on slave");
        Swi_post(obj->swi);
    }
}

/*
 *  ======== SPIMessageQTransport_checksum ========
 */
static Bits32 SPIMessageQTransport_checksum(Char *buffer, Bits32 msgSize)
{
    UInt i;
    Bits32 checksum = 0;

    /*
     *  Skipping the first two words. The first 32-bits are the msg type and
     *  the next has the checksum.
     */
    for (i = 2 * sizeof(Bits32); i < msgSize; i++) {
        checksum += buffer[i];
    }

    return (checksum);
}

/*
 *  ======== SPIMessageQTransport_clkFxn ========
 */
Void SPIMessageQTransport_clkFxn(UArg arg)
{
    SPIMessageQTransport_Object *obj = (SPIMessageQTransport_Object *)arg;

    Swi_post(obj->swi);
}

/*
 *  ======== SPIMessageQTransport_control ========
 */
Bool SPIMessageQTransport_control(SPIMessageQTransport_Object *obj, UInt cmd,
                                  UArg cmdArg)
{
    return (FALSE);
}

/*
 *  ======== SPIMessageQTransport_getStatus ========
 */
Int SPIMessageQTransport_getStatus(SPIMessageQTransport_Object *obj)
{
    return (obj->handshake);
}

/*
 *  ======== SPIMessageQTransport_defaultErrFxn ========
 */
Void SPIMessageQTransport_defaultErrFxn(IMessageQTransport_Reason reason,
                                        IMessageQTransport_Handle handle,
                                        Ptr ptr,
                                        UArg arg)
{
}

/*
 *  ======== SPIMessageQTransport_Instance_finalize ========
 */
Void SPIMessageQTransport_Instance_finalize(SPIMessageQTransport_Object* obj,
                                            Int status)
{
    Memory_free(obj->heap, obj->rxMsg, obj->maxMsgSize);

    if (status == 2) {
        Memory_free(obj->heap, obj->transaction, sizeof(SPI_Transaction));
    }

    if (obj->clock != NULL) {
        Clock_delete(&(obj->clock));
    }

    if (obj->swi != NULL) {
        Swi_delete(&(obj->swi));
    }

    if (obj->spiHandle != NULL) {
        SPI_close(obj->spiHandle);
    }
}

/*
 *  ======== SPIMessageQTransport_Instance_init ========
 */
Int SPIMessageQTransport_Instance_init(SPIMessageQTransport_Object *obj,
        UInt16 procId, const SPIMessageQTransport_Params *params,
        Error_Block *eb)
{
    Queue_Handle txHandle;
    Clock_Params clockParams;
    Swi_Params swiParams;
    SPI_Params spiParams;
    MessageQ_Msg handshakeMsg = NULL;

    /* Grab the Queue handle and initialize it*/
    txHandle = SPIMessageQTransport_Instance_State_txQueue(obj);
    Queue_construct(Queue_struct(txHandle), NULL);

    /* Initialize the rest of the object. Each object gets it's own rx buffer */
    obj->maxMsgSize = params->maxMsgSize;
    obj->procId = procId;
    obj->rxMsgDropped = 0;
    obj->txMsgDropped = 0;
    obj->heap = params->heap;
    obj->handshake = SPIMessageQTransport_LinkStatus_DOWN;
    obj->master = params->master;
    obj->priority = params->transportPriority;
    obj->rxMsg = Memory_alloc(obj->heap, params->maxMsgSize, 0, eb);
    if (obj->rxMsg  == NULL) {
        return (1);
    }

    obj->transaction = Memory_alloc(obj->heap, sizeof(SPI_Transaction), 0, eb);
    if (obj->transaction  == NULL) {
        return (2);
    }

    /*
     *  The  transport instance gets it own clock function if it
     *  the SPI master.
     */
    if (obj->master == TRUE) {
        Clock_Params_init(&clockParams);
        clockParams.period = params->clockRate;
        clockParams.startFlag = TRUE;
        clockParams.arg = (UArg)obj;
        obj->clock = Clock_create(SPIMessageQTransport_clkFxn,
                                  params->clockStartDelay,
                                  &clockParams, eb);
        if (obj->clock == NULL) {
            return (3);
        }
    }
    else {
        obj->clock = NULL;
    }

    Swi_Params_init(&swiParams);
    swiParams.arg0 = (UArg)obj;
    swiParams.priority = params->swiPriority;
    obj->swi = Swi_create(SPIMessageQTransport_swiFxn, &swiParams, eb);
    Log_print1(Diags_USER1, "Created swi 0x%x", (UArg)(obj->swi));
    if (obj->swi == NULL) {
        return (4);
    }

    /* Open the SPI channel */
    SPI_Params_init(&spiParams);
    if (obj->master == TRUE) {
        spiParams.mode = SPI_MASTER;

        /* Allocate a handshake message */
        handshakeMsg = MessageQ_alloc(params->heapId, params->maxMsgSize);
        if (handshakeMsg == NULL) {
            return (5);
        }
        Queue_put(txHandle, (Queue_Elem *)handshakeMsg);
    }
    else {
        spiParams.mode = SPI_SLAVE;
    }
    spiParams.transferCallbackFxn = SPIMessageQTransport_callBack;
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.bitRate = params->spiBitRate;
    spiParams.dataSize = 16;
    obj->spiHandle = (Ptr)SPI_open(params->spiIndex, &spiParams);
    if (obj->spiHandle == NULL) {
        if (handshakeMsg != NULL) {
            MessageQ_free(handshakeMsg);
        }
        return (6);
    }

    obj->spiIndex = params->spiIndex;

    obj->ready = TRUE;

    if (obj->master == FALSE) {
        Swi_post(obj->swi);
    }

    Log_print1(Diags_USER1, "registered as %d", params->transportPriority);

    ti_sdo_ipc_MessageQ_registerTransport(
        SPIMessageQTransport_Handle_upCast(obj), procId, params->transportPriority);

    return (0);
}

/*
 *  ======== SPIMessageQTransport_put ========
 */
Bool SPIMessageQTransport_put(SPIMessageQTransport_Object *obj, Ptr msg)
{
    Queue_Handle txHandle;

    if (obj->handshake == SPIMessageQTransport_LinkStatus_DOWN) {
        Log_print2(Diags_USER2, "MessageQ_put failed for msg 0x%x. \
                   Transport 0x%x has not completed handshake",
                   (IArg)msg, (IArg)obj);
        return (FALSE);
    }

    Log_print2(Diags_USER2, "Placed msg 0x%x onto transport 0x%x's queue",
               (IArg)msg, (IArg)obj);
    txHandle = SPIMessageQTransport_Instance_State_txQueue(obj);
    Queue_put(txHandle, msg);

    return (TRUE);
}

/*
 *  ======== SPIMessageQTransport_setErrFxn ========
 */
Void SPIMessageQTransport_setErrFxn(SPIMessageQTransport_ErrFxn errFxn)
{
    SPIMessageQTransport_module->errFxn = errFxn;
}

/*
 *  ======== SPIMessageQTransport_swiFxn ========
 */
Void SPIMessageQTransport_swiFxn(UArg arg0, UArg arg1)
{
    UInt key;
    Bool flag;
    Bits32 msgSize;
    Queue_Handle txHandle;
    SPIMessageQTransport_Object *obj = (SPIMessageQTransport_Object *)arg0;
    SPI_Transaction *transaction;

    Assert_isTrue((obj != NULL), SPIMessageQTransport_A_nullObject);

    /* Make sure the the previous Tx/Rx is completed */
    key = Hwi_disable();
    if (obj->ready == FALSE) {
        Hwi_restore(key);
        return;
    }
    obj->ready = FALSE;
    Hwi_restore(key);

    /* Get the msg out of the transmit queue */
    transaction = obj->transaction;
    txHandle = SPIMessageQTransport_Instance_State_txQueue(obj);
    transaction->txBuf = Queue_get(txHandle);
    if ((Queue_Elem *)transaction->txBuf == (Queue_Elem *)txHandle) {
        transaction->txBuf = NULL;
    }

    /*
     *  Mark sure the first char is non-zero. The SPI driver will set this to zero
     *  if there is no msg. Note the count is in 16-bit instead of 8-bit
     */
    obj->rxMsg[0] = ~0;
    transaction->rxBuf = (UShort *)(obj->rxMsg);
    transaction->arg = (UArg)obj;
    transaction->count = obj->maxMsgSize/2;

    /* Send a handshake if not completed yet */
    if (transaction->txBuf != NULL) {
        if (obj->handshake == SPIMessageQTransport_LinkStatus_UP) {
            ((Bits32 *)(transaction->txBuf))[0] = STARTOFMSG;
            Log_print1(Diags_USER2, "sending application message 0x%x",
                       (IArg)(transaction->txBuf));
        }
        else {
            ((Bits32 *)(transaction->txBuf))[0] = HANDSHAKEMSG;
            Log_print0(Diags_USER1, "sending handshake");
        }

        /* If checksum enabled, put it in the second 32-bit field in the msg */
        ((Bits32 *)(transaction->txBuf))[1] = 0;
        if (SPIMessageQTransport_checksumEnabled == TRUE) {
            msgSize = MessageQ_getMsgSize((MessageQ_Msg)(transaction->txBuf));
            ((Bits32 *)(transaction->txBuf))[1] =
                SPIMessageQTransport_checksum((Char *)(transaction->txBuf),
                                               msgSize);
        }
    }

    /* Transfer */
    flag = SPI_transfer(obj->spiHandle, transaction);
    if (flag == FALSE) {

        Log_error0("SPIMessageQTransport: SPI_transfer failed.");
        if (transaction->txBuf != NULL) {
            obj->txMsgDropped++;
            SPIMessageQTransport_module->errFxn(
                SPIMessageQTransport_Reason_FAILEDPUT,
                (IMessageQTransport_Handle)obj,
                transaction->txBuf, SPIMessageQTransport_Failure_TRANSFER);
            MessageQ_free((MessageQ_Msg)(transaction->txBuf));
        }
    }
}
