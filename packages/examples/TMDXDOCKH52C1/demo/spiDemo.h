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
 *  ======== spiDemo.h ========
 */

#ifndef __TMDXDOCKH52C1_SSIDEMO_H
#define __TMDXDOCKH52C1_SSIDEMO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/ipc/MessageQ.h>

#define M3MASTER "M3s_queue"
#define C28SLAVE "C28s_queue"

#define HEAPID      0

#ifdef xdc_target__isaCompatible_28

#define NUMBLOCKS 4

typedef struct MyMsg {
    MessageQ_MsgHeader hdr;
    Char data[52]; // get good alignment
} MyMsg;

#else

#define NUMBLOCKS 8

typedef struct MyMsg {
    MessageQ_MsgHeader hdr;
    Char data[104]; // get good alignment
} MyMsg;


#endif

#define BLOCKSIZE  sizeof(MyMsg)

#define MASTERM3PROCID  0
#define MASTERC28PROCID 1
#define SLAVEM3PROCID   2

#define SLAVEM3QUEUEINDEX   0

#ifdef __cplusplus
}
#endif

#endif /* __TMDXDOCKH52C1_SSIDEMO_H */
