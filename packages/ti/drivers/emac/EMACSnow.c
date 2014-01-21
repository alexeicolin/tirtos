/*
 * Copyright (c) 2013 Texas Instruments Incorporated
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
 *  ======== EMACSnow.c ========
 *  This driver is currently supports only 1 EMACSnow port. In the future
 *  when multiple ports are required, this driver needs to move all the
 *  EMACSnow_Data fields into the EMACSnow_Object structure. The APIs need to get
 *  to the fields via the pvt_data field in the NETIF_DEVICE that
 *  is passed in. ROV must be changed to support the change also.
 *  The NETIF_DEVICE structure should go into the EMACSnow_Object also.
 *
 *  This changes are not being done at this time because of the impact on
 *  the complexity of the code.
 */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>

#include <string.h>

#include <ti/drivers/EMAC.h>
#include <ti/drivers/emac/EMACSnow.h>

/* device specific driver includes */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_emac.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/emac.h"
#include <driverlib/sysctl.h>
/* Name of the device. */
#define ETHERNET_NAME "eth0"

#ifndef NUM_RX_DESCRIPTORS
#define NUM_RX_DESCRIPTORS 4
#endif

#ifndef NUM_TX_DESCRIPTORS
#define NUM_TX_DESCRIPTORS 4
#endif

#ifndef EMAC_PHY_CONFIG
#define EMAC_PHY_CONFIG         (EMAC_PHY_TYPE_INTERNAL |                     \
                                 EMAC_PHY_INT_MDIX_EN |                       \
                                 EMAC_PHY_AN_100B_T_FULL_DUPLEX)
#endif

unsigned long g_ulStatus; // TODO fix

static Void EMACSnow_handleRx();
static Void EMACSnow_processTransmitted();

/*
 *  Helper struct holding a DMA descriptor and the pbuf it currently refers
 *  to.
 */
typedef struct {
  tEMACDMADescriptor Desc;
  PBM_Handle hPkt;
} tDescriptor;

typedef struct {
    tDescriptor *pDescriptors;
    unsigned long ulNumDescs;
    unsigned long ulWrite;
    unsigned long ulRead;
} tDescriptorList;

/*
 * Global variable for this interface's private data.  Needed to allow
 * the interrupt handlers access to this information outside of the
 * context of the lwIP netif.
 */
static tDescriptor g_pTxDescriptors[NUM_TX_DESCRIPTORS];
static tDescriptor g_pRxDescriptors[NUM_RX_DESCRIPTORS];

static tDescriptorList g_TxDescList = {
    g_pTxDescriptors, NUM_TX_DESCRIPTORS, 0, 0
};

static tDescriptorList g_RxDescList = {
    g_pRxDescriptors, NUM_RX_DESCRIPTORS, 0, 0
};

/* EMAC function table for snowflake implementation */
const EMAC_FxnTable EMACSnow_fxnTable = {
        EMACSnow_init,
        EMACSnow_isLinkUp
};

/* Application is required to provide this variable */
extern EMAC_Config EMAC_config;

/*
 *  The struct is used to store the private data for the EMACSnow controller.
 *  TODO move some of this into the EMACSnow_Object
 */
typedef struct EMACSnow_Data {
    STKEVENT_Handle hEvent;
    PBMQ            PBMQ_tx;
    PBMQ            PBMQ_rx;
    UInt            rxCount;
    UInt            rxDropped;
    UInt            txSent;
    UInt            txDropped;
    UInt            abnormalInts;
    UInt            isrCount;
    UInt            linkUp;
    tDescriptorList *pTxDescList;
    tDescriptorList *pRxDescList;
} EMACSnow_Data;

/* Only supporting one EMACSnow device */
static EMACSnow_Data EMACSnow_private;

/* Internal flag to make sure the driver is initialized properly */
static volatile Bool _EMACSnow_initialized = FALSE;

#define PHY_PHYS_ADDR 1

/*
 *  ======== EMACSnow_processPendingTx ========
 *  TODO handle buffers being in shared memory...
 */
static Void EMACSnow_processPendingTx()
{
    UINT8  *pBuffer;
    UInt    len;
    PBM_Handle hPkt;
    tDescriptor *pDesc;

    /*
     *  If there are pending packets, send one.
     *  Otherwise quit the loop.
     */
    hPkt = PBMQ_deq(&EMACSnow_private.PBMQ_tx);
    if (hPkt != NULL) {

        pDesc = &(EMACSnow_private.pTxDescList->pDescriptors[EMACSnow_private.pTxDescList->ulWrite]);
        if (pDesc->hPkt) {
            PBM_free(hPkt);
            EMACSnow_private.txDropped++;
            return;
        }

        /* Get the pointer to the buffer and the length */
        pBuffer = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);
        len = PBM_getValidLen(hPkt);

        /* Fill in the buffer pointer and length */
        pDesc->Desc.ui32Count = len;
        pDesc->Desc.pvBuffer1 = pBuffer;
        pDesc->Desc.ui32CtrlStatus = DES0_TX_CTRL_FIRST_SEG;

        pDesc->Desc.ui32CtrlStatus |= (/*DES0_TX_CTRL_IP_ALL_CKHSUMS |*/ DES0_TX_CTRL_CHAINED);
        pDesc->Desc.ui32CtrlStatus |= (DES0_TX_CTRL_LAST_SEG |
                                       DES0_TX_CTRL_INTERRUPT);
        EMACSnow_private.pTxDescList->ulWrite++;
        if (EMACSnow_private.pTxDescList->ulWrite == NUM_TX_DESCRIPTORS) {
            EMACSnow_private.pTxDescList->ulWrite = 0;
        }
        pDesc->hPkt = hPkt;
        pDesc->Desc.ui32CtrlStatus |= DES0_TX_CTRL_OWN;

        EMACSnow_private.txSent++;

        EMACTxDMAPollDemand(EMAC0_BASE);
    }

    return;
}


/*
 *  ======== EMACSnow_handlePackets ========
 */
static Void EMACSnow_handlePackets(UArg arg0, UArg arg1)
{
    Log_print1(Diags_USER1, "EMACSnow_handlePackets handling packets status = 0x%x", g_ulStatus);

    /* Process the transmit DMA list, freeing any buffers that have been
     * transmitted since our last interrupt.
     */
    if (g_ulStatus & EMAC_INT_TRANSMIT) {
        Log_print0(Diags_USER1, "EMACSnow_handlePackets Tx ones...");
        EMACSnow_processTransmitted();
    }

    /*
     * Process the receive DMA list and pass all successfully received packets
     * up the stack.  We also call this function in cases where the receiver has
     * stalled due to missing buffers since the receive function will attempt to
     * allocate new pbufs for descriptor entries which have none.
     */
    if (g_ulStatus & (EMAC_INT_RECEIVE | EMAC_INT_RX_NO_BUFFER |
        EMAC_INT_RX_STOPPED)) {
        Log_print0(Diags_USER1, "EMACSnow_handlePackets Rx ones...");
        EMACSnow_handleRx();
    }

    Log_print0(Diags_USER1, "EMACSnow_handlePackets re-enable peripheral...");
    EMACIntEnable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                        EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                        EMAC_INT_RX_STOPPED | EMAC_INT_PHY));
}

/*
 *  ======== EMACSnow_processTransmitted ========
 */
static Void EMACSnow_processTransmitted()
{
    tDescriptor *pDesc;
    unsigned long ulNumDescs;

    /*
     * Walk the list until we have checked all descriptors or we reach the
     * write pointer or find a descriptor that the hardware is still working
     * on.
     */
    for (ulNumDescs = 0; ulNumDescs < NUM_TX_DESCRIPTORS; ulNumDescs++) {
        pDesc = &(EMACSnow_private.pTxDescList->pDescriptors[EMACSnow_private.pTxDescList->ulRead]);
        /* Has the buffer attached to this descriptor been transmitted? */
        if (pDesc->Desc.ui32CtrlStatus & DES0_TX_CTRL_OWN) {
            /* No - we're finished. */
            break;
        }

        /* Does this descriptor have a buffer attached to it? */
        if (pDesc->hPkt) {
            /* Yes - free it if it's not marked as an intermediate pbuf */
            if (!((uint32_t)(pDesc->hPkt) & 1)) {
                PBM_free(pDesc->hPkt);
            }
            pDesc->hPkt = NULL;
        }
        else {
            /* If the descriptor has no buffer, we are finished. */
            break;
        }

        /* Move on to the next descriptor. */
        EMACSnow_private.pTxDescList->ulRead++;
        if (EMACSnow_private.pTxDescList->ulRead == NUM_TX_DESCRIPTORS) {
            EMACSnow_private.pTxDescList->ulRead = 0;
        }
    }
}

/*
 *  ======== EMACSnow_primeRx ========
 *  TODO make an EMACSnow_primeTx
 */
static Void EMACSnow_primeRx(PBM_Handle hPkt, tDescriptor *desc)
{
    desc->hPkt = hPkt;
    desc->Desc.ui32Count = DES1_RX_CTRL_CHAINED;

    /* We got a buffer so fill in the payload pointer and size. */
    desc->Desc.pvBuffer1 = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);
    desc->Desc.ui32Count |= (ETH_MAX_PAYLOAD << DES1_RX_CTRL_BUFF1_SIZE_S);

    /* Give this descriptor back to the hardware */
    desc->Desc.ui32CtrlStatus = DES0_RX_CTRL_OWN;
}

/*
 *  ======== EMACSnow_handleRx ========
 */
static Void EMACSnow_handleRx()
{
    PBM_Handle  hPkt;
    PBM_Handle  hPktNew;
    Long len;
    tDescriptorList *pDescList;
    unsigned long ulDescEnd;

    /* Get a pointer to the receive descriptor list. */
    pDescList = EMACSnow_private.pRxDescList;

    /* Determine where we start and end our walk of the descriptor list */
    ulDescEnd = pDescList->ulRead ? (pDescList->ulRead - 1) : (pDescList->ulNumDescs - 1);

    /* Step through the descriptors that are marked for CPU attention. */
    while (pDescList->ulRead != ulDescEnd) {

        /* Does the current descriptor have a buffer attached to it? */
        hPkt = pDescList->pDescriptors[pDescList->ulRead].hPkt;
        if (hPkt) {

            /* Determine if the host has filled it yet. */
            if (pDescList->pDescriptors[pDescList->ulRead].Desc.ui32CtrlStatus &
                DES0_RX_CTRL_OWN) {
              /* The DMA engine still owns the descriptor so we are finished. */
              break;
            }

            /* Yes - does the frame contain errors? */
            if (pDescList->pDescriptors[pDescList->ulRead].Desc.ui32CtrlStatus &
               DES0_RX_STAT_ERR) {
                /*
                 *  This is a bad frame so discard it and update the relevant
                 *  statistics.
                 */
                Log_error0("EMACSnow_handleRx: DES0_RX_STAT_ERR");
                EMACSnow_private.rxDropped++;
                EMACSnow_primeRx(hPkt, &(pDescList->pDescriptors[pDescList->ulRead]));
                pDescList->ulRead++;
                break;
            }
            else {
                /* Allocate a new buffer for this descriptor */
                hPktNew = PBM_alloc(ETH_MAX_PAYLOAD);
                if (hPktNew == NULL) {
                    /*
                     *  Leave the packet in the descriptor and owned by the driver.
                     *  Process when the next interrupt occurs.
                     */
                    break;
                }

                /* This is a good frame so pass it up the stack. */
                len = (pDescList->pDescriptors[pDescList->ulRead].Desc.ui32CtrlStatus &
                      DES0_RX_STAT_FRAME_LENGTH_M) >> DES0_RX_STAT_FRAME_LENGTH_S;

                /* Remove the CRC */
                PBM_setValidLen(hPkt, len - 4);

                /*
                 *  Place the packet onto the receive queue to be handled in the
                 *  EMACSnow_pkt_service function (which is called by the
                 *  NDK stack).
                 */
                PBMQ_enq(&EMACSnow_private.PBMQ_rx, hPkt);

                Log_print2(Diags_USER2, "EMACSnow_handleRx: Enqueued recv packet 0x%x, length = %d",
                    (IArg)hPkt, len - 4);

                /* Update internal statistic */
                EMACSnow_private.rxCount++;

                /*
                 *  Notify NDK stack of pending Rx Ethernet packet and
                 *  that it was triggered by an external event.
                 */
                STKEVENT_signal(EMACSnow_private.hEvent, STKEVENT_ETHERNET, 1);

                /* Prime the receive descriptor back up for future packets */
                EMACSnow_primeRx(hPktNew,
                                 &(pDescList->pDescriptors[pDescList->ulRead]));
            }
        }

        /* Move on to the next descriptor in the chain, taking care to wrap. */
        pDescList->ulRead++;
        if (pDescList->ulRead == pDescList->ulNumDescs) {
            pDescList->ulRead = 0;
        }
    }
}

/*
 *  ======== EMACSnow_processPhyInterrupt ========
 */
Void EMACSnow_processPhyInterrupt()
{
    UInt16 value, status;
    UInt32 config, mode, rxMaxFrameSize;

    /*
     * Read the PHY interrupt status.  This clears all interrupt sources.
     * Note that we are only enabling sources in EPHY_MISR1 so we don't
     * read EPHY_MISR2.
     */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);

    /* Read the current PHY status. */
    status = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_STS);

    /* Has the link status changed? */
    if (value & EPHY_MISR1_LINKSTAT) {
        /* Is link up or down now? */
        if (status & EPHY_STS_LINK) {
            EMACSnow_private.linkUp = TRUE;
        }
        else {
            EMACSnow_private.linkUp = FALSE;
        }
    }

    /* Has the speed or duplex status changed? */
    if (value & (EPHY_MISR1_SPEED | EPHY_MISR1_SPEED | EPHY_MISR1_ANC)) {
        /* Get the current MAC configuration. */
        EMACConfigGet(EMAC0_BASE, (uint32_t *)&config, (uint32_t *)&mode,
                        (uint32_t *)&rxMaxFrameSize);

        /* What speed is the interface running at now?
         */
        if (status & EPHY_STS_SPEED) {
            /* 10Mbps is selected */
            config &= ~EMAC_CONFIG_100MBPS;
        }
        else {
            /* 100Mbps is selected */
            config |= EMAC_CONFIG_100MBPS;
        }

        /* Are we in full- or half-duplex mode? */
        if (status & EPHY_STS_DUPLEX) {
            /* Full duplex. */
            config |= EMAC_CONFIG_FULL_DUPLEX;
        }
        else {
            /* Half duplex. */
            config &= ~EMAC_CONFIG_FULL_DUPLEX;
        }

        /* Reconfigure the MAC */
        EMACConfigSet(EMAC0_BASE, config, mode, rxMaxFrameSize);
    }
}

/*
 *  ======== EMACSnow_hwiIntFxn ========
 */
Void EMACSnow_hwiIntFxn(UArg callbacks)
{
    EMACSnow_Object *object = (EMACSnow_Object *)(EMAC_config.objects);
    ULong status;

    EMACSnow_private.linkUp = (EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) & EPHY_BMSR_LINKSTAT);

    EMACSnow_private.isrCount++;

    /* Read and Clear the interrupt. */
    status = EMACIntStatus(EMAC0_BASE, true);
    EMACIntClear(EMAC0_BASE, status);

    /*
     *  Disable the Ethernet interrupts.  Since the interrupts have not been
     *  handled, they are not asserted.  Once they are handled by the Ethernet
     *  interrupt, it will re-enable the interrupts.
     */
    EMACIntDisable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                     EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                     EMAC_INT_RX_STOPPED | EMAC_INT_PHY));

    if (status & EMAC_INT_ABNORMAL_INT) {
        EMACSnow_private.abnormalInts++;
    }

    g_ulStatus = status;

    if (status & EMAC_INT_PHY) {
        EMACSnow_processPhyInterrupt();
    }

    Log_print1(Diags_USER1, "EMACSnow_hwiIntFxn Posting Swi status = 0x%x", g_ulStatus);

    /* Have the Swi handle the incoming packets and re-enable peripheral */
    Swi_post(object->swi);
}

/*
 *  ======== EMACSnow_emacStart ========
 *  The function is used to initialize and start the EMACSnow
 *  controller and device.
 */
Int  EMACSnow_emacStart(struct NETIF_DEVICE* ptr_net_device)
{
    UInt16 value;
	EMACSnow_Object *object = (EMACSnow_Object *)(EMAC_config.objects);
	EMACSnow_HWAttrs *hwAttrs = (EMACSnow_HWAttrs *)(EMAC_config.hwAttrs);
    Error_Block eb;

	/*
     *  Create the Swi that handles incoming packets.
     *  Take default parameters.
     */
    Error_init(&eb);
    object->swi = Swi_create(EMACSnow_handlePackets,
                             NULL, &eb);
    if (object->swi == NULL) {
        Log_error0("EMACSnow_emacStart: Swi_create failed");
		return (-1);
    }

    /* Create the hardware interrupt */
    object->hwi = Hwi_create(hwAttrs->intNum, EMACSnow_hwiIntFxn, NULL, &eb);
    if (object->hwi == NULL) {
        Log_error0("EMACSnow_emacStart: Hwi_create failed");
        return (-1);
    }

    /* Clear any stray PHY interrupts that may be set. */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1);
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR2);

    /* Configure and enable the link status change interrupt in the PHY. */
    value = EMACPHYRead(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_SCR);
    value |= (EPHY_SCR_INTEN_EXT | EPHY_SCR_INTOE_EXT);
    EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_SCR, value);
    EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_MISR1, (EPHY_MISR1_LINKSTATEN |
                 EPHY_MISR1_SPEEDEN | EPHY_MISR1_DUPLEXMEN | EPHY_MISR1_ANCEN));

    /*
     *  Set MAC filtering options.  We receive all broadcast and multicast
     *  packets along with those addressed specifically for us.
     */
    EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_HASH_AND_PERFECT |
                       EMAC_FRMFILTER_PASS_MULTICAST));

    /* Clear any pending interrupts. */
    EMACIntClear(EMAC0_BASE, EMACIntStatus(EMAC0_BASE, false));

    /* Enable the Ethernet MAC transmitter and receiver. */
    EMACTxEnable(EMAC0_BASE);
    EMACRxEnable(EMAC0_BASE);

    /* Enable the Ethernet RX and TX interrupt source. */
    EMACIntEnable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                  EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                  EMAC_INT_RX_STOPPED | EMAC_INT_PHY));

    /* Enable the Ethernet Interrupt handler. */
    Hwi_enableInterrupt(hwAttrs->intNum);

    EMACPHYWrite(EMAC0_BASE, PHY_PHYS_ADDR, EPHY_BMCR, (EPHY_BMCR_ANEN |
                 EPHY_BMCR_RESTARTAN));
    Log_print0(Diags_USER2, "EMACSnow_emacStart: start completed");

    return (0);
}

/*
 *  ======== EMACSnow_emacStop ========
 *  The function is used to de-initialize and stop the EMACSnow
 *  controller and device.
 */
Int  EMACSnow_emacStop(struct NETIF_DEVICE* ptr_net_device)
{
    EMACSnow_Object *object = (EMACSnow_Object *)(EMAC_config.objects);
    EMACSnow_HWAttrs *hwAttrs = (EMACSnow_HWAttrs *)(EMAC_config.hwAttrs);
    PBM_Handle hPkt;

    EMACIntDisable(EMAC0_BASE, (EMAC_INT_RECEIVE | EMAC_INT_TRANSMIT |
                     EMAC_INT_TX_STOPPED | EMAC_INT_RX_NO_BUFFER |
                     EMAC_INT_RX_STOPPED | EMAC_INT_PHY));
    Hwi_disableInterrupt(hwAttrs->intNum);

    if (object->hwi != NULL) {
        Hwi_delete(&(object->hwi));
    }

	if (object->swi != NULL) {
        Swi_delete(&(object->swi));
    }
    while (PBMQ_count(&EMACSnow_private.PBMQ_rx)) {
        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&EMACSnow_private.PBMQ_rx);
        PBM_free(hPkt);
    }

    while (PBMQ_count(&EMACSnow_private.PBMQ_tx)) {
        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&EMACSnow_private.PBMQ_tx);
        PBM_free(hPkt);
    }



    Log_print0(Diags_USER2, "EMACSnow_emacStop: stop completed");

    return (0);
}

/*
 *  ======== EMACSnow_emacPoll ========
 *  The function is used to poll the EMACSnow controller to check
 *  if there has been any activity
 */
Void EMACSnow_emacPoll(struct NETIF_DEVICE* ptr_net_device, uint timer_tick)
{
    /* Send pending Tx packets */
    EMACSnow_processPendingTx();
    EMACSnow_private.linkUp = (EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) & EPHY_BMSR_LINKSTAT);
}

/*
 *  ======== EMACSnow_emacSend ========
 *  The function is the interface routine invoked by the NDK stack to
 *  pass packets to the driver.
 */
Int  EMACSnow_emacSend(struct NETIF_DEVICE* ptr_net_device, PBM_Handle hPkt)
{
    /*
     *  Enqueue the packet onto the end of the transmit queue.
     *  This is done to ensure that the packets are sent in order.
     */
    PBMQ_enq(&EMACSnow_private.PBMQ_tx, hPkt);

    Log_print2(Diags_USER2, "EMACSnow_emacSend: enqueued hPkt = 0x%x, len = %d", (IArg)hPkt, PBM_getValidLen(hPkt));

    /* Transmit pending packets */
    EMACSnow_processPendingTx();

    return (0);
}

/*
 *  ======== EMACSnow_emacioctl ========
 *  The function is called by the NDK core stack to configure the driver
 */
Int EMACSnow_emacioctl(struct NETIF_DEVICE* ptr_net_device, uint cmd,
               Void* pbuf, uint size)
{
    /* No control commands are currently supported for this driver */
    Log_print0(Diags_USER2, "EMACSnow_emacioctl: emacioctl called");

    if ((cmd == NIMU_ADD_MULTICAST_ADDRESS) || (cmd == NIMU_DEL_MULTICAST_ADDRESS)) {
        return (0);
    }

    return (-1);
}

/*
 *  ======== EMACSnow_pkt_service ========
 *  The function is called by the NDK core stack to receive any packets
 *  from the driver.
 */
Void EMACSnow_pkt_service(NETIF_DEVICE* ptr_net_device)
{
    PBM_Handle  hPkt;

    /* Give all queued packets to the stack */
    while (PBMQ_count(&EMACSnow_private.PBMQ_rx)) {

        /* Dequeue a packet from the driver receive queue. */
        hPkt = PBMQ_deq(&EMACSnow_private.PBMQ_rx);

        /*
         *  Prepare the packet so that it can be passed up the networking stack.
         *  If this 'step' is not done the fields in the packet are not correct
         *  and the packet will eventually be dropped.
         */
        PBM_setIFRx(hPkt, ptr_net_device);

        Log_print1(Diags_USER2,
                   "EMACSnow_pkt_service: give packet 0x%x to NDK via NIMUReceivePacket",
                   (IArg)hPkt);

        /* Pass the packet to the NDK Core stack. */
        NIMUReceivePacket(hPkt);
    }

    /* Work has been completed; the receive queue is empty. */
    return;
}

/*
 *  ======== EMACSnow_init ========
 *  The function is used to initialize the EMACSnow driver
 */
Void EMACSnow_init(UInt index)
{
    /* Currently only supports 1 EMACSnow peripheral */
    Assert_isTrue((index == 0), NULL);

    Assert_isTrue(_EMACSnow_initialized == FALSE, NULL);
    _EMACSnow_initialized = TRUE;

    /* Initialize the global structures */
    memset(&EMACSnow_private, 0, sizeof(EMACSnow_Data));

    Log_print0(Diags_USER2, "EMACSnow_init: setup successfully completed");
}

/*
 *  ======== EMACSnow_InitDMADescriptors ========
 * Initialize the transmit and receive DMA descriptor lists.
 */
Void EMACSnow_InitDMADescriptors(Void)
{
    Int i;
    PBM_Handle  hPkt;

    /* Transmit list -  mark all descriptors as not owned by the hardware */
    for (i = 0; i < NUM_TX_DESCRIPTORS; i++) {
        g_pTxDescriptors[i].hPkt = NULL;
        g_pTxDescriptors[i].Desc.ui32Count = 0;
        g_pTxDescriptors[i].Desc.pvBuffer1 = 0;
        g_pTxDescriptors[i].Desc.DES3.pLink = ((i == (NUM_TX_DESCRIPTORS - 1)) ?
               &g_pTxDescriptors[0].Desc : &g_pTxDescriptors[i + 1].Desc);
        g_pTxDescriptors[i].Desc.ui32CtrlStatus = DES0_TX_CTRL_INTERRUPT |
                                                  /*DES0_TX_CTRL_IP_ALL_CKHSUMS |*/
                                                  DES0_TX_CTRL_CHAINED;
    }

    /*
     * Receive list -  tag each descriptor with a buffer and set all fields to
     * allow packets to be received.
     */
    for (i = 0; i < NUM_RX_DESCRIPTORS; i++) {
        hPkt = PBM_alloc(ETH_MAX_PAYLOAD);
        if (hPkt) {
            EMACSnow_primeRx(hPkt, &(g_pRxDescriptors[i]));
        }
        else {
            System_abort("EMACSnow_InitDMADescriptors: PBM_alloc error\n");
            g_pRxDescriptors[i].Desc.pvBuffer1 = 0;
            g_pRxDescriptors[i].Desc.ui32CtrlStatus = 0;
        }
        g_pRxDescriptors[i].Desc.DES3.pLink =
                ((i == (NUM_RX_DESCRIPTORS - 1)) ?
                &g_pRxDescriptors[0].Desc : &g_pRxDescriptors[i + 1].Desc);
    }

    /* Set the descriptor pointers in the hardware. */
    EMACRxDMADescriptorListSet(EMAC0_BASE, &g_pRxDescriptors[0].Desc);
    EMACTxDMADescriptorListSet(EMAC0_BASE, &g_pTxDescriptors[0].Desc);
}

/*
 *  ======== EMACSnow_NIMUInit ========
 *  The function is used to initialize and register the EMACSnow
 *  with the Network Interface Management Unit (NIMU)
 */
Int EMACSnow_NIMUInit(STKEVENT_Handle hEvent)
{
    EMACSnow_HWAttrs *hwAttrs = (EMACSnow_HWAttrs *)(EMAC_config.hwAttrs);
    Types_FreqHz freq;
    NETIF_DEVICE *device;

    Assert_isTrue(_EMACSnow_initialized == TRUE, NULL);

    Log_print0(Diags_USER2, "EMACSnow_NIMUInit: init called");

    /* Allocate memory for the EMAC. Memory freed in the NDK stack shutdown */
    device = mmAlloc(sizeof(NETIF_DEVICE));
    if (device == NULL) {
        Log_error0("EMACSnow: Failed to allocate NETIF_DEVICE structure");
        return (-1);
    }

    /* Initialize the allocated memory block. */
    mmZeroInit (device, sizeof(NETIF_DEVICE));

    device->mac_address[0] = hwAttrs->macAddress[0];
    device->mac_address[1] = hwAttrs->macAddress[1];
    device->mac_address[2] = hwAttrs->macAddress[2];
    device->mac_address[3] = hwAttrs->macAddress[3];
    device->mac_address[4] = hwAttrs->macAddress[4];
    device->mac_address[5] = hwAttrs->macAddress[5];

    /* Initialize the Packet Device Information struct */
    PBMQ_init(&EMACSnow_private.PBMQ_rx);
    PBMQ_init(&EMACSnow_private.PBMQ_tx);
    EMACSnow_private.hEvent       = hEvent;
    EMACSnow_private.pTxDescList  = &g_TxDescList;
    EMACSnow_private.pRxDescList  = &g_RxDescList;
    EMACSnow_private.rxCount      = 0;
    EMACSnow_private.rxDropped    = 0;
    EMACSnow_private.txSent       = 0;
    EMACSnow_private.txDropped    = 0;
    EMACSnow_private.abnormalInts = 0;
    EMACSnow_private.isrCount = 0;
    EMACSnow_private.linkUp       = FALSE;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0)) {
        /* Keep waiting... */
    }

    EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_CONFIG);

    BIOS_getCpuFreq(&freq);
    EMACInit(EMAC0_BASE, freq.lo,
             EMAC_BCONFIG_MIXED_BURST | EMAC_BCONFIG_PRIORITY_FIXED,
             4, 4, 0);

    /* Set MAC configuration options. */
    EMACConfigSet(EMAC0_BASE, (EMAC_CONFIG_FULL_DUPLEX |
                               //EMAC_CONFIG_CHECKSUM_OFFLOAD |
                               EMAC_CONFIG_7BYTE_PREAMBLE |
                               EMAC_CONFIG_IF_GAP_96BITS |
                               EMAC_CONFIG_USE_MACADDR0 |
                               EMAC_CONFIG_SA_FROM_DESCRIPTOR |
                               EMAC_CONFIG_BO_LIMIT_1024),
                  (EMAC_MODE_RX_STORE_FORWARD |
                   EMAC_MODE_TX_STORE_FORWARD |
                   EMAC_MODE_TX_THRESHOLD_64_BYTES |
                   EMAC_MODE_RX_THRESHOLD_64_BYTES), 0);

    /* Program the MAC address into the Ethernet controller. */
    EMACAddrSet(EMAC0_BASE, 0,
                  (unsigned char *)device->mac_address);

    /*
     * Set MAC filtering options.  We receive all broadcast and multicast
     * packets along with those addressed specifically for us.
     */
    EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_SADDR          |
                                    EMAC_FRMFILTER_PASS_MULTICAST |
                                    EMAC_FRMFILTER_PASS_NO_CTRL));

    /* Initialize the DMA descriptors. */
    EMACSnow_InitDMADescriptors();

    /* Populate the Network Interface Object. */
    strcpy(device->name, ETHERNET_NAME);
    device->mtu         = ETH_MAX_PAYLOAD - ETHHDR_SIZE;
    device->pvt_data    = (Void *)&EMACSnow_private;  // TODO alloc instead?

    /* Populate the Driver Interface Functions. */
    device->start       = EMACSnow_emacStart;
    device->stop        = EMACSnow_emacStop;
    device->poll        = EMACSnow_emacPoll;
    device->send        = EMACSnow_emacSend;
    device->pkt_service = EMACSnow_pkt_service;
    device->ioctl       = EMACSnow_emacioctl;
    device->add_header  = NIMUAddEthernetHeader;

    /* Register the device with NIMU */
    if (NIMURegister(device) < 0) {
        Log_print0(Diags_USER1, "EMACSnow_NIMUInit: failed to register with NIMU");
        return (-1);
    }

    Log_print0(Diags_USER2, "EMACSnow_NIMUInit: register with NIMU");

    return (0);
}

/*
 *  ======== EMACSnow_linkUp ========
 */
Bool EMACSnow_isLinkUp(UInt index)
{
    EMACSnow_private.linkUp = (EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) & EPHY_BMSR_LINKSTAT);
    if (EMACSnow_private.linkUp) {
        return (TRUE);
    }
    else {
        return (FALSE);
    }
}
