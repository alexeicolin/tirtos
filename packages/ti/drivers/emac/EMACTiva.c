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
 *  ======== EMACTiva.c ========
 *  This driver is currently supports only 1 EMAC port. In the future
 *  when multiple ports are required, this driver needs to move all the
 *  EMACTiva_Data fields into the EMACTiva_Object structure. The APIs
 *  need to get to the fields via the pvt_data field in the NETIF_DEVICE that
 *  is passed in. ROV must be changed to support the change also.
 *  The NETIF_DEVICE structure should go into the EMACTiva_Object also.
 *
 *  This changes are not being done at this time because of the impact on
 *  the complexity of the code.
 */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <string.h>

 /* NDK includes */
#define _INCLUDE_NIMU_CODE
#include <ti/ndk/inc/stkmain.h>

/* device specific driver includes */
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/ethernet.h>
#include <driverlib/sysctl.h>
#include <inc/hw_ethernet.h>

#include <ti/drivers/EMAC.h>
#include <ti/drivers/emac/EMACTiva.h>

/* Name of the device. */
#define NIMUINTERFACE_NAME "eth0"

/* EMAC function table for Tiva implementation */
const EMAC_FxnTable EMACTiva_fxnTable = {
        EMACTiva_init,
        EMACTiva_isLinkUp
};

/* Application is required to provide this variable */
extern EMAC_Config EMAC_config;

/* Forward prototypes as needed */
static Void EMACTiva_handlePendingTx();
static Void EMACTiva_handleRx(UArg arg0, UArg arg1);
static Void EMACTiva_hwiIntFxn(UArg callbacks);

/*
 *  ======== EMACTiva_emacioctl ========
 *  The function is called by the NDK core stack to configure the driver
 */
static Int EMACTiva_emacioctl(struct NETIF_DEVICE* ptr_net_device, uint cmd,
               Void* pbuf, uint size)
{
    Log_print0(Diags_USER2, "EMACTiva: emacioctl called");

    if ((cmd == NIMU_ADD_MULTICAST_ADDRESS) ||
        (cmd == NIMU_DEL_MULTICAST_ADDRESS)) {
        return (0);
    }

    return (-1);
}

/*
 *  ======== EMACTiva_emacPoll ========
 *  The function is used to poll the EMACTiva controller to check
 *  if there has been any activity
 */
static Void EMACTiva_emacPoll(struct NETIF_DEVICE* ptr_net_device, uint timer_tick)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);

    /* Send pending Tx packets */
    EMACTiva_handlePendingTx();
    object->linkUp = EthernetPHYRead(ETH_BASE, PHY_MR1) & PHY_MR1_LINK;
}

/*
 *  ======== EMACTiva_emacSend ========
 *  The function is the interface routine invoked by the NDK stack to
 *  pass packets to the driver.
 */
static Int EMACTiva_emacSend(struct NETIF_DEVICE* ptr_net_device, PBM_Handle hPkt)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);

    /*
     *  Enqueue the packet onto the end of the transmit queue.
     *  This is done to ensure that the packets are sent in order.
     */
    PBMQ_enq(&object->PBMQ_tx, hPkt);

    /* Transmit pending packets */
    EMACTiva_handlePendingTx();

    return (0);
}

/*
 *  ======== EMACTiva_emacServicePacket ========
 *  The function is called by the NDK core stack to receive any packets
 *  from the driver.
 */
static Void EMACTiva_emacServicePacket(NETIF_DEVICE* ptr_net_device)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    PBM_Handle  hPkt;

    /* Give all queued packets to the stack */
    while ((hPkt = PBMQ_deq(&(object->PBMQ_rx))) != NULL) {

        /*
         *  Prepare the packet so that it can be passed up the networking stack.
         *  If this 'step' is not done the fields in the packet are not correct
         *  and the packet will eventually be dropped.
         */
        PBM_setIFRx(hPkt, ptr_net_device);

        Log_print1(Diags_USER2,
                   "EMACTiva: give packet 0x%x to NDK via NIMUReceivePacket",
                   (IArg)hPkt);

        /* Pass the packet to the NDK Core stack. */
        NIMUReceivePacket(hPkt);
    }

    return;
}

/*
 *  ======== EMACTiva_emacStart ========
 *  The function is used to initialize and start the EMACTiva
 *  controller and device.
 */
static Int EMACTiva_emacStart(struct NETIF_DEVICE* ptr_net_device)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    EMACTiva_HWAttrs *hwAttrs = (EMACTiva_HWAttrs *)(EMAC_config.hwAttrs);
    Error_Block eb;

    Error_init(&eb);

    /*
     *  Create the Swi that handles incoming packets.
     *  Take default parameters.
     */
    object->swi = Swi_create(EMACTiva_handleRx, NULL, &eb);
    if (object->swi == NULL) {
        Log_error0("EMACTiva: Swi_create failed");
        return (-1);
    }

    /* Create the hardware interrupt */
    object->hwi = Hwi_create(hwAttrs->intNum, EMACTiva_hwiIntFxn, NULL, &eb);
    if (object->hwi == NULL) {
        Log_error0("EMACTiva: Hwi_create failed");
        return (-1);
    }

    /* Enable the Ethernet Controller transmitter and receiver. */
    EthernetEnable(ETH_BASE);

    /* Enable the Ethernet Interrupt handler. */
    Hwi_enableInterrupt(hwAttrs->intNum);

    /* Enable Ethernet TX and RX Packet Interrupts. */
    EthernetIntEnable(ETH_BASE, ETH_INT_RX | ETH_INT_TX);

    Log_print0(Diags_USER1, "EMACTiva: start completed");

    return (0);
}

/*
 *  ======== EMACTiva_emacStop ========
 *  The function is used to de-initialize and stop the EMAC
 *  controller and device.
 */
static Int EMACTiva_emacStop(struct NETIF_DEVICE* ptr_net_device)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    EMACTiva_HWAttrs *hwAttrs = (EMACTiva_HWAttrs *)(EMAC_config.hwAttrs);
    PBM_Handle hPkt;

    EthernetIntDisable(ETH_BASE, ETH_INT_RX | ETH_INT_TX);

    Hwi_disableInterrupt(hwAttrs->intNum);

    EthernetDisable(ETH_BASE);

    if (object->hwi != NULL) {
        Hwi_delete(&(object->hwi));
    }

    if (object->swi != NULL) {
        Swi_delete(&(object->swi));
    }

    /* Free all the packets on the Rx and Tx queues */
    while ((hPkt = PBMQ_deq(&object->PBMQ_rx)) != NULL) {
        PBM_free(hPkt);
    }

    while ((hPkt = PBMQ_deq(&object->PBMQ_tx)) != NULL) {
        PBM_free(hPkt);
    }

    Log_print0(Diags_USER1, "EMACTiva: stop completed");

    return (0);
}

/*
 *  ======== EMACTiva_handlePendingTx ========
 */
static Void EMACTiva_handlePendingTx()
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    UINT8  *pBuffer;
    UInt    len;
    Long    sentLen;
    PBM_Handle hPkt;

    /* Transmit pending packets */
    while (EthernetSpaceAvail(ETH_BASE)) {

        /*
         *  If there are pending packets, send one.
         *  Otherwise quit the loop.
         */
        hPkt = PBMQ_deq(&object->PBMQ_tx);
        if (hPkt != NULL) {

            /* Get the pointer to the buffer and the length */
            pBuffer = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);
            len = PBM_getValidLen(hPkt);

            sentLen = EthernetPacketPutNonBlocking(ETH_BASE, pBuffer, len);
            if (sentLen < 0) {
                Log_print2(Diags_USER1,
                    "EMACTiva: failed to transmit packet 0x%x len = %d",
                    (IArg)hPkt, len);
                object->txDropped++;
            }
            else {
                Log_print2(Diags_USER2,
                    "EMACTiva: Sent packet 0x%x to network len = %d",
                    (IArg)hPkt, len);
                object->txSent++;
            }

            PBM_free(hPkt);
        }
        else {
            break;
        }
    }
    return;
}

/*
 *  ======== EMACTiva_handleRx ========
 */
static Void EMACTiva_handleRx(UArg arg0, UArg arg1)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    PBM_Handle  hPkt;
    UInt8 dummyBuffer[4];
    UInt8 *pBuffer;
    Long len;

    /* Process all incoming packets */
    while (EthernetPacketAvail(ETH_BASE) == TRUE) {

        /*
         *  Alloc a PBM packet. The incoming Ethernet packet will be
         *  placed into this buffer.
         */
        hPkt = PBM_alloc(ETH_MAX_PAYLOAD);
        if (hPkt == NULL) {
            Log_print0(Diags_USER1, "EMACTiva: no PBM packets available");

            /*
             *  Drop this incoming packet. The rest of the packet will
             *  be dropped by the Ethernet API.
             */
            EthernetPacketGetNonBlocking(ETH_BASE, dummyBuffer,
                                         sizeof(dummyBuffer));

            object->rxDropped++;
            break;
        }
        pBuffer = PBM_getDataBuffer(hPkt);

        len = EthernetPacketGetNonBlocking(ETH_BASE, pBuffer, ETH_MAX_PAYLOAD);
        if (len < 0) {
           /* Packet is too small */
           Log_print2(Diags_USER1,
               "EMACTiva: packet too small. supplied %d, received %d",
               ETH_MAX_PAYLOAD, -len);
           PBM_free(hPkt);
           object->rxDropped++;
        }
        else {
            /* Set the length of the incoming packet */
            PBM_setValidLen(hPkt, len);

            /*
             *  Place the packet onto the receive queue to be handled in the
             *  EMACTiva_pkt_service function (which is called by the
             *  NDK stack).
             */
            PBMQ_enq(&object->PBMQ_rx, hPkt);

            Log_print1(Diags_USER2, "EMACTiva: enqueued received 0x%x",
                (IArg)hPkt);

            /* Update internal statistic */
            object->rxCount++;

            /*
             *  Notify NDK stack of pending Rx Ethernet packet and
             *  that it was triggered by an external event.
             */
            STKEVENT_signal(object->hEvent, STKEVENT_ETHERNET, 1);

        }
    }

    /* Re-enable the Ethernet interrupts. */
    EthernetIntEnable(ETH_BASE, ETH_INT_RX | ETH_INT_TX);
}

/*
 *  ======== EMACTiva_hwiIntFxn ========
 */
static Void EMACTiva_hwiIntFxn(UArg callbacks)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    unsigned long ulStatus;

    /* Read and Clear the interrupt. */
    ulStatus = EthernetIntStatus(ETH_BASE, false);

    /*
     *  Disable the Ethernet interrupts.  Since the interrupts have not been
     *  handled, they are not asserted.  Once they are handled by the Ethernet
     *  interrupt, it will re-enable the interrupts.
     */
    EthernetIntDisable(ETH_BASE, ETH_INT_RX | ETH_INT_TX);

    EthernetIntClear(ETH_BASE, ulStatus);

    /* Have the Swi handle the incoming packets and re-enable peripheral */
    Swi_post(object->swi);
}

/*
 *  ======== EMACTiva_init ========
 *  The function is used to initialize the EMACTiva driver
 */
Void EMACTiva_init(UInt index)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);

    /* Currently only supports 1 EMACTiva peripheral */
    Assert_isTrue((index == 0), NULL);

    /* Initialize the global structure */
    memset(object, 0, sizeof(EMACTiva_Object));

    Log_print0(Diags_USER1, "EMACTiva: setup successfully completed");
}

/*
 *  ======== EMACTiva_NIMUInit ========
 *  The function is used to initialize and register the EMACTiva
 *  with the Network Interface Management Unit (NIMU)
 */
Int EMACTiva_NIMUInit(STKEVENT_Handle hEvent)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);
    EMACTiva_HWAttrs *hwAttrs = (EMACTiva_HWAttrs *)(EMAC_config.hwAttrs);
    UInt32 temp;
    NETIF_DEVICE *device;
    Types_FreqHz freq;

    Log_print0(Diags_USER1, "EMACTiva: init called");

    /*
     *  Allocate memory for the EMACTiva. Memory freed in
     *  the NDK stack shutdown.
     */
    device = mmAlloc(sizeof(NETIF_DEVICE));
    if (device == NULL) {
        Log_error0("EMACTiva: Failed to allocate NETIF_DEVICE structure");
        return (-1);
    }

    /* Initialize the allocated memory block. */
    memset(device, 0, sizeof(NETIF_DEVICE));

    device->mac_address[0] = hwAttrs->macAddress[0];
    device->mac_address[1] = hwAttrs->macAddress[1];
    device->mac_address[2] = hwAttrs->macAddress[2];
    device->mac_address[3] = hwAttrs->macAddress[3];
    device->mac_address[4] = hwAttrs->macAddress[4];
    device->mac_address[5] = hwAttrs->macAddress[5];

    /* Initialize the Packet Device Information struct */
    PBMQ_init(&(object->PBMQ_rx));
    PBMQ_init(&(object->PBMQ_tx));
    object->hEvent  = hEvent;

    /* Program the MAC address into the Ethernet controller. */
    EthernetMACAddrSet(ETH_BASE, (unsigned char *)device->mac_address);

    /* Disable all Ethernet Interrupts. */
    EthernetIntDisable(ETH_BASE, (ETH_INT_PHY | ETH_INT_MDIO | ETH_INT_RXER |
                       ETH_INT_RXOF | ETH_INT_TX | ETH_INT_TXER | ETH_INT_RX));
    temp = EthernetIntStatus(ETH_BASE, false);
    EthernetIntClear(ETH_BASE, temp);

    BIOS_getCpuFreq(&freq);

    /* Initialize the Ethernet Controller. */
    EthernetInitExpClk(ETH_BASE, freq.lo);

    /*
     * Configure the Ethernet Controller for normal operation.
     * - Enable TX Duplex Mode
     * - Enable TX Padding
     * - Enable TX CRC Generation
     * - Enable RX Multicast Reception
     */
    EthernetConfigSet(ETH_BASE, (ETH_CFG_TX_DPLXEN |ETH_CFG_TX_CRCEN |
                      ETH_CFG_TX_PADEN | ETH_CFG_RX_AMULEN));

    /* Populate the Network Interface Object. */
    strcpy (device->name, NIMUINTERFACE_NAME);
    device->mtu         = ETH_MAX_PAYLOAD - ETHHDR_SIZE;
    device->pvt_data    = (Void *)object;

    /* Populate the Driver Interface Functions. */
    device->start       = EMACTiva_emacStart;
    device->stop        = EMACTiva_emacStop;
    device->poll        = EMACTiva_emacPoll;
    device->send        = EMACTiva_emacSend;
    device->pkt_service = EMACTiva_emacServicePacket;
    device->ioctl       = EMACTiva_emacioctl;
    device->add_header  = NIMUAddEthernetHeader;

    /* Register the device with NIMU */
    if (NIMURegister(device) < 0) {
        mmFree(device);
        Log_print0(Diags_USER1, "EMACTiva: failed to register with NIMU");
        return (-1);
    }

    Log_print0(Diags_USER1, "EMACTiva: register with NIMU");

    return (0);
}


/*
 *  ======== EMACTiva_isLinkUp ========
 */
Bool EMACTiva_isLinkUp(UInt index)
{
    EMACTiva_Object *object = (EMACTiva_Object *)(EMAC_config.objects);

    object->linkUp = EthernetPHYRead(ETH_BASE, PHY_MR1) & PHY_MR1_LINK;
    if (object->linkUp) {
        return (TRUE);
    }
    else {
        return (FALSE);
    }
}
