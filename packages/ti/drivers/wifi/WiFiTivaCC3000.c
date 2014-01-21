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

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Types.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiTivaCC3000.h>

/* CC3000 Host Driver header files */
#include <cc3000_host_driver/core_driver/inc/evnt_handler.h>
#include <cc3000_host_driver/include/wlan.h>
#include <spi.h>

/* driverlib header files */
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>

/* SPI layer header opcodes */
#define READ_OPCODE          3
#define WRITE_OPCODE         1

/* Macros to get put data length in the right format */
#define HI(value)            ((UShort)(value) >> 8)
#define LO(value)            ((UShort)(value) & 0x00FF)

/* Header sizes */
#define SPI_HEADER_SIZE      (5)
#define HEADERS_SIZE_EVNT    (SPI_HEADER_SIZE + 5)

/*
 *  The magic number that resides at the end of the TX/RX buffer (1 byte after
 *  the allocated size) for the purpose of detection of the overrun. The
 *  location of the memory where the magic number resides shall never be
 *  written. In case it is written - the overrun occurred and either receive
 *  function or send function will stuck forever.
 */
#define WiFi_BUF_OVERRUN_DETECT (0xDE)

/* DMA max size */
#define DMA_WINDOW_SIZE     1024

/* Defines to resolve differences between GPIO APIs */
#if defined(MWARE)
#define GPIOIntClear        GPIOPinIntClear
#define GPIOIntDisable      GPIOPinIntDisable
#define GPIOIntEnable       GPIOPinIntEnable
#endif

/*
 *  Since there's no way to tell which instance of the CC3000 driver is getting
 *  calls from the upper-level host driver, we can only support one. We will
 *  save the WiFi handle in the WiFi_init() function and use it everywhere.
 */
static WiFi_Handle wiFiHandle;

/* Static buffer for 5 bytes of SPI HEADER */
static UChar spiReadHeader[] = {READ_OPCODE, 0, 0, 0, 0};

/* Function prototypes */
Void WiFiTivaCC3000_close(WiFi_Handle handle);
Void WiFiTivaCC3000_init(WiFi_Handle handle);
WiFi_Handle WiFiTivaCC3000_open(WiFi_Handle handle, UInt spiIndex,
                                WiFi_evntCallback evntCallback,
                                WiFi_Params *params);
static Void WiFiTivaCC3000_readHeader(Void);
static Long WiFiTivaCC3000_readIrqPin(Void);
static Void WiFiTivaCC3000_spiCallbackFxn(SPI_Handle spiHandle,
                                               SPI_Transaction *transaction);
static Void WiFiTivaCC3000_triggerRxProcessing(Void);
static Void WiFiTivaCC3000_writeWlanEnPin(UChar val);

/* CC3000 function table for Tiva implementation */
const WiFi_FxnTable WiFiTivaCC3000_fxnTable = {
    WiFiTivaCC3000_close,
    WiFiTivaCC3000_init,
    WiFiTivaCC3000_open
};

extern const WiFi_Params WiFi_defaultParams;

/*
 *  Constants for buffer sizes. These are declared and initialized in generated
 *  code based on your project's configuration file.
 */
extern const Int WiFi_TX_BUFFER_SIZE;
extern const Int WiFi_RX_BUFFER_SIZE;

/*
 *  Transfer and receive buffers. These are declared and initialized in
 *  generated code based on your project's configuration file.
 */
extern UChar wlan_tx_buffer[];
extern UChar wlan_rx_buffer[];

/*
 *  ======== WiFiTivaCC3000_close ========
 */
Void WiFiTivaCC3000_close(WiFi_Handle handle)
{
    UInt key;
    WiFiTivaCC3000_Object *object = handle->object;

    /* Disable the CC3000 */
    wlan_stop();

    /* Close the SPI driver */
    if (object->spiHandle != NULL) {
        SPI_close(object->spiHandle);
    }

    /* Delete the IRQ Hwi */
    Hwi_destruct(&(object->hwiIrq));

    /* Delete the semaphore */
    Semaphore_destruct(&(object->writeComplete));

    Log_print0(Diags_USER1, "WiFi: Object closed.");

    key = Hwi_disable();
    object->isOpen = FALSE;
    Hwi_restore(key);
}

/*
 *  ======== WiFiTivaCC3000_disableIrqInt ========
 *  Callback for host driver. Registered with wlan_init().
 */
static Void WiFiTivaCC3000_disableIrqInt(Void)
{
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    GPIOIntDisable(hwAttrs->irqPort, hwAttrs->irqPin);

    Log_print0(Diags_USER1, "WiFi: IRQ interrupt disabled by Host Driver.");
}

/*
 *  ======== WiFiTivaCC3000_enableIrqInt ========
 *  Callback for host driver. Registered with wlan_init().
 */
static Void WiFiTivaCC3000_enableIrqInt(Void)
{
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    GPIOIntEnable(hwAttrs->irqPort, hwAttrs->irqPin);

    Log_print0(Diags_USER1, "WiFi: IRQ interrupt enabled by Host Driver.");
}

/*
 *  ======== WiFiTivaCC3000_firstWrite ========
 *  Function to perform the first write after power up. CC3000 requires a few
 *  50 us delays during this first transaction.
 */
Void WiFiTivaCC3000_firstWrite(UChar *userBuffer, UShort length)
{
    UInt          delayAmt;
    Types_FreqHz  freq;
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    Log_print0(Diags_USER1, "WiFi: Performing first write to CC3000 after "
                            "wlan_start().");

    /* Calculate delay amount */
    BIOS_getCpuFreq(&freq);
    delayAmt = freq.lo/60000;

    /* Assert CS */
    GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, 0);

    /* Need to wait at least 50 us after asserting CS */
    SysCtlDelay(delayAmt);

    /* SPI writes first 4 bytes of data */
    object->transaction.txBuf = userBuffer;
    object->transaction.count = 4;

    object->spiState = STATE_WRITE_FIRST_PART;
    if(!SPI_transfer(object->spiHandle, &object->transaction)){
        Log_error0("WiFi: SPI transfer (write) failed to begin!");
    }

    Semaphore_pend(Semaphore_handle(&(object->writeComplete)), BIOS_WAIT_FOREVER);

    /* Need to wait at least 50 us again */
    SysCtlDelay(delayAmt);

    /* SPI writes another 4 bytes of data */
    object->transaction.txBuf = userBuffer + 4;
    object->transaction.count = length - 4;

    object->spiState = STATE_WRITE_EOT;
    if(!SPI_transfer(object->spiHandle, &object->transaction)){
        Log_error0("WiFi: SPI transfer (write) failed to begin!");
    }
}

/*
 *  ======== WiFiTivaCC3000_hwiIrqFxn ========
 *  GPIO interrupt handler for the IRQ line.
 */
static Void WiFiTivaCC3000_hwiIrqFxn(UArg arg)
{
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Clear flag */
    GPIOIntClear(hwAttrs->irqPort, hwAttrs->irqPin);

    /* If the IRQ pin has been asserted... */
    if(!GPIOPinRead(hwAttrs->irqPort, hwAttrs->irqPin)) {
        if (object->spiState == STATE_POWERUP) {
            /* Wi-Fi device is powered up and ready to interact with host. */
            object->spiState = STATE_INITIALIZED;
            Log_print0(Diags_USER1, "WiFi: CC3000 is powered up and ready.");
        }
        else if (object->spiState == STATE_IDLE) {
            /*
             *  When in the IDLE state and an IRQ occurs, the Wi-Fi device is
             *  saying that there is data that needs to be read.
             */
            object->spiState = STATE_READ_IRQ;

            /* Assert chip select and start reading */
            GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, 0);
            WiFiTivaCC3000_readHeader();
        }
        else if ((object->spiState == STATE_WRITE_FIRST_PART) ||
                 (object->spiState == STATE_WRITE_EOT)) {
            /* Wi-Fi device is ready to receive a write from the host */
            if(!SPI_transfer(object->spiHandle, &object->transaction)){
                Log_error0("WiFi: SPI transfer (write) failed to begin!");
            }
        }
    }
}

/*
 *  ======== WiFiTivaCC3000_init ========
 */
Void WiFiTivaCC3000_init(WiFi_Handle handle)
{
    /*
     *  We save the handle to a global variable to be used in the functions that
     *  are called from the CC3000 Host Driver.
     */
    wiFiHandle = handle;

    /* Mark the object as available. */
    ((WiFiTivaCC3000_Object*)wiFiHandle->object)->isOpen = FALSE;
}

/*
 *  ======== WiFiTivaCC3000_open ========
 */
WiFi_Handle WiFiTivaCC3000_open(WiFi_Handle handle, UInt spiIndex,
                                WiFi_evntCallback evntCallback,
                                WiFi_Params *params)
{
    UInt                          key;
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;
    union {
        Hwi_Params                hwiParams;
        Semaphore_Params          semParams;
        SPI_Params                spiParams;
    } paramsUnion;

    /* Determine if the WiFi object has already been opened */
    key = Hwi_disable();
    if(object->isOpen == TRUE) {
        Hwi_restore(key);
        Log_warning0("WiFi: This index is already in use.");
        return (NULL);
    }
    /* Mark the handle as being used */
    object->isOpen = TRUE;
    Hwi_restore(key);

    /* Create a semaphore to block for the write transaction. */
    Semaphore_Params_init(&(paramsUnion.semParams));
    paramsUnion.semParams.mode = Semaphore_Mode_BINARY;
    paramsUnion.semParams.instance->name = "WiFi.writeComplete";
    Semaphore_construct(&(object->writeComplete), 0, &(paramsUnion.semParams));

    /* BIOS Hwi create for IRQ interrupt */
    Hwi_Params_init(&(paramsUnion.hwiParams));

    Hwi_construct(&(object->hwiIrq), hwAttrs->irqIntNum, WiFiTivaCC3000_hwiIrqFxn,
            &(paramsUnion.hwiParams), NULL);

    /* Set up the parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (WiFi_Params *) &WiFi_defaultParams;
    }

    /* GPIO Configuration */
    /* Configure IRQ pin */
    GPIOPinTypeGPIOInput(hwAttrs->irqPort, hwAttrs->irqPin);
    GPIOIntTypeSet(hwAttrs->irqPort, hwAttrs->irqPin, GPIO_FALLING_EDGE);

    /* Configure WLAN EN pin */
    GPIOPinTypeGPIOOutput(hwAttrs->enPort, hwAttrs->enPin);
    GPIOPinWrite(hwAttrs->enPort, hwAttrs->enPin, 0);

    /* Configure CS pin */
    GPIOPinTypeGPIOOutput(hwAttrs->csPort, hwAttrs->csPin);
    GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, (~0));

    /* Initialize SPI parameters */
    SPI_Params_init(&(paramsUnion.spiParams));
    paramsUnion.spiParams.transferMode = SPI_MODE_CALLBACK;
    paramsUnion.spiParams.transferCallbackFxn = WiFiTivaCC3000_spiCallbackFxn;
    paramsUnion.spiParams.mode = SPI_MASTER;
    paramsUnion.spiParams.bitRate = params->bitRate;
    paramsUnion.spiParams.dataSize = 8;
    paramsUnion.spiParams.frameFormat = SPI_POL0_PHA1;

    /* Open SPI driver */
    object->spiHandle = SPI_open(spiIndex, &(paramsUnion.spiParams));
    if (object->spiHandle == NULL) {
        Log_error0("WiFi: Error opening SPI. WiFi instance was not opened.");
        WiFiTivaCC3000_close(handle);
        return (NULL);
    }

    Log_print0(Diags_USER1, "WiFi: Object created.");

    /* Register callbacks with the host driver */
    wlan_init(evntCallback, NULL, NULL, NULL,
              WiFiTivaCC3000_readIrqPin,
              WiFiTivaCC3000_enableIrqInt,
              WiFiTivaCC3000_disableIrqInt,
              WiFiTivaCC3000_writeWlanEnPin);

    /* Start the CC3000. Pass in 0 because no patches are available. */
    wlan_start(0);

    return (handle);
}

/*
 *  ======== WiFiTivaCC3000_readData ========
 *  Analyze first ten bytes received and continue to read if necessary. Not to
 *  be called by user.
 */
static Long WiFiTivaCC3000_readData(Void)
{
    Long    dataToRecv;
    UChar   type;
    WiFiTivaCC3000_Object *object = wiFiHandle->object;

    /* Determine what type of packet we have */
    dataToRecv = 0;
    STREAM_TO_UINT8((Char *)(wlan_rx_buffer + SPI_HEADER_SIZE),
                    HCI_PACKET_TYPE_OFFSET, type);

    switch(type)
    {
        case HCI_TYPE_DATA:
            /* Calculate amount of remaining data */
            STREAM_TO_UINT16((Char *)(wlan_rx_buffer + SPI_HEADER_SIZE),
                             HCI_DATA_LENGTH_OFFSET, dataToRecv);

            if (dataToRecv > DMA_WINDOW_SIZE) {
                object->spiState = STATE_READ_FIRST_PART;
                dataToRecv = DMA_WINDOW_SIZE;
            }
            else {
                /* Make amount 16-bit aligned if needed */
                if (!((HEADERS_SIZE_EVNT + dataToRecv) & 1)) {
                    dataToRecv++;
                }

                object->spiState = STATE_READ_EOT;
            }
            break;

        case HCI_TYPE_EVNT:
            /* Calculate amount of remaining data */
            STREAM_TO_UINT8((Char *)(wlan_rx_buffer + SPI_HEADER_SIZE),
                            HCI_EVENT_LENGTH_OFFSET, dataToRecv);
            dataToRecv -= 1;

            /* Make amount 16-bit aligned if needed */
            if ((HEADERS_SIZE_EVNT + dataToRecv) & 1) {
                dataToRecv++;
            }

            object->spiState = STATE_READ_EOT;
            break;
    }

    /* If there is data remaining, call the read function*/
    if (dataToRecv) {
        object->transaction.rxBuf = wlan_rx_buffer + 10;
        object->transaction.txBuf = wlan_tx_buffer;
        object->transaction.count = dataToRecv;

        if(!SPI_transfer(object->spiHandle, &object->transaction)){
            Log_error0("WiFi: SPI transfer (read) failed to begin!");
        }
    }

    return (dataToRecv);
}

/*
 *  ======== WiFiTivaCC3000_readDataCont ========
 *  This function is called by the SPI interrupt handler after reading the
 *  header.
 */
static Void WiFiTivaCC3000_readDataCont(Void)
{
    if (!WiFiTivaCC3000_readData())
    {
        /* All data has been read */
        WiFiTivaCC3000_triggerRxProcessing();
    }
}

/*
 *  ======== WiFiTivaCC3000_readHeader ========
 *  Function reads 10 bytes--5 bytes of SPI Header and 5 bytes of data.
 */
static Void WiFiTivaCC3000_readHeader(Void)
{
    WiFiTivaCC3000_Object *object = wiFiHandle->object;

    /* Read first 10 bytes */
    object->transaction.count = 10;
    object->transaction.txBuf = spiReadHeader;

    if(!SPI_transfer(object->spiHandle, &object->transaction)){
        Log_error0("WiFi: SPI transfer (read header) failed to begin!");
    }
}

/*
 *  ======== WiFiTivaCC3000_readIrqPin ========
 *  Callback for host driver. Registered with wlan_init().
 */
static Long WiFiTivaCC3000_readIrqPin(Void)
{
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    return (GPIOPinRead(hwAttrs->irqPort, hwAttrs->irqPin));
}

/*
 *  ======== WiFiTivaCC3000_spiCallbackFxn ========
 *  Called by the SPI driver when a transmit has completed.
 */
static Void WiFiTivaCC3000_spiCallbackFxn(SPI_Handle spiHandle,
                                               SPI_Transaction *transaction)
{
    UShort dataToRecv = 0;
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    if (object->spiState == STATE_READ_IRQ) {
        /* Header has been read. Analyze it and proceed. */
        WiFiTivaCC3000_readDataCont();
    }
    else if (object->spiState == STATE_READ_FIRST_PART) {
        /* First part of read has completed. Read the rest. */
        STREAM_TO_UINT16((char *)(wlan_rx_buffer + SPI_HEADER_SIZE),
                         HCI_DATA_LENGTH_OFFSET, dataToRecv);

        dataToRecv -= DMA_WINDOW_SIZE;

        /* Make amount 16-bit aligned if needed */
        if (!((HEADERS_SIZE_EVNT + dataToRecv) & 1)) {
            dataToRecv++;
        }

        object->spiState = STATE_READ_EOT;

        /* Read the last of the data. */
        object->transaction.rxBuf = wlan_rx_buffer + 10 + DMA_WINDOW_SIZE;
        object->transaction.txBuf = wlan_tx_buffer;
        object->transaction.count = dataToRecv;

        if(!SPI_transfer(object->spiHandle, &object->transaction)){
            Log_error0("WiFi: SPI transfer (read) failed to begin!");
        }
    }
    else if (object->spiState == STATE_READ_EOT) {
        /* Read has completed. Trigger processing. */
        WiFiTivaCC3000_triggerRxProcessing();
    }
    else if (object->spiState == STATE_WRITE_FIRST_PART) {
        /* First part of the write has completed. Finish it. */
        Semaphore_post(Semaphore_handle(&(object->writeComplete)));
    }
    else if (object->spiState == STATE_WRITE_EOT) {
        /* Write has completed. Deassert chip select and return to idle. */
        GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, hwAttrs->csPin);
        object->spiState = STATE_IDLE;

        /* Post write semaphore */
        Semaphore_post(Semaphore_handle(&(object->writeComplete)));
    }
}

/*
 *  ======== WiFiTivaCC3000_triggerRxProcessing ========
 *  End transaction by deasserting CS and calling the receive handler that the
 *  host driver passed in through SpiOpen(). Not to be called by user.
 */
static Void WiFiTivaCC3000_triggerRxProcessing(Void)
{
    SPITivaDMA_HWAttrs     const *spiAttrs;
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Interrupts will be reenabled by the host driver */
    spiAttrs = (SPITivaDMA_HWAttrs *)(object->spiHandle->hwAttrs);
    Hwi_disableInterrupt(spiAttrs->intNum);
    Hwi_disableInterrupt(hwAttrs->irqIntNum);

    /* Deassert chip select */
    GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, hwAttrs->csPin);

    /*
     *  The magic number that resides at the end of the TX/RX buffer (1 byte
     *  after the allocated size) for the purpose of detection of the overrun.
     *  If the magic number is overwritten - buffer overrun occurred - and we
     *  will stuck here forever!
     */
    if (wlan_rx_buffer[WiFi_RX_BUFFER_SIZE - 1] != WiFi_BUF_OVERRUN_DETECT) {
        Log_error0("WiFi: Receive buffer overrun. Closing CC3000.");
        WiFiTivaCC3000_close(wiFiHandle);
        return;
    }

    Log_print0(Diags_USER2, "WiFi: Read transaction from CC3000 has completed. "
                            "Calling receive handler.");

    object->spiState = STATE_IDLE;
    object->spiRxHandler(wlan_rx_buffer + SPI_HEADER_SIZE);
    object->transaction.rxBuf = wlan_rx_buffer;
}

/*
 *  ======== WiFiTivaCC3000_writeWlanEnPin ========
 *  Callback for host driver. Registered with wlan_init() and called by
 *  wlan_start() and wlan_stop().
 */
static Void WiFiTivaCC3000_writeWlanEnPin(UChar val)
{
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    if (val) {
        /* Enable CC3000 */
        GPIOPinWrite(hwAttrs->enPort, hwAttrs->enPin, (~0));
    }
    else {
        /* Disable CC3000 */
        GPIOPinWrite(hwAttrs->enPort, hwAttrs->enPin, 0);
    }

    Log_print1(Diags_USER1, "WiFi: 0x%x was written to the WLAN EN pin.", val);
}

/*
 *  ======== SpiClose ========
 *  Called by wlan_stop().
 */
Void SpiClose(Void)
{
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

#ifndef xdc_runtime_Assert_DISABLE_ALL
    WiFiTivaCC3000_Object *object = wiFiHandle->object;

    /* Make sure WiFi is open */
    Assert_isTrue(object->isOpen != FALSE, NULL);
#endif

    /* Disable IRQ Interrupt */
    GPIOIntDisable(hwAttrs->irqPort, hwAttrs->irqPin);

    Log_print0(Diags_USER1, "WiFi: wlan_stop() was called. IRQ interrupt is now"
                            " disabled.");
}

/*
 *  ======== SpiOpen ========
 *  Called by wlan_start().
 */
Void SpiOpen(RxHandlerFxn rxHandler)
{
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Make sure WiFi is open */
    Assert_isTrue(object->isOpen != FALSE, NULL);

    object->spiState = STATE_POWERUP;
    object->spiRxHandler = rxHandler;

    /* Initialize transaction structure */
    object->transaction.count = 0;
    object->transaction.rxBuf = wlan_rx_buffer;
    object->transaction.txBuf = NULL;

    wlan_rx_buffer[WiFi_RX_BUFFER_SIZE - 1] = WiFi_BUF_OVERRUN_DETECT;
    wlan_tx_buffer[WiFi_TX_BUFFER_SIZE - 1] = WiFi_BUF_OVERRUN_DETECT;

    /* Enable interrupts on IRQ */
    GPIOIntClear(hwAttrs->irqPort, hwAttrs->irqPin);
    GPIOIntEnable(hwAttrs->irqPort, hwAttrs->irqPin);

    /* Enable interrupt in NVIC */
    Hwi_enableInterrupt(hwAttrs->irqIntNum);

    Log_print0(Diags_USER1, "WiFi: wlan_start() was called. IRQ interrupt is "
                            "now enabled.");
}

/*
 *  ======== SpiResumeSpi ========
 *  This function is called by the host driver to reenable interrupts. Called
 *  by hci_unsolicited_event_handler().
 */
Void SpiResumeSpi(Void)
{
    SPITivaDMA_HWAttrs     const *spiAttrs;
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    spiAttrs = (SPITivaDMA_HWAttrs *)(object->spiHandle->hwAttrs);

    Hwi_enableInterrupt(spiAttrs->intNum);
    Hwi_enableInterrupt(hwAttrs->irqIntNum);
}

/*
 *  ======== SpiWrite ========
 *  Function used by the upper-level host-driver to send data and commands to
 *  the CC3000. Called by HCI layer.
 */
Void SpiWrite(UChar *userBuffer, UShort length)
{
    UChar padByte = 0;
    WiFiTivaCC3000_Object        *object  = wiFiHandle->object;
    WiFiTivaCC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Make sure WiFi is open */
    Assert_isTrue(object->isOpen != FALSE, NULL);

    /* Add a padding bit if necessary to make everything 16-bit aligned */
    if (!(length & 0x0001)) {
        padByte = 1;
    }

    userBuffer[0] = WRITE_OPCODE;
    userBuffer[1] = HI(length + padByte);
    userBuffer[2] = LO(length + padByte);
    userBuffer[3] = 0;
    userBuffer[4] = 0;

    length += (SPI_HEADER_SIZE + padByte);

    /*
     *  The magic number that resides at the end of the TX/RX buffer (1 byte
     *  after the allocated size) for the purpose of detection of the overrun.
     *  If the magic number is overwritten - buffer overrun for the purpose of
     *  detection of the overrun. If the magic number is overwritten, buffer
     *  overrun occurred.
     */
    if (wlan_tx_buffer[WiFi_TX_BUFFER_SIZE - 1] != WiFi_BUF_OVERRUN_DETECT) {
        Log_error0("WiFi: Transfer buffer overrun. Closing CC3000.");
        WiFiTivaCC3000_close(wiFiHandle);
        return;
    }

    if (object->spiState == STATE_POWERUP) {
        /* Wait for CC3000 to assert IRQ line */
        while(object->spiState != STATE_INITIALIZED) {
        }
    }

    if (object->spiState == STATE_INITIALIZED) {
        /* IRQ line has been asserted and CC3000 is ready */
        WiFiTivaCC3000_firstWrite(userBuffer, length);
    }
    else {
        /*
         *  This is needed in the case where an IRQ interrupt for a read occurs
         *  as the host device is preparing to perform a write. Must wait until
         *  the state returns to IDLE.
         */
        GPIOIntDisable(hwAttrs->irqPort, hwAttrs->irqPin);
        while(object->spiState != STATE_IDLE) {
        }

        if (length > DMA_WINDOW_SIZE) {
            /* Prepare for a write */
            object->spiState = STATE_WRITE_FIRST_PART;
            object->transaction.txBuf = userBuffer;
            object->transaction.count = DMA_WINDOW_SIZE;

            /* Assert CS line. There will be an IRQ when CC3000 is ready. */
            GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, 0);

            GPIOIntEnable(hwAttrs->irqPort, hwAttrs->irqPin);
            /* Block until write finishes. SPI callback will post semaphore. */
            Semaphore_pend(Semaphore_handle(&(object->writeComplete)), BIOS_WAIT_FOREVER);

            /* Send the rest of the data. */
            object->spiState = STATE_WRITE_EOT;
            object->transaction.txBuf = userBuffer + DMA_WINDOW_SIZE;
            object->transaction.count = length - DMA_WINDOW_SIZE;

            if(!SPI_transfer(object->spiHandle, &object->transaction)){
                Log_error0("WiFi: SPI transfer (write) failed to begin!");
            }
        }
        else {
            /* Prepare for a write */
            object->spiState = STATE_WRITE_EOT;
            object->transaction.txBuf = userBuffer;
            object->transaction.count = length;

            /* Assert CS line. There will be an IRQ when CC3000 is ready. */
            GPIOPinWrite(hwAttrs->csPort, hwAttrs->csPin, 0);
            GPIOIntEnable(hwAttrs->irqPort, hwAttrs->irqPin);
        }
    }

    /* Block until write has finished. SPI callback will post semaphore. */
    Semaphore_pend(Semaphore_handle(&(object->writeComplete)), BIOS_WAIT_FOREVER);

    Log_print0(Diags_USER2, "WiFi: Write transaction to CC3000 has completed.");
}
