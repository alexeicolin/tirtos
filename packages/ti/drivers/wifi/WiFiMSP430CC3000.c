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
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drivers/SPI.h>

#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiMSP430CC3000.h>

/* CC3000 Host Driver header files */
#include <cc3000_host_driver/core_driver/inc/evnt_handler.h>
#include <cc3000_host_driver/include/wlan.h>
#include <spi.h>

/* driverlib header files */
#include <gpio.h>

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

/*
 *  Since there's no way to tell which instance of the CC3000 driver is getting
 *  calls from the upper-level host driver, we can only support one. We will
 *  save the WiFi handle in the WiFi_init() function and use it everywhere.
 */
static WiFi_Handle wiFiHandle;

/* Static buffer for 5 bytes of SPI HEADER */
static UChar spiReadHeader[] = {READ_OPCODE, 0, 0, 0, 0};

/* Function prototypes */
static Void WiFiMSP430CC3000_close(WiFi_Handle handle);
static Void WiFiMSP430CC3000_init(WiFi_Handle handle);
static WiFi_Handle WiFiMSP430CC3000_open(WiFi_Handle handle, UInt spiIndex,
                                         WiFi_evntCallback evntCallback,
                                         WiFi_Params *params);
static Void WiFiMSP430CC3000_readHeader(Void);
static Long WiFiMSP430CC3000_readIrqPin(Void);
static Void WiFiMSP430CC3000_spiCallbackFxn(SPI_Handle spiHandle,
                                            SPI_Transaction *transaction);
static Void WiFiMSP430CC3000_triggerRxProcessing(Void);
static Void WiFiMSP430CC3000_writeWlanEnPin(UChar val);

/* CC3000 function table for MSP430 implementation */
const WiFi_FxnTable WiFiMSP430CC3000_fxnTable = {
    WiFiMSP430CC3000_close,
    WiFiMSP430CC3000_init,
    WiFiMSP430CC3000_open
};

/* Default WiFi params */
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
 *  ======== WiFiMSP430CC3000_close ========
 */
static Void WiFiMSP430CC3000_close(WiFi_Handle handle)
{
    UInt key;
    WiFiMSP430CC3000_Object *object = handle->object;

    /* Disable the CC3000 */
    wlan_stop();

    /* Close the SPI driver */
    if (object->spiHandle != NULL) {
        SPI_close(object->spiHandle);
    }

    /* Delete the semaphore */
    Semaphore_destruct(&(object->writeComplete));

    Log_print0(Diags_USER1, "WiFi: Object closed.");

    key = Hwi_disable();
    object->isOpen = FALSE;
    Hwi_restore(key);
}

/*
 *  ======== WiFiMSP430CC3000_disableIrqInt ========
 *  Callback for host driver. Registered with wlan_init().
 */
static Void WiFiMSP430CC3000_disableIrqInt(Void)
{
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    GPIO_disableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);

    Log_print0(Diags_USER1, "WiFi: IRQ interrupt disabled by Host Driver.");
}

/*
 *  ======== WiFiMSP430CC3000_enableIrqInt ========
 *  Callback for host driver. Registered with wlan_init().
 */
static Void WiFiMSP430CC3000_enableIrqInt(Void)
{
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    GPIO_enableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);

    Log_print0(Diags_USER1, "WiFi: IRQ interrupt enabled by Host Driver.");
}

/*
 *  ======== WiFiMSP430CC3000_firstWrite ========
 *  Function to perform the first write after power up. CC3000 requires a few
 *  50 us delays during this first transaction.
 */
Void WiFiMSP430CC3000_firstWrite(UChar *userBuffer, UShort length)
{
    UInt            delayAmt;
    UInt volatile   i;
    Types_FreqHz    freq;
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    Log_print0(Diags_USER1, "WiFi: Performing first write to CC3000 after "
                            "wlan_start().");

    /* Calculate delay amount */
    BIOS_getCpuFreq(&freq);
    delayAmt = freq.lo/60000;

    /* Assert CS */
    GPIO_setOutputLowOnPin(hwAttrs->csPort, hwAttrs->csPin);

    /* Need to wait at least 50 us after asserting CS */
    for (i = delayAmt; i > 0; i--) {
    }

    /* SPI writes first 4 bytes of data */
    object->transaction.txBuf = userBuffer;
    object->transaction.count = 4;

    if(!SPI_transfer(object->spiHandle, &object->transaction)){
        Log_error0("WiFi: SPI transfer (write) failed to begin!");
    }

    Semaphore_pend(Semaphore_handle(&(object->writeComplete)),
            BIOS_WAIT_FOREVER);

    /* Need to wait at least 50 us again */
    for (i = delayAmt; i > 0; i--) {
    }

    /* SPI writes another 4 bytes of data */
    object->transaction.txBuf = userBuffer + 4;
    object->transaction.count = length - 4;

    object->spiState = STATE_WRITE_EOT;
    if(!SPI_transfer(object->spiHandle, &object->transaction)){
        Log_error0("WiFi: SPI transfer (write) failed to begin!");
    }
}

/*
 *  ======== WiFiMSP430CC3000_hwiIntFxn ========
 *  GPIO interrupt handler for the IRQ line.
 */
Void WiFiMSP430CC3000_hwiIntFxn(UArg arg)
{
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Clear flag */
    GPIO_clearInterruptFlag(hwAttrs->irqPort, hwAttrs->irqPin);

    /* If the IRQ pin has been asserted... */
    if(!GPIO_getInputPinValue(hwAttrs->irqPort, hwAttrs->irqPin)) {
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
            GPIO_setOutputLowOnPin(hwAttrs->csPort, hwAttrs->csPin);
            WiFiMSP430CC3000_readHeader();
        }
        else if (object->spiState == STATE_WRITE_EOT) {
            /* Wi-Fi device is ready to receive a write from the host */
            if(!SPI_transfer(object->spiHandle, &object->transaction)){
                Log_error0("WiFi: SPI transfer (write) failed to begin!");
            }
        }
    }
}

/*
 *  ======== WiFiMSP430CC3000_init ========
 */
static Void WiFiMSP430CC3000_init(WiFi_Handle handle)
{
    /*
     *  We save the handle to a global variable to be used in the functions that
     *  are called from the CC3000 Host Driver.
     */
    wiFiHandle = handle;

    /* Mark the object as available. */
    ((WiFiMSP430CC3000_Object*)wiFiHandle->object)->isOpen = FALSE;
}

/*
 *  ======== WiFiMSP430CC3000_open ========
 */
static WiFi_Handle WiFiMSP430CC3000_open(WiFi_Handle handle, UInt spiIndex,
                                         WiFi_evntCallback evntCallback,
                                         WiFi_Params *params)
{
    UInt              key;
    SPI_Params        spiParams;
    Semaphore_Params  semParams;
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Determine if the WiFi object has already been opened */
    key = Hwi_disable();
    if (object->isOpen == TRUE) {
        Hwi_restore(key);
        Log_warning0("WiFi: This index is already in use.");
        return (NULL);
    }

    /* Mark the handle as being used */
    object->isOpen = TRUE;
    Hwi_restore(key);

    /* Create a semaphore to block for the write transaction. */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&(object->writeComplete), 0, &semParams);

    /* Set up the parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (WiFi_Params *) &WiFi_defaultParams;
    }

    /* GPIO Configuration */
    /* Configure IRQ pin */
    GPIO_setAsInputPinWithPullUpresistor(hwAttrs->irqPort, hwAttrs->irqPin);
    GPIO_interruptEdgeSelect(hwAttrs->irqPort, hwAttrs->irqPin,
                   GPIO_HIGH_TO_LOW_TRANSITION);

    /* Configure WLAN EN pin */
    GPIO_setAsOutputPin(hwAttrs->enPort, hwAttrs->enPin);
    GPIO_setOutputLowOnPin(hwAttrs->enPort, hwAttrs->enPin);

    /* Configure CS pin */
    GPIO_setAsOutputPin(hwAttrs->csPort, hwAttrs->csPin);
    GPIO_setOutputHighOnPin(hwAttrs->csPort, hwAttrs->csPin);

    /* Initialize SPI parameters */
    SPI_Params_init(&spiParams);
    spiParams.transferMode = SPI_MODE_CALLBACK;
    spiParams.transferCallbackFxn = WiFiMSP430CC3000_spiCallbackFxn;
    spiParams.mode = SPI_MASTER;
    spiParams.bitRate = params->bitRate;
    spiParams.dataSize = 8;
    spiParams.frameFormat = SPI_POL0_PHA1;

    /* Open SPI driver */
    object->spiHandle = SPI_open(spiIndex, &spiParams);
    if (object->spiHandle == NULL) {
        Log_error0("WiFi: Error opening SPI. WiFi instance was not opened.");
        WiFiMSP430CC3000_close(handle);
        return (NULL);
    }

    Log_print0(Diags_USER1, "WiFi: Object created.");

    /* Register callbacks with the host driver */
    wlan_init(evntCallback, NULL, NULL, NULL,
              WiFiMSP430CC3000_readIrqPin,
              WiFiMSP430CC3000_enableIrqInt,
              WiFiMSP430CC3000_disableIrqInt,
              WiFiMSP430CC3000_writeWlanEnPin);

    /* Start the CC3000. Pass in 0 because no patches are available. */
    wlan_start(0);

    return (handle);
}

/*
 *  ======== WiFiMSP430CC3000_readData ========
 *  Analyze first ten bytes received and continue to read if necessary. Not to
 *  be called by user.
 */
static Long WiFiMSP430CC3000_readData(Void)
{
    Long    dataToRecv;
    UChar   type;
    WiFiMSP430CC3000_Object *object = wiFiHandle->object;

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
            break;

        case HCI_TYPE_EVNT:
            /* Calculate amount of remaining data */
            STREAM_TO_UINT8((Char *)(wlan_rx_buffer + SPI_HEADER_SIZE),
                            HCI_EVENT_LENGTH_OFFSET, dataToRecv);
            dataToRecv -= 1;
            break;
    }

    /* Make amount 16-bit aligned if needed */
    if ((HEADERS_SIZE_EVNT + dataToRecv) & 1) {
        dataToRecv++;
    }

    /* If there is data remaining, call the read function*/
    if (dataToRecv) {
        object->transaction.rxBuf = wlan_rx_buffer + 10;
        object->transaction.txBuf = wlan_tx_buffer;
        object->transaction.count = dataToRecv;
        object->spiState = STATE_READ_EOT;

        if(!SPI_transfer(object->spiHandle, &object->transaction)){
            Log_error0("WiFi: SPI transfer (read) failed to begin!");
        }
    }

    return (dataToRecv);
}

/*
 *  ======== WiFiMSP430CC3000_readDataCont ========
 *  This function is called by the SPI interrupt handler after reading the
 *  header.
 */
static Void WiFiMSP430CC3000_readDataCont(Void)
{
    if (!WiFiMSP430CC3000_readData())
    {
        /* All data has been read */
        WiFiMSP430CC3000_triggerRxProcessing();
    }
}

/*
 *  ======== WiFiMSP430CC3000_readHeader ========
 *  Function reads 10 bytes--5 bytes of SPI Header and 5 bytes of data.
 */
static Void WiFiMSP430CC3000_readHeader(Void)
{
    WiFiMSP430CC3000_Object *object = wiFiHandle->object;

    /* Read first 10 bytes */
    object->transaction.count = 10;
    object->transaction.txBuf = spiReadHeader;

    if(!SPI_transfer(object->spiHandle, &object->transaction)){
        Log_error0("WiFi: SPI transfer (read header) failed to begin!");
    }
}

/*
 *  ======== WiFiMSP430CC3000_readIrqPin ========
 *  Callback for host driver. Registered with wlan_init().
 */
static Long WiFiMSP430CC3000_readIrqPin(Void)
{
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    return (GPIO_getInputPinValue(hwAttrs->irqPort, hwAttrs->irqPin));
}

/*
 *  ======== WiFiMSP430CC3000_spiCallbackFxn ========
 *  Called by the SPI driver when a transmit has completed.
 */
static Void WiFiMSP430CC3000_spiCallbackFxn(SPI_Handle spiHandle,
                                            SPI_Transaction *transaction)
{
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    if (object->spiState == STATE_READ_IRQ) {
        /* Header has been read. Analyze it and proceed. */
        WiFiMSP430CC3000_readDataCont();
    }
    else if (object->spiState == STATE_READ_EOT) {
        /* Read has completed. Trigger processing. */
        WiFiMSP430CC3000_triggerRxProcessing();
    }
    else if ((object->spiState == STATE_INITIALIZED) ||
             (object->spiState == STATE_WRITE_EOT)) {
        if (object->spiState == STATE_WRITE_EOT) {
            /* Write has completed. Deassert chip select and return to idle. */
            GPIO_setOutputHighOnPin(hwAttrs->csPort, hwAttrs->csPin);
            object->spiState = STATE_IDLE;
        }

        /* Post write semaphore */
        Semaphore_post(Semaphore_handle(&(object->writeComplete)));
    }
}

/*
 *  ======== WiFiMSP430CC3000_triggerRxProcessing ========
 *  End transaction by deasserting CS and calling the receive handler that the
 *  host driver passed in through SpiOpen(). Not to be called by user.
 */
static Void WiFiMSP430CC3000_triggerRxProcessing(Void)
{
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Interrupts will be reenabled by the host driver */
    GPIO_disableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);

    /* Deassert chip select */
    GPIO_setOutputHighOnPin(hwAttrs->csPort, hwAttrs->csPin);

    /*
     *  The magic number that resides at the end of the TX/RX buffer (1 byte
     *  after the allocated size) for the purpose of detection of the overrun.
     *  If the magic number is overwritten - buffer overrun occurred - and we
     *  will stuck here forever!
     */
    if (wlan_rx_buffer[WiFi_RX_BUFFER_SIZE - 1] != WiFi_BUF_OVERRUN_DETECT) {
        Log_error0("WiFi: Receive buffer overrun. Closing CC3000.");
        WiFiMSP430CC3000_close(wiFiHandle);
        return;
    }

    Log_print0(Diags_USER2, "WiFi: Read transaction from CC3000 has completed. "
                            "Calling receive handler.");

    object->spiState = STATE_IDLE;
    object->spiRxHandler(wlan_rx_buffer + SPI_HEADER_SIZE);
    object->transaction.rxBuf = wlan_rx_buffer;
}

/*
 *  ======== WiFiMSP430CC3000_writeWlanEnPin ========
 *  Callback for host driver. Registered with wlan_init() and called by
 *  wlan_start() and wlan_stop().
 */
static Void WiFiMSP430CC3000_writeWlanEnPin(UChar val)
{
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    if (val) {
        /* Enable CC3000 */
        GPIO_setOutputHighOnPin(hwAttrs->enPort, hwAttrs->enPin);
    }
    else {
        /* Disable CC3000 */
        GPIO_setOutputLowOnPin(hwAttrs->enPort, hwAttrs->enPin);
    }

    Log_print1(Diags_USER1, "WiFi: 0x%x was written to the WLAN EN pin.", val);
}

/*
 *  ======== SpiClose ========
 *  Called by wlan_stop().
 */
Void SpiClose(Void)
{
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

#ifndef xdc_runtime_Assert_DISABLE_ALL
    WiFiMSP430CC3000_Object *object = wiFiHandle->object;

    /* Make sure WiFi is open */
    Assert_isTrue(object->isOpen == TRUE, NULL);
#endif

    /* Disable IRQ Interrupt */
    GPIO_disableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);

    Log_print0(Diags_USER1, "WiFi: wlan_stop() was called. IRQ interrupt is now"
                            " disabled.");
}

/*
 *  ======== SpiOpen ========
 *  Called by wlan_start().
 */
Void SpiOpen(RxHandlerFxn rxHandler)
{
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Make sure WiFi is open */
    Assert_isTrue(object->isOpen == TRUE, NULL);

    object->spiState = STATE_POWERUP;
    object->spiRxHandler = rxHandler;

    /* Initialize transaction structure */
    object->transaction.count = 0;
    object->transaction.rxBuf = wlan_rx_buffer;
    object->transaction.txBuf = NULL;

    wlan_rx_buffer[WiFi_RX_BUFFER_SIZE - 1] = WiFi_BUF_OVERRUN_DETECT;
    wlan_tx_buffer[WiFi_TX_BUFFER_SIZE - 1] = WiFi_BUF_OVERRUN_DETECT;

    /* Enable interrupts on IRQ */
    GPIO_clearInterruptFlag(hwAttrs->irqPort, hwAttrs->irqPin);
    GPIO_enableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);

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
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    GPIO_enableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);
}

/*
 *  ======== SpiWrite ========
 *  Function used by the upper-level host-driver to send data and commands to
 *  the CC3000. Called by HCI layer.
 */
Void SpiWrite(UChar *userBuffer, UShort length)
{
    UChar padByte = 0;
    WiFiMSP430CC3000_Object        *object  = wiFiHandle->object;
    WiFiMSP430CC3000_HWAttrs const *hwAttrs = wiFiHandle->hwAttrs;

    /* Make sure WiFi is open */
    Assert_isTrue(object->isOpen == TRUE, NULL);

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
        WiFiMSP430CC3000_close(wiFiHandle);
        return;
    }

    if (object->spiState == STATE_POWERUP) {
        /* Wait for CC3000 to assert IRQ line */
        while(object->spiState != STATE_INITIALIZED) {
        }
    }

    if (object->spiState == STATE_INITIALIZED) {
        /* IRQ line has been asserted and CC3000 is ready */
        WiFiMSP430CC3000_firstWrite(userBuffer, length);
    }
    else {
        /*
         *  This is needed in the case where an IRQ interrupt for a read occurs
         *  as the host device is preparing to perform a write. Must wait until
         *  the state returns to IDLE.
         */
        GPIO_disableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);
        while(object->spiState != STATE_IDLE) {
        }

        /* Prepare for a write */
        object->spiState = STATE_WRITE_EOT;
        object->transaction.txBuf = userBuffer;
        object->transaction.count = length;

        /* Assert CS line. There will be an IRQ when CC3000 is ready. */
        GPIO_setOutputLowOnPin(hwAttrs->csPort, hwAttrs->csPin);
        GPIO_enableInterrupt(hwAttrs->irqPort, hwAttrs->irqPin);
    }

    /* Block until write has finished. SPI callback will post semaphore. */
    Semaphore_pend(Semaphore_handle(&(object->writeComplete)),
            BIOS_WAIT_FOREVER);

    Log_print0(Diags_USER2, "WiFi: Write transaction to CC3000 has completed.");
}
