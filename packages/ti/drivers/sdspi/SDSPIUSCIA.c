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

/*-----------------------------------------------------------------------*/
/* MMC/SDC (in SPI mode) control module  (C)ChaN, 2007                   */
/*-----------------------------------------------------------------------*/

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Types.h>
#include <xdc/runtime/Timestamp.h>

#include <ti/drivers/sdspi/SDSPIUSCIA.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/msp430/ClockFreqs.h>
#include <ti/sysbios/hal/Hwi.h>

/* driverlib header files */
#include <gpio.h>
#include <usci_a_spi.h>

/* Definitions for MMC/SDC command */
#define CMD0                        (0x40+0)    /* GO_IDLE_STATE */
#define CMD1                        (0x40+1)    /* SEND_OP_COND */
#define CMD8                        (0x40+8)    /* SEND_IF_COND */
#define CMD9                        (0x40+9)    /* SEND_CSD */
#define CMD10                       (0x40+10)   /* SEND_CID */
#define CMD12                       (0x40+12)   /* STOP_TRANSMISSION */
#define CMD16                       (0x40+16)   /* SET_BLOCKLEN */
#define CMD17                       (0x40+17)   /* READ_SINGLE_BLOCK */
#define CMD18                       (0x40+18)   /* READ_MULTIPLE_BLOCK */
#define CMD23                       (0x40+23)   /* SET_BLOCK_COUNT */
#define CMD24                       (0x40+24)   /* WRITE_BLOCK */
#define CMD25                       (0x40+25)   /* WRITE_MULTIPLE_BLOCK */
#define CMD41                       (0x40+41)   /* SEND_OP_COND (ACMD) */
#define CMD55                       (0x40+55)   /* APP_CMD */
#define CMD58                       (0x40+58)   /* READ_OCR */

#define SD_SECTOR_SIZE              512

#define START_BLOCK_TOKEN           0xFE
#define START_MULTIBLOCK_TOKEN      0xFC
#define STOP_MULTIBLOCK_TOKEN       0xFD

#define DRIVE_NOT_MOUNTED           ~0

/*
 * Array of SDSPI_Handles to determine the association of the FatFs drive number
 * with a SDSPI_Handle
 * _VOLUMES is defined in <ti/sysbios/fatfs/ffconf.h>
 */
static SDSPI_Handle sdspiHandles[_VOLUMES];

/* ms scaling to function timeouts */
static Bits32 mSScalingFactor = 0;

/* Function prototypes */
static UINT         rcvr_datablock(SDSPIUSCIA_HWAttrs const *hwAttrs,
                                   UChar *buf, UINT btr);
static inline Void  releaseSPIBus(SDSPIUSCIA_HWAttrs const *hwAttrs);
static inline UChar rxSPI(SDSPIUSCIA_HWAttrs const *hwAttrs);
static UChar        send_cmd(SDSPIUSCIA_HWAttrs const *hwAttrs, UChar cmd,
                             ULong arg);
static Void         send_initial_clock_train(SDSPIUSCIA_HWAttrs const *hwAttrs);
static inline Void  takeSPIBus(SDSPIUSCIA_HWAttrs const *hwAttrs);
static inline Void  txSPI(SDSPIUSCIA_HWAttrs const *hwAttrs, UChar dat);
static UChar        wait_ready(SDSPIUSCIA_HWAttrs const *hwAttrs);
static UINT         xmit_datablock(SDSPIUSCIA_HWAttrs const *hwAttrs,
                                   const UChar *buf, UChar token);

/* FatFs disk I/O functions */
DSTATUS             SDSPIUSCIA_diskInitialize(UChar drv);
DRESULT             SDSPIUSCIA_diskIOctrl(UChar drv, UChar ctrl, Void *buf);
DRESULT             SDSPIUSCIA_diskRead(UChar drv, UChar *buf, ULong sector,
                                        UChar count);
DSTATUS             SDSPIUSCIA_diskStatus(UChar drv);
DRESULT             SDSPIUSCIA_diskWrite(UChar drv, const UChar *buf,
                                         ULong sector, UChar count);

/* SDSPIUSCIA functions */
Void         SDSPIUSCIA_init(SDSPI_Handle handle);
SDSPI_Handle SDSPIUSCIA_open(SDSPI_Handle handle, UInt drv,
                             SDSPI_Params *params);
Void         SDSPIUSCIA_close(SDSPI_Handle handle);

/* SDSPI function table for SDSPIUSCIA implementation */
const SDSPI_FxnTable SDSPIUSCIA_fxnTable = {
    SDSPIUSCIA_init,
    SDSPIUSCIA_open,
    SDSPIUSCIA_close,
};

/* Default SDSPI params */
extern const SDSPI_Params SDSPI_defaultParams;

/*
 *  ======== rcvr_datablock ========
 *  Function to receive a block of data from the SDCard
 *
 *  btr count must be an even number
 */
static UInt rcvr_datablock(SDSPIUSCIA_HWAttrs const *hwAttrs, UChar *buf,
                           UInt btr)
{
    UChar   token;
    Bits32  timeoutLimit;

    /* Wait for data packet in timeout of 100 ms */
    timeoutLimit = Timestamp_get32() + 100 * mSScalingFactor;
    do {
        token = rxSPI(hwAttrs);
    } while ((token == 0xFF) && (Timestamp_get32() <= timeoutLimit));

    if (token != START_BLOCK_TOKEN) {
        /* If not valid data token, return error */
        return (0);
    }

    /* Receive the data block into buffer */
    do {
        *(buf++) = rxSPI(hwAttrs);
    } while (--btr);

    /* Read the CRC, but discard it */
    rxSPI(hwAttrs);
    rxSPI(hwAttrs);

    /* Return with success */
    return (1);
}

/*
 *  ======== releaseSPIBus ========
 *  Function to release the SPI bus
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 */
static inline Void releaseSPIBus(SDSPIUSCIA_HWAttrs const *hwAttrs)
{
    /* Deselect the SD card. */
    GPIO_setOutputHighOnPin(hwAttrs->portCS, hwAttrs->pinCS);
}

/*
 *  ======== rxSPI ========
 *  Function to receive one byte onto the SPI bus. Polling (Blocked)
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 */
static inline UChar rxSPI(SDSPIUSCIA_HWAttrs const *hwAttrs)
{
    UChar   rcvdat;

    /* Wait for all TX/RX to finish */
    while (!USCI_A_SPI_getInterruptStatus(hwAttrs->baseAddr,
                USCI_A_SPI_TRANSMIT_INTERRUPT)) {
    };

    /* write dummy data */
    USCI_A_SPI_transmitData(hwAttrs->baseAddr, 0xFF);

    /* Wait while not ready for RX */
    while (!USCI_A_SPI_getInterruptStatus(hwAttrs->baseAddr,
                USCI_A_SPI_RECEIVE_INTERRUPT)) {
    };

    /* Read data frm RX */
    rcvdat = USCI_A_SPI_receiveData(hwAttrs->baseAddr);

    return (rcvdat);
}

/*
 *  ======== send_cmd ========
 *  Function that will transmit an command to the SDCard
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 *
 *  @param  cmd         SD command
 *
 *  @param  arg         SD command argument
 */
static UChar send_cmd(SDSPIUSCIA_HWAttrs const *hwAttrs, UChar cmd, ULong arg)
{
    UChar   n;
    UChar   res;

    if (wait_ready(hwAttrs) != 0xFF) {
        Log_print1(Diags_USER1, "SDSPI:(%p) send_cmd: SD card wait time expired",
                                 hwAttrs->baseAddr);
        return (0xFF);
    }

    /* Send command packet */
    txSPI(hwAttrs, cmd);                  /* Command */
    txSPI(hwAttrs, (UChar)(arg >> 24));   /* Argument[31..24] */
    txSPI(hwAttrs, (UChar)(arg >> 16));   /* Argument[23..16] */
    txSPI(hwAttrs, (UChar)(arg >> 8));    /* Argument[15..8] */
    txSPI(hwAttrs, (UChar)arg);           /* Argument[7..0] */

    if (cmd == CMD0) {
        /* CRC for CMD0(0) */
        n = 0x95;
    }
    else if (cmd == CMD8) {
        /* CRC for CMD8(0x1AA) */
        n = 0x87;
    }
    else {
        /* Default CRC should be at least 0x01 */
        n = 0x01;
    }

    /* Future enhancement to add CRC support */
    txSPI(hwAttrs, n);

    /* Receive command response */
    if (cmd == CMD12) {
        /* Skip a stuff byte when stop reading */
        rxSPI(hwAttrs);
    }

    /* Wait for a valid response in timeout; 10 attempts */
    n = 10;
    do {
        res = rxSPI(hwAttrs);
    } while ((res & 0x80) && --n);

    /* Return with the response value */
    return (res);
}

/*
 *  ======== send_initial_clock_train ========
 *  Function to get the SDCard into SPI mode
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 */
static Void send_initial_clock_train(SDSPIUSCIA_HWAttrs const *hwAttrs)
{
    UInt            i;

    /* Deselect the SD card. */
    GPIO_setOutputHighOnPin(hwAttrs->portCS, hwAttrs->pinCS);

    /* Switch the SPI TX line to a GPIO and drive it high too. */
    GPIO_setAsOutputPin(hwAttrs->portSPI, hwAttrs->pinMOSI);
    GPIO_setOutputHighOnPin(hwAttrs->portSPI, hwAttrs->pinMOSI);

    /*
     * Send 10 bytes over the SPI bus. This causes the clock to toggle several
     * times to get the SD Card into SPI mode.
     */
    for (i = 0; i < 10; i++) {
        rxSPI(hwAttrs);
    }

    /* Revert to hardware control of theSPITX line. */
    GPIO_setAsPeripheralModuleFunctionOutputPin(hwAttrs->portSPI,
                                                hwAttrs->pinMOSI);

    Log_print1(Diags_USER1, "SDSPI:(%p) initialized SD card to SPI mode",
                             hwAttrs->baseAddr);
}

/*
 *  ======== takeSPIBus ========
 *  Function to take the SPI bus
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 */
static inline Void takeSPIBus(SDSPIUSCIA_HWAttrs const *hwAttrs)
{
    /* Select the SD card. */
    GPIO_setOutputLowOnPin(hwAttrs->portCS, hwAttrs->pinCS);
}

/*
 *  ======== txSPI ========
 *  Function to transmit one byte onto the SPI bus. Polling (Blocked)
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 *
 *  @param  dat         Data to be sent onto the SPI bus
 */
static inline Void txSPI(SDSPIUSCIA_HWAttrs const *hwAttrs, UChar dat)
{
    UInt key;

    key = Hwi_disable();

    /* Wait for all TX/RX to finish */
    while (!USCI_A_SPI_getInterruptStatus(hwAttrs->baseAddr,
                USCI_A_SPI_TRANSMIT_INTERRUPT)) {
    };

    /* Write the data to the TX */
    USCI_A_SPI_transmitData(hwAttrs->baseAddr, dat);

    /* Wait for all TX/RX to finish */
    while (!USCI_A_SPI_getInterruptStatus(hwAttrs->baseAddr,
                USCI_A_SPI_RECEIVE_INTERRUPT)) {
    };

    /* flush data read during the write */
    USCI_A_SPI_receiveData(hwAttrs->baseAddr);

    Hwi_restore(key);
}

/*
 *  ======== wait_ready ========
 *  Function to check if the SDCard is busy
 *
 *  This function queries the SDCard to see if it is in a busy state or ready
 *  state
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 */
static UChar wait_ready(SDSPIUSCIA_HWAttrs const *hwAttrs)
{
    UChar   res;
    Bits32  timeoutLimit;

    /* Wait for data packet in timeout of 500 ms */
    timeoutLimit = Timestamp_get32() + 500 * mSScalingFactor;

    rxSPI(hwAttrs);
    do {
        res = rxSPI(hwAttrs);
    } while ((res != 0xFF) && (Timestamp_get32() <= timeoutLimit));

    return (res);
}

/* _READONLY is defined in <ti/sysbios/fatfs/diskio.h> */
#if _READONLY == 0
/*
 *  ======== xmit_datablock ========
 *  Function to transmit a block of data to the SDCard
 *
 *  @param  hwAttrs     Pointer to hardware attributes
 *
 *  @param  params      SDSPIUSCIA hardware attributes
 *
 *  @param  buf         pointer to const data buffer
 *
 *  @param  token       command token to be sent to the SD card prior to
 *                      sending the data block. The available tokens are:
 *                      START_BLOCK_TOKEN
 *                      START_MULTIBLOCK_TOKEN
 *                      STOP_MULTIBLOCK_TOKEN
 */
static UINT xmit_datablock(SDSPIUSCIA_HWAttrs const *hwAttrs,
                           const UChar *buf, UChar token)
{
    UChar   resp;
    UChar   wc;

    if (wait_ready(hwAttrs) != 0xFF) {
        /* Return with error */
        return (0);
    }

    /* Xmit data token */
    txSPI(hwAttrs, token);

    /* Send data only when token != STOP_MULTIBLOCK_TOKEN */
    if (token != STOP_MULTIBLOCK_TOKEN) {
        /* Is data token */
        wc = 0;
        /* Transferring 512 byte blocks using a 8 bit counter */
        do {
            /* Xmit the SD_SECTOR_SIZE byte data block */
            txSPI(hwAttrs, *buf++);
            txSPI(hwAttrs, *buf++);
        } while (--wc);

        /* Future enhancement to add CRC support */
        txSPI(hwAttrs, 0xFF);
        txSPI(hwAttrs, 0xFF);

        /* Reveive data response */
        resp = rxSPI(hwAttrs);

        /* If not accepted, return error */
        if ((resp & 0x1F) != 0x05) {
            return (0);
        }
    }

    /* Return with success */
    return (1);
}
#endif /* _READONLY */

/*
 *  ======== SDSPIUSCIA_close ========
 *  Function to unmount the FatFs filesystem and unregister the SDSPIUSCIA
 *  disk I/O functions from SYS/BIOS' FatFS module.
 *
 *  @param  handle      SDSPI_Handle returned by SDSPI_open()
 */
Void SDSPIUSCIA_close(SDSPI_Handle handle)
{
    UInt                        key;
    DRESULT                     dresult;
    FRESULT                     fresult;
    SDSPIUSCIA_Object          *object = handle->object;
    SDSPIUSCIA_HWAttrs const   *hwAttrs = handle->hwAttrs;

    /* Unmount the FatFs drive */
    fresult = f_mount(object->driveNumber, NULL);
    if (fresult != FR_OK) {
        Log_print2(Diags_USER1, "SDSPI:(%p) Could not unmount FatFs volume @ "
                                "drive number %d",
                                 hwAttrs->baseAddr, object->driveNumber);
    }

    /* Unregister the disk_*() functions */
    dresult = disk_unregister(object->driveNumber);
    if (dresult != RES_OK) {
        Log_print2(Diags_USER1, "SDSPI:(%p) Error unregistering disk functions "
                                "@ drive number %d",
                                 hwAttrs->baseAddr, object->driveNumber);
    }

    USCI_A_SPI_disable(hwAttrs->baseAddr);

    Log_print1(Diags_USER1, "SDSPI:(%p) closed", hwAttrs->baseAddr);

    key = Hwi_disable();
    object->driveNumber = DRIVE_NOT_MOUNTED;
    Hwi_restore(key);
}

/*
 *  ======== SDSPIUSCIA_diskInitialize ========
 *  Function to initialize the SD Card.  This function is called by the FatFs
 *  module and must not be called by the application!
 *
 *  @param  drv         Drive Number
 */
DSTATUS SDSPIUSCIA_diskInitialize(UChar drv)
{
    UChar                       n;
    UChar                       ocr[4];
    Bits32                      clockFreq;
    Bits32                      timeoutLimit;
    SDSPIUSCIA_CardType         cardType;
    SDSPIUSCIA_Object          *object = sdspiHandles[drv]->object;
    SDSPIUSCIA_HWAttrs const   *hwAttrs = sdspiHandles[drv]->hwAttrs;

    /* No card in the socket */
    if (object->diskState & STA_NODISK) {
        Log_error1("SDSPI:(%p) disk initialization failed: No disk",
                    hwAttrs->baseAddr);

        return (object->diskState);
    }

    /* Initialize the SD Card for SPI mode */
    send_initial_clock_train(hwAttrs);

    /* Select the SD Card's chip select */
    takeSPIBus(hwAttrs);
    cardType = NOCARD;

    /* Send the CMD0 to put the SD Card in "Idle" state */
    if (send_cmd(hwAttrs, CMD0, 0) == 1) {

        /*
         * Determine what SD Card version we are dealing with
         * Depending on which SD Card version, we need to send different SD
         * commands to the SD Card, which will have different response fields.
         */

        if (send_cmd(hwAttrs, CMD8, 0x1AA) == 1) {
            /* SDC Ver2+ */
            for (n = 0; n < 4; n++) {
                ocr[n] = rxSPI(hwAttrs);
            }

            /*
             * Ensure that the card's voltage range is valid
             * The card can work at vdd range of 2.7-3.6V
             */
            if ((ocr[2] == 0x01) && (ocr[3] == 0xAA)) {
                /* Wait for data packet in timeout of 1s */
                timeoutLimit = Timestamp_get32() + 1000 * mSScalingFactor;
                do {
                    /* ACMD41 with HCS bit */
                    if (send_cmd(hwAttrs, CMD55, 0) <= 1 &&
                        send_cmd(hwAttrs, CMD41, 1UL << 30) == 0) {
                        timeoutLimit = 1;
                        break;
                    }
                } while (Timestamp_get32() <= timeoutLimit);

                /*
                 * Check CCS bit to determine which type of capacity we are
                 * dealing with
                 */
                if ((timeoutLimit) && send_cmd(hwAttrs, CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) {
                        ocr[n] = rxSPI(hwAttrs);
                    }
                    cardType = (ocr[0] & 0x40) ? SDHC : SDSC;
                }
            }
        }

        /* SDC Ver1 or MMC */
        else {
            /*
             * The card verion is not SDC V2+ so check if we are dealing with a
             * SDC or MMC card
             */
            if ((send_cmd(hwAttrs, CMD55, 0) <= 1 &&
                 send_cmd(hwAttrs, CMD41, 0) <= 1)) {
                cardType = SDSC;
            }
            else {
                cardType = MMC;
            }

            /* Wait for data packet in timeout of 1s */
            timeoutLimit = Timestamp_get32() + 1000 * mSScalingFactor;
            do {
                if (cardType == SDSC) {
                    /* ACMD41 */
                    if (send_cmd(hwAttrs, CMD55, 0) <= 1 &&
                        send_cmd(hwAttrs, CMD41, 0) == 0) {
                        timeoutLimit = 1;
                        break;
                    }
                } else {
                    /* CMD1 */
                    if (send_cmd(hwAttrs, CMD1, 0) == 0) {
                        timeoutLimit = 1;
                        break;
                    }
                }
            } while (Timestamp_get32() <= timeoutLimit);

            /* Select R/W block length */
            if (!(timeoutLimit) ||
                  send_cmd(hwAttrs, CMD16, SD_SECTOR_SIZE) != 0) {
                cardType = NOCARD;
            }
        }
    }

    object->cardType = cardType;

    /* Deselect the SD Card's chip select */
    releaseSPIBus(hwAttrs);

    /* Idle (Release DO) */
    rxSPI(hwAttrs);

    /* Check to see if a card type was determined */
    if (cardType != NOCARD) {
        /*
         * Reconfigure the SPI bust at the new frequency rate
         * Since we've gotten this far, we know that hwAttrs->clockSource
         * is valid. No need to add a default error case.
         */
        switch (hwAttrs->clockSource) {
            case USCI_A_SPI_CLOCKSOURCE_ACLK:
                clockFreq = ClockFreqs_getFrequency(ClockFreqs_Clock_ACLK);
                Log_print1(Diags_USER1, "ClockFreqs_getFrequency ACLK: %d", clockFreq);
                break;

            case USCI_A_SPI_CLOCKSOURCE_SMCLK:
                clockFreq = ClockFreqs_getFrequency(ClockFreqs_Clock_SMCLK);
                Log_print1(Diags_USER1, "ClockFreqs_getFrequency SMCLK: %d", clockFreq);
                break;
        }

        USCI_A_SPI_disable(hwAttrs->baseAddr);

        USCI_A_SPI_masterChangeClock(hwAttrs->baseAddr,
                                     clockFreq,
                                     object->bitRate);

        USCI_A_SPI_enable(hwAttrs->baseAddr);

        Log_print3(Diags_USER1, "SDSPI:(%p) CPU freq: %d; Reconfiguring SDSPI "
                                "freq to %d",
                                 hwAttrs->baseAddr, clockFreq, object->bitRate);

        /* Initialization succeeded */
        object->diskState &= ~STA_NOINIT;
    }
    else {
        Log_print1(Diags_USER1, "SDSPI:(%p) disk initialization failed",
                                 hwAttrs->baseAddr);
    }

    return (object->diskState);
}

/*
 *  ======== SDSPIUSCIA_diskIOctrl ========
 *  Function to perform specifed disk operations. This function is called by the
 *  FatFs module and must not be called by the application!
 *
 *  @param  drv         Drive Number
 *
 *  @param  ctrl        Control code
 *
 *  @param  buf         Buffer to send/receive control data
 */
DRESULT SDSPIUSCIA_diskIOctrl(UChar drv, UChar ctrl, Void *buf)
{
    DRESULT                     res = RES_ERROR;
    UChar                       n;
    UChar                       csd[16];
    WORD                        csize;
    SDSPIUSCIA_Object          *object = sdspiHandles[drv]->object;
    SDSPIUSCIA_HWAttrs const   *hwAttrs = sdspiHandles[drv]->hwAttrs;

    if (object->diskState & STA_NOINIT) {
        Log_error1("SDSPI:(%p) disk IO control: disk not initialized",
                    hwAttrs->baseAddr);

        return (RES_NOTRDY);
    }

    /* Select the SD Card's chip select */
    takeSPIBus(hwAttrs);

    switch (ctrl) {
        case GET_SECTOR_COUNT:
            /* Get number of sectors on the disk (ULong) */
            if ((send_cmd(hwAttrs, CMD9, 0) == 0) &&
                 rcvr_datablock(hwAttrs, csd, 16)) {

                /* SDC ver 2.00 */
                if ((csd[0] >> 6) == 1) {
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(ULong*)buf = (ULong)csize << 10;
                }
                /* MMC or SDC ver 1.XX */
                else {
                    n =  (csd[5] & 15) +
                        ((csd[10] & 128) >> 7) +
                        ((csd[9] & 3) << 1) + 2;

                    csize =        (csd[8] >> 6) +
                             ((WORD)csd[7] << 2) +
                            ((WORD)(csd[6] & 3) << 10) + 1;

                    *(ULong*)buf = (ULong)csize << (n - 9);
                }
                Log_print2(Diags_USER2, "SDSPI:(%p) disk IO control: sector "
                                        "count: %d",
                                         hwAttrs->baseAddr, *(ULong*)buf);
                res = RES_OK;
            }
            break;

        case GET_SECTOR_SIZE:
            /* Get sectors on the disk (WORD) */
            *(WORD*)buf = SD_SECTOR_SIZE;
                Log_print2(Diags_USER2, "SDSPI:(%p) disk IO control: sector "
                                        "size: %d",
                                         hwAttrs->baseAddr, *(WORD*)buf);
            res = RES_OK;
            break;

        case CTRL_SYNC:
            /* Make sure that data has been written */
            if (wait_ready(hwAttrs) == 0xFF) {
                Log_print1(Diags_USER2, "SDSPI:(%p) disk IO control: control "
                                        "sync: ready",
                                         hwAttrs->baseAddr);
                res = RES_OK;
            }
            else {
                Log_print1(Diags_USER2, "SDSPI:(%p) disk IO control: control "
                                        "sync: not ready",
                                         hwAttrs->baseAddr);
                res = RES_NOTRDY;
            }
            break;

        default:
            Log_print1(Diags_USER2, "SDSPI:(%p) disk IO control: parameter "
                                    "error",
                                     hwAttrs->baseAddr);
            res = RES_PARERR;
            break;
    }

    /* Deselect the SD Card's chip select */
    releaseSPIBus(hwAttrs);

    /* Idle (Release DO) */
    rxSPI(hwAttrs);

    return (res);
}

/*
 *  ======== SDSPIUSCIA_diskRead ========
 *  Function to perform a disk read from the SDCard. This function is called by
 *  the FatFs module and must not be called by the application!
 *
 *  @param  drv         Drive Number
 *
 *  @param  buf         Pointer to a buffer on which to store data
 *
 *  @param  sector      Starting sector number (LBA)
 *
 *  @param  count       Sector count (1...255)
 */
DRESULT SDSPIUSCIA_diskRead(UChar drv, UChar *buf, ULong sector,UChar count)
{
    SDSPIUSCIA_Object          *object = sdspiHandles[drv]->object;
    SDSPIUSCIA_HWAttrs const   *hwAttrs = sdspiHandles[drv]->hwAttrs;

    if (!count) {
        Log_print1(Diags_USER1, "SDSPI:(%p) disk read: 0 sectors to read",
                                 hwAttrs->baseAddr);

        return (RES_PARERR);
    }

    if (object->diskState & STA_NOINIT) {
        Log_error1("SDSPI:(%p) disk read: disk not initialized",
                    hwAttrs->baseAddr);

        return (RES_NOTRDY);
    }

    /*
     * On a SDSC card, the sector address is a byte address on the SD Card
     * On a SDHC card, the sector address is address by sector blocks
     */
    if (object->cardType != SDHC) {
        /* Convert to byte address */
        sector *= SD_SECTOR_SIZE;
    }

    /* Select the SD Card's chip select */
    takeSPIBus(hwAttrs);

    /* Single block read */
    if (count == 1) {
        if ((send_cmd(hwAttrs, CMD17, sector) == 0) &&
             rcvr_datablock(hwAttrs, buf, SD_SECTOR_SIZE)) {
            count = 0;
        }
    }
    /* Multiple block read */
    else {
        if (send_cmd(hwAttrs, CMD18, sector) == 0) {
            do {
                if (!rcvr_datablock(hwAttrs, buf, SD_SECTOR_SIZE)) {
                    break;
                }
                buf += SD_SECTOR_SIZE;
            } while (--count);

            /* STOP_TRANSMISSION */
            send_cmd(hwAttrs, CMD12, 0);
        }
    }

    /* Deselect the SD Card's chip select */
    releaseSPIBus(hwAttrs);

    /* Idle (Release DO) */
    rxSPI(hwAttrs);

    return (count ? RES_ERROR : RES_OK);
}

/*
 *  ======== SDSPIUSCIA_diskStatus ========
 *  Function to return the current disk status. This function is called by
 *  the FatFs module and must not be called by the application!
 *
 *  @param(drv)         Drive Number
 */
DSTATUS SDSPIUSCIA_diskStatus(UChar drv)
{
    /* Get the pointer to the object */
    SDSPIUSCIA_Object  *object = sdspiHandles[drv]->object;

    /* Use Diags_USER1 to reduce noise in the logs */
    Log_print2(Diags_USER2, "SDSPI:(%p) disk status: diskState: %d",
        ((SDSPIUSCIA_HWAttrs const *)(sdspiHandles[drv]->hwAttrs))->baseAddr,
        object->diskState);

    return (object->diskState);
}

#if _READONLY == 0
/*
 *  ======== SDSPIUSCIA_diskWrite ========
 *  Function to perform a disk write from the SDCard. This function is called by
 *  the FatFs module and must not be called by the application!
 *
 *  @param  drv         Drive Number
 *
 *  @param  buf         Pointer to a buffer from which to read data
 *
 *  @param  sector      Starting sector number (LBA)
 *
 *  @param  count       Sector count (1...255)
 */
DRESULT SDSPIUSCIA_diskWrite(UChar drv,
                             const UChar *buf,
                             ULong sector,
                             UChar count)
{
    SDSPIUSCIA_Object          *object = sdspiHandles[drv]->object;
    SDSPIUSCIA_HWAttrs const   *hwAttrs = sdspiHandles[drv]->hwAttrs;

    if (!count) {
        Log_print1(Diags_USER1, "SDSPI:(%p) disk write: 0 sectors to write",
                                 hwAttrs->baseAddr);

        return (RES_PARERR);
    }
    if (object->diskState & STA_NOINIT) {
        Log_error1("SDSPI:(%p) disk write: disk not initialized",
                    hwAttrs->baseAddr);

        return (RES_NOTRDY);
    }
    if (object->diskState & STA_PROTECT) {
        Log_error1("SDSPI:(%p) disk write: disk protected",
                    hwAttrs->baseAddr);

        return (RES_WRPRT);
    }

    /*
     * On a SDSC card, the sector address is a byte address on the SD Card
     * On a SDHC card, the sector address is address by sector blocks
     */
    if (object->cardType != SDHC) {
        /* Convert to byte address if needed */
        sector *= SD_SECTOR_SIZE;
    }

    /* Select the SD Card's chip select */
    takeSPIBus(hwAttrs);

    /* Single block write */
    if (count == 1) {
        if ((send_cmd(hwAttrs, CMD24, sector) == 0) &&
             xmit_datablock(hwAttrs, buf, START_BLOCK_TOKEN)) {
            count = 0;
        }
    }
    /* Multiple block write */
    else {
        if ((object->cardType == SDSC) || (object->cardType == SDHC)) {
            send_cmd(hwAttrs, CMD55, 0);
            send_cmd(hwAttrs, CMD23, count);    /* ACMD23 */
        }
        /* WRITE_MULTIPLE_BLOCK */
        if (send_cmd(hwAttrs, CMD25, sector) == 0) {
            do {
                if (!xmit_datablock(hwAttrs, buf, START_MULTIBLOCK_TOKEN)) {
                    break;
                }
                buf += SD_SECTOR_SIZE;
            } while (--count);

            /* STOP_TRAN token */
            if (!xmit_datablock(hwAttrs, 0, STOP_MULTIBLOCK_TOKEN)) {
                count = 1;
            }
        }
    }

    /* Deselect the SD Card's chip select */
    releaseSPIBus(hwAttrs);

    /* Idle (Release DO) */
    rxSPI(hwAttrs);

    return (count ? RES_ERROR : RES_OK);
}
#endif /* _READONLY */

/*
 *  ======== SDSPIUSCIA_init ========
 *  Function to initialize SDSPI module
 */
Void SDSPIUSCIA_init(SDSPI_Handle handle)
{
    SDSPIUSCIA_Object       *object = handle->object;

    /* Mark the object as available */
    object->driveNumber = DRIVE_NOT_MOUNTED;
    object->diskState = STA_NOINIT;
    object->cardType = NOCARD;
}

/*
 *  ======== SDSPIUSCIA_open ========
 *  Function to mount the FatFs filesystem and register the SDSPIUSCIA disk
 *  I/O functions with SYS/BIOS' FatFS module.
 *
 *  This function also configures some basic GPIO settings needed for the
 *  software chip select with the SDCard.
 *
 *  @param  handle      SDSPI handle
 *  @param  drv         Drive Number
 *  @param  params      SDSPI parameters
 */
SDSPI_Handle SDSPIUSCIA_open(SDSPI_Handle handle, UInt drv,
                                    SDSPI_Params *params)
{
    UInt                        key;
    DRESULT                     dresult;
    FRESULT                     fresult;
    UInt32                      clockFreq;
    Types_FreqHz                freq;
    SDSPIUSCIA_Object          *object = handle->object;
    SDSPIUSCIA_HWAttrs const   *hwAttrs = handle->hwAttrs;

    /* Determine if the device was already opened */
    key = Hwi_disable();
    if (object->driveNumber != DRIVE_NOT_MOUNTED) {
        Hwi_restore(key);
        return (NULL);
    }
    /* Mark as being used */
    object->driveNumber = drv;
    Hwi_restore(key);

    /* Store the SDSPI parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (SDSPI_Params *) &SDSPI_defaultParams;
    }

    /* Determine time scaling for ms timeouts */
    Timestamp_getFreq(&freq);
    mSScalingFactor =  freq.lo / 1000;

    /* Store desired bitRate */
    object->bitRate = params->bitRate;

    /* Pins used for SPI ( CLK | RX | TX ) */
    GPIO_setAsInputPinWithPullUpresistor(hwAttrs->portSPI, hwAttrs->pinMISO);
    GPIO_setAsPeripheralModuleFunctionInputPin(hwAttrs->portSPI,
                                               hwAttrs->pinMISO);

    GPIO_setAsPeripheralModuleFunctionOutputPin(hwAttrs->portSPI,
                                                hwAttrs->pinSCK |
                                                hwAttrs->pinMOSI);
    /* Pin used for Chip Select */
    GPIO_setAsOutputPin(hwAttrs->portCS, hwAttrs->pinCS);

    /* Raise the chip select pin */
    GPIO_setOutputHighOnPin(hwAttrs->portCS, hwAttrs->pinCS);

    /* Get the SPI clock input frequency */
    switch (hwAttrs->clockSource) {
        case USCI_A_SPI_CLOCKSOURCE_ACLK:
            clockFreq = ClockFreqs_getFrequency(ClockFreqs_Clock_ACLK);
            Log_print1(Diags_USER1, "ClockFreqs_getFrequency ACLK: %d", clockFreq);
            break;

        case USCI_A_SPI_CLOCKSOURCE_SMCLK:
            clockFreq = ClockFreqs_getFrequency(ClockFreqs_Clock_SMCLK);
            Log_print1(Diags_USER1, "ClockFreqs_getFrequency SMCLK: %d", clockFreq);
            break;

        default:
            Log_error0("SDSPI: Error determining clock source");
            SDSPIUSCIA_close(handle);
            return (NULL);
    }

    /*
     * Configure the SPI bus to 400 kHz as required per SD specs. This frequency
     * will be adjusted later once the SD card has been successfully initialized
     */
    USCI_A_SPI_masterInit(hwAttrs->baseAddr,
                        hwAttrs->clockSource,
                        clockFreq,
                        400000,
                        USCI_A_SPI_MSB_FIRST,
                        USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
                        USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW);

    Log_print2(Diags_USER1, "SDSPI:(%p) CPU freq: %d; SDSPI freq to 400000 kHz",
                             hwAttrs->baseAddr, clockFreq);

    USCI_A_SPI_enable(hwAttrs->baseAddr);

    /* Register the new disk_*() functions */
    dresult = disk_register(object->driveNumber,
                            SDSPIUSCIA_diskInitialize,
                            SDSPIUSCIA_diskStatus,
                            SDSPIUSCIA_diskRead,
                            SDSPIUSCIA_diskWrite,
                            SDSPIUSCIA_diskIOctrl);

    /* Check for drive errors */
    if (dresult != RES_OK) {
        Log_error1("SDSPI:(%p) disk functions not registered",
                    hwAttrs->baseAddr);

        SDSPIUSCIA_close(handle);
        return (NULL);
    }

    /*
     * Register the filesystem with FatFs. This operation does not access the
     * SDCard yet.
     */
    fresult = f_mount(object->driveNumber, &(object->filesystem));
    if (fresult != FR_OK) {
        Log_error2("SDSPI:(%p) drive %d not mounted",
                    hwAttrs->baseAddr, object->driveNumber);

        SDSPIUSCIA_close(handle);
        return (NULL);
    }

    /* Store the new SDSPI handle for this FatFs drive number */
    sdspiHandles[drv] = handle;

    Log_print1(Diags_USER1, "SDSPI:(%p) opened",
                             hwAttrs->baseAddr);

    return (handle);
}
