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
 *  ======== cc3000patcher.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <string.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/WiFi.h>

/* SimpleLink Wi-Fi Host Driver Header files */
#include <cc3000_host_driver/include/common/cc3000_common.h>
#include <cc3000_host_driver/include/nvmem.h>
#include <cc3000_host_driver/include/wlan.h>

/* Example/Board Header files */
#include "Board.h"
#include "cc3000patcharrays.h"

#define PACKAGEID           1
#define PACKAGEBLDNUM       24

#define RM_PARAM_LENGTH     128
#define FAT_FILEID          16

/*
 *  ======== fatWriteContent ========
 *  This function is a slightly modified version of one taken from the
 *  LM4F232+CC3000 Patch Programmer v1.10.2 offered on the SimpleLink Wi-Fi
 *  CC3000 Wiki download page:
 *  http://processors.wiki.ti.com/index.php/CC3000_Wi-Fi_Downloads#CC3000.2BStellaris_Cortex-M4
 *
 *  fileAddress         array of file address in FAT table:
 *                      this is the absolute address of the file in the EEPROM.
 *  fileLength          array of file length in FAT table:
 *                      this is the upper limit of the file size in the EEPROM.
 *
 *  Return on success 0, error otherwise.
 */
UChar fatWriteContent(UShort const *fileAddress,
                      UShort const *fileLength)
{
    UShort  index = 0;
    UChar   status;
    UChar   fatTable[48];
    UChar*  fatTablePtr = fatTable;

    /* First, write the magic number */
    status = nvmem_write(FAT_FILEID, 2, 0, (UChar *)"LS");

    for (; index <= NVMEM_RM_FILEID; index++) {
        /* Write address low char and mark as allocated */
        *fatTablePtr++ = (UChar)(fileAddress[index] & 0xff) | 0x01;

        /* Write address high char */
        *fatTablePtr++ = (UChar)((fileAddress[index]>>8) & 0xff);

        /* Write length low char */
        *fatTablePtr++ = (UChar)(fileLength[index] & 0xff);

        /* Write length high char */
        *fatTablePtr++ = (UChar)((fileLength[index]>>8) & 0xff);
    }

    /* Second, write the FAT */
    status = nvmem_write(FAT_FILEID, 48, 4, fatTable);

    /* Third, erase any user files */
    memset(fatTable, 0, sizeof(fatTable));
    status = nvmem_write(FAT_FILEID, 16, 52, fatTable);

    return (status);
}

/*
 *  ======== patchFxn ========
 *  Runs the patching routine.
 *
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void patchFxn(UArg arg0, UArg arg1)
{
    UChar      *rmParams;
    UChar       drStatus;
    UChar       fwStatus;
    UChar       retStatus;
    UChar       counter = 0;
    UChar       spNum[2] = {0};
    UChar       macFromEEPROM[MAC_ADDR_LEN];
    Char        macStatus;
    WiFi_Params params;
    WiFi_Handle handle;

    /* Turn LED off. It will be used as a patching status indicator */
    GPIO_write(Board_LED0, Board_LED_OFF);

    /* Open WiFi */
    WiFi_Params_init(&params);
    params.bitRate = 1000000;
    handle = WiFi_open(Board_WIFI, Board_SPI_CC3000, NULL, &params);

    /* Check service pack version */
    nvmem_read_sp_version(spNum);
    if ((spNum[0] == PACKAGEID) && (spNum[1] == PACKAGEBLDNUM)) {
        System_printf("You are using service pack version %d.%d already. No "
                      "need to patch the CC3000.\n\n", spNum[0], spNum[1]);

        GPIO_write(Board_LED0, Board_LED_ON);
        BIOS_exit(0);
    }

    /* Restart CC3000 for patching */
    wlan_stop();
    Task_sleep(500);
    wlan_start(1);

    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE
                      | HCI_EVNT_WLAN_UNSOL_INIT
                      | HCI_EVNT_WLAN_ASYNC_PING_REPORT
                      | HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE
                      | HCI_EVNT_WLAN_UNSOL_CONNECT
                      | HCI_EVNT_WLAN_UNSOL_DISCONNECT);

    GPIO_write(Board_LED0, Board_LED_ON);
    Task_sleep(1000);
    GPIO_write(Board_LED0, Board_LED_OFF);

    /* Save MAC address so we can restore it at the end */
    macStatus = nvmem_get_mac_address(macFromEEPROM);

    do {
        /* Read RM parameters */
        retStatus = nvmem_read(NVMEM_RM_FILEID, RM_PARAM_LENGTH, 0,
                               cRMParamsFromEeprom);
        counter++;
    } while ((retStatus) && (counter < 3));

    /* If RM file is not valid, load the default one */
    if (counter == 3) {
        rmParams = (UChar *)cRMdefaultParams;
    }
    else {
        rmParams = cRMParamsFromEeprom;
    }

    do {
        /* Write new FAT */
        retStatus = fatWriteContent(aFATEntries[0], aFATEntries[1]);
    } while (retStatus);

    wlan_stop();
    Task_sleep(500);
    wlan_start(1);
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE
                      | HCI_EVNT_WLAN_UNSOL_INIT
                      | HCI_EVNT_WLAN_ASYNC_PING_REPORT
                      | HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE
                      | HCI_EVNT_WLAN_UNSOL_CONNECT
                      | HCI_EVNT_WLAN_UNSOL_DISCONNECT);

    GPIO_write(Board_LED0, Board_LED_ON);
    Task_sleep(1000);
    GPIO_write(Board_LED0, Board_LED_OFF);

    do {
        /* Write RM parameters */
        retStatus = nvmem_write(NVMEM_RM_FILEID, RM_PARAM_LENGTH, 0, rmParams);
    } while (retStatus);

    /* Write back the MAC address if it exists */
    if (macStatus == 0) {
        /* Zero out MCAST bit if set */
        macFromEEPROM[0] &= 0xfe;
        do {
            retStatus = nvmem_set_mac_address(macFromEEPROM);
        } while (retStatus);
    }

    do {
        /* Writing driver patch to EEPROM */
        drStatus = nvmem_write_patch(NVMEM_WLAN_DRIVER_SP_FILEID, drv_length,
                                     wlan_drv_patch);
    } while (drStatus);

    do {
        /* Writing FW patch to EEPROM */
        fwStatus = nvmem_write_patch(NVMEM_WLAN_FW_SP_FILEID, fw_length,
                                     fw_patch);
    } while (fwStatus);

    System_printf("Patching process complete!\n\n");
    GPIO_write(Board_LED0, Board_LED_ON);

    WiFi_close(handle);
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initGPIO();
    Board_initWiFi();

    System_printf("Starting the SimpleLink Wi-Fi CC3000 Patcher example\n");

    /* Start BIOS */
    BIOS_start();

    return (0);
}
