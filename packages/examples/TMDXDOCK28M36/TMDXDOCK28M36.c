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
 *  ======== TMDXDOCK28M36.c ========
 *  This file is responsible for setting up the board specific items for the
 *  TMDXDOCK28M36 board.
 *
 *  The following defines are used to determine which TI-RTOS peripheral drivers
 *  to include:
 *     TI_DRIVERS_EMAC_INCLUDED
 *     TI_DRIVERS_GPIO_INCLUDED
 *     TI_DRIVERS_I2C_INCLUDED
 *     TI_DRIVERS_SDSPI_INCLUDED
 *     TI_DRIVERS_SPI_INCLUDED
 *     TI_DRIVERS_UART_INCLUDED
 *     TI_DRIVERS_USBMSCHFATFS_INCLUDED
 *     TI_DRIVERS_WATCHDOG_INCLUDED
 *  These defines are created when a useModule is done on the driver in the
 *  application's .cfg file. The actual #define is in the application
 *  generated header file that is brought in via the xdc/cfg/global.h.
 *  For example the following in the .cfg file
 *     var GPIO = xdc.useModule('ti.drivers.GPIO');
 *  Generates the following
 *     #define TI_DRIVERS_GPIO_INCLUDED 1
 *  If there is no useModule of ti.drivers.GPIO, the constant is set to 0.
 *
 *  Note: a useModule is generated in the .cfg file via the graphical
 *  configuration tool when the "Add xxx to my configuration" is checked
 *  or "Use xxx" is selected.

 */

#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <inc/hw_sysctl.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/udma.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "TMDXDOCK28M36.h"

#pragma DATA_SECTION(TMDXDOCK28M36_DMAControlTable, ".dma");
#pragma DATA_ALIGN(TMDXDOCK28M36_DMAControlTable, 1024)
static tDMAControlTable TMDXDOCK28M36_DMAControlTable[32];
static Bool DMA_initialized = FALSE;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct hwiStruct;
/*
 *  ======== TMDXDOCK28M36_errorDMAHwi ========
 */
static Void TMDXDOCK28M36_errorDMAHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== TMDXDOCK28M36_initDMA ========
 */
Void TMDXDOCK28M36_initDMA(Void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if(!DMA_initialized){

        Error_init(&eb);

        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(hwiStruct), INT_UDMAERR, TMDXDOCK28M36_errorDMAHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't create DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(TMDXDOCK28M36_DMAControlTable);

        DMA_initialized = TRUE;
    }
}

#if TI_DRIVERS_EMAC_INCLUDED
#include <ti/drivers/EMAC.h>
#include <ti/drivers/emac/EMACTiva.h>

/* EMAC objects */
EMACTiva_Object emacObjects[TMDXDOCK28M36_EMACCOUNT];

/*
 *  EMAC configuration structure
 *  Set user/company specific MAC octates. The following sets the address
 *  to ff-ff-ff-ff-ff-ff. Users need to change this to make the label on
 *  their boards.
 */
UInt8 macAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

const EMACTiva_HWAttrs emacHWAttrs[TMDXDOCK28M36_EMACCOUNT] = {
    {INT_ETH, macAddress}
};

const EMAC_Config EMAC_config[] = {
    {&EMACTiva_fxnTable, emacObjects, emacHWAttrs},
    {NULL, NULL, NULL}
};

/* Required by the NDK. This array must be NULL terminated. */
NIMU_DEVICE_TABLE_ENTRY  NIMUDeviceTable[2] = {EMACTiva_NIMUInit, NULL};

/*
 *  ======== TMDXDOCK28M36_initEMAC ========
 */
Void TMDXDOCK28M36_initEMAC(Void)
{
    /*
     *  Set up the pins that are used for Ethernet
     *  MII_TXD3
     */
    GPIODirModeSet(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTM_BASE + GPIO_O_APSEL)|= 0x00000002;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) &= 0xFFFFFF0F;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) |= 0x000000C0;

    /* MII_MDIO */
    GPIODirModeSet(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTM_BASE + GPIO_O_APSEL)|= 0x00000001;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) &= 0xFFFFFFF0;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) |= 0x0000000C;

    /* MII_RXD3 */
    GPIODirModeSet(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTL_BASE + GPIO_O_APSEL)|= 0x00000001;
    HWREG(GPIO_PORTL_BASE + GPIO_O_PCTL) &= 0xFFFFFFF0;
    HWREG(GPIO_PORTL_BASE + GPIO_O_PCTL) |= 0x0000000C;

    /* MMI_TXEN, MMI_TXCK, MMI_TXER, MMI_CRS */
    GPIODirModeSet(GPIO_PORTK_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
                   GPIO_PIN_4, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
                     GPIO_PIN_4, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTK_BASE + GPIO_O_APSEL)|= 0x000000F0;
    HWREG(GPIO_PORTK_BASE + GPIO_O_PCTL) &= 0x0000FFFF;
    HWREG(GPIO_PORTK_BASE + GPIO_O_PCTL) |= 0xCCCC0000;

    /* MMI_RXD2, MMI_RXD1, MMI_RXD0, MMI_COL, MMI_PHYRSTn, MMI_PHYINTRn,
       MMI_MDC */
    GPIODirModeSet(GPIO_PORTL_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
                   GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1,
                   GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTL_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
                   GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1,
                     GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTL_BASE + GPIO_O_APSEL)|= 0x000000FE;
    HWREG(GPIO_PORTL_BASE + GPIO_O_PCTL) &= 0x0000000F;
    HWREG(GPIO_PORTL_BASE + GPIO_O_PCTL) |= 0xCCCCCCC0;

    /* MMI_TXD2, MMI_TXD1, MMI_TXD0, MMI_RXDV, MMI_RXER, MMI_RXCK */
    GPIODirModeSet(GPIO_PORTM_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
                   GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2,
                   GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|
                   GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2,
                     GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTM_BASE + GPIO_O_APSEL)|= 0x000000FC;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) &= 0x000000FF;
    HWREG(GPIO_PORTM_BASE + GPIO_O_PCTL) |= 0xCCCCCC00;

    /* Enable and Reset the Ethernet Controller. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ETH);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ETH);

    if (macAddress[0] == 0xff && macAddress[1] == 0xff &&
        macAddress[2] == 0xff && macAddress[3] == 0xff &&
        macAddress[4] == 0xff && macAddress[5] == 0xff) {
        System_abort("Change the macAddress variable to match your board's MAC sticker");
    }

    /* Once EMAC_init is called, EMAC_config cannot be changed */
    EMAC_init();
}
#endif /* TI_DRIVERS_EMAC_INCLUDED */

/*
 *  ======== TMDXDOCK28M36_initGeneral ========
 */
Void TMDXDOCK28M36_initGeneral(Void)
{
    /* Disable Protection */
    HWREG(SYSCTL_MWRALLOW) =  0xA5A5A5A5;

    /* Enable clock supply for the following peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);

    /* Disable clock supply for the watchdog modules */
    SysCtlPeripheralDisable(SYSCTL_PERIPH_WDOG1);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_WDOG0);
}

#if TI_DRIVERS_GPIO_INCLUDED
#include <ti/drivers/GPIO.h>

/* Callback function for the GPIO interrupt example. */
Void gpioButtonFxn0(Void);

/* GPIO configuration structure */
const GPIO_HWAttrs gpioHWAttrs[TMDXDOCK28M36_GPIOCOUNT] = {
    {GPIO_PORTE_BASE, GPIO_PIN_7, GPIO_OUTPUT},  /* TMDXDOCK28M36_D1 */
    {GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_OUTPUT},  /* TMDXDOCK28M36_D2 */
    {GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_INPUT},   /* TMDXDOCK28M36_BUTTON */
};

/* Memory for the GPIO module to construct a Hwi */
Hwi_Struct callbackHwi;

/* GPIO callback structure to set callbacks for GPIO interrupts */
const GPIO_Callbacks TMDXDOCK28M36_gpioPortBCallbacks = {
    GPIO_PORTB_BASE, INT_GPIOB, &callbackHwi,
    {NULL, NULL, NULL, NULL, gpioButtonFxn0, NULL, NULL, NULL}
};

const GPIO_Config GPIO_config[] = {
    {&gpioHWAttrs[0]},
    {&gpioHWAttrs[1]},
    {&gpioHWAttrs[2]},
    {NULL},
};

/*
 *  ======== TMDXDOCK28M36_initGPIO ========
 */
Void TMDXDOCK28M36_initGPIO(Void)
{
    /* Setup the LED GPIO pins used */
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_7);  /* TMDXDOCK28M36_D1 */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);  /* TMDXDOCK28M36_D2 */

    /* Setup the button GPIO pins used */
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_4);   /* TMDXDOCK28M36_BUTTON */
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_TYPE_STD_WPU);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();

    GPIO_write(TMDXDOCK28M36_D1, TMDXDOCK28M36_LED_OFF);
    GPIO_write(TMDXDOCK28M36_D2, TMDXDOCK28M36_LED_OFF);
}
#endif /* TI_DRIVERS_GPIO_INCLUDED */

#if TI_DRIVERS_I2C_INCLUDED
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

/* I2C objects */
I2CTiva_Object i2cTivaObjects[TMDXDOCK28M36_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const I2CTiva_HWAttrs i2cTivaHWAttrs[TMDXDOCK28M36_I2CCOUNT] = {
    {I2C0_MASTER_BASE, INT_I2C0}
};

const I2C_Config I2C_config[] = {
    {&I2CTiva_fxnTable, &i2cTivaObjects[0], &i2cTivaHWAttrs[0]},
    {NULL, NULL, NULL}
};

/*
 *  ======== TMDXDOCK28M36_initI2C ========
 */
Void TMDXDOCK28M36_initI2C(Void)
{
    /* I2C0 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PF0_I2C0SDA); /* GPIO32--85 on dock*/
    GPIOPinConfigure(GPIO_PF1_I2C0SCL); /* GPIO33--87 on dock */
    GPIOPinTypeI2C(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    I2C_init();
}
#endif /* TI_DRIVERS_I2C_INCLUDED */

#if TI_DRIVERS_SDSPI_INCLUDED
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPITiva.h>

/* SDSPI objects */
SDSPITiva_Object sdspiTivaobjects[TMDXDOCK28M36_SDSPICOUNT];

/* SDSPI configuration structure, describing which pins are to be used */
const SDSPITiva_HWAttrs sdspiTivaHWattrs[TMDXDOCK28M36_SDSPICOUNT] = {
    {
        SSI3_BASE,          /* SPI base address */

        GPIO_PORTR_BASE,    /* The GPIO port used for the SPI pins */
        GPIO_PIN_2,         /* SCK */
        GPIO_PIN_1,         /* MISO */
        GPIO_PIN_0,         /* MOSI */

        GPIO_PORTR_BASE,    /* Chip select port */
        GPIO_PIN_3,         /* Chip select pin */

        GPIO_PORTR_BASE,    /* GPIO TX port */
        GPIO_PIN_0,         /* GPIO TX pin */
    }
};

const SDSPI_Config SDSPI_config[] = {
    {&SDSPITiva_fxnTable, &sdspiTivaobjects[0], &sdspiTivaHWattrs[0]},
    {NULL, NULL, NULL}
};

/*
 *  ======== TMDXDOCK28M36_initSDSPI ========
 */
Void TMDXDOCK28M36_initSDSPI(Void)
{
    /* Enable the peripherals used by the SD Card */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    /* Configure SPI Peripheral pin muxing */
    GPIOPinConfigure(GPIO_PR2_SSI3CLK);
    GPIOPinConfigure(GPIO_PR0_SSI3TX);
    GPIOPinConfigure(GPIO_PR1_SSI3RX);

    /* Configure pad settings */
    GPIOPadConfigSet(GPIO_PORTR_BASE,
            GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0,
            GPIO_PIN_TYPE_STD_WPU);

    GPIOPadConfigSet(GPIO_PORTR_BASE, GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);

    SDSPI_init();
}
#endif /* TI_DRIVERS_SDSPI_INCLUDED */

#if TI_DRIVERS_SPI_INCLUDED
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

/* SPI objects */
SPITivaDMA_Object spiTivaDMAobjects[TMDXDOCK28M36_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[TMDXDOCK28M36_SPICOUNT] = {
    {
        SSI0_BASE,
        INT_SSI0,
        UDMA_CHANNEL_SSI0RX,
        UDMA_CHANNEL_SSI0TX,
        uDMAChannel8_15SelectDefault,
        UDMA_CHAN10_DEF_SSI0RX_M,
        UDMA_CHAN11_DEF_SSI0TX_M
    },
    {
        SSI1_BASE,
        INT_SSI1,
        UDMA_CHANNEL_SSI1RX,
        UDMA_CHANNEL_SSI1TX,
        uDMAChannel24_31SelectDefault,
        UDMA_CHAN24_DEF_SSI1RX_M,
        UDMA_CHAN25_DEF_SSI1TX_M
    },
    {
        SSI2_BASE,
        INT_SSI2,
        UDMA_THRD_CHANNEL_SSI2RX,
        UDMA_THRD_CHANNEL_SSI2TX,
        uDMAChannel8_15SelectAltMapping,
        UDMA_CHAN12_THRD_SSI2RX,
        UDMA_CHAN13_THRD_SSI2TX
    },
    {
        SSI3_BASE,
        INT_SSI3,
        UDMA_THRD_CHANNEL_SSI3RX,
        UDMA_THRD_CHANNEL_SSI3TX,
        uDMAChannel8_15SelectAltMapping,
        UDMA_CHAN14_THRD_SSI3RX,
        UDMA_CHAN15_THRD_SSI3TX
    }
};

const SPI_Config SPI_config[] = {
    {&SPITivaDMA_fxnTable, &spiTivaDMAobjects[0], &spiTivaDMAHWAttrs[0]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAobjects[1], &spiTivaDMAHWAttrs[1]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAobjects[2], &spiTivaDMAHWAttrs[2]},
    {&SPITivaDMA_fxnTable, &spiTivaDMAobjects[3], &spiTivaDMAHWAttrs[3]},
    {NULL, NULL, NULL},
};

/*
 *  ======== TMDXDOCK28M36_initSPI ========
 */
Void TMDXDOCK28M36_initSPI(Void)
{
    /* SSI0 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PD2_SSI0CLK);
    GPIOPinConfigure(GPIO_PD3_SSI0FSS);
    GPIOPinConfigure(GPIO_PD1_SSI0RX);
    GPIOPinConfigure(GPIO_PD0_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                    GPIO_PIN_2 | GPIO_PIN_3);

    /* SSI1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    GPIOPinConfigure(GPIO_PE0_SSI1CLK);
    GPIOPinConfigure(GPIO_PE1_SSI1FSS);
    GPIOPinConfigure(GPIO_PE2_SSI1RX);
    GPIOPinConfigure(GPIO_PE3_SSI1TX);

    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                    GPIO_PIN_2 | GPIO_PIN_3);
    TMDXDOCK28M36_initDMA();
    SPI_init();
}
#endif /* TI_DRIVERS_SPI_INCLUDED */

#if TI_DRIVERS_UART_INCLUDED
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTTiva.h>

/* UART objects */
UARTTiva_Object uartTivaObjects[TMDXDOCK28M36_UARTCOUNT];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[TMDXDOCK28M36_UARTCOUNT] = {
    {UART0_BASE, INT_UART0}, /* TMDXDOCK28M36_UART0 */
};

const UART_Config UART_config[] = {
    {
        &UARTTiva_fxnTable,
        &uartTivaObjects[0],
        &uartTivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== TMDXDOCK28M36_initUART ========
 */
Void TMDXDOCK28M36_initUART()
{
    /* Enable and configure the peripherals used by the uart. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PE4_U0RX);
    GPIOPinConfigure(GPIO_PE5_U0TX);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_TYPE_STD_WPU);

    /* Initialize the UART driver */
    UART_init();
}
#endif /* TI_DRIVERS_UART_INCLUDED */

/*
 *  ======== TMDXDOCK28M36_initUSB ========
 *  This function just turns on the USB
 */
Void TMDXDOCK28M36_initUSB(TMDXDOCK28M36_USBMode usbMode)
{
    /*
     * Setup USB clock tree for 60MHz
     * 20MHz * 12 / 4 = 60
     */
    SysCtlUSBPLLConfigSet(
            (SYSCTL_UPLLIMULT_M & 12) | SYSCTL_UPLLCLKSRC_X1 | SYSCTL_UPLLEN);

    /* Setup pins for USB operation */
    GPIOPinTypeUSBAnalog(GPIO_PORTF_BASE, GPIO_PIN_6);
    GPIOPinTypeUSBAnalog(
            GPIO_PORTG_BASE, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

    /* Additional configurations for Host mode */
    if (usbMode == TMDXDOCK28M36_USBHOST) {
        /* Configure the pins needed */
        GPIOPinConfigure(GPIO_PN6_USB0EPEN);
        GPIOPinConfigure(GPIO_PN7_USB0PFLT);
        GPIOPinTypeUSBDigital(GPIO_PORTN_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    }
}

#if TI_DRIVERS_USBMSCHFATFS_INCLUDED
#include <ti/drivers/USBMSCHFatFs.h>
#include <ti/drivers/usbmschfatfs/USBMSCHFatFsTiva.h>

/* USBMSCHFatFs objects */
USBMSCHFatFsTiva_Object usbmschfatfstivaObjects[TMDXDOCK28M36_USBMSCHFatFsCOUNT];

/* USBMSCHFatFs configuration structure, describing which pins are to be used */
const USBMSCHFatFsTiva_HWAttrs usbmschfatfstivaHWAttrs[TMDXDOCK28M36_USBMSCHFatFsCOUNT] = {
    {INT_USB0}
};

const USBMSCHFatFs_Config USBMSCHFatFs_config[] = {
    {
        &USBMSCHFatFsTiva_fxnTable,
        &usbmschfatfstivaObjects[0],
        &usbmschfatfstivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== TMDXDOCK28M36_initUSBMSCHFatFs ========
 */
Void TMDXDOCK28M36_initUSBMSCHFatFs(Void)
{
    /* Call the USB initialization function for the USB Reference modules */
    TMDXDOCK28M36_initUSB(TMDXDOCK28M36_USBHOST);
    USBMSCHFatFs_init();
}
#endif /* TI_DRIVERS_USBMSCHFATFS_INCLUDED */

#if TI_DRIVERS_WATCHDOG_INCLUDED
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

/* Watchdog objects */
WatchdogTiva_Object watchdogTivaObjects[TMDXDOCK28M36_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[TMDXDOCK28M36_WATCHDOGCOUNT] = {
    /* TMDXDOCK28M36_WATCHDOG0 with 1 sec period at default CPU clock freq */
    {WATCHDOG0_BASE, INT_WATCHDOG, 75000000},
};

const Watchdog_Config Watchdog_config[] = {
    {&WatchdogTiva_fxnTable, &watchdogTivaObjects[0], &watchdogTivaHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== TMDXDOCK28M36_initWatchdog ========
 */
Void TMDXDOCK28M36_initWatchdog()
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}
#endif /* TI_DRIVERS_WATCHDOG_INCLUDED */
