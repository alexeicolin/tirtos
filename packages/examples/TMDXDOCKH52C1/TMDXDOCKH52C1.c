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
 *  ======== TMDXDOCKH52C1.c ========
 *  This file is responsible for setting up the board specific items for the
 *  TMDXDOCKH52C1 board.
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

#include "TMDXDOCKH52C1.h"

#pragma DATA_SECTION(TMDXDOCKH52C1_DMAControlTable, ".dma");
#pragma DATA_ALIGN(TMDXDOCKH52C1_DMAControlTable, 1024)
static tDMAControlTable TMDXDOCKH52C1_DMAControlTable[32];
static Bool DMA_initialized = FALSE;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct hwiStruct;
/*
 *  ======== TMDXDOCKH52C1_errorDMAHwi ========
 */
static Void TMDXDOCKH52C1_errorDMAHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== TMDXDOCKH52C1_initDMA ========
 */
Void TMDXDOCKH52C1_initDMA(Void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if(!DMA_initialized){

        Error_init(&eb);

        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(hwiStruct), INT_UDMAERR, TMDXDOCKH52C1_errorDMAHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't create DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(TMDXDOCKH52C1_DMAControlTable);

        DMA_initialized = TRUE;
    }
}

#if TI_DRIVERS_EMAC_INCLUDED
#include <ti/drivers/EMAC.h>
#include <ti/drivers/emac/EMACTiva.h>

/* EMAC objects */
EMACTiva_Object emacObjects[TMDXDOCKH52C1_EMACCOUNT];

/*
 *  EMAC configuration structure
 *  Set user/company specific MAC octates. The following sets the address
 *  to ff-ff-ff-ff-ff-ff. Users need to change this to make the label on
 *  their boards.
 */
UInt8 macAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

const EMACTiva_HWAttrs emacHWAttrs[TMDXDOCKH52C1_EMACCOUNT] = {
    {INT_ETH, macAddress}
};

const EMAC_Config EMAC_config[] = {
    {&EMACTiva_fxnTable, emacObjects, emacHWAttrs},
    {NULL, NULL, NULL}
};

/* Required by the NDK. This array must be NULL terminated. */
NIMU_DEVICE_TABLE_ENTRY  NIMUDeviceTable[2] = {EMACTiva_NIMUInit, NULL};

/*
 *  ======== TMDXDOCKH52C1_initEMAC ========
 */
Void TMDXDOCKH52C1_initEMAC(Void)
{
    /*
     *  Set up the pins that are used for Ethernet
     *  MII_TXD3
     */
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) &= 0xFFF0FFFF;
    HWREG(GPIO_PORTC_BASE + GPIO_O_PCTL) |= 0x00030000;

    /* MII_MDIO */
    GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_6, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_6, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTE_BASE + GPIO_O_APSEL)|= 0x00000040;
    HWREG(GPIO_PORTE_BASE + GPIO_O_PCTL) &= 0xF0FFFFFF;
    HWREG(GPIO_PORTE_BASE + GPIO_O_PCTL) |= 0x0C000000;

    /* MII_RXD3 */
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_5, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_5, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) &= 0xFF0FFFFF;
    HWREG(GPIO_PORTF_BASE + GPIO_O_PCTL) |= 0x00300000;

    /* MII_TXER , MII_RXDV , MII_RXD1 , MII_RXD2 */
    GPIODirModeSet(GPIO_PORTG_BASE, GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_1|GPIO_PIN_0,
                   GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_1|
                     GPIO_PIN_0,
                     GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTG_BASE + GPIO_O_APSEL)|= 0x0000000B;
    HWREG(GPIO_PORTG_BASE + GPIO_O_PCTL) &= 0x0FFF0F00;
    HWREG(GPIO_PORTG_BASE + GPIO_O_PCTL) |= 0x3000C0CC;

    /* MII_TXCK , MII_TXEN , MII_TXD0 , MII_TXD1 , MII_TXD2 , MII_RXD0 */
    GPIODirModeSet(
        GPIO_PORTH_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|
        GPIO_PIN_3|
        GPIO_PIN_1, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(
        GPIO_PORTH_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|
        GPIO_PIN_3|
        GPIO_PIN_1, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTH_BASE + GPIO_O_APSEL)|= 0x000000C2;
    HWREG(GPIO_PORTH_BASE + GPIO_O_PCTL) &= 0x00000F0F;
    HWREG(GPIO_PORTH_BASE + GPIO_O_PCTL) |= 0xCC9990C0;

    /*
     *  MII_PHYRSTn , MII_PHYINTRn , MII_CRS , MII_COL , MII_MDC , MII_RXCK ,
     *  MII_RXER
     */
    GPIODirModeSet(
        GPIO_PORTJ_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|
        GPIO_PIN_3|
        GPIO_PIN_2|GPIO_PIN_0, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(
        GPIO_PORTJ_BASE, GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|
        GPIO_PIN_3|
        GPIO_PIN_2|GPIO_PIN_0, GPIO_PIN_TYPE_STD);
    HWREG(GPIO_PORTJ_BASE + GPIO_O_APSEL)|= 0x000000FC;
    HWREG(GPIO_PORTJ_BASE + GPIO_O_PCTL) &= 0x000000F0;
    HWREG(GPIO_PORTJ_BASE + GPIO_O_PCTL) |= 0xCCCCCC03;

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
 *  ======== TMDXDOCKH52C1_initGeneral ========
 */
Void TMDXDOCKH52C1_initGeneral(Void)
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

    /* Disable clock supply for the watchdog modules */
    SysCtlPeripheralDisable(SYSCTL_PERIPH_WDOG1);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_WDOG0);
}

#if TI_DRIVERS_GPIO_INCLUDED
#include <ti/drivers/GPIO.h>

/* Callback function for the GPIO interrupt example. */
Void gpioButtonFxn0(Void);

/* GPIO configuration structure */
const GPIO_HWAttrs gpioHWAttrs[TMDXDOCKH52C1_GPIOCOUNT] = {
    {GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_OUTPUT},  /* TMDXDOCKH52C1_LD2 */
    {GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_OUTPUT},  /* TMDXDOCKH52C1_LD3 */
    {GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_INPUT},   /* TMDXDOCKH52C1_BUTTON */
};

/* Memory for the GPIO module to construct a Hwi */
Hwi_Struct callbackHwi;

/* GPIO callback structure to set callbacks for GPIO interrupts */
const GPIO_Callbacks TMDXDOCKH52C1_gpioPortBCallbacks = {
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
 *  ======== TMDXDOCKH52C1_initGPIO ========
 */
Void TMDXDOCKH52C1_initGPIO(Void)
{
    /* Setup the LED GPIO pins used */
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);  /* TMDXDOCKH52C1_LD2 */
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);  /* TMDXDOCKH52C1_LD3 */

    /* Setup the button GPIO pins used */
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_4);   /* TMDXDOCKH52C1_BUTTON */
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_TYPE_STD_WPU);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();

    GPIO_write(TMDXDOCKH52C1_LD2, TMDXDOCKH52C1_LED_OFF);
    GPIO_write(TMDXDOCKH52C1_LD3, TMDXDOCKH52C1_LED_OFF);
}
#endif /* TI_DRIVERS_GPIO_INCLUDED */

#if TI_DRIVERS_I2C_INCLUDED
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

/* I2C objects */
I2CTiva_Object i2cTivaObjects[TMDXDOCKH52C1_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const I2CTiva_HWAttrs i2cTivaHWAttrs[TMDXDOCKH52C1_I2CCOUNT] = {
    {I2C0_MASTER_BASE, INT_I2C0},
    {I2C1_MASTER_BASE, INT_I2C1}
};

const I2C_Config I2C_config[] = {
    {&I2CTiva_fxnTable, &i2cTivaObjects[0], &i2cTivaHWAttrs[0]},
    {&I2CTiva_fxnTable, &i2cTivaObjects[1], &i2cTivaHWAttrs[1]},
    {NULL, NULL, NULL}
};

/*
 *  ======== TMDXDOCKH52C1_initI2C ========
 */
Void TMDXDOCKH52C1_initI2C(Void)
{
    /* I2C0 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinUnlock(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB7_I2C0SCL); /* GPIO15 on Concerto base board */
    GPIOPinConfigure(GPIO_PB6_I2C0SDA); /* GPIO14 on Concerto base board */
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /* I2C1 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PA0_I2C1SCL); /* GPIO00 on Concerto base board */
    GPIOPinConfigure(GPIO_PA1_I2C1SDA); /* GPIO01 on Concerto base board */
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    I2C_init();
}
#endif /* TI_DRIVERS_I2C_INCLUDED */

#if TI_DRIVERS_SDSPI_INCLUDED
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPITiva.h>

/* SDSPI objects */
SDSPITiva_Object sdspiTivaobjects[TMDXDOCKH52C1_SDSPICOUNT];

/* SDSPI configuration structure, describing which pins are to be used */
const SDSPITiva_HWAttrs sdspiTivaHWattrs[TMDXDOCKH52C1_SDSPICOUNT] = {
    {
        SSI0_BASE,          /* SPI base address */

        GPIO_PORTD_BASE,    /* The GPIO port used for the SPI pins */
        GPIO_PIN_2,         /* SCK */
        GPIO_PIN_1,         /* MISO */
        GPIO_PIN_0,         /* MOSI */

        GPIO_PORTD_BASE,    /* Chip select port */
        GPIO_PIN_3,         /* Chip select pin */

        GPIO_PORTA_BASE,    /* GPIO TX port */
        GPIO_PIN_5,         /* GPIO TX pin */
    }
};

const SDSPI_Config SDSPI_config[] = {
    {&SDSPITiva_fxnTable, &sdspiTivaobjects[0], &sdspiTivaHWattrs[0]},
    {NULL, NULL, NULL}
};

/*
 *  ======== TMDXDOCKH52C1_initSDSPI ========
 */
Void TMDXDOCKH52C1_initSDSPI(Void)
{
    /* Enable the peripherals used by the SD Card */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    /* Configure SSI Peripheral pin muxing */
    GPIOPinConfigure(GPIO_PD2_SSI0CLK);
    GPIOPinConfigure(GPIO_PD0_SSI0TX);
    GPIOPinConfigure(GPIO_PD1_SSI0RX);

    /* Configure pad settings */
    GPIOPadConfigSet(GPIO_PORTD_BASE,
            GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0,
            GPIO_PIN_TYPE_STD_WPU);

    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_TYPE_STD_WPU);

    SDSPI_init();
}
#endif /* TI_DRIVERS_SDSPI_INCLUDED */

#if TI_DRIVERS_SPI_INCLUDED
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

/* SPI objects */
SPITivaDMA_Object spiTivaDMAobjects[TMDXDOCKH52C1_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[TMDXDOCKH52C1_SPICOUNT] = {
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
 *  ======== TMDXDOCKH52C1_initSPI ========
 */
Void TMDXDOCKH52C1_initSPI(Void)
{
    /* SSI0 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 |
                                    GPIO_PIN_4 | GPIO_PIN_5);

    /* SSI1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    GPIOPinConfigure(GPIO_PE0_SSI1CLK);
    GPIOPinConfigure(GPIO_PE1_SSI1FSS);
    GPIOPinConfigure(GPIO_PE2_SSI1RX);
    GPIOPinConfigure(GPIO_PE3_SSI1TX);

    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                    GPIO_PIN_2 | GPIO_PIN_3);
    TMDXDOCKH52C1_initDMA();
    SPI_init();
}
#endif /* TI_DRIVERS_SPI_INCLUDED */

#if TI_DRIVERS_UART_INCLUDED
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTTiva.h>

/* UART objects */
UARTTiva_Object uartTivaObjects[TMDXDOCKH52C1_UARTCOUNT];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[TMDXDOCKH52C1_UARTCOUNT] = {
    {UART0_BASE, INT_UART0}, /* TMDXDOCKH52C1_UART0 */
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
 *  ======== TMDXDOCKH52C1_initUART ========
 */
Void TMDXDOCKH52C1_initUART()
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
 *  ======== TMDXDOCKH52C1_initUSB ========
 *  This function just turns on the USB
 */
Void TMDXDOCKH52C1_initUSB(TMDXDOCKH52C1_USBMode usbMode)
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
    if (usbMode == TMDXDOCKH52C1_USBHOST) {
        /* Configure the pins needed */
        GPIOPinConfigure(GPIO_PC5_USB0EPEN);
        GPIOPinConfigure(GPIO_PJ1_USB0PFLT);
        GPIOPinTypeUSBDigital(GPIO_PORTC_BASE, GPIO_PIN_5);
        GPIOPinTypeUSBDigital(GPIO_PORTJ_BASE, GPIO_PIN_1);
    }
}

#if TI_DRIVERS_USBMSCHFATFS_INCLUDED
#include <ti/drivers/USBMSCHFatFs.h>
#include <ti/drivers/usbmschfatfs/USBMSCHFatFsTiva.h>

/* USBMSCHFatFs objects */
USBMSCHFatFsTiva_Object usbmschfatfstivaObjects[TMDXDOCKH52C1_USBMSCHFatFsCOUNT];

/* USBMSCHFatFs configuration structure, describing which pins are to be used */
const USBMSCHFatFsTiva_HWAttrs usbmschfatfstivaHWAttrs[TMDXDOCKH52C1_USBMSCHFatFsCOUNT] = {
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
 *  ======== TMDXDOCKH52C1_initUSBMSCHFatFs ========
 */
Void TMDXDOCKH52C1_initUSBMSCHFatFs(Void)
{
    /* Call the USB initialization function for the USB Reference modules */
    TMDXDOCKH52C1_initUSB(TMDXDOCKH52C1_USBHOST);
    USBMSCHFatFs_init();
}
#endif /* TI_DRIVERS_USBMSCHFATFS_INCLUDED */

#if TI_DRIVERS_WATCHDOG_INCLUDED
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

/* Watchdog objects */
WatchdogTiva_Object watchdogTivaObjects[TMDXDOCKH52C1_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[TMDXDOCKH52C1_WATCHDOGCOUNT] = {
    /* TMDXDOCKH52C1_WATCHDOG0 with 1 sec period at default CPU clock freq */
    {WATCHDOG0_BASE, INT_WATCHDOG, 75000000},
};

const Watchdog_Config Watchdog_config[] = {
    {&WatchdogTiva_fxnTable, &watchdogTivaObjects[0], &watchdogTivaHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== TMDXDOCKH52C1_initWatchdog ========
 */
Void TMDXDOCKH52C1_initWatchdog()
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}
#endif /* TI_DRIVERS_WATCHDOG_INCLUDED */
