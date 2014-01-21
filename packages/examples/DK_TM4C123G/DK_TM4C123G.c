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
 *  ======== DK_TM4C123G.c ========
 *  This file is responsible for setting up the board specific items for the
 *  DK_TM4C123G board.
 *
 *  The following defines are used to determine which TI-RTOS peripheral drivers
 *  to include:
 *     TI_DRIVERS_GPIO_INCLUDED
 *     TI_DRIVERS_I2C_INCLUDED
 *     TI_DRIVERS_SDSPI_INCLUDED
 *     TI_DRIVERS_SPI_INCLUDED
 *     TI_DRIVERS_UART_INCLUDED
 *     TI_DRIVERS_USBMSCHFATFS_INCLUDED
 *     TI_DRIVERS_WATCHDOG_INCLUDED
 *     TI_DRIVERS_WIFI_INCLUDED
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

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/udma.h>
#include <driverlib/pin_map.h>

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include "DK_TM4C123G.h"

#ifdef ccs
#pragma DATA_ALIGN(DK_TM4C123G_DMAControlTable, 1024)
#elif ewarm
#pragma data_alignment=1024
#endif
static tDMAControlTable DK_TM4C123G_DMAControlTable[32];
static Bool DMA_initialized = FALSE;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct hwiStruct;

/*
 *  ======== DK_TM4C123G_errorDMAHwi ========
 */
static Void DK_TM4C123G_errorDMAHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== DK_TM4C123G_initDMA ========
 */
Void DK_TM4C123G_initDMA(Void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if(!DMA_initialized) {

        Error_init(&eb);

        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(hwiStruct), INT_UDMAERR, DK_TM4C123G_errorDMAHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't create DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(DK_TM4C123G_DMAControlTable);

        DMA_initialized = TRUE;
    }
}

/*
 *  ======== DK_TM4C123G_initGeneral ========
 */
Void DK_TM4C123G_initGeneral(Void)
{
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
}

#if TI_DRIVERS_GPIO_INCLUDED
#include <ti/drivers/GPIO.h>

/* Callback functions for the GPIO interrupt example. */
Void gpioButtonFxn0(Void);
Void gpioButtonFxn1(Void);

/* GPIO configuration structure */
const GPIO_HWAttrs gpioHWAttrs[DK_TM4C123G_GPIOCOUNT] = {
    {GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_OUTPUT}, /* DK_TM4C123G_LED */
    {GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_INPUT}, /* DK_TM4C123G_SW1(Up) */
    {GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_INPUT}, /* DK_TM4C123G_SW2(Down) */
    {GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_INPUT}, /* DK_TM4C123G_SW3(Left) */
    {GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_INPUT}, /* DK_TM4C123G_SW4(Right) */
    {GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_INPUT}, /* DK_TM4C123G_SW5(Select/Wake) */
};

/* Memory for the GPIO module to construct a Hwi */
Hwi_Struct callbackHwi;

/* GPIO callback structure to set callbacks for GPIO interrupts */
const GPIO_Callbacks DK_TM4C123G_gpioPortMCallbacks = {
    GPIO_PORTM_BASE, INT_GPIOM, &callbackHwi,
    {NULL, NULL, gpioButtonFxn0, gpioButtonFxn1, NULL, NULL, NULL, NULL}
};

const GPIO_Config GPIO_config[] = {
    {&gpioHWAttrs[0]},
    {&gpioHWAttrs[1]},
    {&gpioHWAttrs[2]},
    {&gpioHWAttrs[3]},
    {&gpioHWAttrs[4]},
    {&gpioHWAttrs[5]},
    {NULL},
};

/*
 *  ======== DK_TM4C123G_initGPIO ========
 */
Void DK_TM4C123G_initGPIO(Void)
{
    /* DK_TM4C123G_SW1 - DK_TM4C123G_SW5 */
    UChar allButtons = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;

    /* Setup the LED GPIO pins used */
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2); /* DK_TM4C123G_LED */

    /* Setup the button GPIO pins used */
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, allButtons);
    GPIOPadConfigSet(GPIO_PORTM_BASE, allButtons, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}
#endif /* TI_DRIVERS_GPIO_INCLUDED */

#if TI_DRIVERS_I2C_INCLUDED
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

/* I2C objects */
I2CTiva_Object i2cTivaObjects[DK_TM4C123G_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const I2CTiva_HWAttrs i2cTivaHWAttrs[DK_TM4C123G_I2CCOUNT] = {
    {I2C0_BASE, INT_I2C0},
    {I2C2_BASE, INT_I2C2}
};

const I2C_Config I2C_config[] = {
    {&I2CTiva_fxnTable, &i2cTivaObjects[0], &i2cTivaHWAttrs[0]},
    {&I2CTiva_fxnTable, &i2cTivaObjects[1], &i2cTivaHWAttrs[1]},
    {NULL, NULL, NULL}
};

/*
 *  ======== DK_TM4C123G_initI2C ========
 */
Void DK_TM4C123G_initI2C(Void)
{
    /* I2C0 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    /* I2C2 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PF6_I2C2SCL);
    GPIOPinConfigure(GPIO_PF7_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTF_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTF_BASE, GPIO_PIN_7);

    I2C_init();
}
#endif /* TI_DRIVERS_I2C_INCLUDED */

#if TI_DRIVERS_SDSPI_INCLUDED
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPITiva.h>

/* SDSPI objects */
SDSPITiva_Object sdspiTivaobjects[DK_TM4C123G_SDSPICOUNT];

/* SDSPI configuration structure, describing which pins are to be used */
const SDSPITiva_HWAttrs sdspiTivaHWattrs[DK_TM4C123G_SDSPICOUNT] = {
    {
        SSI0_BASE,          /* SPI base address */

        GPIO_PORTA_BASE,    /* The GPIO port used for the SPI pins */
        GPIO_PIN_2,         /* SCK */
        GPIO_PIN_4,         /* MISO */
        GPIO_PIN_5,         /* MOSI */

        GPIO_PORTA_BASE,    /* Chip select port */
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
 *  ======== DK_TM4C123G_initSDSPI ========
 */
Void DK_TM4C123G_initSDSPI(Void)
{
    /* Enable the peripherals used by the SD Card */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    /* Configure pad settings */
    GPIOPadConfigSet(GPIO_PORTA_BASE,
            GPIO_PIN_2 | GPIO_PIN_5,
            GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    GPIOPadConfigSet(GPIO_PORTA_BASE,
            GPIO_PIN_4,
            GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPadConfigSet(GPIO_PORTA_BASE,
            GPIO_PIN_3,
            GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    SDSPI_init();
}
#endif /* TI_DRIVERS_SDSPI_INCLUDED */

#if TI_DRIVERS_SPI_INCLUDED
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

/* SPI objects */
SPITivaDMA_Object spiTivaDMAobjects[DK_TM4C123G_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[DK_TM4C123G_SPICOUNT] = {
    {
        SSI0_BASE,
        INT_SSI0,
        UDMA_CHANNEL_SSI0RX,
        UDMA_CHANNEL_SSI0TX,
        uDMAChannelAssign,
        UDMA_CH10_SSI0RX,
        UDMA_CH11_SSI0TX
    },
    {
        SSI1_BASE,
        INT_SSI1,
        UDMA_CHANNEL_SSI1RX,
        UDMA_CHANNEL_SSI1TX,
        uDMAChannelAssign,
        UDMA_CH24_SSI1RX,
        UDMA_CH25_SSI1TX
    },
    {
        SSI2_BASE,
        INT_SSI2,
        UDMA_SEC_CHANNEL_UART2RX_12,
        UDMA_SEC_CHANNEL_UART2TX_13,
        uDMAChannelAssign,
        UDMA_CH12_SSI2RX,
        UDMA_CH13_SSI2TX
    },
    {
        SSI3_BASE,
        INT_SSI3,
        UDMA_SEC_CHANNEL_TMR2A_14,
        UDMA_SEC_CHANNEL_TMR2B_15,
        uDMAChannelAssign,
        UDMA_CH14_SSI3RX,
        UDMA_CH15_SSI3TX
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
 *  ======== DK_TM4C123G_initSPI ========
 */
Void DK_TM4C123G_initSPI(Void)
{
    /* SPI1 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    /* Need to unlock PF0 */
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    GPIOPinConfigure(GPIO_PF0_SSI1RX);
    GPIOPinConfigure(GPIO_PF1_SSI1TX);
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);

    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                    GPIO_PIN_2 | GPIO_PIN_3);
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_M;

    /* SSI3 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    GPIOPinConfigure(GPIO_PK0_SSI3CLK);
    GPIOPinConfigure(GPIO_PK1_SSI3FSS);
    GPIOPinConfigure(GPIO_PK2_SSI3RX);
    GPIOPinConfigure(GPIO_PK3_SSI3TX);

    GPIOPinTypeSSI(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                                    GPIO_PIN_2 | GPIO_PIN_3);

    DK_TM4C123G_initDMA();
    SPI_init();
}
#endif /* TI_DRIVERS_SPI_INCLUDED */

#if TI_DRIVERS_UART_INCLUDED
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTTiva.h>

/* UART objects */
UARTTiva_Object uartTivaObjects[DK_TM4C123G_UARTCOUNT];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[DK_TM4C123G_UARTCOUNT] = {
    {UART0_BASE, INT_UART0}, /* DK_TM4C123G_UART0 */
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
 *  ======== DK_TM4C123G_initUART ========
 */
Void DK_TM4C123G_initUART()
{
    /* Enable and configure the peripherals used by the uart. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the UART driver */
    UART_init();
}
#endif /* TI_DRIVERS_UART_INCLUDED */

/*
 *  ======== DK_TM4C123G_initUSB ========
 *  This function just turns on the USB
 */
Void DK_TM4C123G_initUSB(DK_TM4C123G_USBMode usbMode)
{
    /* Enable the USB peripheral and PLL */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    SysCtlUSBPLLEnable();

    /* Setup pins for USB operation */
    GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /* Additional configurations for Host mode */
    if (usbMode == DK_TM4C123G_USBHOST) {
        /* Configure the pins needed */
        GPIOPinConfigure(GPIO_PG4_USB0EPEN);
        GPIOPinConfigure(GPIO_PG5_USB0PFLT);
        GPIOPinTypeUSBDigital(GPIO_PORTG_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    }
}

#if TI_DRIVERS_USBMSCHFATFS_INCLUDED
#include <ti/drivers/USBMSCHFatFs.h>
#include <ti/drivers/usbmschfatfs/USBMSCHFatFsTiva.h>

/* USBMSCHFatFs objects */
USBMSCHFatFsTiva_Object usbmschfatfstivaObjects[DK_TM4C123G_USBMSCHFatFsCOUNT];

/* USBMSCHFatFs configuration structure, describing which pins are to be used */
const USBMSCHFatFsTiva_HWAttrs usbmschfatfstivaHWAttrs[DK_TM4C123G_USBMSCHFatFsCOUNT] = {
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
 *  ======== DK_TM4C123G_initUSBMSCHFatFs ========
 */
Void DK_TM4C123G_initUSBMSCHFatFs(Void)
{
    /* Initialize the DMA control table */
    DK_TM4C123G_initDMA();

    /* Call the USB initialization function for the USB Reference modules */
    DK_TM4C123G_initUSB(DK_TM4C123G_USBHOST);
    USBMSCHFatFs_init();
}
#endif /* TI_DRIVERS_USBMSCHFATFS_INCLUDED */

#if TI_DRIVERS_WATCHDOG_INCLUDED
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

/* Watchdog objects */
WatchdogTiva_Object watchdogTivaObjects[DK_TM4C123G_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[DK_TM4C123G_WATCHDOGCOUNT] = {
    /* DK_TM4C123G_WATCHDOG0 with 1 sec period at default CPU clock freq */
    {WATCHDOG0_BASE, INT_WATCHDOG, 80000000},
};

const Watchdog_Config Watchdog_config[] = {
    {&WatchdogTiva_fxnTable, &watchdogTivaObjects[0], &watchdogTivaHWAttrs[0]},
    {NULL, NULL, NULL},
};

/*
 *  ======== DK_TM4C123G_initWatchdog ========
 *
 * NOTE: To use the other watchdog timer with base address WATCHDOG1_BASE,
 *       an additional function call may need be made to enable PIOSC. Enabling
 *       WDOG1 does not do this. Enabling another peripheral that uses PIOSC
 *       such as ADC0 or SSI0, however, will do so. Example:
 *
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
 *
 *       See the following forum post for more information:
 *       http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/176487/654390.aspx#654390
 */
Void DK_TM4C123G_initWatchdog()
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}
#endif /* TI_DRIVERS_WATCHDOG_INCLUDED */

#if TI_DRIVERS_WIFI_INCLUDED
#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiTivaCC3000.h>

/* WiFi objects */
WiFiTivaCC3000_Object wiFiTivaCC3000Objects[DK_TM4C123G_WIFICOUNT];

/* WiFi configuration structure */
const WiFiTivaCC3000_HWAttrs wiFiTivaCC3000HWAttrs[DK_TM4C123G_WIFICOUNT] = {
    { GPIO_PORTC_BASE, /* IRQ port */
      GPIO_PIN_7,      /* IRQ pin */
      INT_GPIOC,       /* IRQ port interrupt vector */

      GPIO_PORTH_BASE, /* CS port */
      GPIO_PIN_1,      /* CS pin */

      GPIO_PORTC_BASE, /* WLAN EN port */
      GPIO_PIN_6       /* WLAN EN pin */
    }
};

const WiFi_Config WiFi_config[] = {
    {
        &WiFiTivaCC3000_fxnTable,
        &wiFiTivaCC3000Objects[0],
        &wiFiTivaCC3000HWAttrs[0]
    },
    {NULL,NULL, NULL},
};

/*
 *  ======== DK_TM4C123G_initWiFi ========
 */
Void DK_TM4C123G_initWiFi(Void)
{
    /* Configure SSI3 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    GPIOPinConfigure(GPIO_PH0_SSI3CLK);
    GPIOPinConfigure(GPIO_PH2_SSI3RX);
    GPIOPinConfigure(GPIO_PH3_SSI3TX);

    GPIOPinTypeSSI(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

    /* Call necessary SPI init functions */
    SPI_init();
    DK_TM4C123G_initDMA();

    /* Initialize WiFi driver */
    WiFi_init();
}
#endif /* TI_DRIVERS_WIFI_INCLUDED */
