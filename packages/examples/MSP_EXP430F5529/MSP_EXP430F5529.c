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
 *  ======== MSP_EXP430F5529.c ========
 *  This file is responsible for setting up the board specific items for the
 *  MSP_EXP430F5529 board.
 *
 *  The following defines are used to determine which TI-RTOS peripheral drivers
 *  to include:
 *     TI_DRIVERS_GPIO_INCLUDED
 *     TI_DRIVERS_I2C_INCLUDED
 *     TI_DRIVERS_SDSPI_INCLUDED
 *     TI_DRIVERS_SPI_INCLUDED
 *     TI_DRIVERS_UART_INCLUDED
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

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

#include <msp430.h>
#include <inc/hw_memmap.h>
#include <dma.h>
#include <usci_a_uart.h>
#include <usci_b_i2c.h>
#include <usci_b_spi.h>
#include <usci_a_spi.h>
#include <ucs.h>
#include <gpio.h>
#include <pmm.h>

#include "MSP_EXP430F5529.h"

#include <ti/drivers/SPI.h>

const SPI_Config SPI_config[];

/*
 *  ======== MSP_EXP430F5529_initGeneral ========
 */
Void MSP_EXP430F5529_initGeneral(Void) {
}

/*
 *  ======== MSP_EXP430F5529_isrDMA ========
 *  This is a application defined DMA ISR. This ISR must map and call the
 *  appropriate Driver_event(handle) API to indicate completed DMA transfers.
 */
Void MSP_EXP430F5529_isrDMA(UArg arg)
{
#if TI_DRIVERS_WIFI_INCLUDED
    /* Call the SPI DMA function, passing the SPI handle used for WiFi */
    SPI_serviceISR((SPI_Handle) &(SPI_config[1]));
#elif TI_DRIVERS_SPI_INCLUDED
    /* Use the SPI_Handle that operates the SPI driver */
    SPI_serviceISR((SPI_Handle) &(SPI_config[0]));
#endif
}

#if TI_DRIVERS_GPIO_INCLUDED
#include <ti/drivers/GPIO.h>

/* GPIO configuration structure */
const GPIO_HWAttrs gpioHWAttrs[MSP_EXP430F5529_GPIOCOUNT] = {
    {GPIO_PORT_P1, GPIO_PIN0, GPIO_OUTPUT}, /* MSP_EXP430F5529_LED1 */
    {GPIO_PORT_P8, GPIO_PIN1, GPIO_OUTPUT}, /* MSP_EXP430F5529_LED2 */
    {GPIO_PORT_P8, GPIO_PIN2, GPIO_OUTPUT}, /* MSP_EXP430F5529_LED3 */
    {GPIO_PORT_P1, GPIO_PIN7, GPIO_INPUT},  /* MSP_EXP430F5529_S1 */
    {GPIO_PORT_P2, GPIO_PIN2, GPIO_INPUT},  /* MSP_EXP430F5529_S2 */
    {GPIO_PORT_P1, GPIO_PIN1, GPIO_OUTPUT}, /* MSP_EXP430F5529_PAD1(Cross) */
    {GPIO_PORT_P1, GPIO_PIN2, GPIO_OUTPUT}, /* MSP_EXP430F5529_PAD2(Square) */
    {GPIO_PORT_P1, GPIO_PIN3, GPIO_OUTPUT}, /* MSP_EXP430F5529_PAD3(Octagon) */
    {GPIO_PORT_P1, GPIO_PIN4, GPIO_OUTPUT}, /* MSP_EXP430F5529_PAD4(Triangle) */
    {GPIO_PORT_P1, GPIO_PIN5, GPIO_OUTPUT}, /* MSP_EXP430F5529_PAD5(Circle) */
};

const GPIO_Config GPIO_config[] = {
    {&gpioHWAttrs[0]},
    {&gpioHWAttrs[1]},
    {&gpioHWAttrs[2]},
    {&gpioHWAttrs[3]},
    {&gpioHWAttrs[4]},
    {&gpioHWAttrs[5]},
    {&gpioHWAttrs[6]},
    {&gpioHWAttrs[7]},
    {&gpioHWAttrs[8]},
    {&gpioHWAttrs[9]},
    {NULL},
};

/*
 *  ======== MSP_EXP430F5529_initGPIO ========
 */
Void MSP_EXP430F5529_initGPIO()
{
    /* Buttons are active low with pullup resistor */
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P2, GPIO_PIN2);

    /* LEDs */
    GPIO_setAsOutputPin(GPIO_PORT_P1,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5);
    GPIO_setAsOutputPin(GPIO_PORT_P8,
            GPIO_PIN1 | GPIO_PIN2);
    GPIO_setDriveStrength(GPIO_PORT_P1,
            GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5,
            GPIO_FULL_OUTPUT_DRIVE_STRENGTH);
    GPIO_setDriveStrength(GPIO_PORT_P8, GPIO_PIN1 | GPIO_PIN2,
            GPIO_FULL_OUTPUT_DRIVE_STRENGTH);

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}
#endif /* TI_DRIVERS_GPIO_INCLUDED */

#if TI_DRIVERS_I2C_INCLUDED
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CUSCIB.h>

/* I2C objects */
I2CUSCIB_Object i2cUSCIBObjects[MSP_EXP430F5529_I2CCOUNT];

/* I2C configuration structure */
const I2CUSCIB_HWAttrs i2cUSCIBHWAttrs[MSP_EXP430F5529_I2CCOUNT] = {
    {
        USCI_B0_BASE,
        USCI_B_I2C_CLOCKSOURCE_SMCLK
    },
    {
        USCI_B1_BASE,
        USCI_B_I2C_CLOCKSOURCE_SMCLK
    }
};

const I2C_Config I2C_config[] = {
    {
        &I2CUSCIB_fxnTable,
        &i2cUSCIBObjects[0],
        &i2cUSCIBHWAttrs[0]
    },
    {
        &I2CUSCIB_fxnTable,
        &i2cUSCIBObjects[1],
        &i2cUSCIBHWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430F5529_initI2C ========
 */
Void MSP_EXP430F5529_initI2C()
{
    /* USCIB0 */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P3,
        GPIO_PIN0 | GPIO_PIN1);

    /* USCIB1 */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P4,
        GPIO_PIN1 | GPIO_PIN2);

    I2C_init();
}
#endif /* TI_DRIVERS_I2C_INCLUDED */

#if TI_DRIVERS_SDSPI_INCLUDED
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPIUSCIB.h>

/* SDSPI objects */
SDSPIUSCIB_Object sdspiUSCIBobjects[MSP_EXP430F5529_SDSPICOUNT];

/* SDSPI configuration structure, describing which pins are to be used */
const SDSPIUSCIB_HWAttrs sdspiUSCIBHWAttrs[MSP_EXP430F5529_SDSPICOUNT] = {
    {
        USCI_B1_BASE,   /* SPI base address */

        USCI_B_SPI_CLOCKSOURCE_SMCLK, /* Clock source */

        GPIO_PORT_P4,   /* The GPIO port used for the SPI pins */
        GPIO_PIN3,      /* SCK */
        GPIO_PIN2,      /* MISO */
        GPIO_PIN1,      /* MOSI */

        GPIO_PORT_P3,   /* Chip select port */
        GPIO_PIN7,      /* Chip select pin */
    }
};

const SDSPI_Config SDSPI_config[] = {
    {
        &SDSPIUSCIB_fxnTable,
        &sdspiUSCIBobjects[0],
        &sdspiUSCIBHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430F5529_initSDSPI ========
 */
Void MSP_EXP430F5529_initSDSPI()
{
    SDSPI_init();
}
#endif /* TI_DRIVERS_SDSPI_INCLUDED */

#if TI_DRIVERS_SPI_INCLUDED
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPIUSCIBDMA.h>

/* SPI objects */
SPIUSCIBDMA_Object spiUSCIBDMAobjects[MSP_EXP430F5529_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPIUSCIBDMA_HWAttrs spiUSCIBDMAHWAttrs[MSP_EXP430F5529_SPICOUNT] = {
    {
        USCI_B1_BASE,
        USCI_B_SPI_CLOCKSOURCE_SMCLK,
        USCI_B_SPI_MSB_FIRST,

        /* DMA */
        DMA_BASE,
        /* Rx Channel */
        DMA_CHANNEL_1,
        DMA_TRIGGERSOURCE_22,
        /* Tx Channel */
        DMA_CHANNEL_0,
        DMA_TRIGGERSOURCE_23
    },
    {
        USCI_B0_BASE,
        USCI_B_SPI_CLOCKSOURCE_SMCLK,
        USCI_B_SPI_MSB_FIRST,

        /* DMA */
        DMA_BASE,
        /* Rx Channel */
        DMA_CHANNEL_1,
        DMA_TRIGGERSOURCE_18,
        /* Tx Channel */
        DMA_CHANNEL_0,
        DMA_TRIGGERSOURCE_19
    }
};

const SPI_Config SPI_config[] = {
    {
        &SPIUSCIBDMA_fxnTable,
        &spiUSCIBDMAobjects[0],
        &spiUSCIBDMAHWAttrs[0]
    },
    {
        &SPIUSCIBDMA_fxnTable,
        &spiUSCIBDMAobjects[1],
        &spiUSCIBDMAHWAttrs[1]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== MSP_EXP430F5529_initSPI ========
 */
Void MSP_EXP430F5529_initSPI(Void)
{
    /* USCIB1 */
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P4,
            GPIO_PIN3 | GPIO_PIN2 | GPIO_PIN1);

    /* SOMI/MISO */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
            GPIO_PIN2);

    /* CLK and SIMO/MOSI */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4,
            GPIO_PIN3 | GPIO_PIN1);

    SPI_init();
}
#endif /* TI_DRIVERS_SPI_INCLUDED */

#if TI_DRIVERS_UART_INCLUDED
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTUSCIA.h>

/* UART objects */
UARTUSCIA_Object uartUSCIAObjects[MSP_EXP430F5529_UARTCOUNT];

/*
 * The baudrate dividers were determined by using the MSP430 baudrate
 * calculator
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const UARTUSCIA_BaudrateConfig uartUSCIABaudrates[] = {
    /* baudrate, input clock, prescalar, UCBRFx, UCBRSx, oversampling */
    {115200, 8192000, 4,   7, 0, 1},
    {9600,   8192000, 53,  5, 0, 1},
    {9600,   32768,   3,   0, 3, 0},
};

/* UART configuration structure */
const UARTUSCIA_HWAttrs uartUSCIAHWAttrs[MSP_EXP430F5529_UARTCOUNT] = {
    {
        USCI_A1_BASE,
        USCI_A_UART_CLOCKSOURCE_SMCLK,
        USCI_A_UART_LSB_FIRST,
        sizeof(uartUSCIABaudrates)/sizeof(UARTUSCIA_BaudrateConfig),
        uartUSCIABaudrates
    },
};

const UART_Config UART_config[] = {
    {
        &UARTUSCIA_fxnTable,
        &uartUSCIAObjects[0],
        &uartUSCIAHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430F5529_initUART ========
 */
Void MSP_EXP430F5529_initUART()
{
    /* P4.4,5 = USCI_A1 TXD/RXD */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
            GPIO_PIN4 | GPIO_PIN5);

    /* Initialize the UART driver */
    UART_init();
}
#endif /* TI_DRIVERS_UART_INCLUDED */

/*
 *  ======== MSP_EXP430F5529_initUSB ========
 */
Void MSP_EXP430F5529_initUSB(UInt arg)
{
	PMM_setVCore(PMM_BASE, PMM_CORE_LEVEL_3);

}

#if TI_DRIVERS_WATCHDOG_INCLUDED
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogMSP430.h>

/* Watchdog objects */
WatchdogMSP430_Object watchdogMSP430Objects[MSP_EXP430F5529_WATCHDOGCOUNT];

/* Watchdog configuration structure */
const WatchdogMSP430_HWAttrs watchdogMSP430HWAttrs[MSP_EXP430F5529_WATCHDOGCOUNT] = {
    {
        WDT_A_BASE,
        SFR_BASE,
        WATCHDOG_CLOCKSOURCE_SMCLK,
        WATCHDOG_CLOCKDIVIDER_8192K
    }, /* MSP430F5529_WATCHDOG */
};

const Watchdog_Config Watchdog_config[] = {
    {
        &WatchdogMSP430_fxnTable,
        &watchdogMSP430Objects[0],
        &watchdogMSP430HWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== MSP_EXP430F5529_initWatchdog ========
 */
Void MSP_EXP430F5529_initWatchdog()
{
    /* Initialize the Watchdog driver */
    Watchdog_init();
}
#endif /* TI_DRIVERS_WATCHDOG_INCLUDED */

#if TI_DRIVERS_WIFI_INCLUDED
#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiMSP430CC3000.h>

/* WiFi objects */
WiFiMSP430CC3000_Object wiFiMSP430CC3000Objects[MSP_EXP430F5529_WIFICOUNT];

/* WiFi configuration structure */
const WiFiMSP430CC3000_HWAttrs wiFiMSP430CC3000HWAttrs[MSP_EXP430F5529_WIFICOUNT] = {
    {
        GPIO_PORT_P2, /* IRQ port */
        GPIO_PIN4,    /* IRQ pin */

        GPIO_PORT_P2, /* CS port */
        GPIO_PIN6,    /* CS pin */

        GPIO_PORT_P2, /* WLAN EN port */
        GPIO_PIN3     /* WLAN EN pin */
    }
};

const WiFi_Config WiFi_config[] = {
    {
        &WiFiMSP430CC3000_fxnTable,
        &wiFiMSP430CC3000Objects[0],
        &wiFiMSP430CC3000HWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== MSP_EXP430F5529_initWiFi ========
 */
Void MSP_EXP430F5529_initWiFi(Void)
{
    /* Configure SPI */
    /* SOMI/MISO */
    GPIO_setAsInputPinWithPullUpresistor(GPIO_PORT_P3, GPIO_PIN1);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN1);

    /* CLK and SIMO/MOSI */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
            GPIO_PIN0 | GPIO_PIN2);

    /* Initialize SPI and WiFi drivers */
    SPI_init();
    WiFi_init();
}
#endif /* TI_DRIVERS_WIFI_INCLUDED */
