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
/** ============================================================================
 *  @file       SDSPIUSCIB.h
 *
 *  @brief      SDSPI driver implementation for a USCI SPI peripheral used
 *              with the SDSPI driver.
 *
 *  The SDSPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SDSPI.h>
 *  #include <ti/drivers/sdspi/SDSPIUSCIB.h>
 *  @endcode
 *
 *  This SDSPI driver implementation is designed to operate on a USCI SPI
 *  controller in a simple polling method.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_sdspi_SDSPIUSCIB__include
#define ti_drivers_sdspi_SDSPIUSCIB__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/drivers/SDSPI.h>

/*
 * DIR gets defined in msp430.h and ff.h
 * We need the defined DIR data structure from ff.h
 */
#undef DIR

#include <ti/sysbios/fatfs/ff.h>
#include <ti/sysbios/fatfs/diskio.h>

/* SDSPI function table */
extern const SDSPI_FxnTable SDSPIUSCIB_fxnTable;

/*!
 *  @brief  SD Card type inserted
 */
typedef enum SDSPIUSCIB_CardType {
    NOCARD = 0, /*!< Unrecognized Card */
    MMC = 1,    /*!< Multi-media Memory Card (MMC) */
    SDSC = 2,   /*!< Standard SDCard (SDSC) */
    SDHC = 3    /*!< High Capacity SDCard (SDHC) */
} SDSPIUSCIB_CardType;

/*!
 *  @brief  SDSPIUSCIB Hardware attributes
 *
 *  The SDSPIUSCIB configuration structure describes to the SDSPIUSCIB
 *  driver implementation hardware specifies on which SPI peripheral, GPIO Pins
 *  and Ports are to be used.
 *
 *  The SDSPIUSCIB driver uses this information to:
 *  - configure and reconfigure specific ports/pins to initialize the SD Card
 *    for SPI mode
 *  - identify which SPI peripheral is used for data communications
 *  - identify which GPIO port and pin is used for the SPI chip select
 *    mechanism
 *  - identify which GPIO port and pin is concurrently located on the SPI's MOSI
 *    (TX) pin.
 *
 *  @remark
 *  To initialize the SD Card into SPI mode, the SDSPI driver changes the SPI's
 *  MOSI pin into a GPIO pin so it can kept driven HIGH while the SPI SCK pin
 *  can toggle. After the initialization, the TX pin is reverted back to the SPI
 *  MOSI mode.
 *
 *  These fields are used by driverlib APIs and therefore must be populated by
 *  driverlib macro definitions. For TivaWare these definitions are found in:
 *      - inc/hw_memmap.h
 *      - usci_b_spi.h
 *
 *  @struct SDSPIUSCIB_HWAttrs
 *  An example configuration structure could look as the following:
 *  @code
 *  const SPIUSCIBDMA_HWAttrs sdspiUSCIBHWAttrs = {
 *      {
 *          // SPI Peripheral's base address
 *          USCI_B1_BASE,
 *          // Clock source
 *          USCI_B_SPI_CLOCKSOURCE_SMCLK,
 *
 *          // The GPIO port used for the SPI pins
 *          GPIO_PORT_P4,
 *          // SPI SCK pin
 *          GPIO_PIN3,
 *          // SPI MISO pin
 *          GPIO_PIN2,
 *          // SPI MOSI pin
 *          GPIO_PIN1,
 *
 *          // GPIO Port used for the chip select
 *          GPIO_PORT_P3,
 *          // GPIO Pin used for the chip select
 *          GPIO_PIN7,
 *
 *          // GPIO Port that sits on the same pin as pinMOSI
 *          GPIO_PORT_P4,
 *          // GPIO Pin that sits on the same pin as pinMOSI
 *          GPIO_PIN1,
 *       },
 *  };
 *  @endcode
 */
typedef struct SDSPIUSCIB_HWAttrs {
    ULong   baseAddr; /*!< SPI Peripheral's base address */

    UChar   clockSource; /*!< SPIUSCI Clock source */

    ULong   portSPI;  /*!< SPI port uses for the SCK, MISO, and MOSI pins */
    ULong   pinSCK;   /*!< SPI SCK pin */
    ULong   pinMISO;  /*!< SPI MISO pin */
    ULong   pinMOSI;  /*!< SPI MOSI pin */

    ULong   portCS;   /*!< GPIO Port used for the chip select */
    ULong   pinCS;    /*!< GPIO Pin used for the chip select */

} SDSPIUSCIB_HWAttrs;

/*!
 *  @brief  SDSPIUSCIB Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SDSPIUSCIB_Object {
    UInt                 driveNumber;   /*!< Drive number used by FatFs */
    DSTATUS              diskState;     /*!< Disk status */
    SDSPIUSCIB_CardType  cardType;      /*!< SDCard Card Command Class (CCC) */
    ULong                bitRate;       /*!< SPI bus bit rate (Hz) */
    FATFS                filesystem;    /*!< FATFS data object */
} SDSPIUSCIB_Object, *SDSPIUSCIB_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_sdspi_SDSPIUSCIB__include */
