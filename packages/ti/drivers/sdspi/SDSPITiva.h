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
 *  @file       SDSPITiva.h
 *
 *  @brief      SDSPI driver implementation for a Tiva SPI peripheral used
 *              with the SDSPI driver.
 *
 *  The SDSPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SDSPI.h>
 *  #include <ti/drivers/sdspi/SDSPITiva.h>
 *  @endcode
 *
 *  This SDSPI driver implementation is designed to operate on a Tiva SPI
 *  controller using a polling method.
 *
 *  ============================================================================
 */

#ifndef ti_drivers_sdspi_SDSPITiva__include
#define ti_drivers_sdspi_SDSPITiva__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/drivers/SDSPI.h>

#include <ti/sysbios/fatfs/ff.h>
#include <ti/sysbios/fatfs/diskio.h>

#if defined(TIVAWARE)
/* c99 types needed by TivaWare */
#include <stdint.h>
typedef uint32_t            SDSPIBaseAddrType;
typedef uint32_t            SDSPIDataType;
#else /* MWARE */
typedef ULong               SDSPIBaseAddrType;
typedef ULong               SDSPIDataType;
#endif

/* SDSPI function table */
extern const SDSPI_FxnTable SDSPITiva_fxnTable;

/*!
 *  @brief  SD Card type inserted
 */
typedef enum SDSPITiva_CardType {
    NOCARD = 0, /*!< Unrecognized Card */
    MMC = 1,    /*!< Multi-media Memory Card (MMC) */
    SDSC = 2,   /*!< Standard SDCard (SDSC) */
    SDHC = 3    /*!< High Capacity SDCard (SDHC) */
} SDSPITiva_CardType;

/*!
 *  @brief  SDSPITiva Hardware attributes
 *
 *  The SDSPITiva configuration structure describes to the SDSPITiva
 *  driver implementation hardware specifies on which SPI peripheral, GPIO Pins
 *  and Ports are to be used.
 *
 *  The SDSPITiva driver uses this information to:
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
 *      - inc/hw_ints.h
 *
 *  @struct SDSPITiva_HWAttrs
 *  An example configuration structure could look as the following:
 *  @code
 *  const SDSPITiva_HWAttrs sdspiTivaHWattrs = {
 *      {
 *          // SPI Peripheral's base address
 *          SSI0_BASE,
 *
 *          // SPI Port for SCK, MISO, and MOSI pins
 *          GPIO_PORTD_BASE,
 *          // SPI SCK pin
 *          GPIO_PIN_2,
 *          // SPI MISO pin
 *          GPIO_PIN_1,
 *          // SPI MOSI pin
 *          GPIO_PIN_0,
 *
 *          // GPIO Port used for the chip select
 *          GPIO_PORTD_BASE,
 *          // GPIO Pin used for the chip select
 *          GPIO_PIN_3,
 *
 *          // GPIO Port that sits on the same pin as pinMOSI
 *          GPIO_PORTA_BASE,
 *          // GPIO Pin that sits on the same pin as pinMOSI
 *          GPIO_PIN_5,
 *       },
 *  };
 *  @endcode
 */
typedef struct SDSPITiva_HWAttrs {
    /*!< SSI Peripheral's base address */
    SDSPIBaseAddrType baseAddr;

    /*!< SSI port uses for the SCK, MISO, and MOSI pins */
    ULong portSPI;
    /*!< SSI SCK pin */
    ULong pinSCK;
    /*!< SSI MISO pin */
    ULong pinMISO;
    /*!< SSI MOSI pin */
    ULong pinMOSI;

    /*!< GPIO Port used for the chip select */
    ULong portCS;
    /*!< GPIO Pin used for the chip select */
    ULong pinCS;

    /*!< GPIO Port that sits on the same pin as pinMOSI */
    ULong portTX;
    /*!< GPIO Pin that sits on the same pin at pinMOSI */
    ULong pinTX;
} SDSPITiva_HWAttrs;

/*!
 *  @brief  SDSPITiva Object
 *
 *  The application must not access any member variables of this structure!
 */
typedef struct SDSPITiva_Object {
    UInt                 driveNumber;   /*!< Drive number used by FatFs */
    DSTATUS              diskState;     /*!< Disk status */
    SDSPITiva_CardType   cardType;      /*!< SDCard Card Command Class (CCC) */
    ULong                bitRate;       /*!< SPI bus bit rate (Hz) */
    FATFS                filesystem;    /*!< FATFS data object */
} SDSPITiva_Object, *SDSPITiva_Handle;

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_sdspi_SDSPITiva__include */
