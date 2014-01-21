var Boards = xdc.loadCapsule("Boards.xs");

var MSP430WareVer = Boards.MSP430WareVer;

if (MSP430WareVer == undefined){
    MSP430WareVer = "MSP430Ware_1_60_00_23a";
    print("USING unwanted MSP430 dir");
}

var usbStackPath = "../../products/" + MSP430WareVer + "/usblib430/MSP430_USB_Software/MSP430_USB_API/USB_API/";
var msp430USBstackFiles = [

    /* USB_CDC_API */
    {path: usbStackPath + "USB_CDC_API/UsbCdc.c",               targetDirectory: "USB_API/USB_CDC_API"},
    {path: usbStackPath + "USB_CDC_API/UsbCdc.h",               targetDirectory: "USB_API/USB_CDC_API"},

    /* USB_Common */
    {path: usbStackPath + "USB_Common/defMSP430USB.h",          targetDirectory: "USB_API/USB_Common"},
    {path: usbStackPath + "USB_Common/device.h",                targetDirectory: "USB_API/USB_Common"},
    {path: usbStackPath + "USB_Common/types.h",                 targetDirectory: "USB_API/USB_Common"},
    {path: usbStackPath + "USB_Common/usb.c",                   targetDirectory: "USB_API/USB_Common"},
    {path: usbStackPath + "USB_Common/usb.h",                   targetDirectory: "USB_API/USB_Common"},
    {path: usbStackPath + "USB_Common/usbdma.c",                targetDirectory: "USB_API/USB_Common"},
    {path: usbStackPath + "USB_Common/UsbIsr.h",                targetDirectory: "USB_API/USB_Common"},

    /* USB_HID_API */
    {path: usbStackPath + "USB_HID_API/UsbHid.c",               targetDirectory: "USB_API/USB_HID_API"},
    {path: usbStackPath + "USB_HID_API/UsbHid.h",               targetDirectory: "USB_API/USB_HID_API"},
    {path: usbStackPath + "USB_HID_API/UsbHidReq.c",            targetDirectory: "USB_API/USB_HID_API"},
    {path: usbStackPath + "USB_HID_API/UsbHidReq.h",            targetDirectory: "USB_API/USB_HID_API"},

    /* USB_MSC_API */
    {path: usbStackPath + "USB_MSC_API/UsbMsc.h",               targetDirectory: "USB_API/USB_MSC_API"},
    {path: usbStackPath + "USB_MSC_API/UsbMscReq.c",            targetDirectory: "USB_API/USB_MSC_API"},
    {path: usbStackPath + "USB_MSC_API/UsbMscReq.h",            targetDirectory: "USB_API/USB_MSC_API"},
    {path: usbStackPath + "USB_MSC_API/UsbMscScsi.c",           targetDirectory: "USB_API/USB_MSC_API"},
    {path: usbStackPath + "USB_MSC_API/UsbMscScsi.h",           targetDirectory: "USB_API/USB_MSC_API"},
    {path: usbStackPath + "USB_MSC_API/UsbMscStateMachine.c",   targetDirectory: "USB_API/USB_MSC_API"},
    {path: usbStackPath + "USB_MSC_API/UsbMscStateMachine.h",   targetDirectory: "USB_API/USB_MSC_API"},

    /* USB_PHDC_API */
    {path: usbStackPath + "USB_PHDC_API/UsbPHDC.c",             targetDirectory: "USB_API/USB_PHDC_API"},
    {path: usbStackPath + "USB_PHDC_API/UsbPHDC.h",             targetDirectory: "USB_API/USB_PHDC_API"},

    /* USB linker cmd file */
    {path: usbStackPath + "msp430USB.cmd",                      targetDirectory: "USB_API"},
];

/*
 *
 ********* EMPTY EXAMPLES **********
 *
 */
var allExamples = [
    {
        title: "Empty Project",
        name: "empty",
        description: "An empty TI-RTOS project",
        cFile: "empty.c",
        cfgFile: "empty.cfg",
        readme: "empty_readme.txt",
        fileList: [],
        requiredProducts: [],
        options: "NPW",
        type: "example",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.DK_TM4C129X,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "Empty (Minimal) Project",
        name: "empty_min",
        description: "An empty TI-RTOS project with minimal footprint",
        cFile: "empty_min.c",
        cfgFile: "empty_min.cfg",
        readme: "empty_min_readme.txt",
        fileList: [],
        requiredProducts: [],
        options: "NPW",
        type: "example",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.DK_TM4C129X,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
/*
 *
 ********* Ethernet EXAMPLES **********
 *
 */
    {
        title: "TCP Echo",
        name: "tcpEcho",
        description: "The TCP Echo example showing how to use TCP sockets",
        cFile: "tcpEcho.c",
        cfgFile: "tcpEcho.cfg",
        fileList:[
            {path: "tcpEcho_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "Ethernet",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "TCP Echo IPv6",
        name: "tcpEchoIPv6",
        description: "The TCP Echo example showing how to use TCP sockets for IPv6",
        cFile: "tcpEchoIPv6.c",
        cfgFile: "tcpEchoIPv6.cfg",
        fileList:[
            {path: "tcpEchoIPv6_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "Ethernet",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "UDP Echo",
        name: "udpEcho",
        description: "The UDP Echo example showing how to use UDP sockets",
        cFile: "udpEcho.c",
        cfgFile: "udpEcho.cfg",
        fileList: [
            {path: "udpEcho_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "Ethernet",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "UDP Echo IPv6",
        name: "udpEchoIPv6",
        description: "The UDP Echo example showing how to use UDP sockets for IPv6",
        cFile: "udpEchoIPv6.c",
        cfgFile: "udpEchoIPv6.cfg",
        fileList:[
            {path: "udpEchoIPv6_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "Ethernet",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.DK_TM4C129X
        ],
    },
/*
 *
 ********* FatFS EXAMPLES **********
 *
 */
    {
        title: "FatSD",
        name: "fatsd",
        description: "An example using an SDcard",
        cFile: "fatsd.c",
        cfgFile: "fatsd.cfg",
        fileList: [
            {path: "fatsd_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "FatFS",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "FatSD Raw",
        name: "fatsdraw",
        description: "An example using an SDcard with FatFs API calls",
        cFile: "fatsdraw.c",
        cfgFile: "fatsd.cfg",
        fileList:[
            {path: "fatsdraw_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "FatFS",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "FatSD",
        name: "fatsd",
        description: "An example using an SDcard",
        cFile: "fatsd.c",
        cfgFile: "fatsd.cfg",
        fileList:[
            {path: "fatsd_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "FatFS",
        tools: ["TI"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "FatSD Raw",
        name: "fatsdraw",
        description: "An example using an SDcard with FatFs API calls",
        cFile: "fatsdraw.c",
        cfgFile: "fatsd.cfg",
        fileList:[
            {path: "fatsdraw_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "FatFS",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "FatSD USB Copy",
        name: "fatsdusbcopy",
        description: "An example copying a file from a SD Card to a USB Drive",
        cFile: "fatsdusbcopy.c",
        cfgFile: "fatsdusbcopy.cfg",
        fileList:[
            {path: "fatsdusbcopy_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.DK_TM4C129X
        ],
    },
/*
 *
 ********* GPIO EXAMPLES **********
 *
 */
    {
        title: "GPIO Interrupt",
        name: "gpiointerrupt",
        description: "An example using interrupts to toggle an LED after a button press",
        cFile: "gpiointerrupt.c",
        cfgFile: "gpiointerrupt.cfg",
        fileList:[
            {path: "gpiointerrupt_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "GPIO",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "GPIO Interrupt",
        name: "gpiointerrupt",
        description: "An example using interrupts to toggle an LED after a button press",
        cFile: "gpiointerrupt_msp430.c",
        cfgFile: "gpiointerrupt_msp430.cfg",
        fileList: [
            {path: "gpiointerrupt_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "GPIO",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
/*
 *
 ********* SPI EXAMPLES **********
 *
 */
    {
        title: "SPI Loopback",
        name: "spiloopback",
        description: "An example that performs an external SPI loopback with master and slave SPI",
        cFile: "spiloopback.c",
        cfgFile: "spiloopback.cfg",
        fileList: [
            {path: "spiloopback_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "SPI",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
//    {
//        title: "SPI AT45DB161D",
//        name: "spi_at45db161d",
//        description: "This example uses an Atmel AT45Db161D SPI NAND Flash. It performs a page write and read using the AT4DB SPI driver",
//        cFile: "spi_at45db161d.c",
//        cfgFile: "spi_at45db161d.cfg",
//        fileList: [
//            {path: "spi_at45db161d_readme.txt", targetDirectory: "."},
//            {path: "AT45DB.h", targetDirectory: "."},
//            {path: "AT45DB.c", targetDirectory: "."}
//        ],
//        requiredProducts: [],
//        options: "TREX",
//        type: "example",
//        group: "SPI",
//        tools: ["TI", "IAR"],
//        boards:[
//            Boards.MSP_EXP430F5529LP
//        ],
//    },
//    {
//        title: "SPI AT45DB161D FatFs",
//        name: "spi_at45db161dFatFs",
//        description: "This example uses an Atmel AT45Db161D SPI NAND Flash. It uses the AT45DBFatFs driver to implement a FatFs file system for creating, reading and writing files.",
//        cFile: "spi_at45db161dFatFs.c",
//        cfgFile: "spi_at45db161dFatFs.cfg",
//        fileList: [
//            {path: "spi_at45db161dFatFs_readme.txt", targetDirectory: "."},
//            {path: "AT45DB.h", targetDirectory: "."},
//            {path: "AT45DB.c", targetDirectory: "."},
//            {path: "AT45DBFatFs.h", targetDirectory: "."},
//            {path: "AT45DBFatFs.c", targetDirectory: "."},
//        ],
//        requiredProducts: [],
//        options: "TREX",
//        type: "example",
//        group: "SPI",
//        tools: ["TI", "IAR"],
//        boards:[
//            Boards.MSP_EXP430F5529LP
//        ],
//    },
/*
 *
 ********* I2C EXAMPLES **********
 *
 */
    {
        title: "I2C EEPROM",
        name: "i2ceeprom",
        description: "An example that performs page reads and writes to the onboard EEPROM",
        cFile: "i2ceeprom.c",
        cfgFile: "i2ceeprom.cfg",
        fileList: [
            {path: "i2ceeprom_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCKH52C1_M3
        ],
    },
    {
        title: "I2C TPL0401EVM",
        name: "i2ctpl0401evm",
        description: "An example on how to use the I2C driver with the TPL0401EVM boosterpack",
        cFile: "i2ctpl0401evm.c",
        cfgFile: "i2ctpl0401evm.cfg",
        fileList: [
            {path: "i2ctpl0401evm_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X,
        ],
    },
    {
        title: "I2C TMP006",
        name: "i2ctmp006",
        description: "An example on how to use the I2C driver with the TMP006 boosterpack",
        cFile: "i2ctmp006.c",
        cfgFile: "i2ctmp006.cfg",
        fileList: [
            {path: "i2ctmp006_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "I2C RF430CL330 Load",
        name: "i2crf430cl330_load",
        description: "An example NFC RF430CL330 example that sends the CPU Load",
        cFile: "i2crf430cl330_load.c",
        cfgFile: "i2crf430cl330_load.cfg",
        fileList: [
            {path: "i2crf430cl330_load_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X,
        ],
    },
    {
        title: "I2C TPL0401EVM",
        name: "i2ctpl0401evm",
        description: "An example on how to use the I2C driver with the TPL0401EVM boosterpack",
        cFile: "i2ctpl0401evm.c",
        cfgFile: "i2ctpl0401evm_msp430.cfg",
        fileList: [
            {path: "i2ctpl0401evm_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP
        ],
    },
    {
        title: "I2C TMP006",
        name: "i2ctmp006",
        description: "An example on how to use the I2C driver with the TMP006 boosterpack",
        cFile: "i2ctmp006.c",
        cfgFile: "i2ctmp006_msp430.cfg",
        fileList: [
            {path: "i2ctmp006_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP
        ],
    },
    {
        title: "I2C RF430CL330 Load",
        name: "i2crf430cl330_load",
        description: "An example NFC RF430CL330 example that sends the CPU Load",
        cFile: "i2crf430cl330_load.c",
        cfgFile: "i2crf430cl330_load_msp430.cfg",
        fileList: [
            {path: "i2crf430cl330_load_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "I2C",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
/*
 *
 ********* UART EXAMPLES **********
 *
 */
    {
        title: "UART Console",
        name: "uartconsole",
        description: "A basic console implemented with the UART driver and cstdio",
        cFile: "uartconsole.c",
        cfgFile: "uartconsole.cfg",
        fileList: [
            {path: "uartconsole_readme.txt", targetDirectory: "."},
            {path: "UARTUtils.c", targetDirectory: "."},
            {path: "UARTUtils.h", targetDirectory: "."},
            {path: "USBCDCD_LoggerIdle.c", targetDirectory: "."},
            {path: "USBCDCD_LoggerIdle.h", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "UART Echo",
        name: "uartecho",
        description: "Echos characters back using the UART driver",
        cFile: "uartecho.c",
        cfgFile: "uartecho.cfg",
        fileList: [
            {path: "uartecho_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "UART Logging",
        name: "uartlogging",
        description: "A simple example that sends Log data to the UART",
        cFile: "uartlogging.c",
        cfgFile: "uartlogging.cfg",
        fileList: [
            {path: "uartlogging_readme.txt",  targetDirectory: "."},
            {path: "UARTUtils.c", targetDirectory: "."},
            {path: "UARTUtils.h", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "UART Logging",
        name: "uartlogging",
        description: "A simple example that sends Log data to the UART",
        cFile: "uartlogging.c",
        cfgFile: "uartlogging_msp430.cfg",
        fileList: [
            {path: "uartlogging_readme.txt",  targetDirectory: "."},
            {path: "UARTUtils.c", targetDirectory: "."},
            {path: "UARTUtils.h", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI"],
        boards: [
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "UART Echo",
        name: "uartecho",
        description: "Echos characters back using the UART driver.",
        cFile: "uartecho.c",
        cfgFile: "uartecho_msp430.cfg",
        fileList: [
            {path: "uartecho_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI", "IAR"],
        boards: [
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "UART Echo",
        name: "uartecho",
        description: "Echos characters back using the UART driver.",
        cFile: "uartecho_115200.c",
        cfgFile: "uartecho.cfg",
        fileList: [
            {path: "uartecho_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G
        ],
    },
    {
        title: "UART Console",
        name: "uartconsole",
        description: "A basic console implemented with the UART driver and cstdio",
        cFile: "uartconsole.c",
        cfgFile: "uartconsole_msp430.cfg",
        fileList: msp430USBstackFiles.concat([
            {path: "uartconsole_readme.txt", targetDirectory: "."},
            {path: "UARTUtils.c", targetDirectory: "."},
            {path: "UARTUtils.h", targetDirectory: "."},
            {path: "usbmsp430/cdclogging/USBCDCD_LoggerIdle.c", targetDirectory: "."},
            {path: "usbmsp430/cdclogging/USBCDCD_LoggerIdle.h", targetDirectory: "."},

            /* USB_config */
            {path: "usbmsp430/cdclogging/USB_config/descriptors.c",    targetDirectory: "USB_config"},
            {path: "usbmsp430/cdclogging/USB_config/descriptors.h",    targetDirectory: "USB_config"},
            {path: "usbmsp430/cdclogging/USB_config/UsbIsr.c",         targetDirectory: "USB_config"},
            {path: "usbmsp430/cdclogging/USB_config/MSP430_CDC.inf",   targetDirectory: "USB_config"},
        ]),
        compilerBuildOptions: {
            TI: " -I${workspace_loc:/${ProjName}} -I${workspace_loc:/${ProjName}/USB_config}",
	        TI_makefile: " -I ./USB_config"
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "UART",
        tools: ["TI"],
        boards: [
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
/*
 *
 ********* USB EXAMPLES **********
 *
 */
    {
        title: "USB Keyboard Device",
        name: "usbkeyboarddevice",
        description: "USB HID Keyboard Device example that simluates a keyboard",
        cFile: "usbkeyboarddevice.c",
        cfgFile: "usbkeyboarddevice.cfg",
        fileList: [
            {path: "USBKBD.h", targetDirectory: "."},
            {path: "USBKBD.c", targetDirectory: "."},
            {path: "usbkeyboarddevice_readme.txt", targetDirectory: "."}
        ],
        isHybrid: true,
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "USB Mouse Device",
        name: "usbmousedevice",
        description: "USB HID Mouse Device example that moves a mouse cursor and simulates mouse clicks",
        cFile: "usbmousedevice.c",
        cfgFile: "usbmousedevice.cfg",
        fileList: [
            {path: "USBMD.h", targetDirectory: "."},
            {path: "USBMD.c", targetDirectory: "."},
            {path: "usbmousedevice_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "USB Serial Device",
        name: "usbserialdevice",
        description: "USB CDC Device that allows for serial communications using a virtual serial COM port",
        cFile: "usbserialdevice.c",
        cfgFile: "usbserialdevice.cfg",
        fileList: [
            {path: "USBCDCD.h", targetDirectory: "."},
            {path: "USBCDCD.c", targetDirectory: "."},
            {path: "usbserialdevice_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "USB Keyboard Host",
        name: "usbkeyboardhost",
        description: "USB HID Keyboard Host example that reads characters from a USB Keyboard",
        cFile: "usbkeyboardhost.c",
        cfgFile: "usbkeyboardhost.cfg",
        fileList: [
            {path: "USBKBH.h", targetDirectory: "."},
            {path: "USBKBH.c", targetDirectory: "."},
            {path: "usbkeyboardhost_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "USB Mouse Host",
        name: "usbmousehost",
        description: "USB HID Mouse Host example that displays the status of a USB Mouse",
        cFile: "usbmousehost.c",
        cfgFile: "usbmousehost.cfg",
        fileList: [
            {path: "USBMH.h", targetDirectory: "."},
            {path: "USBMH.c", targetDirectory: "."},
            {path: "usbmousehost_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR", "GNU"],
        boards: [
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "USB CDC Mouse Device",
        name: "usbcdcmousedevice",
        description: "USB composite example that features a CDC virtual serial port communications along with a HID Mouse device",
        cFile: "usbcdcmousedevice.c",
        cfgFile: "usbcdcmousedevice.cfg",
        fileList: [
            {path: "USBCDCMOUSE.h", targetDirectory: "."},
            {path: "USBCDCMOUSE.c", targetDirectory: "."},
            {path: "usbcdcmousedevice_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
        ],
    },
    {
        title: "USB Serial Device",
        name: "usbserialdevice",
        description: "USB CDC Device that allows for serial communications using a virtual serial COM port",
        cFile: "usbserialdevice.c",
        cfgFile: "usbserialdevice_msp430.cfg",
        fileList: msp430USBstackFiles.concat([
            {path: "usbmsp430/cdc/USBCDCD.h", targetDirectory: "."},
            {path: "usbmsp430/cdc/USBCDCD.c", targetDirectory: "."},
            {path: "usbserialdevice_readme.txt", targetDirectory: "."},

            /* USB_config */
            {path: "usbmsp430/cdc/USB_config/descriptors.c",    targetDirectory: "USB_config"},
            {path: "usbmsp430/cdc/USB_config/descriptors.h",    targetDirectory: "USB_config"},
            {path: "usbmsp430/cdc/USB_config/UsbIsr.c",         targetDirectory: "USB_config"},
            {path: "usbmsp430/cdc/USB_config/MSP430_CDC.inf",   targetDirectory: "USB_config"},
        ]),
        compilerBuildOptions: {
            TI: " -I${workspace_loc:/${ProjName}} -I${workspace_loc:/${ProjName}/USB_config}",
            TI_makefile: " -I ./USB_config",
            IAR_makefile: " -I ./USB_config"
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "USB Mouse Device",
        name: "usbmousedevice",
        description: "USB HID Mouse Device example that moves a mouse cursor and simulates mouse clicks",
        cFile: "usbmousedevice.c",
        cfgFile: "usbmousedevice_msp430.cfg",
        fileList: msp430USBstackFiles.concat([
            {path: "usbmsp430/hidm/USBMD.h", targetDirectory: "."},
            {path: "usbmsp430/hidm/USBMD.c", targetDirectory: "."},
            {path: "usbmousedevice_readme.txt", targetDirectory: "."},

            /* USB_config */
            {path: "usbmsp430/hidm/USB_config/descriptors.c",   targetDirectory: "USB_config"},
            {path: "usbmsp430/hidm/USB_config/descriptors.h",   targetDirectory: "USB_config"},
            {path: "usbmsp430/hidm/USB_config/UsbIsr.c",        targetDirectory: "USB_config"},
        ]),
        compilerBuildOptions: {
            TI: " -I${workspace_loc:/${ProjName}} -I${workspace_loc:/${ProjName}/USB_config}",
            TI_makefile: " -I ./USB_config",
            IAR_makefile: " -I ./USB_config"
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "USB Keyboard Device",
        name: "usbkeyboarddevice",
        description: "USB HID Keyboard Device example that simluates a keyboard",
        cFile: "usbkeyboarddevice.c",
        cfgFile: "usbkeyboarddevice_msp430.cfg",
        fileList: msp430USBstackFiles.concat([
            {path: "usbmsp430/hidk/USBKBD.h", targetDirectory: "."},
            {path: "usbmsp430/hidk/USBKBD.c", targetDirectory: "."},
            {path: "usbkeyboarddevice_readme.txt", targetDirectory: "."},

            /* USB_config */
            {path: "usbmsp430/hidk/USB_config/descriptors.c",   targetDirectory: "USB_config"},
            {path: "usbmsp430/hidk/USB_config/descriptors.h",   targetDirectory: "USB_config"},
            {path: "usbmsp430/hidk/USB_config/UsbIsr.c",        targetDirectory: "USB_config"},
        ]),
        compilerBuildOptions: {
            TI: " -I${workspace_loc:/${ProjName}} -I${workspace_loc:/${ProjName}/USB_config}",
            TI_makefile: " -I ./USB_config",
            IAR_makefile: " -I ./USB_config"
        },
        isHybrid: true,
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "USB CDC Mouse Device",
        name: "usbcdcmousedevice",
        description: "USB composite example that features a CDC virtual serial port communications along with a HID Mouse device",
        cFile: "usbcdcmousedevice.c",
        cfgFile: "usbcdcmousedevice_msp430.cfg",
        fileList: msp430USBstackFiles.concat([
            {path: "usbmsp430/cdchidm/USBCDCMOUSE.h", targetDirectory: "."},
            {path: "usbmsp430/cdchidm/USBCDCMOUSE.c", targetDirectory: "."},
            {path: "usbcdcmousedevice_readme.txt", targetDirectory: "."},

            /* USB_config */
            {path: "usbmsp430/cdchidm/USB_config/descriptors.c", targetDirectory: "USB_config"},
            {path: "usbmsp430/cdchidm/USB_config/descriptors.h", targetDirectory: "USB_config"},
            {path: "usbmsp430/cdchidm/USB_config/UsbIsr.c",      targetDirectory: "USB_config"},
            {path: "usbmsp430/cdchidm/USB_config/MSP430_CDC.inf",targetDirectory: "USB_config"},
        ]),
        compilerBuildOptions: {
            TI: " -I${workspace_loc:/${ProjName}} -I${workspace_loc:/${ProjName}/USB_config}",
            TI_makefile: " -I ./USB_config",
            IAR_makefile: " -I ./USB_config"
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "USB",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
/*
 *
 ********* Watchdog EXAMPLES **********
 *
 */
    {
        title: "Watchdog",
        name: "watchdog",
        description: "Watchdog Timer example that services a watchdog until a button press and release",
        cFile: "watchdog.c",
        cfgFile: "watchdog.cfg",
        fileList: [
            {path: "watchdog_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "Watchdog",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.TMDXDOCK28M36_M3,
            Boards.TMDXDOCKH52C1_M3,
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL,
            Boards.DK_TM4C129X
      ],
    },
    {
        title: "Watchdog",
        name: "watchdog",
        description: "Watchdog Timer example that services a watchdog until a button press and release",
        cFile: "watchdog_msp430.c",
        cfgFile: "watchdog_msp430.cfg",
        fileList: [
            {path: "watchdog_msp430_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "Watchdog",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
            Boards.MSP_EXP430F5529
        ],
    },
/*
 *
 ********* WiFi EXAMPLES **********
 *
 */
    {
        title: "TCP Echo for CC3000",
        name: "tcpEchoCC3000",
        description: "The TCP Echo example showing how to use sockets with the SimpleLink Wi-Fi CC3000",
        cFile: "tcpEchoCC3000.c",
        cfgFile: "tcpEchoCC3000.cfg",
        fileList: [
            {path: "tcpEchoCC3000_readme.txt", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            GNU: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        GNU_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL
        ],
    },
    {
        title: "UDP Echo for CC3000",
        name: "udpEchoCC3000",
        description: "The UDP Echo example showing how to use sockets with the SimpleLink Wi-Fi CC3000",
        cFile: "udpEchoCC3000.c",
        cfgFile: "udpEchoCC3000.cfg",
        fileList: [
            {path: "udpEchoCC3000_readme.txt", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            GNU: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        GNU_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL
        ],
    },
    {
        title: "CC3000 Patcher",
        name: "cc3000patcher",
        description: "An example that uses the WiFi driver to patch the CC3000 device with the recommended service pack",
        cFile: "cc3000patcher.c",
        cfgFile: "cc3000patcher.cfg",
        fileList: [
            {path: "cc3000patcher_readme.txt", targetDirectory: "."},
            {path: "cc3000patcharrays.h", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            GNU: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        GNU_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR", "GNU"],
        boards:[
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G,
            Boards.EK_TM4C123GXL,
            Boards.EK_LM4F120XL
        ],
    },
    {
        title: "TCP Echo for CC3000",
        name: "tcpEchoCC3000",
        description: "The TCP Echo example showing how to use sockets with the SimpleLink Wi-Fi CC3000",
        cFile: "tcpEchoCC3000.c",
        cfgFile: "tcpEchoCC3000_msp430f5529lp.cfg",
        fileList: [
            {path: "tcpEchoCC3000_msp430_readme.txt", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I $(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makfile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR"],
        boards: [
            Boards.MSP_EXP430F5529LP
        ],
    },
    {
        title: "TCP Echo for CC3000",
        name: "tcpEchoCC3000",
        description: "The TCP Echo example showing how to use sockets with the SimpleLink Wi-Fi CC3000",
        cFile: "tcpEchoCC3000.c",
        cfgFile: "tcpEchoCC3000_msp430f5529exp.cfg",
        fileList: [
            {path: "tcpEchoCC3000_msp430_readme.txt", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I $(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makfile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR"],
        boards: [
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "UDP Echo for CC3000",
        name: "udpEchoCC3000",
        description: "The UDP Echo example showing how to use sockets with the SimpleLink Wi-Fi CC3000",
        cFile: "udpEchoCC3000.c",
        cfgFile: "udpEchoCC3000_msp430f5529lp.cfg",
        fileList: [
            {path: "udpEchoCC3000_msp430_readme.txt", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I $(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP
        ],
    },
    {
        title: "UDP Echo for CC3000",
        name: "udpEchoCC3000",
        description: "The UDP Echo example showing how to use sockets with the SimpleLink Wi-Fi CC3000",
        cFile: "udpEchoCC3000.c",
        cfgFile: "udpEchoCC3000_msp430f5529exp.cfg",
        fileList: [
            {path: "udpEchoCC3000_msp430_readme.txt", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile: " -I $(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529
        ],
    },
    {
        title: "CC3000 Patcher",
        name: "cc3000patcher",
        description: "An example that uses the WiFi driver to patch the CC3000 device with the recommended service pack",
        cFile: "cc3000patcher.c",
        cfgFile: "cc3000patcher_msp430f5529lp.cfg",
        fileList: [
            {path: "cc3000patcher_readme.txt", targetDirectory: "."},
            {path: "cc3000patcharrays.h", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile:" -I $(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529LP,
        ],
    },
    {
        title: "CC3000 Patcher",
        name: "cc3000patcher",
        description: "An example that uses the WiFi driver to patch the CC3000 device with the recommended service pack",
        cFile: "cc3000patcher.c",
        cfgFile: "cc3000patcher_msp430f5529exp.cfg",
        fileList: [
            {path: "cc3000patcher_readme.txt", targetDirectory: "."},
            {path: "cc3000patcharrays.h", targetDirectory: "."}
        ],
        compilerBuildOptions: {
            TI: " -I${COM_TI_RTSC_TIRTOS_INSTALL_DIR}/packages/ti/drivers/wifi/cc3000",
	        TI_makefile:" -I $(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
            IAR_makefile: " -I$(TIRTOS_INSTALLATION_DIR)/packages/ti/drivers/wifi/cc3000",
        },
        requiredProducts: [],
        options: "TREX",
        type: "example",
        group: "WiFi",
        tools: ["TI", "IAR"],
        boards:[
            Boards.MSP_EXP430F5529
        ],
    },

/*
 *
 ********* DEMO PROJECTS **********
 *
 */
    {
        title: "Demo [M3]",
        name: "demo",
        description: "Demo showcasing SYS/BIOS, Networking, IPC and UIA.  Uses UART, USB, I2C, SDSPI, EMAC and GPIO.",
        cFile: "TMDXDOCKH52C1/demo/demo.c",
        cfgFile: "TMDXDOCKH52C1/demo/demo.cfg",
        fileList: [
            {path: "TMDXDOCKH52C1/demo/webpage.c", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/cmdline.c", targetDirectory: "."},
            {path: "USBCDCD_LoggerIdle.c", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/demo.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/logobar.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/dspchip.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/default.h", targetDirectory: "."},
            {path: "USBCDCD_LoggerIdle.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/jquery.min.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/jquery.flot.min.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/layout.css.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/demo_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        linkeroptions: " --stack_size=1024 --heap_size=0 --warn_sections --rom_model",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCKH52C1_M3
        ],
    },
    {
        title: "Demo [M3]",
        name: "demo",
        description: "Demo showcasing SYS/BIOS, Networking, IPC and UIA.  Uses UART, USB, I2C, SDSPI, EMAC and GPIO.",
        cFile: "TMDXDOCK28M36/demo/demo.c",
        cfgFile: "TMDXDOCK28M36/demo/demo.cfg",
        fileList: [
            {path: "TMDXDOCK28M36/demo/webpage.c", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/cmdline.c", targetDirectory: "."},
            {path: "USBCDCD_LoggerIdle.c", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/demo.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/logobar.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/dspchip.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/default.h", targetDirectory: "."},
            {path: "USBCDCD_LoggerIdle.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/jquery.min.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/jquery.flot.min.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/layout.css.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/demo_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        linkeroptions: " --stack_size=1024 --heap_size=0 --warn_sections --rom_model",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCK28M36_M3
        ],
    },
    {
        title: "Demo [C28]",
        name: "demo",
        description: "Demo showcasing SYS/BIOS, Networking, IPC and UIA.  Uses UART, USB, I2C, SDSPI, EMAC and GPIO.",
        cFile: "TMDXDOCKH52C1/demo/demo_c28.c",
        cfgFile: "TMDXDOCKH52C1/demo/demo_c28.cfg",
        fileList: [
            {path: "TMDXDOCKH52C1/demo/demo.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/demo_c28_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCKH52C1_C28
        ],
    },
    {
        title: "Demo [C28]",
        name: "demo",
        description: "Demo showcasing SYS/BIOS, Networking, IPC and UIA. Uses UART, USB, I2C, SDSPI, EMAC and GPIO.",
        cFile: "TMDXDOCK28M36/demo/demo_c28.c",
        cfgFile: "TMDXDOCK28M36/demo/demo_c28.cfg",
        fileList: [
            {path: "TMDXDOCK28M36/demo/demo.h", targetDirectory: "."},
            {path: "TMDXDOCK28M36/demo/demo_c28_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCK28M36_C28
        ],
    },
    {
        title: "IPC SPI Slave",
        name: "spislave",
        description: "Slave side of the MessageQ over SPI example.",
        cFile: "TMDXDOCKH52C1/demo/spislave.c",
        cfgFile: "TMDXDOCKH52C1/demo/spislave.cfg",
        fileList: [
            {path: "TMDXDOCKH52C1/demo/spiDemo.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/spislave_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCKH52C1_M3
        ],
    },
    {
        title: "IPC SPI Master",
        name: "spimaster",
        description: "Master side of the MessageQ over SPI example.",
        cFile: "TMDXDOCKH52C1/demo/spimaster.c",
        cfgFile: "TMDXDOCKH52C1/demo/spimaster.cfg",
        fileList: [
            {path: "TMDXDOCKH52C1/demo/spiDemo.h", targetDirectory: "."},
            {path: "TMDXDOCKH52C1/demo/spimaster_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        tools: ["TI"],
        boards:[
            Boards.TMDXDOCKH52C1_M3
        ],
    },
    {
        title: "Graphic Library Demo",
        name: "grlibdemo",
        description: "Display picture/text on the onboard LED using the graphic library.",
        cFile: "DK_TM4C123G/demo/grlibdemo.c",
        cfgFile: "DK_TM4C123G/demo/grlibdemo.cfg",
        fileList: [
            {path: "DK_TM4C123G/demo/cfal96x64x16.h", targetDirectory: "."},
            {path: "DK_TM4C123G/demo/cfal96x64x16.c", targetDirectory: "."},
            {path: "DK_TM4C123G/demo/images.c", targetDirectory: "."},
            {path: "USBCDCD.h", targetDirectory: "."},
            {path: "USBCDCD.c", targetDirectory: "."},
            {path: "DK_TM4C123G/demo/grlibdemo_readme.txt", targetDirectory: "."}
        ],
        requiredProducts: [],
        options: "TREX",
        type: "Demo",
        tools: ["TI"],
        boards:[
            Boards.EKS_LM4F232,
            Boards.DK_TM4C123G
        ],
    },
];

/*
var allExamples = [
    emptyExamples,
    ethernetExamples,
    fatfsExamples,
    gpioExamples,
    spiExamples,
    i2cExamples,
    ipcExamples,
    uartExamples,
    usbExamples,
    watchdogExamples,
    wifiExamples,
    demoProjects
];*/

function supportsBoard(example, board) {
    for(var i = 0; i < example.boards.length; i++) {
        if(example.boards[i].name == board.name)
            return true;
    }
    return false;
}

function supportsTool(example, tool){
    for(var i = 0; i < example.tools.length; i++){
        if(example.tools[i] == tool)
            return true;
    }
    return false;
}
