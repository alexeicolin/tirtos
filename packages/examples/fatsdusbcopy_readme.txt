Example Summary
----------------
Sample application to read data from a SD Card (SPI interface) and onto a
USB flash drive (USB Host Mass Storage Class).

Peripherals Exercised
---------------------
Board_LED0           Indicates that the board was initialized within main()
Board_SDSPI0         Interface to communicate with SD Card
Board_USBMSCHFatFs0  USB mass storage driver instance

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any additional
settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver initialization
is complete.

The example proceeds to read the SD card. If an "input.txt" file is not found,
the file is created on the SD card with the following text:

    "***********************************************************************\n"
    "0         1         2         3         4         5         6         7\n"
    "01234567890123456789012345678901234567890123456789012345678901234567890\n"
    "This is some text to be inserted into the inputfile if there isn't     \n"
    "already an existing file located on the SDCard.                        \n"
    "If an inputfile already exists, or if the file was already once        \n"
    "generated, then the inputfile will NOT be modified.                    \n"
    "\n"
    "This output file is copied from the SDCard to the USB Thumb Drive      \n"
    "End of Demo. Now, get back to work!                                    \n"
    "***********************************************************************\n"

The "input.txt" file on the SD card is read and it's contents are written to a
new file called "output.txt" on the Flash Drive. If the file already exists on
the Flash Drive, it will be overwritten.

The contents of the "output.txt" file are then written to the console.

Application Design Details
--------------------------
This application demonstrates how to use TI-RTOS's SD Card driver to read and
write data onto a SD Card or a USB Flash Drive using C I/O API calls
(fopen, fread, fwrite, etc...).

This application uses SysStd instead of SysMin. This was done because
real-time was not a concern and the size of the internal SysMin had to
be large to hold the output. Please refer to the TI-RTOS User Guide's
"Generating printf Output" for a comparison of the different System
Support implementations.

The TMDXDOCKH52C1 development board has a KNOWN issue with USB HOST operation.
If you are using Rev 1.0, you must remove AND short R230 on the control card!!
Please refer to the Getting Started Guide's Board section for more details.

This application uses one task:
  'fatsdUSBCopyTask' performs the following actions:
      Create and initialize SDSPI and USBMSCHFatFS driver objects.

      Block execution until the USB Flash Drive is enumerated.

      Open the "input.txt" file on the SD Card.  If not found, create the file
      for reading and writing.  Write the default message to the file.

      Open a "output.txt" file on the USB Drive.  Content will be overwritten
      if found.

      Copy the contents of "input.txt" on the SD Card to "output.txt" on the
      USB Drive.  Close both files afterwards.

      Open "output.txt", read the file and print the contents to System_printf.

      The "output.txt" file, SDSPI driver, and USBMSCHFatFS driver are closed
      and the example is terminated.
