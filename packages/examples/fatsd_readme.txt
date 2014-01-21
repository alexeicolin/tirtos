Example Summary
----------------
Sample application to read and write data onto a SD Card (SPI interface)

Board Overview
--------------
Board_LED0    Indicates that the board was initialized within main()
Board_SDSPI0  Connection to SD card

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs and any additional settings
(e.g. jumpers) for your specific board.

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
    "***********************************************************************\n"

The "input.txt" file is then read and it's contents are written to a new file
called "output.txt". If the file already exists on the SD card, it will be
overwritten.

The contents of the "output.txt" file are then written to the console.

Application Design Details
--------------------------
This application demonstrates how to use TI-RTOS's SD Card driver to read and
write data onto a SD Card using standard CIO runtime system calls
(fopen, fread, fwrite, etc...).

This application uses SysStd instead of SysMin. This was done because
real-time was not a concern and the size of the internal SysMin had to
be large to hold the output. Please refer to the TI-RTOS User Guide's
"Generating printf Output" for a comparison of the different System
Support implementations.

This application uses one task:
  'fatSDTask' performs the following actions:
      Create and initialize SDSPI driver object.

      Open the "input.txt" file.  If not found, create the file for reading and
      writing.  Write the default message to the file.

      Open the "output.txt" file.  Content will be overwritten if found.

      Copy the contents of "input.txt" to "output.txt".  Close both files
      afterwards.

      Open "output.txt", read the file and print the contents to System_printf.

      The "output.txt" file and SDSPI driver are closed and the example is
      terminated.