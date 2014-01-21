Example Summary
----------------
Application that uses the graphics library to display image/text on the on-board
LCD. It also shows SD card interaction.

Peripherals Exercised
---------------------
DK_TM4C123G_LED          Indicates that the board was initialized within main()
DK_TM4C123G_SW3          Displays Texas Instruments Logo
DK_TM4C123G_SW4          Displays an image of a small country scene
DK_TM4C123G_SDSPI0       Connection to SD card
DK_TM4C123G_USBDEVICE    Used for serial communication*

*  Please ensure that the board is connected to your host with a second USB
   cable.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections, and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. DK_TM4C123G_LED turns ON to indicate TI-RTOS driver
initialization is complete.

Once the example is running if the DK_TM4C123G_SW3 (LEFT) button is pressed, the
TI Logo is displayed on the on-board LCD. If the DK_TM4C123G_SW4 (RIGHT) button
is pressed, an image of a small country scene is displayed.

When the application is running, open a serial session (e.g. HyperTerminal,
puTTY, etc.) to the appropriate COM port. Note: the COM port can be determined
via Device Manager in Windows or via ls /dev/tty* in Linux.

The characters typed into this session are displayed on the on-board LCD (once
you hit ENTER). If you type in "ls" and hit ENTER, a list of the files on the
SD card are displayed on the LCD.

This example also demonstrates System Analyzer in CCS. This is accomplished
via stop-mode reading of the logs on the target. Halt the target and open
System Analyzer as described in the TI-RTOS User Guide's "Viewing the Logs".

The "Live Session" should have records like the following
   - "LS_cpuLoad: 0%"
   - "LD_block: tsk: 0x20000650, func: 0x3525"

Application Design Details
--------------------------
This example shows the graphics library being used in a task.  Other tasks and
interrupts communicate with the graphics task; using the mailbox module; to
change what is displayed on the LCD.

There is a keyboard listening task which receives keyboard commands through
USB CDC and sends them to the mailbox for displaying. The GPIO interrupts also
sent an "image message" is sent to the mailbox.

To generate different images, please refer to readme.txt at TIRTOS installation
directory/products/StellarisWare_####/tools/pnmtoc.
