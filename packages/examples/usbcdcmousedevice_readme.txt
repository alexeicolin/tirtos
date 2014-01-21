Example Summary
----------------
Sample application to send a string to a USB host as a mouse HID device.

Peripherals Exercised
---------------------
Board_LED0      Indicator LED
Board_BUTTON0   Used to simulate the primary button
Board_USBDEVICE Used as HID Mouse and CDC device*

*  Please ensure that the board is connected to your host via a USB cable.
   Depending on your board, this may require a second USB connection.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

The example acts as a mouse to your host; it moves the cursor in a figure eight
pattern.  Pressing Board_BUTTON0 performs a primary click.

When the application is running, pen a serial session (e.g. HyperTerminal,
puTTY, etc.) to the appropriate COM port. Note: the COM port can be determine
via Device Manager in Windows or via ls /dev/tty* in Linux.

Once the connection is made, the board transmits the following text every
two seconds:

    "TI-RTOS controls USB.\r\n"

The Board_LED0 is toggled whenever a transmission occurs.

Board_LED1 toggles once character(s)* are received. Typed characters are printed
to the SysMin internal buffer. These can be viewed by halting the target and
looking at SysMin in ROV.

*  The MSP430 collects 31 characters before returning. The Tiva and Concerto
   family devices collect a single character before returning.

** Note characters typed into the serial session are not echoed back, so you
   will not see them (unless you enable echo on the host).

USB drivers can be found at the following locations:
    Tiva USB Drivers:
    Windows USB drivers are located in the products directory:
    <tirtos_install_dir>\products\<TivaWare_install_dir>\windows_drivers

    MSP430 USB Drivers:
    Windows USB drivers are located within the CCS project's USB_config/
    directory.

    Concerto Family USB Drivers:
    Windows USB drivers are located in the products directory:
    <tirtos_install_dir>\products\<MWare_install_dir>\windows_drivers

Application Design Details
--------------------------
This application uses three tasks:

  'mouse' performs the following actions:
      Waits for the device to connected to a USB host.

      Once connected it sends predefined mouse offsets (from the
      mouseLookupTable[]) along with primary click status.

  'transmit' performs the following actions:
      Determine if the device is connected to a USB host.

      If connected, periodically sends an array of characters to the USB host.

  'receive' performs the following actions:
      Determine if the device is connected to a USB host.

      If connected, it prints, via System_printf, any received data and the
      number of bytes. SysMin is used for this example, so the output goes into
      its internal buffer.

The tasks are at different priorities to show the functionality. The mouse Task
is the lowest to make sure it does not impact the other tasks.