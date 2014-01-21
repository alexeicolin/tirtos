Example Summary
----------------
Sample application to connect to a USB host as a mouse HID device.

Peripherals Exercised
---------------------
Board_LED0      Indicator LED
Board_BUTTON0   Used to simulate a primary mouse button
Board_USBDEVICE Used as HID Mouse device*

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
This application uses one tasks:

  'mouse' performs the following actions:
      Waits for the device to connected to a USB host.

      Once connected it sends predefined mouse offsets (from the
      mouseLookupTable[]) along with primary click status.
