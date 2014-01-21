Example Summary
----------------
Sample application to get updates from a mouse HID device.

Peripherals Exercised
---------------------
Board_LED0      Primary Button Indicator LED
Board_LED1      Secondary Button Indicator LED
Board_USBHOST   Used as HID mouse host*

*  Please ensure that the HID mouse device is connected to your board on the
   appropriate USB host port.

** The TMDXDOCKH52C1 development board has a KNOWN issue with USB HOST
   operation.  If you are using Rev 1.0, you must remove AND short R230 from
   the control card!!

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

Once the enumeration occurs, Board_LED0 and Board_LED1 to show the status
of the Primary and Secondary HID mouse buttons respectively. For example, if
the Primary button is pressed Board_LED0 will turn ON.  Once Primary button is
release, Board_LED0 will turn OFF.

The HID mouse's movements are tracked as offsets.  The offsets are printed to
the SysMin internal buffer.  These can  be viewed by stopping the target and
looking at SysMin in ROV.

Application Design Details
--------------------------
This application uses one tasks:
  'mouseHostTask' performs the following actions:
      Waits for a HID mouse device to be connected to the USB host port.

      Gets the status of the device's buttons and updates the LEDs accordingly.

      Prints the mouse's movement offsets via System_printf.  SysMin is used
      for this example; the output is redirected to the internal buffer.
