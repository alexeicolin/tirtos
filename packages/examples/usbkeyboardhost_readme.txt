Example Summary
----------------
Sample application to receive characters from a keyboard HID device.

Peripherals Exercised
---------------------
Board_LED0      Caps Lock Indicator LED
Board_LED1      Scroll Lock Indicator LED
Board_USBHOST   Used as HID keyboard host*

*  Please ensure that the HID keyboard device is connected to your board on the
   appropriate USB Host port.

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

Once the enumeration occurs on the host, Board_LED0 and Board_LED1 to show
Caps Lock and Scroll lock status respectively. For example, press Caps Lock on
your keyboard and Board_LED0 will toggle.  The Host then updates the status LEDs
on the HID keyboard (if any).

Keys pressed on the HID Keyboard are read by the Host and are printed to the
SysMin internal buffer.  These can be viewed by stopping the target and looking
at SysMin in ROV.

Application Design Details
--------------------------
This application uses one tasks:

  'keyboardHostTask' performs the following actions:
      Waits for a HID keyboard device to be connected to the USB host port.

      Gets the status of the keyboard's buttons and updates the LEDs accordingly.

      If USEGETCHAR is defined in the example, characters received from the
      keyboard are printed directly to System_printf.

      Otherwise, keyboard inputs are printed as character arrays delimited by
      the keyboard's Enter key (a <LF> character).
