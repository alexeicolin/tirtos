Example Summary
----------------
Application that uses the WiFi driver to patch the CC3000 device.

Example Usage
-------------
This application provides an easy way to load the CC3000 with the version of
the CC3000 Service Pack with which the TI-RTOS WiFi driver has been tested and
verified. The application uses the SP reprogramming technique that uses CC3000
Host Driver APIs to write to the EEPROM. Any user files on the EEPROM will be
deleted.

Peripherals Exercised
---------------------
Board_LED0        Caps Lock indicator LED
Board_WIFI        Wireless driver instance
Board_SPI_CC3000  Interface to communicate with CC3000

*  This example was designed to use a CC3000 evaluation module (CC3000BOOST or
   CC3000EM).  The booster pack is required to successfully complete this
   example.  A wireless router or access point (AP) is also required for this
   example.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
This application loads the CC3000 with the version of the CC3000 Service Pack
with which the TI-RTOS WiFi driver has been tested and verified.
The application uses CC3000 Host Driver APIs to write to the EEPROM. Any user
files on the EEPROM will be deleted.

Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

Afterwards, Board_LED0 is turned OFF and used as patch status indicator.  A
WiFi driver object is created and initialized.

The current Service Pack (SP) version is read from the EEPROM.  If already loaded
with the proper SP, Board_LED0 is turned ON and the application will exit
without patching the device. Otherwise, the Board_LED0 will turn ON and OFF twice
and then remain ON, indicating that the process has completed.

Allow at least 20 seconds for it to finish.

Application Design Details
--------------------------
This application uses one tasks:

*  This application uses SysStd instead of SysMin. This was done because
   real-time was not a concern and the size of the internal SysMin had to
   be large to hold the output. Please refer to the TI-RTOS User Guide's
   "Generating printf Output" for a comparison of the different System
   Support implementations.

  'patchTask' performs the following actions:
      Opens and initializes a WIFI driver object.

      Verifies the firmware version on the CC3000.  If already loaded with proper
      service pack, A message is printed to System_printf, Board_LED0 is turned
      ON and the example is terminated.

      Otherwise, the CC3000 is restarted and all interrupt events are masked.

      The MAC address and radio parameters are read from the EEPROM.  If radio
      parameters are valid, they will be restored later.  Otherwise they will
      be replaced with defaults.

      A new FAT file system is written to the EEPROM.  Board_LED0 is blinked for
      a second before writing the radio parameters and MAC address back to the
      EEPROM.

      The driver patch and firmware are written to the EEPROM.

      Finally, Board_LED0 is turned ON, the WiFi driver object is closed and
      the example is terminated after printing a patch complete message on
      System_printf.
