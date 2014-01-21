Example Summary
----------------
Sample application to read and write onto an on-board I2C EEPROM IC.

Peripherals Exercised
---------------------
Board_LED0        Indicator LED
Board_I2C_EEPROM  I2C used to communicate with I2C EEPROM.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

A confirmation message will be printed on System_printf after the EEPROM is
erased.

A page with incremental values is written to the EEPROM.  The memory is compared
to ensure the data was written correctly.  A confirmation message will
be written to System_printf.  The following message should appear:

    "Page was successfully written with data\n"

The example exits afterwards.

Application Design Details
--------------------------
This application uses one task:
  'eepromTask' performs the following actions:
      Create and initialize I2C driver object.

      Clear EEPROM memory by writing 0xFF's to memory.

      Wait for the I2C EEPROM to acknowledge, indicating the page write
      operation was completed.

      Read the EEPROM page and verify was erased.

      Write incrementing data into the "Erased" page.

      Wait for the I2C EEPROM to acknowledge, indicating the page write
      operation has completed.

      Read the EEPROM page to verify it was written correctly.

      Close the I2C object and terminate execution.
