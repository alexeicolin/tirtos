Example Summary
----------------
Sample application to communicate with the I2C peripherals on a TPL0401EVM
Booster Pack.

Peripherals Exercised
---------------------
Board_LED0         Indicator LED
Board_LED1         Indicator LED
Board_I2C_TPL0401  I2C used to communicate with I2C peripherals on TPL0401EVM.*

*  This example was designed to use the TPL0401 Booster Pack (TPL0401EVM).  The
   booster pack is required to successfully complete this example.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Connect the TPL0401EVM Booster Pack before powering the hardware.

Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

The TLC59108 will read the color patterns in the rgbcmd variable and toggle
the RGB LEDs ON/OFF.  Board_LED1 will turn ON if there has been a problem
with I2C communication.

Application Design Details
--------------------------
This application uses one tasks:
  'cycleLED' performs the following actions:
      Opens and initializes an I2C driver object.

      Uses the I2C driver to initialize the PWM oscillator on the TLC59108 and
      set the LED outputs to PWM mode.

      A RGBCMD structure (contains LED address and individual color values for
      Red, Green and Blue) is read from the rgbcmd array and sent via I2C driver
      to the TLC59108.

      If the end of the array is reached, the index is reset to 0 and the Task
      sleeps for 100 system ticks before starting the next iteration.
