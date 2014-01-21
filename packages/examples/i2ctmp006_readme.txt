Example Summary
----------------
Sample application to read temperature value from a TMP006 I2C temperature
sensor.

Peripherals Exercised
---------------------
Board_LED0      Indicator LED
Board_I2C_TMP   I2C used to communicate with TMP006 sensor*.

*This example was designed to use the TMP006 Booster Pack (430BOOST-TMP006).
 The booster pack is required to successfully complete this example.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Connect the TMP006 Booster Pack before powering the hardware.

Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

The example will request temperature samples from the TMP006 and print them on
the console.  A total of 20 temperature samples are read/printed before the
example exits.  Console output should resemble:

        I2C Initialized!
        Sample 0: 24 (C)
        Sample 1: 24 (C)
        Sample 2: 24 (C)
        Sample 3: 24 (C)

Application Design Details
--------------------------
This application uses one tasks:
  'getTempTask' performs the following actions:
      Opens and initializes an I2C driver object.

      Uses the I2C driver to get data from the TMP006 sensor.

      Extracts the temperature (in Celsius) and prints the value on the console.

      The task sleeps for 1000 system ticks.

      After 20 temperature samples are recovered, the I2C peripheral is closed
      and the example exits.
