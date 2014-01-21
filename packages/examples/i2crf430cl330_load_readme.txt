Example Summary
----------------
Sample application to read CPU load using a RF430CL330H I2C NFC transponder.

Peripherals Exercised
---------------------
Board_LED0      Indicator LED
Board_LED1      Indicator LED
Board_LED2      Indicator LED
Board_I2C_NFC   I2C used to communicate with RF430CL330H NFC transponder*.

*  This example was designed to use the RF430CL330H Booster Pack
   (RF430CL330HTB - RF430CL330H NFC Dynamic Tag Target Board).  The booster pack
   is required to successfully complete this example.

** A NFC transceiver (such as a NFC enabled smart-phone) is required to recover
   CPU load data.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Connect the RF430CL330H Booster Pack before powering the hardware.

Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

Once initialized, Board_LED0 will be turned OFF.  From this point onward
Board_LED0, Board_LED1, and Board_LED2 will blink randomly to show example is
executing.  When a LED is toggled, a message is printed to the internal SysMin
buffer.  These can be viewed by stopping the target and opening in SysMin in ROV.

Hold the NFC transceiver directly above the NFC transponder. The
NFC transceiver will receive the following message:

    "CPU Load: x"

Where "x" is the current host processor CPU load.  The example will periodically
update the CPU load until the target is halted and example is terminated.

Application Design Details
--------------------------
This application uses four tasks:

nfcLoadTask  is used to update CPU load messages on the RF430CL330H device.
dummyTask    is used to randomly toggle the indicator LEDs Board_LED0,
             Board_LED1, and Board_LED2.  This function is executed in three
             statically created tasks (one for each LED).

  'nfcLoadTask' performs the following actions:
      Opens and initializes an I2C driver object.

      Uses the I2C driver to reset and initialize the RF430CL330H device.

      The task then clears the RF430CL330H control register to disable RF
      communication.

      The message to be transmitted is composed using System_sprintf.  The CPU
      load is provided by the SYS/BIOS Load module.

      The I2C driver is used to write the message to the NFC transponder.

      nfcLoadTask sleeps for 1000 system ticks before executing again and
      updating the CPU load message.

  'dummyTask' performs the following actions:
      LED index is provided in arg0.  The index is used to access a single entry
      in the dummyCfg array.

      Sleeps for a randomly determined amount of system ticks.  The value is
      limited by the ticks member of the DUMMYTASKCFG structure.

      The LED is toggled.  A message with the Task sleep and count is printed
      to System_printf.
