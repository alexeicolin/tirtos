Example Summary
----------------
Example that uses the UART driver to echo back to the console.  It also
demonstrates System Analyzer usage via a stop-mode logger.

Peripherals Exercised
---------------------
Board_LED0      Indicates that the board was initialized within main()
Board_UART0     Used to echo characters from host serial session

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

When the application is running, open a serial session (e.g. HyperTerminal,
puTTY, etc.) to the appropriate COM port. Note: the COM port can be determine
via Device Manager in Windows or via ls /dev/tty* in Linux.

The connection should have the following settings
    Baud-rate:    9600  (the DK_TM4C123G/EKS_LM4F232 example uses 115200)
    Data bits:       8
    Stop bits:       1
    Parity:       None
    Flow Control: None

The target echoes back any character that is typed in the serial session.

If the serial session is started before the target completes initialization,
the following is displayed:
    "Echoing characters:"

This example also demonstrates System Analyzer in CCS. This is accomplished
via stop-mode reading of the logs on the target. Halt the target and open
System Analyzer as described in the TI-RTOS User Guide's "Viewing the Logs".

The "Live Session" should have record like the following
   - "LS_cpuLoad: 0%"
   - "Wrote character 0xa"

Application Design Details
--------------------------
This example shows how to initialize the UART driver in blocking read and write
mode with no data processing and echo characters back to a console.

A single task, "echo", reads a character from Board_UART0 and writes it back.

This example uses the default StopMode logger with LoggingSetup. Log records
are stored in internal buffers. They are read by System Analyzer when the target
is halted. If the target is not halted and the buffers are full, the old records
are over-written.

To help fill up the logs, the kernel is configured to generate Log records
via the following from the .cfg file
    BIOS.logsEnabled = true;

The drivers by default are instrumented. So the following pulls in the
instrumented UART and GPIO libraries
    var GPIO = xdc.useModule('ti.drivers.GPIO');
    var UART = xdc.useModule('ti.drivers.UART');

Note: most of the other examples do not use instrumented libraries to improve
performance and minimize footprint.
