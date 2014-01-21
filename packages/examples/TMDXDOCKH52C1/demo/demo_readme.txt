Example Summary
----------------
Demo (kitchen-sink) TI-RTOS example. Requires both the M3 and C28 sides
to be running.

Peripherals Exercised
---------------------
TMDXDOCKH52C1_LD2       Indicates that the board was initialized within main()
                        Also is controlled in the UART console
TMDXDOCKH52C1_LD3       Indicates that the SD is being written to
TMDXDOCKH52C1_USBDEVICE Connection to send out Log records*
TMDXDOCKH52C1_I2C1      I2C to read external temperature**
TMDXDOCKH52C1_EMAC0     Connection to HTTP server on target
Board_UART0             Connection to the host "console"
Board_SDSPI0            Connection to SD card

*  Please ensure that the board is connected to your host via a USB cable.
   This requires a second USB connection.

** External Temperature (Optional)
   An external Texas Instruments TMP102 I2C temperature sensor can be used
   and needs to be connected in the following manner:
       TMP102's SDA - Docking Station's GPIO01
       TMP102's SCL - Docking Station's GPIO00
       TMP102's ADD0 - GND
   A TMP102 I2C Temperature breakout board can be found at:
       http://www.sparkfun.com/products/9418
       http://www.ti.com/tool/tmp102evm

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections, EMAC
address and any additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
By default the M3 example does not use the optional external temperature sensor.
If you have one and want to use it, you must connect it (as described above)
and change the '0' to a '1' in following line in demo.c

    #define EXTERNALTEMP 0

Build, load and run both the M3 and C28 Demo examples.

The M3 turns ON TMDXDOCKH52C1_LD2 and starts the network stack. When the stack
receives an IP address from a DHCP server, the IP address is written to the
console.

You can point a browser to the board (e.g. simply add the IP address into the
browser). The application will give you a web page where you can select different
requests (e.g. see CPU load, Task status, etc). It also shows the current
temperature in both Celsius and Fahrenheit (the graph is not visible on older
browsers namely IE 7). If the external temperature sensor is not used, a
sawtooth pattern is displayed instead.

You can select, via the web page, to write the temperature values to an micro SD
card. Note: when you enable the writing to the SD card, TMDXDOCKH52C1_LD3 is
turned on. When you disable it, TMDXDOCKH52C1_LD3 is turned off.

Please note, if the SD card is not present, the example terminates with an error
message to the console.

When the application is running, open a serial session (e.g. HyperTerminal,
puTTY, etc.) to the appropriate COM port. Note: the COM port can be determine
via Device Manager in Windows or via ls /dev/tty* in Linux.

The connection should have the following settings
    Baud-rate:   115200
    Data bits:       8
    Stop bits:       1
    Parity:       None
    Flow Control: None

The serial session is now a console to the target. You can get type 'h' to
get help.
    Available commands
    ------------------
    help  : Display list of commands
    h     : alias for help
    ?     : alias for help
    led   : Set LED mode (on|off|toggle|activity)

The LED being controlled is TMDXDOCKH52C1_LD2.  "activity" means
TMDXDOCKH52C1_LD2 toggles every time there is a RETURN entered in the serial
session.

This example also demonstrates System Analyzer in CCS. This is accomplished
by sending out the logs records via USB from the target to the host. Please
refer to TI-RTOS User Guide's for more details.

Application Design Details
--------------------------
The Celsius temperature is converted to Fahrenheit by the C28 via MessageQ.
The temperature reading is provided by an external I2C TMP102 chip.

Note: there is a temperature sensor on the C28, but we wanted to demonstrate
the I2C on the M3.

Here are the following task and their functionality:

Temperature Task  Reads temperature sensor via I2C.
                  Sends temperature value to the C28 via MessageQ. It also
                  writes to the filesystem if instructed to via the web page.

Serial Task       Opens Board_UART0 and communicates with a host serial
                  application. It provides a "console" to the user.

NDK Stack Thread  Main Networking Task

DHCPclient        Task that renews IP address

IdleTask          Kernel Idle task. Moves Log records to System Analyzer via
                  USB.

daemon            HTTP Server. Uses jquery for the graph. Please refer to
                  http://processors.wiki.ti.com/index.php/TI-RTOS_HTTP_Example
                  for details on how to make a HTTP server.

Note the M3 application is large because the web pages are stored in flash
(especially the jquery code). The URL above discusses how to have the web pages
on the SD card instead.
