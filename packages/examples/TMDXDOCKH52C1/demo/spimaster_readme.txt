Example Summary
----------------
This example uses the SPI driver to demonstrate SPI communication in master
and slave modes. Two examples are provided, one to run on the master processor
and the other on the slave processor.

Peripherals Exercised
---------------------
TMDXDOCKH52C1_LD2   Message flow indicator
TMDXDOCKH52C1_SPI0  SPI channel for messages

Please refer to the Getting Started Guide's Board section to get details
regarding on the location of the buttons, LEDs, USB connections, and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
The following pins must be connected between the two boards.

                    Master (SSI0)    Slave (SSI1)
SCK (Clock)             02              24
_CS (Slave select)      03              25
MOSI                    04 (MO)         27 (SI)
MISO                    05 (MI)         26 (SO)
GND                     Need a common ground

It is up to the user to load and run the slave first and then the master.

The master M3 sends a MessageQ msg to the slave every millisecond. The slave
receives the message and discards it. Both processors toggle the
TMDXDOCKH52C1_LD2 LED every 250 messages.

Please refer to the following URL for more details on how to load and run
the examples in CCS:
http://processors.wiki.ti.com/index.php/TI-RTOS_Debugging_Multiple_Boards

Application Design Details
--------------------------
The example takes advantage of the new MessageQ_create2 API. This allows the
caller to specify a queueIndex to be used. Then the sender uses
MessageQ_openQueueId() instead of MessageQ_open(). One restriction is that
the receiver (i.e. slave) must be running with the message queue created before
the sender (i.e. master) starts to run.

This example does not handle flow control. In a real system, this could be
accomplished via another GPIO line that the slave asserts. The master would
either poll on this pin (or use the interrupt capability of the TI-RTOS
GPIO module).
