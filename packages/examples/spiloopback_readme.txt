Example Summary
----------------
Application to demonstrate use of the SPI driver to create a simple external
loop-back.

Peripherals Exercised
---------------------
Board_LED0  Indicator LED
Board_SPI0  SPI peripheral assigned as Master
Board_SPI1  SPI peripheral assigned as Slave

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

Once initialized, the Slave SPI will send a message to the Master SPI.
The Master SPI will also send a message to the Slave SPI.  After the transfer is
complete, the messages are printed on the console.  Messages should appear as
follows:

        SPI initialized
        SPI initialized
        Slave: Hello, this is master SPI
        Master: Hello, this is slave SPI
        Done

Application Design Details
--------------------------
This application uses two tasks:

masterTaskFxn  creates the Master SPI message and initiate the SPI transfer.
slaveTaskFxn   creates the Slave SPI message and waits for the Master to
               start the SPI transaction.  This task runs at a higher priority
               than the masterTaskFxn, the Slave SPI must be ready before the
               Master SPI starts the transaction.

  'masterTask' performs the following actions:
      Opens and initializes an SPI driver object.

      Creates a SPI transaction structure and sets txBuffer to "Hello, this is
      master SPI".

      Transfers the message.  If the transfer is successful, the message received
      from the Slave SPI is printed.  Otherwise, an error message is printed.

      Closes the SPI driver object and terminates execution.

  'slaveTask' performs the following actions:
      Opens and initializes an SPI driver object.

      Creates a SPI transaction structure and sets txBuffer to "Hello, this is
      slave SPI".

      Waits for the Master SPI before sending the message.  If the transfer is
      successful, the message received from the Master SPI is printed.
      Otherwise, an error message is printed.

      Closes the SPI driver object.
