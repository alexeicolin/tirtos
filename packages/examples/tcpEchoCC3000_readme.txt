Example Summary
----------------
Application that uses the WiFi driver and CC3000 device to echo TCP packets.

Peripherals Exercised
---------------------
Board_LED0        Caps Lock Indicator LED
Board_WIFI        Wireless driver instance
Board_SPI_CC3000  Interface to communicate with CC3000
Board_BUTTON0     Used to put CC3000 in Smart Config mode

*  This example was designed to use a CC3000 evaluation module (CC3000BOOST or
   CC3000EM).  The booster pack is required to successfully complete this
   example.  A wireless router or access point (AP) is also required for this
   example.

** This example uses the CC3000's SmartConfig technology to connect the
   CC3000 to a wireless network.  A SmartConfig application will be required
   to complete the connection process.

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections and any
additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
Run the example. Board_LED0 turns ON to indicate TI-RTOS driver
initialization is complete.

Afterwards, Board_LED0 is turned OFF and used as a connection indicator for the
remainder of the example.  The CC3000 firmware version is checked for
compatibility.  If firmware update is necessary, a message will be printed on
the console with the minimum required firmware version.

If connecting the CC3000 to the AP for the first time, Board_BUTTON0 must be
pressed to start the SmartConfig process.  Use another wireless device (computer,
phone, etc.) connected to the AP to launch a SmartConfig application.  Follow
the application instructions to connect the CC3000 to your AP.  Once connected,
the CC3000's IP address is printed on the console and Board_LED0 is ON.  The
following message should be displayed:

        CC3000 has connected to AP and acquired an IP address.
        IP Address: xxx.xxx.xxx.xxx

Launch the tcpSendReceive Linux or Windows executable shipped with TI-RTOS. The
executable is found in:

  <tirtos_install_dir>\packages\examples\tools\tcpSendReceive

  Usage: ./tcpSendReceive <IP-addr> <port> <id> -l[length] -s[sleep in uS]

  <IP-addr> is the IP address of the CC3000
  <port>    is the TCP port being listened to (1000)
  <id>      is a unique id for the executable. Printed out when 1000 packets are
            transmitted. It allows the user to run multiple instances
            of tcpSendReceive.

  Optional:
    -l[length]      size of the packet in bytes. Default is 1024 bytes.
    -s[sleep in uS] usleep time to between sends. Default is 1000 uSecs.

  Example:
        tcpSendReceive 192.168.1.100 1000 1 -s100

Messages such as the following will begin to appear on the terminal window when
a TCP packet has been echoed back:

        Starting test with a 1000 uSec delay between transmits
        [id 1] count = 1000, time = 12
        [id 1] count = 2000, time = 24
        [id 1] count = 3000, time = 36

Application Design Details
--------------------------
This application uses one tasks:

* Note: a SYS/BIOS Task is statically created in tcpEchoCC3000.cfg and the
  RX and TX payload sizes are configured to 1024 bytes.  This is the largest
  possible amount of data sent from the client application.

  'tcpEchoTask' performs the following actions:
      Opens a WIFI driver object.

      Verifies the firmware version on the CC3000.  Will print a message if
      current firmware is incompatible with the example.

      Waits until the CC3000 is connected to the AP.  If connecting to AP
      for the first time, the SmartConfig procedure must be used.  The
      smartConfigFxn() initializes the configuration process on the CC3000.

      The asynchCallback function manages the connection events.  Allows execution
      to continue once connected to the AP and DHCP has returned an IP address.

      A TCP socket is created, bound and listening for connections on port 1000.

      If a client is acquired, the application echoes back data received until
      the remote client closes the socket.

      The socket and WIFI driver object are closed the the application is exited.
