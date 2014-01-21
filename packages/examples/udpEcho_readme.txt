Example Summary
----------------
This application demonstrates how to use UDP.

Peripherals Exercised
---------------------
Board_LED0      Indicates that the board was initialized within main()
Board_EMAC      Connection to network

Please refer to the Getting Started Guide's Board section to get details
regarding the location of the buttons, LEDs, USB connections, EMAC
address and any additional settings (e.g. jumpers) for your specific board.

Example Usage
-------------
The device must be connected to a network with a DHCP server to run this example
successfully.

The example turns ON Board_LED0 and starts the network stack. When the stack
receives an IP address from the DHCP server, the IP address is written to the
console.

Run the udpSendReceive Linux or Windows executable that is shipped with TI-RTOS.
The executable is found in:

  <tirtos_install_dir>\packages\examples\tools\udpSendReceive

  Usage: ./udpSendReceive <IP-addr> <port> <id> -l[length] -s[sleep in uS]

  <IP-addr> is the IP address
  <port>    is the UDP port being listened to (1000)
  <id>      is a unique id for the executable. Printed out when 1000 packets are
            transmitted. It allows the user to run multiple instances
            of udpSendReceive.

  Optional:
    -l[length]      size of the packet in bytes. Default is 1024 bytes.
    -s[sleep in uS] usleep time to between sends. Default is 1000 uSecs.

  Example:
        udpSendReceive 192.168.1.5 1000 1 -s100

Messages such as the following will begin to appear on the terminal window when
a UDP packet has been echoed back:

        Starting test with a 1000 uSec delay between transmits
        [id 1] count = 1000, time = 12
        [id 1] count = 2000, time = 24
        [id 1] count = 3000, time = 36

Application Design Details
--------------------------
This application uses one task:
  'udpHandler' performs the following actions:
      Create a socket and bind it to a port (1000 for this example).

      Wait for client before timing-out.

      Receive data from socket if a client is connected.

      Echo the UDP packet back to the client.

      When client closes the socket, close server socket, and exit the task.
