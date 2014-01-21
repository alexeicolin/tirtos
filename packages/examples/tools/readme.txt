tcpSendReceive: linux commandline tool that that makes a connection to the 
                sockets example on the target. It sends a packet to the target
                and waits for a reply. If verifies the first and last byte for
                correctness. It prints out the status every 1000 packets.
                tcpSendReceive.c was used to build this tool.

 usage: ./tcpSendReceive <IPv4 or IPv6 addr> <port> <id> -l[length] -s[sleep in uS]

 params: <IPv4 or IPv6 addr> IPv4 or IPv6  address of target
         <port>    port to make TCP connection with
         <id>      id of executable (helps when there is more than 1 of these)
 optional:
         -l[length]      size of the packet in bytes. Default is 1024 bytes.
         -s[sleep in uS] usleep time to between sends. Default is 1000 uSecs.


 IPv4 example: ./tcpSendReceive 146.252.161.26 1000 1

 IPv6 example (Linux): ./tcpSendReceive fe80::aa63:f2ff:fe00:491%eth1 1000 1

 IPv6 example (Windows): ./tcpSendReceive fe80::aa63:f2ff:fe00:491 1000 1

 output:
    [id 1] count = 1000, time = 11
    [id 1] count = 2000, time = 22
    [id 1] count = 3000, time = 33
    [id 1] count = 4000, time = 43
