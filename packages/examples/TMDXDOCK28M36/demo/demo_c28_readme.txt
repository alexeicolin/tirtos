Example Summary
----------------
Refer to the Demo [M3] demo_readme.txt file for a description.

Peripherals Exercised
---------------------
N/A. TI-RTOS does not support peripherals on the C28

Example Usage
-------------
Refer to the Demo [M3] demo_readme.txt file for a description.

Application Design Details
--------------------------
This example create a Message Queue called "C28s_queue". It then calls
MessageQ_open() to get the handle of the queue that was created on the M3.

Then it calls MessageQ_get() and waits until a message comes. It converts
the temperature and sends the message back to the M3.
