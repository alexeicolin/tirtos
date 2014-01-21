Example Summary
----------------
This example is intended to be a starting point for new development where
a fuller set of kernel features and debug capabilities are enabled.

Peripherals Exercised
---------------------
Board_LED0  Indicates that the board was initialized within main()

Example Usage
-------------
The example only lights Board_LED0 as part of the initialization in main().

Application Design Details
--------------------------
This examples is the same as the "Empty (Minimal)" example except many
developement and debug features are enabled. For example:
    - Logging is enabled
    - Assert checking is enabled
    - Kernel Idle task
    - Instrumented driver modules are used
    - Stack overflow checking
    - Default kernel heap is present

Please refer to the "Memory Footprint Reduction" section in the TI-RTOS User
Guide (spruhd4.pdf) for a complete and detailed list of the differences
between the empty minimal and empty projects.
