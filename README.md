# STM32_UART_ECHO
Example of how to use interrupts to receive chars into a ring buffer for processing.

The STM32 HAL function for the UART does not provide a byte-wise access to processing
received data.  The provided functions assume you will be receiving a block of data.

This code uses a ring buffer to received single chars on receive.  Main loop can read 
charactors from the ring-buffer to process them.

