# Dishwasher Pico Controller

This project aims to replace the original I/O board of my dishwasher with a Raspberry Pi Pico.  
The Pico acts as a drop-in replacement that:

- Receives infrared remote control signals (NEC protocol) via an external IR sensor.
- Translates these commands into UART communication that mimics the original dishwasher I/O board.
- Passes the data on to the main control board of the dishwasher.

By doing so, the Pico fully emulates the behavior of the broken I/O board and allows controlling the machine with a generic IR remote.
