# SPI flash programmer

This python module uses several simple commands to communicate with a small state machine over an RS-232 port which sends and receives bytes over SPI to implement a SPI flash programmer.

## Details

This script goes with the AVR code to implement a SPI flash programmer.
It assumes that there is a black box on the other end of the COM port that
implements the following protocol:
* Send 'l' -> Lower #CS line (enable communications)
* Send 'r' -> Raise #CS line (disable communications)
* Send 'b' and a byte -> send that byte to the SPI flash, and receive a byte back


That's the basic protocol. On top of that abstraction, this file implements the
SPI flash commands:

* 0x9F - read identification
* 0x05 - read status register
* 0x06 - write enable

* 0x03 - read data
* 0x02 - program page

* 0x20 - sector erase

The data manipulation commands (the last three) are followed by a three byte
address.

To read data, after sending the command, keep sending 0's until you've
received the number of bytes you want.

Data is written a page (256 bytes) at a time, and requires that page first be
blank.

To write data, first enable write. Then, erase the relevant sector if needed,
and wait for bit 0 of the status register, write in progress, to clear.

Then, enable write again and send a page of data, once again waiting until the
status register bit 0 is clear.


In addition to the above, the following fast programming protocols are
implemented, which speed up programming the flash greatly by reducing the back
and forth time over the serial port:

* Send 'f', then the program page, the 3 byte address, and 256 bytes of data all
at once.
The other end will send all of that to the SPI flash, and send back 'a' and 'd'
when complete.

* Send 'g', then the read data command and the 3 byte address all at once.
The other end will send back 256 bytes consisting of the page of data from the
flash.

## Example

* spi_ctrl.py com12 hardware.bin

## License

This project is licensed in the public domain, under the Creative Commons CC0 license. More details can be found at https://creativecommons.org/publicdomain/zero/1.0/
