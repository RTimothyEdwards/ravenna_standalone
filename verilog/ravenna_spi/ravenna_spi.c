#include "../ravenna_defs.h"

// --------------------------------------------------------

void main()
{
	int i;
	int spival;

	/* Configuration register:
	bit   13:  SPI master enable
	bit   12:  stream mode
	bit   11:  read/write on same clock edge
        bit   10:  invert sck sense
	bit    9:  invert csb sense
	bit    8:  lsb first
	bits 7-0:  clock prescaler
	*/

	// Test bench:  Write QSPI flash command
	// and read back data.

	// config:
	// set SPI enable (0x2000), prescaler = 2;

	reg_spi_config = 1 + 0x2000;

	// data to send:  ab (powerup) 03 (read command) 00 00 00 (address 0)
	reg_spi_data = 0xab;

	// apply stream mode (0x1000)
	reg_spi_config = 1 + 0x3000;
	reg_spi_data = 0x03;
	for (i = 0; i < 3; i++) reg_spi_data = 0x00;

	// read back 10 bytes
        for (i = 0; i < 10; i++) {
	    reg_spi_data = 0x00;	// Write dummy value
	    spival = reg_spi_data;	// Read back value from slave
	}

	// release CSB, ending stream mode.
	reg_spi_config = 2 + 0x2100;

	// to do:  Need more tests covering other modes.
}

