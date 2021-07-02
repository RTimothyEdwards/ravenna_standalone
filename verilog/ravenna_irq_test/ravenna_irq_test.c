#include "../ravenna_defs.h"

// -------------------------------------------------------------------------
// This routine uses the IRQ to force an SPI read/write cycle at a rate of
// 16MHz.  This works in conjunction with a Digilent PMOD MIC3 module,
// which emits 12 bit data in 2-byte transfers, read-only.  The IRQ is
// optimized to take as little processor time as possible.  The IRQ
// routine operates in 3 cycles which it handles by counting from a static
// register:
//
//  The first cycle initiates an SPI transfer by writing to the SPI configuration
//  register (memory-mapped location).  The timer is restarted for a count
//  long enough for the SPI transfer to complete.
//
//  The second cycle copies the SPI result into memory, increments the
//  memory pointer, and initiates the second SPI transfer, and resets the
//  timer.
//
//  The third cycles copies the SPI result into memory, increments the
//  memory pointer, and resets the timer such that the total of the three
//  cycles is 62.5us (16 kHz)
//
// Development:
// (1) Run timer and confirm IRQ and 16kHz cycle time
// (2) Initiate SPI transfer from within IRQ
// (3) Initiate two SPI transfers from within IRQ
// (4) Set up memory block for holding SPI data
// (5) Write a handler that copies the IRQ routine into memory so that the
//     handler does not run from the SPI flash.
// (6) Write emulator for the PMOD, have it output a sine wave, sample that
//     sine wave at 16kHz and display the result to the LEDs as a thermometer
//     code.
// (7) Implement a flag that indicates what is the position of the most
//     recent sample in the data block
//
// -------------------------------------------------------------------------

// Ring buffer latest valid position held here
#define reg_ringbuf_ptr (*(volatile uint32_t*)0x0000000c)

void main()
{
	int16_t data;
	uint16_t led_val;

	// Test bench:
	reg_gpio_pub =  0xffff;         // NOT(pullup)
	reg_gpio_pdb =  0xffff;         // NOT(pulldown)
	reg_gpio_enb =  0x0000;         // NOT(output enable)

	reg_gpio_data =  0x5555;        // initial test value

	// loop writing last received value from SPI to the LEDs.
	// NOTE:  Use the 1st line for a verilog testbench.  For
	// the Ravenna development board (with only 4 LEDs) use
	// the 2nd line to bounce the output down by 12 bits.
	while (1) {
            // data = *((int16_t *)(reg_ringbuf_ptr) - 1);
            data = *((int16_t *)(reg_ringbuf_ptr) + 1000);	/* fixed */
	    led_val = 0;
	    if (data > 0x8000) continue;
	    if (data > 0x1000) led_val |= 0x1;
	    if (data > 0x2000) led_val |= 0x2;
	    if (data > 0x3000) led_val |= 0x4;
	    if (data > 0x4000) led_val |= 0x8;
	    reg_gpio_data = led_val;
	}
}

