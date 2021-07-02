#include "../ravenna_defs.h"

// -------------------------------------------------------------------------
// Test SPI interface running on a timed loop 
// -------------------------------------------------------------------------

// Ring buffer latest valid position held here
#define reg_ringbuf_ptr (*(volatile uint32_t*)0x0000000c)
#define reg_ringbuf_start (*(volatile uint16_t*)0x00000030)

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
        data = *((int16_t *)(reg_ringbuf_ptr - 1)); 
        // data = *((int16_t *)(reg_ringbuf_ptr) - 1);
	if (data < 0x800) continue;
        led_val = 0;
        if (data > 0x900) led_val |= 0x1;
        if (data > 0xa00) led_val |= 0x2;
        if (data > 0xb00) led_val |= 0x4;
        if (data > 0xc00) led_val |= 0x8;
        reg_gpio_data = led_val;
	// The following works correctly, shows the ring buffer address incrementing
        // reg_gpio_data = (int16_t *)(reg_ringbuf_ptr) << 1; 
    }
}

