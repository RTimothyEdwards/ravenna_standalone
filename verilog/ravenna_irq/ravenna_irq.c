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

// Routines needed to set quad speed mode

extern uint32_t flashio_worker_begin;
extern uint32_t flashio_worker_end;

void flashio(uint32_t *data, int len, uint8_t wrencmd)
{
        uint32_t func[&flashio_worker_end - &flashio_worker_begin];

        uint32_t *src_ptr = &flashio_worker_begin;
        uint32_t *dst_ptr = func;

        while (src_ptr != &flashio_worker_end)
                *(dst_ptr++) = *(src_ptr++);

        ((void(*)(uint32_t*, uint32_t, uint32_t))func)(data, len, wrencmd);
}

void set_flash_latency(uint8_t value)
{
        reg_spictrl = (reg_spictrl & ~0x007f0000) | ((value & 15) << 16);

        uint32_t buffer_wr[2] = {0x01000260, ((0x70 | value) << 24)};
        flashio(buffer_wr, 5, 0x50);
}

// Ring buffer latest valid position held here
#define reg_ringbuf_ptr (*(volatile uint32_t*)0x0000000c)

void main()
{
	int16_t data;
	uint16_t led_val;

	// Do not use this on the Ravenna board!
	// set_flash_latency(8);

	// Test bench:
	reg_gpio_pub =  0xffff;         // NOT(pullup)
	reg_gpio_pdb =  0xffff;         // NOT(pulldown)
	reg_gpio_enb =  0x0000;         // NOT(output enable)

	reg_gpio_data =  0x5555;        // initial test value

	// Speed up to DSPI + CRM
	// reg_spictrl = 0x80580000;	// Flash SPI speed mode DSPI + CRM

	// Speed up to QSPI + DDR + CRM
	// reg_spictrl = 0x807c0000;	// Flash SPI speed mode DDR + QSPI + CRM

	// loop writing last received value from SPI to the LEDs.
	while (1) {
            // data = *((uint16_t *)(reg_ringbuf_ptr) - 1);
            data = *((uint16_t *)(reg_ringbuf_ptr));
            // reg_gpio_data = (uint32_t)data;
	    led_val = 0;
	    if (data >= 0x999) led_val |= 0x1;
	    if (data >= 0xb32) led_val |= 0x2;
	    if (data >= 0xccb) led_val |= 0x4;
	    if (data >= 0xe64) led_val |= 0x8;

	    reg_gpio_data = led_val;
	}
}

