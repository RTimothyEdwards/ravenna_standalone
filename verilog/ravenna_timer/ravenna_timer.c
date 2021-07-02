#include "../ravenna_defs.h"

// --------------------------------------------------------
// Define the irq handling procedure (see start.S)
// There is only one interrupt used, so this routine
// does not check which interrupt was raised.
// --------------------------------------------------------

uint32_t *irq(uint32_t *regs, uint32_t irqs)
{
	static uint32_t count = 0;

	if (irqs & 0x2000)
	{
		count++;

		if (count == 3)
		{
			/* Turn counter off */
			reg_timer_config = 0;
			count = 0;
		}
	}
	return regs;
}

// --------------------------------------------------------

void main()
{
	int i;
	int value;

	/* Configuration register:
	bit  3:	 IRQ enable
	bit  2:  updown (1 = up, 0 = down)
	bit  1:  oneshot (1 = one-shot, 0 = continous)
	bit  0:  enable
	*/

	// Test bench:

	// config:
	// set value = 100, enable = 0x01

	reg_timer_value = 255;
	reg_timer_config = 1;

	// read back 10 times
        for (i = 0; i < 10; i++) {
	    value = reg_timer_data;	// Read back value from counter
	}

	// Change to up counter
	reg_timer_config = 5;

	// read back 12 times
        for (i = 0; i < 12; i++) {
	    value = reg_timer_data;	// Read back value from counter
	}

	// switch to one-shot mode
	reg_timer_config = 7;

	// read back 10 times
        for (i = 0; i < 10; i++) {
	    value = reg_timer_data;	// Read back value from counter
	}
}

