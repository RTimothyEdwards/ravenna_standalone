#include "../ravenna_defs.h"

// --------------------------------------------------------

void main()
{
	/* Test : toggle the GPIO pins once as the first action */
	reg_gpio_pub = 0x5a5a;		// NOT(pullup)
	reg_gpio_enb =  0xa5a5;		// NOT(output enable)

	reg_gpio_data = 0xffff;
	reg_gpio_data = 0x0000;
}

