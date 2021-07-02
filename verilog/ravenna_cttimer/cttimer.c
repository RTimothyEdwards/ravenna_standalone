#include "../ravenna_defs.h"



//
//
//
//
//
//
//
//
//
//	Make a script that exercises the counter time
//	Refer to page 27 of the manual.
//
//	The counter/timer is a general-purpose 32-bit adder and subrator that can be configured for
//	a variety of timing functions including one-shot counts, continuous timing, and interval
//	interripts. At a core clock rate of 80 MHz, the longest single time interval is 26.84 seconds
//	(Maybe 80MHz and 26.84 seconds).
//
//	Timer configuration bit definitions
//	Bit 3 CT/T enable	|	1 = ct/t enabled	|	0 = ct/t disabled
//	Bit 2 Oneshot mode	|	1 = oneshot mode	|	0 = continuous mode
//	Bit 1 = Updown		|	1 = count up		|	0 = count down
//	Bit = Interrupt enable	|	1 = interrupt enabled	|	0 - interrupt disabled
//
//
//



void config()
{
	//reg_timer_config = 2;
	//reg_timer_config = 4; //	One-shot mode, where the counter triggers an interrupt
				//	when it reaches the value of reg_timer_data
	reg_timer_config = 8;
	//reg_timer_config = 1;
	//reg_timer_config = 15;


	//reg_timer_value;
	reg_timer_value = 0xffffffff;
}


void runct()
{
	uint32_t v;

	//v = &reg_timer_value; //	reg_timer_value is the current value of the counter
	while ( reg_uart_data == -1 )
	{
		v = reg_timer_data;
		if ( v > 50 )
		{
			reg_gpio_data = 1024;
		} else {
			reg_gpio_data = 0;
		}
		//reg_gpio_data
	}
	print("reset timer");

	//	If counting down, the counter resets to the value in reg_timer_value
}

void putchar(char c)
{
	        if (c == '\n')
			putchar('\r');
		        reg_uart_data = c & 0xff;
}                                                                                            

void print(const char *p)
{
	        while (*p)
			putchar(*(p++));
}


uint32_t getchar()
{
	int32_t c = -1;
	while (c == -1) {
		c = reg_uart_data;
	}
	return c;
}

void main()
{
	int c;
	uint32_t v;

	reg_uart_clkdiv = 6667;
	print("starting");
	reg_gpio_enb = 0;
	reg_gpio_pub = 0xff;
	reg_gpio_pdb = 0xff;
	reg_gpio_data = 0x01;
	//reg_timer_config
	//
	//reg_timer_value
	//reg_timer_data
	config();
	while (1)
	{
	c = reg_uart_data;
	if (c != -1) {
		//print();
		print("a - on/off counter/timer\n");
		print("b - count up/down toggle\n");
		print("c - one-shot and continuous toggle\n");
		//print("d - output count value to GPIO\n");
		print(" f - quit\n");
		//	output count value backwards so that the opper four bits of the count
		//	value go to GPIO 0-3 so that they show up on the LEDs. The low bits
		//	of the count are going to change too fast to see the LEDs blinkingg;
		//	they'll just look dim. The count reset value should be a full count
		//	(value = 0xffffffff)
		//c = getchar();
		switch (c) {
			//case 'r':
				//runct();
			//	break;
			case 'a':
				if ((reg_timer_config & 4) == 4) {
				       reg_timer_config &= ~4;
				       print("disabled cttimer\n");
				} else {
					reg_timer_config |= 4;
					print("enabled cttimer\n");
				}
				break;
			case 'b':
				if ((reg_timer_config & 1) == 1) {
				       reg_timer_config &= ~1;
					print("now counting down\n");
				} else {
			 		reg_timer_config |= 1;	
					print("now counting up\n");
				}	
				break;
			case 'c':
				if ((reg_timer_config & 2) == 2) {
					reg_timer_config &= ~2;
					print("now in continuous mode\n");
				} else {
					reg_timer_config |= 2;
					print("now in one-shot mode\n");
				}
				break;
			case 'd':

				break;
			case 'e':
				break;
			case 'f':
				break;
			default:
				print("Invalid response. \n");
				break;
		}
	}
	v = reg_timer_data;
	reg_gpio_data = ((v >> 12) & 0xfff0) | ((v >> 28) & 0xf);
	}


}











//
