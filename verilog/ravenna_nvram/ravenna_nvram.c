#include "../ravenna_defs.h"

// --------------------------------------------------------------
// NVRAM configuration register:
//
//	upper 26 bits zero
//	bit 5:	rdy	(read-only) NVRAM idle (not programming)
//	bit 4:	ena	enable NVRAM
//	bit 3:	hs	store (program)
//	bit 2:	hr	recall
//	bit 1:	mem_sel	select bank (0 or 1, normally 0)
//	bit 0:	mem_all select all banks (normally leave 0)
// --------------------------------------------------------------

void main()
{
	// NVRAM testbench
	// 1. Test basic SRAM mode reads/writes

	int i, v;
	for (i = 0; i < 32; i++) {
	    *(&reg_nvram_datatop + i) = i * 2 - 1;
	}
	v = *(&reg_nvram_datatop + 7);

	// 2. NV Program bank 1 (128 x 32 bits)
	// NOTE:  clkdiv is 24 by default to maintain 4MHz. . .
	// But XO does not run at 12.5MHz, so compensate here.
	// Then again, just speed way up so it doesn't take forever.
	reg_nvram_clkdiv = 7;
	reg_nvramctrl = 0x08;
	reg_nvramctrl = 0x00;
	while ((reg_nvramctrl & 0x10) == 0);

	// 3. Overwrite data with volatile contents
	for (i = 0; i < 32; i++) {
	    *(&reg_nvram_datatop + i) = i * 7 + 3;
	}

	// 4. Read back temporary data
	v = *(&reg_nvram_datatop + 7);

	// 5. NV Refresh bank 1
	reg_nvramctrl = 0x04;
	reg_nvramctrl = 0x00;
	while ((reg_nvramctrl & 0x10) == 0);

	// 6. Read back NV data
	v = *(&reg_nvram_datatop + 7);
}

