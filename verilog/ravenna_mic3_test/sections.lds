SECTIONS {
	.memory : {
		sram = 0x1000;
		. = 0x100000;
		start*(.text);
		*(.text);
		*(*);
	}
}
