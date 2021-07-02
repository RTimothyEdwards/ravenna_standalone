# ---- Test patterns for project ravenna ----

GCC_PATH?=/opt/riscv32imc/bin
GCC_PREFIX?=riscv32-unknown-elf

.SUFFIXES:

PATTERN = ravenna_irq2

SIM?=RTL

all:  ${PATTERN:=.vcd}

hex:  ${PATTERN:=.hex}

%.vvp: %_tb.v %.hex
	iverilog -I .. \
	$< -o $@

%.vcd: %.vvp
	vvp $<

%.elf: %.c sections.lds start.S
	$(GCC_PATH)/$(GCC_PREFIX)-gcc -march=rv32imc -Wl,-Bstatic,-T,sections.lds,--strip-debug -ffreestanding -nostdlib -o $@ start.S $<

%.hex: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objcopy -O verilog $< $@

%.bin: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objcopy -O binary $< /dev/stdout | tail -c +1048577 > $@

# ---- Clean ----

clean:
	rm -f *.elf *.hex *.bin *.vvp *.vcd

.PHONY: clean hex all

