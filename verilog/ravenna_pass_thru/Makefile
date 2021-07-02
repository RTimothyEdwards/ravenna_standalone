# ---- Test patterns for project ravenna ----

GCC_PATH?=/opt/riscv32imc/bin
GCC_PREFIX?=riscv32-unknown-elf

.SUFFIXES:

PATTERN = ravenna_pass_thru

all:  ${PATTERN:=.vcd}

hex:  ${PATTERN:=.hex}

%.vvp: %_tb.v %.hex
	iverilog -I .. \
	$< -o $@

%.vcd: %.vvp
	vvp $<

%.elf: %.c ../sections.lds ../start.s
	$(GCC_PATH)/$(GCC_PREFIX)-gcc -march=rv32imc -Wl,-Bstatic,-T,../sections.lds,--strip-debug -ffreestanding -nostdlib -o $@ ../start.s $<

%.hex: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objcopy -O verilog $< /dev/stdout | sed -e '1 s/@00000000/@00100000/; 2,65537 d;' > $@

%.bin: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objcopy -O binary $< /dev/stdout | tail -c +1048577 > $@

# ---- Clean ----

clean:
	rm -f *.elf *.hex *.bin *.vvp *.vcd

.PHONY: clean hex all

