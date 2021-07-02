# ---- Test patterns for project raven ----

GCC_PATH?=/opt/riscv32imc/bin
GCC_PREFIX?=riscv32-unknown-elf

.SUFFIXES:

PATTERN = cttimer

all: hex lst client

hex:  ${PATTERN:=.hex}
lst:  ${PATTERN:=.lst}
sim:  ${PATTERN:=.vcd}

%.vvp: cttimer_tb.v %.hex
	iverilog -I .. \
	$< -o $@

%.vcd: cttimer.vvp
	vvp $<

%.elf: %.c ../sections.lds ../start.s ../i2c_read.s
	$(GCC_PATH)/$(GCC_PREFIX)-gcc -falign-functions=4 -march=rv32im -Wl,-Bstatic,-T,../sections.lds,--strip-debug -ffreestanding -nostdlib -o $@ ../start.s ../i2c_read.s $<

%.hex: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objcopy -O verilog $< $@

%.bin: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objcopy -O binary $< $@

%.lst: %.elf
	$(GCC_PATH)/$(GCC_PREFIX)-objdump -d -s $< > $@

client: client.c
	gcc client.c -o client

flash: cttimer.hex
#	python3 ../../test/ravenna_hkspi.py cttimer.hex
	python3 ../../test/ravenna_flash.py cttimer.hex

# ---- Clean ----

clean:
	rm -f *.elf *.hex *.bin *.vvp *.vcd *.lst

.PHONY: clean hex all flash

