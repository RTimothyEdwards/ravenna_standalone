# ---- Test patterns for project ravenna ----
#
# This testbench is used with the Tensorflow Lite micro example
# micro_speech.  It does not have a compile target because it
# needs to be compiled in Tensorflow.

GCC_PATH?=/opt/riscv32imc/bin
GCC_PREFIX?=riscv32-unknown-elf

.SUFFIXES:

PATTERN = micro_speech

all:  ${PATTERN:=.vcd}

%.vvp: %_tb.v %.hex
	iverilog -I .. \
	$< -o $@

%.vcd: %.vvp
	vvp $<

# ---- Clean ----

clean:
	rm -f *.vvp *.vcd

.PHONY: clean all

