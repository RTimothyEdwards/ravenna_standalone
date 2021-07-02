# ---- Test patterns for project ravenna ----
#
# This is for use with Tensorflow-Lite micro example hello_world
# and therefore does not have a compile target, as the hex file
# must be compiled within Tensorflow.

GCC_PATH?=/opt/riscv32imc/bin
GCC_PREFIX?=riscv32-unknown-elf

.SUFFIXES:

PATTERN = hello_world

SIM?=RTL

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

