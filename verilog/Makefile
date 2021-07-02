# ---- Test patterns for project ravenna ----

.SUFFIXES:

PATTERNS = ravenna_gpio ravenna_adc ravenna_dac ravenna_uart

all:  ${PATTERNS}
	for i in ${PATTERNS}; do \
		( cd $$i && make $${i}.vcd ) ; \
	done

clean:  ${PATTERNS}
	for i in ${PATTERNS}; do \
		( cd $$i && make clean ) ; \
	done

.PHONY: clean all
