/*
 *  Ravenna - A full example SoC using PicoRV32 in X-Fab XH018
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *  Copyright (C) 2018  Tim Edwards <tim@efabless.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

`timescale 1 ns / 1 ps

`include "../ravenna.v"
`include "../spiflash.v"
`include "../tbuart.v"
`include "spi_test.v"

module micro_speech_tb;
	reg XCLK;
	reg real VDD3V3;

	reg real XI;
	wire real XO;

	reg real adc_h, adc_l;
	reg real adc_0, adc_1;
	reg real comp_n, comp_p;
	wire real ana_out;

	always #100 XCLK <= (XCLK === 1'b0);
	always #62.5 XI = VDD3V3 - XO;

	initial begin
		XCLK <= 1'b0;
		XI = 0.0;
		#150;
		XI = VDD3V3 / 2 - 0.1;
	end

	initial begin
		// Ramp VDD3V3 from 0 to 3.3V
		VDD3V3 = 0.0;
		#50;
		repeat (33) begin
		   #3;
		   VDD3V3 = VDD3V3 + 0.1;
		end
	end

	initial begin
		// Analog input pin values (static)

		adc_h = 0.0;
		adc_l = 0.0;
		adc_0 = 0.0;
		adc_1 = 0.0;
		comp_n = 0.0;
		comp_p = 0.0;
	end

	initial begin
		$dumpfile("micro_speech.vcd");
		// $dumpvars(0, micro_speech_tb);
		$dumpvars(1, micro_speech_tb);
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.trap);
		// $dumpvars(0, micro_speech_tb.uut.soc.cpu.timer);
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.dbg_ascii_instr);
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.dbg_insn_addr);

		// Example:  XSPRAM dump
		// 4091 = FFB = position of led_val
		// $dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[4091]);

		// Monitor the main memory positions, which are:
		// step count (10),
		// sample time sub-count (11),
		// ring buffer position (12-15),
		// timestamp (16-19),
		// ring buffer start address (20-23)
		// (NOTE each RAM position is 32 bits, so divide address by 4)
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[0]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[1]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[2]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[3]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[4]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[5]);

		// Main test is to watch how the samples are copied into the
		// ring buffer.  The position of the ring buffer must be
		// computed from the disassembler file:  Take the address of "start"
		// and subtract 0x00100004 (and divide by 4 (words->bytes)).
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[93]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[94]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[95]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[96]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[97]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[98]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[148]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[149]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[150]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[151]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[152]);
		$dumpvars(0, micro_speech_tb.uut.xfab_mem0.sub1.RAM_matrix[153]);

`ifndef GL
		// Example: CPU register dump (RTL only)
		// regs[5] = t0
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[5]);
		// regs[6] = t1
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[6]);
		// regs[7] = t2
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[7]);
		// regs[29] = t4
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[29]);
		// regs[30] = t5
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[30]);
		// regs[10] = a0
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[10]);
		// regs[11] = a1
		$dumpvars(0, micro_speech_tb.uut.soc.cpu.cpuregs.regs[11]);
`endif

		$display("No output, check signals with gtkwave.");
		// repeat (500) begin
		repeat (200) begin
			repeat (1000) @(posedge XCLK);
			// Diagnostic
			$display("+1000 cycles");
		end
		$finish;
	end

	wire [15:0] gpio;

	wire flash_csb;
	wire flash_clk;
	wire flash_io0;
	wire flash_io1;
	wire flash_io2;
	wire flash_io3;
	wire mic_csb, mic_sdi, mic_sdo, mic_sck;

	reg SDI, CSB, SCK;
	wire SDO;

	initial begin
		CSB <= 1'b1;
		SCK <= 1'b0;
		SDI <= 1'b0;
		#1000;
		CSB <= 1'b0;
	end

	wire real VDD1V8;
	wire real VSS;

	assign VSS = 0.0;

	ravenna uut (
		.VDD3V3	  (VDD3V3  ),
		.VDD1V8	  (VDD1V8),
		.VSS	  (VSS),
		.XI	  (XI),
		.XO	  (XO),
		.XCLK	  (XCLK),
		.SDI	  (SDI),
		.SDO	  (SDO),
		.CSB	  (CSB),
		.SCK	  (SCK),
		.ser_rx	  (1'b0	    ),
		.ser_tx	  (tbuart_rx),
                .i2c_sda  (         ),
                .i2c_scl  (         ),
                .spi_sdi  (mic_sdo  ),
                .spi_csb  (mic_csb  ),
                .spi_sck  (mic_sck  ),
                .spi_sdo  (mic_sdi  ),
		.irq	  (1'b0	    ),
		.gpio     (gpio     ),
		.flash_csb(flash_csb),
		.flash_clk(flash_clk),
		.flash_io0(flash_io0),
		.flash_io1(flash_io1),
		.flash_io2(flash_io2),
		.flash_io3(flash_io3),
		.adc_high (adc_h),
		.adc_low  (adc_l),
		.adc0_in  (adc_0),
		.adc1_in  (adc_1),
		.analog_out(ana_out),
		.comp_inp (comp_p),
		.comp_inn (comp_n)
	);

	spiflash #(
		.FILENAME("micro_speech.hex")
	) spiflash (
		.csb(flash_csb),
		.clk(flash_clk),
		.io0(flash_io0),
		.io1(flash_io1),
		.io2(flash_io2),
		.io3(flash_io3)
	);

	spi_test spi_mic (
		.csb(mic_csb),
		.clk(mic_sck),
		.sdi(mic_sdi),
		.sdo(mic_sdo)
	);

	// Testbench UART

	tbuart tbuart (
		.ser_rx(tbuart_rx)
	);

endmodule
