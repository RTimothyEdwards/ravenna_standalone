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

module ravenna_pass_thru_tb;
	reg XCLK;
	reg real VDD3V3;

	reg real XI;
	wire real XO;

	reg real adc_h, adc_l;
	reg real adc_0, adc_1;
	reg real comp_n, comp_p;
	wire real ana_out;

	integer i;

	always #100 XCLK <= (XCLK === 1'b0);
	always #50 XI = VDD3V3 - XO;

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

	wire [15:0] gpio;

	wire flash_csb;
	wire flash_clk;
	wire flash_io0;
	wire flash_io1;
	wire flash_io2;
	wire flash_io3;

	reg SDI, CSB, SCK;
	wire SDO;

        // The main testbench is here.  Put the housekeeping SPI into
        // pass-thru mode and read several bytes from the flash SPI.

        // First define tasks for SPI functions

	task start_csb;
	    begin
		SCK <= 1'b0;
		SDI <= 1'b0;
		CSB <= 1'b0;
		#50;
	    end
	endtask

	task end_csb;
	    begin
		SCK <= 1'b0;
		SDI <= 1'b0;
		CSB <= 1'b1;
		#50;
	    end
	endtask

	task write_byte;
	    input [7:0] odata;
	    begin
		SCK <= 1'b0;
		for (i=7; i >= 0; i--) begin
		    #50;
		    SDI <= odata[i];
                    #50;
		    SCK <= 1'b1;
                    #100;
		    SCK <= 1'b0;
		end
	    end
	endtask

	task read_byte;
	    output [7:0] idata;
	    begin
		SCK <= 1'b0;
		SDI <= 1'b0;
		for (i=7; i >= 0; i--) begin
		    #50;
                    idata[i] = SDO;
                    #50;
		    SCK <= 1'b1;
                    #100;
		    SCK <= 1'b0;
		end
	    end
	endtask

	task read_write_byte
	    (input [7:0] odata,
	    output [7:0] idata);
	    begin
		SCK <= 1'b0;
		for (i=7; i >= 0; i--) begin
		    #50;
		    SDI <= odata[i];
                    idata[i] = SDO;
                    #50;
		    SCK <= 1'b1;
                    #100;
		    SCK <= 1'b0;
		end
	    end
	endtask

        // Now drive the digital signals on the housekeeping SPI
	reg [7:0] tbdata;

	initial begin
	    $dumpfile("ravenna_pass_thru.vcd");
	    $dumpvars(0, ravenna_pass_thru_tb);

	    CSB <= 1'b1;
	    SCK <= 1'b0;
	    SDI <= 1'b0;

	    // Chip does not come out of reset until 20us.
	    #21000;

            // First do a normal read from the housekeeping SPI to
	    // make sure the housekeeping SPI works.

            start_csb();
            write_byte(8'h40);	// Read stream command
            write_byte(8'h03);	// Address (register 3 = product ID)
	    read_byte(tbdata);
	    end_csb();
	    #10;
	    $display("Read data = 0x%02x (should be 0x03)", tbdata);
	

            start_csb();
            write_byte(8'hc4);	// Pass-thru mode
            write_byte(8'h03);	// Command 03 (read values w/3-byte address)
            write_byte(8'h10);	// Address is next 3 bytes (0x100000)
            write_byte(8'h00);
	    write_byte(8'h00);

	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);
	    read_byte(tbdata);
	    $display("Read flash data = 0x%02x", tbdata);

            end_csb();

	    // Run longer to check processor restart.
	    #10000;
 	    $finish;
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
                .spi_sdi  (1'b0     ),
                .spi_csb  (         ),
                .spi_sck  (         ),
                .spi_sdo  (         ),
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
		.FILENAME("ravenna_pass_thru.hex")
	) spiflash (
		.csb(flash_csb),
		.clk(flash_clk),
		.io0(flash_io0),
		.io1(flash_io1),
		.io2(flash_io2),
		.io3(flash_io3)
	);

	// Testbench UART

	tbuart tbuart (
		.ser_rx(tbuart_rx)
	);
		
endmodule
