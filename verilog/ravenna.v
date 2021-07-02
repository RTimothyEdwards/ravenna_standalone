/*
 *  ravenna - A full example SoC using PicoRV32 in X-Fab XH018
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *  Copyright (C) 2018,2019  Tim Edwards <tim@efabless.com>
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

`ifndef LVS

// Local files
`include "ravenna_soc.v"
`include "ravenna_spi.v"
`include "spi_slave.v"

// Marketplace IP
// Behavioral verilog components
`include "AMUX2_3V.v"
`include "AMUX4_3V.v"
`include "LS_3VX2.v"

// `include "XSPRAMBLP_4096X32_M8P.v"
// TEST:  Memory increased from 4x32 to 16x32.  This represents memory that
// does not actually exist on the chip!
`include "sram_test.v"

`include "XCPF_136X32DP128_VD03.v"
`include "XNVR_136X32P128_VD01.v"

// PDK IP
// 3.3V digital standard cells
`include "D_CELLS_3V.v"
// 1.8V digital standard cells
`include "D_CELLS.v"

// 1.8V core / 3.3V I/O padframe cells
`include "IO_CELLS_F3V.v"

// 3.3V core / 3.3V I/O padframe cells (isolate from 1.8V core cells!)
`include "IO_CELLS_FC3V.v"

// 1.8V Analog cells
`include "apllc03_1v8.v"
`include "acsoc04_1v8.v"

// 3.3V Analog cells
`include "atmpc01_3v3.v"
`include "aadcc01_3v3.v"
`include "adacc01_3v3.v"
`include "aopac01_3v3.v"
`include "abgpc01_3v3.v"
`include "acmpc01_3v3.v"
`include "axtoc02_3v3.v"
`include "arcoc01_3v3.v"
`include "aregc01_3v3.v"
`include "aporc02_3v3.v"
`include "acsoc01_3v3.v"
`include "acsoc02_3v3.v"

`endif

// Primitive devices (for LVS, and need (empty) model to prevent error on simulation).
`include "cmm5t.v"
`include "dn.v"
`include "dn3.v"

// ravenna, a picosoc implementation in X-Fab XH018

module ravenna (
	// Padframe I/O
	input  wire real VDD3V3,	// 3V power supply
	output wire real VDD1V8,	// 1.8V from regulator (for external cap)
	input  wire real VSS,	// ground

	// Crystal
	input  wire real XI,	// Crystal oscillator in
	output wire real XO,	// Crystal oscillator out
	input  XCLK,	// External clock (PLL bypass mode)

	// housekeeping SPI
	input  SDI,	// SPI controller data in
	output SDO,	// SPI controller data out
	input  CSB,	// SPI controller select
	input  SCK,	// SPI controller clock

	// UART
	output ser_tx,	// uart transmit
	input  ser_rx,	// uart receive

	// I2C
	inout  i2c_sda,
	inout  i2c_scl,

	// SPI master
	input  spi_sdi,
	output spi_csb,
	output spi_sck,
	output spi_sdo,

	// IRQ
	input  irq,	// dedicated IRQ pin

	// GPIO
	output [15:0] gpio,	// general-purpose I/O

	// Flash
	output flash_csb,	// SPI flash memory
	output flash_clk,
	inout  flash_io0,
	inout  flash_io1,
	inout  flash_io2,
	inout  flash_io3,

	// Analog I/O
	input wire real adc_high,
	input wire real adc_low,
	input wire real adc0_in,
	input wire real adc1_in,

	output wire real analog_out,	// test analog port (multiplexed, buffered)

	input wire real comp_inp,
	input wire real comp_inn,
	input wire real nvref_ext	// NVRAM test analog port
);
	wire dground;
	wire dvdd;
	wire reset;
	wire resetn;
	wire clk;
	wire irq;

	wire flash_io0_oeb, flash_io0_do, flash_io0_di;
	wire flash_io1_oeb, flash_io1_do, flash_io1_di;
	wire flash_io2_oeb, flash_io2_do, flash_io2_di;
	wire flash_io3_oeb, flash_io3_do, flash_io3_di;
	wire flash_clk_oeb;

	wire [15:0] gpio_in_core;
	wire [15:0] gpio_out_core;
	wire 	    irq_pin_core;
	wire	    flash_csb_core;
	wire	    flash_clk_core;
	wire	    ser_rx_core;
	wire	    ser_tx_core;

	/* Analog values represented by reals */
	// wire real VDD3V3;
	// wire real VDD1V8;
	// wire real VSS;

	// wire real adc_high;
	// wire real adc_low;
	// wire real adc0_in;
	// wire real adc1_in;
	// wire real analog_out;
	// wire real comp_inp;
	// wire real comp_inn;

	// Declare bus widths 
	wire [15:0] gpio_pullupb;
	wire [15:0] gpio_pulldownb;
	wire [15:0] gpio_outenb;
	wire [9:0]  adc0_data;
	wire [1:0]  adc0_inputsrc;
	wire [9:0]  adc1_data;
	wire [1:0]  adc1_inputsrc;
	wire [9:0]  dac_value;
	wire [1:0]  comp_ninputsrc;
	wire [1:0]  comp_pinputsrc;
	wire [7:0]  spi_config;
	wire [3:0]  spi_pll_trim;
	wire [11:0] spi_mfgr_id;
	wire [7:0]  spi_prod_id;
	wire [3:0]  spi_mask_rev;

	// Declare level-shifted signals

	wire spi_trap_3v;

	wire SCK_core_lv;

	wire spi_pll_vco_ena_lv;
	wire spi_pll_cp_ena_lv;
	wire spi_pll_bias_ena_lv;
	wire [3:0] spi_pll_trim_lv;
	wire spi_irq_lv;
	wire spi_reset_lv;
	wire spi_pll_bypass_lv;
 	wire [7:0] spi_config_lv;
 	wire spi_xtal_ena_lv;
 	wire spi_reg_ena_lv;
 	wire [11:0] spi_mfgr_id_lv;
 	wire [7:0] spi_prod_id_lv;
 	wire [3:0] spi_mask_rev_lv;

	wire opamp_ena_3v;
	wire opamp_bias_ena_3v;
	wire bg_ena_3v;
	wire comp_out_lv;
	wire comp_ena_3v;
	wire xtal_out_lv;
	wire rcosc_ena_3v;
	wire rcosc_out_lv;
	wire reset_lv;
	wire overtemp_ena_3v;
	wire overtemp_lv;

	/* Padframe pads */

	/* Analog input/output pads */
	APR00DF adc0_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(adc0_in),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF adc1_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(adc1_in),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF adc_low_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(adc_low),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF adc_high_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(adc_high),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF ana_out_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(analog_out),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF comp_inn_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(comp_inn),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF comp_inp_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(comp_inp),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	APR00DF nvref_ext_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .PAD(nvref_ext),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	/* Power supplies (there are multiple pads that need to be represented) */

	VDDORPADF vddor_pad [4:0] (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .VDD(VDD1V8),
	   .VDDOR(VDD3V3)
	);

	VDDPADF vdd_pad [1:0] (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	VDDPADFC vdd3_pad (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .VDD3(VDD3V3),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	GNDORPADF gndor_pad [6:0] (
	   .GNDOR(VSS),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	CORNERESDF padframe_corner [3:0] (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	/* Custom-designed power cut cell isolates the VDD3 and VDD buses */

	POWERCUTVDD3FC pwr_cut [1:0] (
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

        /* Implement bidirectional I/O with X-Fab pads */
        /* See: /ef/tech/XFAB.3/EFXH018D/libs.ref/verilog/IO_CELLS_3V/IO_CELLS_3V.v */

	BBC4F flash_io_buf_3 (
		.PAD(flash_io3),
		.EN(flash_io3_oeb),
		.A(flash_io3_do),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(flash_io3_di)
	);

	BBC4F flash_io_buf_2 (
		.PAD(flash_io2),
		.EN(flash_io2_oeb),
		.A(flash_io2_do),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(flash_io2_di)
	);

	BBC4F flash_io_buf_1 (
		.PAD(flash_io1),
		.EN(flash_io1_oeb),
		.A(flash_io1_do),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(flash_io1_di)
	);

	BBC4F flash_io_buf_0 (
		.PAD(flash_io0),
		.EN(flash_io0_oeb),
		.A(flash_io0_do),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(flash_io0_di)
	);

	/* Standalone SPI controller (3V) */
	ICFC sck_buf (
		.PAD(SCK),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD3(VDD3V3),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(SCK_core)
	);

	ICFC csb_buf (
		.PAD(CSB),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD3(VDD3V3),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(CSB_core)
	);

	ICFC sdi_buf (
		.PAD(SDI),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD3(VDD3V3),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(SDI_core)
	);

	BT4FC sdo_buf (
		.PAD(SDO),
		.EN(sdo_enb),
		.A(SDO_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD3(VDD3V3),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
	);

	/* Implement digital input on irq dedicated pin */
	ICF irq_buf (
		.PAD(irq),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(irq_pin_core)
	);

	/* Implement digital input on ser_rx */
	ICF ser_rx_buf (
		.PAD(ser_rx),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(ser_rx_core)
	);

	/* Implement digital outputs on ser_tx, i2c, LEDs, csb, and clk */
	BT4F ser_tx_buf (
		.PAD(ser_tx),
		.EN(reset_lv),
		.A(ser_tx_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
	);

	BBCUD4F i2c_scl_pad (
		.A(i2c_scl_o),
		.EN(i2c_scl_oeb),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(i2c_scl),
		.PDEN(dground),
		.PI(dground),
		.PO(),
		.PUEN(dground),	// To do: enable PU, optional 5, 10k?
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(i2c_scl_i)
	);

	BBCUD4F i2c_sda_pad (
		.A(i2c_sda_o),
		.EN(i2c_sda_oeb),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(i2c_sda),
		.PDEN(dground),
		.PI(dground),
		.PO(),
		.PUEN(dground),	// To do: enable PU, optional 5, 10k?
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(i2c_sda_i)
	);

	// GPIO is digital bidirectional buffer with selectable pull-up and pull-down

	BBCUD4F GPIO_buf_15 (
		.A(gpio_out_core[15]),
		.EN(gpio_outenb[15]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[15]),
		.PDEN(gpio_pulldownb[15]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[15]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[15])
	);

	BBCUD4F GPIO_buf_14 (
		.A(gpio_out_core[14]),
		.EN(gpio_outenb[14]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[14]),
		.PDEN(gpio_pulldownb[14]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[14]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[14])
	);

	BBCUD4F GPIO_buf_13 (
		.A(gpio_out_core[13]),
		.EN(gpio_outenb[13]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[13]),
		.PDEN(gpio_pulldownb[13]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[13]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[13])
	);

	BBCUD4F GPIO_buf_12 (
		.A(gpio_out_core[12]),
		.EN(gpio_outenb[12]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[12]),
		.PDEN(gpio_pulldownb[12]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[12]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[12])
	);

	BBCUD4F GPIO_buf_11 (
		.A(gpio_out_core[11]),
		.EN(gpio_outenb[11]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[11]),
		.PDEN(gpio_pulldownb[11]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[11]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[11])
	);

	BBCUD4F GPIO_buf_10 (
		.A(gpio_out_core[10]),
		.EN(gpio_outenb[10]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[10]),
		.PDEN(gpio_pulldownb[10]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[10]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[10])
	);

	BBCUD4F GPIO_buf_9 (
		.A(gpio_out_core[9]),
		.EN(gpio_outenb[9]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[9]),
		.PDEN(gpio_pulldownb[9]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[9]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[9])
	);

	BBCUD4F GPIO_buf_8 (
		.A(gpio_out_core[8]),
		.EN(gpio_outenb[8]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[8]),
		.PDEN(gpio_pulldownb[8]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[8]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[8])
	);

	BBCUD4F GPIO_buf_7 (
		.A(gpio_out_core[7]),
		.EN(gpio_outenb[7]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[7]),
		.PDEN(gpio_pulldownb[7]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[7]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[7])
	);

	BBCUD4F GPIO_buf_6 (
		.A(gpio_out_core[6]),
		.EN(gpio_outenb[6]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[6]),
		.PDEN(gpio_pulldownb[6]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[6]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[6])
	);

	BBCUD4F GPIO_buf_5 (
		.A(gpio_out_core[5]),
		.EN(gpio_outenb[5]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[5]),
		.PDEN(gpio_pulldownb[5]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[5]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[5])
	);

	BBCUD4F GPIO_buf_4 (
		.A(gpio_out_core[4]),
		.EN(gpio_outenb[4]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[4]),
		.PDEN(gpio_pulldownb[4]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[4]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[4])
	);

	BBCUD4F GPIO_buf_3 (
		.A(gpio_out_core[3]),
		.EN(gpio_outenb[3]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[3]),
		.PDEN(gpio_pulldownb[3]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[3]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[3])
	);

	BBCUD4F GPIO_buf_2 (
		.A(gpio_out_core[2]),
		.EN(gpio_outenb[2]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[2]),
		.PDEN(gpio_pulldownb[2]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[2]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[2])
	);

	BBCUD4F GPIO_buf_1 (
		.A(gpio_out_core[1]),
		.EN(gpio_outenb[1]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[1]),
		.PDEN(gpio_pulldownb[1]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[1]),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.Y(gpio_in_core[1])
	);

	BBCUD4F GPIO_buf_0 (
		.A(gpio_out_core[0]),
		.EN(gpio_outenb[0]),
		.GNDO(VSS),
		.GNDR(VSS),
		.PAD(gpio[0]),
		.PDEN(gpio_pulldownb[0]),
		.PI(dground),
		.PO(),
		.PUEN(gpio_pullupb[0]),
		.VDD(VDD1V8),
		.VDDR(VDD3V3),
		.VDDO(VDD3V3),
		.Y(gpio_in_core[0])
	);

	BT4F flash_csb_buf (
		.PAD(flash_csb),
		.EN(flash_csb_oeb),
		.A(flash_csb_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
		
	);

	BT4F flash_clk_buf (
		.PAD(flash_clk),
		.EN(flash_clk_oeb),
		.A(flash_clk_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
	);

	ICF clk_ext_buf (	// External digital clock for PLL bypass mode
		.PAD(XCLK),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(clk_ext_core)
	);

	ICF spi_master_sdi_buf (
		.PAD(spi_sdi),
		.PO(),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3),
		.PI(dground),
		.Y(spi_sdi_core)
	);

	BT4F spi_master_csb_buf (
		.PAD(spi_csb),
		.EN(reset_lv),
		.A(spi_csb_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
	);

	BT4F spi_master_sck_buf (
		.PAD(spi_sck),
		.EN(reset_lv),
		.A(spi_sck_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
	);

	BT4F spi_master_sdo_buf (
		.PAD(spi_sdo),
		.EN(spi_sdoenb),
		.A(spi_sdo_core),
		.GNDO(VSS),
		.GNDR(VSS),
		.VDD(VDD1V8),
		.VDDO(VDD3V3),
		.VDDR(VDD3V3)
	);

	/* To avoid antenna violations, each digital pad with inputs A	*/
	/* and EN, and GPIO pads with PUEN and PDEN, have a diode with	*/
	/* minimum area on each of the input pins.			*/

	dn #(
	    .area(2.025e-13)
	) gpio_out_core_tie [15:0] (
	    .pos(VSS),
	    .neg(gpio_out_core)
	);
	dn #(
	    .area(2.025e-13)
	) gpio_outenb_tie [15:0] (
	    .pos(VSS),
	    .neg(gpio_outenb)
	);
	dn #(
	    .area(2.025e-13)
	) gpio_pulldownb_tie [15:0] (
	    .pos(VSS),
	    .neg(gpio_pulldownb)
	);
	dn #(
	    .area(2.025e-13)
	) gpio_pullupb_tie [15:0] (
	    .pos(VSS),
	    .neg(gpio_pullupb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io0_oeb_tie (
	    .pos(VSS),
	    .neg(flash_io0_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io0_do_tie (
	    .pos(VSS),
	    .neg(flash_io0_do)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io1_oeb_tie (
	    .pos(VSS),
	    .neg(flash_io1_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io1_do_tie (
	    .pos(VSS),
	    .neg(flash_io1_do)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io2_oeb_tie (
	    .pos(VSS),
	    .neg(flash_io2_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io2_do_tie (
	    .pos(VSS),
	    .neg(flash_io2_do)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io3_oeb_tie (
	    .pos(VSS),
	    .neg(flash_io3_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_io3_do_tie (
	    .pos(VSS),
	    .neg(flash_io3_do)
	);
	dn #(
	    .area(2.025e-13)
	) reset_lv_tie [2:0] (
	    .pos(VSS),
	    .neg(reset_lv)
	);
	dn #(
	    .area(2.025e-13)
	) ser_tx_core_tie (
	    .pos(VSS),
	    .neg(ser_tx_core)
	);
	dn #(
	    .area(2.025e-13)
	) i2c_scl_o_tie (
	    .pos(VSS),
	    .neg(i2c_scl_o)
	);
	dn #(
	    .area(2.025e-13)
	) i2c_scl_oeb_tie (
	    .pos(VSS),
	    .neg(i2c_scl_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) i2c_sda_o_tie (
	    .pos(VSS),
	    .neg(i2c_sda_o)
	);
	dn #(
	    .area(2.025e-13)
	) i2c_sda_oeb_tie (
	    .pos(VSS),
	    .neg(i2c_sda_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_csb_oeb_tie (
	    .pos(VSS),
	    .neg(flash_csb_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_csb_core_tie (
	    .pos(VSS),
	    .neg(flash_csb_core)
	);
	dn #(
	    .area(2.025e-13)
	) flash_clk_oeb_tie (
	    .pos(VSS),
	    .neg(flash_clk_oeb)
	);
	dn #(
	    .area(2.025e-13)
	) flash_clk_core_tie (
	    .pos(VSS),
	    .neg(flash_clk_core)
	);
	dn #(
	    .area(2.025e-13)
	) spi_csb_core_tie (
	    .pos(VSS),
	    .neg(spi_csb_core)
	);
	dn #(
	    .area(2.025e-13)
	) spi_sck_core_tie (
	    .pos(VSS),
	    .neg(spi_sck_core)
	);
	dn #(
	    .area(2.025e-13)
	) spi_sdoenb_tie (
	    .pos(VSS),
	    .neg(spi_sdoenb)
	);
	dn #(
	    .area(2.025e-13)
	) spi_sdo_core_tie (
	    .pos(VSS),
	    .neg(spi_sdo_core)
	);

	/* Implement MiM capacitors.  Layout uses 20x20um devices, so A=400um^2, P=80um */

	cmm5t #(
	   .A(4e-10),
	   .P(8e-05)
	) cap_area_fill_0 [468:0] (
	   .top(VDD1V8),
	   .bottom(VSS),
	   .subs(VSS)
	);

	cmm5t #(
	   .A(4e-10),
	   .P(8e-05)
	) cap_area_fill_1 [276:0] (
	   .top(VDD3V3),
	   .bottom(VSS),
	   .subs(VSS)
	);

	/* Each cap area has an associated tie-down diode */
	dn3 #(
	    .area(2.25e-12)
	) VDD3V3_tie [2:0] (
	    .pos(VSS),
	    .neg(VDD3V3)
	);

	dn #(
	    .area(2.25e-12)
	) VDD1V8_tie [4:0] (
	    .pos(VSS),
	    .neg(VDD1V8)
	);

	wire [3:0]  ram_wenb;
	// wire [11:0] ram_addr;
	wire [13:0] ram_addr;
	wire [31:0] ram_wdata;
	wire [31:0] ram_rdata;

	wire [7:0]  nvram_addr;
	wire [31:0] nvram_wdata;
	wire [31:0] nvram_rdata;

	// NVRAM test mode from housekeeping SPI
	wire [3:0] tm_nvcp;
	wire [3:0] tm_nvcp_lv;

	/* NOTE:  Hardwired digital 0 disallowed in structural netlist.	*/
	/* Must generate from tie-low standard cell.			*/

	LOGIC0_3V ground_digital [7:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .Q(dground)
	);

	LOGIC1 vdd_digital (
`ifdef LVS
	   .gnd(VSS),
	   .vdd(VDD1V8),
`endif
	   .Q(dvdd)
	);


	/* SCK_core is also input to ravenna_soc but needs to be shifted to 1.8V */
	/* Level shift down */
	BU_3VX2 SCK_core_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(SCK_core),
	   .Q(SCK_core_lv)
	);

	/* Due to lack of any SPI configuration behavior on the 1st generation	*/
	/* Raven chip, the spi_config is just grounded.  However, this requires	*/
	/* tie-low inputs.							*/

	LOGIC0_3V spi_config_zero [7:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .Q(spi_config)
	);

	/* SPI internal registers to be read from memory mapped I/O must also	*/
	/* be shifted down.  Those that are sent to the PLL already have	*/
	/* shifted versions.							*/

	BU_3VX2 spi_config_level [7:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_config),
	   .Q(spi_config_lv)
	);
	BU_3VX2 spi_xtal_ena_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_xtal_ena),
	   .Q(spi_xtal_ena_lv)
	);
	BU_3VX2 spi_reg_ena_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_reg_ena),
	   .Q(spi_reg_ena_lv)
	);
	/* This is a spare level shift buffer */
	BU_3VX2 spare_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(dground),
	   .Q(spare_dg)
	);
	BU_3VX2 spi_mfgr_id_level [11:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_mfgr_id),
	   .Q(spi_mfgr_id_lv)
	);
	BU_3VX2 spi_prod_id_level [7:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_prod_id),
	   .Q(spi_prod_id_lv)
	);
	BU_3VX2 spi_mask_rev_level [3:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_mask_rev),
	   .Q(spi_mask_rev_lv)
	);

	BU_3VX2 spi_reset_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_reset),
	   .Q(spi_reset_lv)
	);
	BU_3VX2 spi_pll_bypass_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_pll_bypass),
	   .Q(spi_pll_bypass_lv)
	);

	ravenna_soc soc (
`ifdef LVS
	   	.gnd	      (VSS    ),
	   	.vdd	      (VDD1V8 ),
`endif
		.pll_clk      (pll_clk ),
		.ext_clk      (clk_ext_core),
		.ext_clk_sel  (spi_pll_bypass_lv),
		.reset        (reset_lv ),
		.ext_reset    (spi_reset_lv ),

		.pass_thru (pass_thru_lv),
		.pass_thru_csb (pass_thru_csb_lv),
		.pass_thru_sck (pass_thru_sck_lv),
		.pass_thru_sdi (pass_thru_sdi_lv),
		.pass_thru_sdo (pass_thru_sdo),

		.ram_wenb     (ram_wenb    ),
		.ram_addr     (ram_addr	   ),
		.ram_wdata    (ram_wdata   ),
		.ram_rdata    (ram_rdata   ),

		.nvram_ena      (nvram_ena     ),
		.nvram_wen      (nvram_wen     ),
		.nvram_addr     (nvram_addr    ),
		.nvram_wdata    (nvram_wdata   ),
		.nvram_rdata    (nvram_rdata   ),
		.nvram_hs    	(nvram_hs      ),
		.nvram_hr    	(nvram_hr      ),
		.nvram_mem_all  (nvram_mem_all ),
		.nvram_mem_sel  (nvram_mem_sel ),
		.nvram_clk 	(nvram_clk     ),
		.nvram_rdy	(nvram_rdy     ),

		.gpio_out       (gpio_out_core),
		.gpio_in        (gpio_in_core),
		.gpio_pullupb   (gpio_pullupb),
		.gpio_pulldownb (gpio_pulldownb),
		.gpio_outenb    (gpio_outenb),

		.adc0_ena     (adc0_ena),
		.adc0_convert (adc0_convert),
		.adc0_data    (adc0_data),
		.adc0_done    (adc0_done),
		.adc0_clk     (adc0_clk),
		.adc0_inputsrc (adc0_inputsrc),

		.adc1_ena      (adc1_ena),
		.adc1_convert  (adc1_convert),
		.adc1_data     (adc1_data),
		.adc1_done     (adc1_done),
		.adc1_clk      (adc1_clk),
		.adc1_inputsrc (adc1_inputsrc),

		.dac_ena     (dac_ena),
		.dac_value   (dac_value),

		.analog_out_sel (analog_out_sel),
		.opamp_ena	(opamp_ena),
		.opamp_bias_ena	(opamp_bias_ena),
		.bg_ena		(bg_ena),

		.comp_ena       (comp_ena),
		.comp_ninputsrc (comp_ninputsrc),
		.comp_pinputsrc (comp_pinputsrc),
		.rcosc_ena	(rcosc_ena),

		.overtemp_ena	(overtemp_ena),
		.overtemp	(overtemp_lv),
		.rcosc_in	(rcosc_out_lv),
		.xtal_in	(xtal_div8),
		.comp_in	(comp_out_lv),
		.spi_sck	(SCK_core_lv),

		.spi_ro_config	(spi_config_lv),
		.spi_ro_xtal_ena (spi_xtal_ena_lv),
		.spi_ro_reg_ena	(spi_reg_ena_lv),
		.spi_ro_pll_cp_ena (spi_pll_cp_ena_lv),
		.spi_ro_pll_vco_ena (spi_pll_vco_ena_lv),
		.spi_ro_pll_bias_ena (spi_pll_bias_ena_lv),
		.spi_ro_pll_trim (spi_pll_trim_lv),
		.spi_ro_mfgr_id	(spi_mfgr_id_lv),
		.spi_ro_prod_id	(spi_prod_id_lv),
		.spi_ro_mask_rev (spi_mask_rev_lv),

		.ser_tx    (ser_tx_core ),
		.ser_rx    (ser_rx_core ),

		.scl_pad_i  (i2c_scl_i),
		.scl_pad_o  (i2c_scl_o),
		.scl_padoeb (i2c_scl_oeb),
		.sda_pad_i  (i2c_sda_i),
		.sda_pad_o  (i2c_sda_o),
		.sda_padoeb (i2c_sda_oeb),

		.spi_master_sdi (spi_sdi_core),
		.spi_master_csb (spi_csb_core),
		.spi_master_sck (spi_sck_core),
		.spi_master_sdo (spi_sdo_core),
		.spi_master_sdoenb (spi_sdoenb),

		.irq_pin   (irq_pin_core),
		.irq_spi   (spi_irq_lv),

		.trap	   (spi_trap),

		.flash_csb (flash_csb_core),
		.flash_clk (flash_clk_core),

		.flash_csb_oeb (flash_csb_oeb),
		.flash_clk_oeb (flash_clk_oeb),

		.flash_io0_oeb (flash_io0_oeb),
		.flash_io1_oeb (flash_io1_oeb),
		.flash_io2_oeb (flash_io2_oeb),
		.flash_io3_oeb (flash_io3_oeb),

		.flash_io0_do (flash_io0_do),
		.flash_io1_do (flash_io1_do),
		.flash_io2_do (flash_io2_do),
		.flash_io3_do (flash_io3_do),

		.flash_io0_di (flash_io0_di),
		.flash_io1_di (flash_io1_di),
		.flash_io2_di (flash_io2_di),
		.flash_io3_di (flash_io3_di)
	);

	/* Level shift up */

	LS_3VX2 spi_trap_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(spi_trap),
	   .Q(spi_trap_3v)
	);
	LS_3VX2 pass_thru_sdo_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(pass_thru_sdo),
	   .Q(pass_thru_sdo_3v)
	);

	/* Metal programming for mask revision */

	wire [3:0] pground;
	wire [3:0] ppower;

	LOGIC0_3V prog_ground [3:0] (
`ifdef LVS
	    .gnd(VSS),
	    .vdd3(VDD3V3),
`endif
	    .Q(pground)
	);
	LOGIC1_3V prog_power [3:0] (
`ifdef LVS
	    .gnd(VSS),
	    .vdd3(VDD3V3),
`endif
	    .Q(ppower)
	);

	/* Standalone SPI (3V)*/
	/* Operates at 3V so that it can control the xtal oscillator, PLL, */
	/* and 1.8V regulator, which cannot be changed from the CPU 	   */
	/* without potentially killing it.				   */

	ravenna_spi spi (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .RST(reset),
	   .SCK(SCK_core),
	   .SDI(SDI_core),
	   .CSB(CSB_core),
	   .SDO(SDO_core),
	   .sdo_enb(sdo_enb),
	   .xtal_ena(spi_xtal_ena),
	   .reg_ena(spi_reg_ena),
	   .pll_vco_ena(spi_pll_vco_ena),
	   .pll_cp_ena(spi_pll_cp_ena),
	   .pll_vco_in(spi_pll_vco_in),
	   .pll_bias_ena(spi_pll_bias_ena),
	   .pll_trim(spi_pll_trim),
	   .pll_bypass(spi_pll_bypass),
	   .tm_nvcp(tm_nvcp),
	   .irq(spi_irq),
	   .reset(spi_reset),
	   .trap(spi_trap_3v),
	   .pass_thru_reset (pass_thru),
	   .pass_thru_csb (pass_thru_csb),
	   .pass_thru_sck (pass_thru_sck),
	   .pass_thru_sdi (pass_thru_sdi),
	   .pass_thru_sdo (pass_thru_sdo_3v),
	   .mask_rev_in(pground),		// Metal programmed
	   .mfgr_id(spi_mfgr_id),
	   .prod_id(spi_prod_id),
	   .mask_rev(spi_mask_rev)
	);

	/* Level shift down.  Unfortunately, PLL is in 1.8V only or	*/
	/* else this would be easier.					*/

	BU_3VX2 pass_thru_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(pass_thru),
	   .Q(pass_thru_lv)
	);
	BU_3VX2 pass_thru_csb_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(pass_thru_csb),
	   .Q(pass_thru_csb_lv)
	);
	BU_3VX2 pass_thru_sdi_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(pass_thru_sdi),
	   .Q(pass_thru_sdi_lv)
	);
	BU_3VX2 pass_thru_sck_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(pass_thru_sck),
	   .Q(pass_thru_sck_lv)
	);

	BU_3VX2 pll_vco_in_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_pll_vco_in),
	   .Q(spi_pll_vco_in_lv)
	);
	BU_3VX2 pll_vco_ena_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_pll_vco_ena),
	   .Q(spi_pll_vco_ena_lv)
	);
	BU_3VX2 pll_cp_ena_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_pll_cp_ena),
	   .Q(spi_pll_cp_ena_lv)
	);
	BU_3VX2 pll_trim_level [3:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_pll_trim),
	   .Q(spi_pll_trim_lv)
	);
	BU_3VX2 pll_bias_ena_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_pll_bias_ena),
	   .Q(spi_pll_bias_ena_lv)
	);
	BU_3VX2 spi_irq_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(spi_irq),
	   .Q(spi_irq_lv)
	);

	/* NVRAM module and charge pump */

	/* List of wires passed between NVRAM and charge pump */
	wire nvram_busynvc;
	wire [15:0] nvram_trim;
	wire nvram_clki;
	wire nvram_vsestart;
	wire nvram_rclt;
	wire nvram_mem1_ent;
	wire nvram_mem2_ent;
	wire nvram_clk4;
	wire nvram_vse1;
	wire nvram_vse2;
	wire nvram_vsebusy;
	wire nvram_vcore;
	wire [3:0] nvram_tm_nvcpi;

	XNVR_136X32P128_VD01 nvram (
`ifdef LVS
	    .VDD18M(VDD1V8),
	    .VSSM(VSS),
`endif
	    .BUSYNVC(nvram_busynvc),
	    .CLKI(nvram_clki),
	    .DOUT(nvram_rdata),
	    .DSO(),
	    .MEM1_ENT(nvram_mem1_ent),
	    .MEM2_ENT(nvram_mem2_ent),
	    .RCLT(nvram_rclt),
	    .RDY(nvram_rdy),
	    .TM_NVCPI(nvram_tm_nvcpi),
	    .TRIM(nvram_trim),
	    .VSESTART(nvram_vsestart),
	    .A(nvram_addr),
	    .CE(nvram_ena),
	    .CLK(nvram_clk),
	    .CLK4(nvram_clk4),
	    .DIN(nvram_wdata),
	    .DRSTN(dvdd),
	    .DSCLK(dground),
	    .DSI(dground),
	    .DUP(dground),
	    .HR(nvram_hr),
	    .HS(nvram_hs),
	    .MEM_ALLC(nvram_mem_all),
	    .MEM_SEL(nvram_mem_sel),
	    .PEIN(dground),
	    .POR(reset_lv),
	    .TM_NVCP(tm_nvcp_lv),
	    .VCORE(nvram_vcore),
	    .VSE1(nvram_vse1),
	    .VSE2(nvram_vse2),
	    .VSEBUSY(nvram_vsebusy),
	    .WE(nvram_wen)
	);

	XCPF_136X32DP128_VD03 nvram_cp (
`ifdef LVS
	    .VDD18M(VDD1V8),
	    .VDD33M(VDD3V3),
	    .VSSM(VSS),
`endif
	    .CLK4(nvram_clk4),
	    .CLK4M(),
	    .VCORE(nvram_vcore),
	    .VSE1(nvram_vse1),
	    .VSE2(nvram_vse2),
	    .VSEBUSY(nvram_vsebusy),
	    .BUSYNVC(nvram_busynvc),
	    .CLKI(nvram_clki),
	    .MEM1_ENT(nvram_mem1_ent),
	    .MEM2_ENT(nvram_mem2_ent),
	    .NVREF_EXT(nvref_ext),
	    .POR(reset_lv),
	    .RCLT(nvram_rclt),
	    .TM_NVCPI(nvram_tm_nvcpi),
	    .TRIM(nvram_trim),
	    .VSESTART(nvram_vsestart)
	);

	/* Level shift down */

	BU_3VX2 tm_nvcp_level [3:0] (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(tm_nvcp),
	   .Q(tm_nvcp_lv)
	);

	/* RAM modules (32 bits x 4k words) */

        wire [31:0] ram_wenb_bits;

	/* Split the four byte-wide write-enable lines into the X-Fab SRAM bit-wide write-enable lines */
	assign ram_wenb_bits[31:24] = {ram_wenb[3], ram_wenb[3], ram_wenb[3], ram_wenb[3], ram_wenb[3], ram_wenb[3], ram_wenb[3], ram_wenb[3]};
	assign ram_wenb_bits[23:16] = {ram_wenb[2], ram_wenb[2], ram_wenb[2], ram_wenb[2], ram_wenb[2], ram_wenb[2], ram_wenb[2], ram_wenb[2]};
	assign ram_wenb_bits[15:8] = {ram_wenb[1], ram_wenb[1], ram_wenb[1], ram_wenb[1], ram_wenb[1], ram_wenb[1], ram_wenb[1], ram_wenb[1]};
	assign ram_wenb_bits[7:0] = {ram_wenb[0], ram_wenb[0], ram_wenb[0], ram_wenb[0], ram_wenb[0], ram_wenb[0], ram_wenb[0], ram_wenb[0]};

        // XSPRAMBLP_4096X32_M8P xfab_mem0 (
        XSPRAMBLP_16384X32_M8P xfab_mem0 (
`ifdef LVS
	    .VSSM(VSS),
	    .VDD18M(VDD1V8),
`endif
            .Q(ram_rdata),
            .D(ram_wdata),
            .A(ram_addr),
            .CLK(pll_clk),
            .CEn(reset_lv),		// SRAM enable
            .WEn(ram_wenb_bits),	// one bit per byte 
            .RDY()			// unused
        );

	/* Analog components (multiplexers) */
	wire real adc0_input;
	wire real adc1_input;
	wire real comp_ninput;
	wire real comp_pinput;
	wire real opamp_input;
	wire real dac_out;
	wire real bandgap_out;

        AMUX4_3V adc0_input_mux (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .AIN1(adc0_in),
	   .AIN2(VDD1V8),
	   .AIN3(dac_out),
	   .AIN4(VSS),
	   .AOUT(adc0_input),
	   .SEL(adc0_inputsrc)
	);

        AMUX4_3V adc1_input_mux (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .AIN1(adc1_in),
	   .AIN2(VDD3V3),
	   .AIN3(bandgap_out),
	   .AIN4(comp_inp),
	   .AOUT(adc1_input),
	   .SEL(adc1_inputsrc)
	);

        AMUX4_3V comp_ninput_mux (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .AIN1(comp_inn),
	   .AIN2(dac_out),
	   .AIN3(bandgap_out),
	   .AIN4(VDD1V8),
	   .AOUT(comp_ninput),
	   .SEL(comp_ninputsrc)
	);

        AMUX4_3V comp_pinput_mux (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .AIN1(comp_inp),
	   .AIN2(dac_out),
	   .AIN3(bandgap_out),
	   .AIN4(VDD1V8),
	   .AOUT(comp_pinput),
	   .SEL(comp_pinputsrc)
	);

        AMUX2_3V analog_out_mux (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .AIN1(dac_out),
	   .AIN2(bandgap_out),
	   .AOUT(opamp_input),
	   .SEL(analog_out_sel)
	);

	/* ADC 0 */
	aadcc01_3v3 adc0 (
	   .VDD(VDD1V8),
	   .VIN(adc0_input),
	   .CLK(adc0_clk),
	   .VREFH(adc_high),
	   .EN(adc0_ena),
	   .VSSA(VSS),
	   .VDDA(VDD3V3),
	   .VREFL(adc_low),
	   .START(adc0_convert),
	   .EOC(adc0_done),
	   .D(adc0_data),
	   .VSS(VSS)
	);

	/* ADC 1 */
	aadcc01_3v3 adc1 (
	   .VDD(VDD1V8),
	   .VIN(adc1_input),
	   .CLK(adc1_clk),
	   .VREFH(adc_high),
	   .EN(adc1_ena),
	   .VSSA(VSS),
	   .VDDA(VDD3V3),
	   .VREFL(adc_low),
	   .START(adc1_convert),
	   .EOC(adc1_done),
	   .D(adc1_data),
	   .VSS(VSS)
	);

	/* DAC */
	adacc01_3v3 dac (
	   .OUT(dac_out),
	   .D(dac_value),
	   .EN(dac_ena),
	   .VDD(VDD1V8),
	   .VDDA(VDD3V3),
	   .VREFH(adc_high),
	   .VREFL(adc_low),
	   .VSS(VSS),
	   .VSSA(VSS)
	);

	wire real bias3u;

	/* Opamp (analog output buffer) */
	aopac01_3v3 opamp (
	   .OUT(analog_out),
	   .EN(opamp_ena_3v),
	   .IB(bias3u),
	   .INN(analog_out),
	   .INP(opamp_input),
	   .VDDA(VDD3V3),
	   .VSSA(VSS)
	);

	/* Level shift up */

	LS_3VX2 opamp_ena_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(opamp_ena),
	   .Q(opamp_ena_3v)
	);

	LS_3VX2 opamp_bias_ena_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(opamp_bias_ena),
	   .Q(opamp_bias_ena_3v)
	);

	/* Biasing for op-amp */
	acsoc02_3v3 opamp_bias (
	   .EN(opamp_bias_ena_3v),
	   .VDDA(VDD3V3),
	   .VSSA(VSS),
	   .CS_8U(),
	   .CS_4U(),
	   .CS_2U(bias3u),
	   .CS_1U(bias3u)
	);

	/* Level shift up */

	LS_3VX2 bg_ena_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(bg_ena),
	   .Q(bg_ena_3v)
	);

	/* Bandgap */
	abgpc01_3v3 bandgap (
	   .EN(bg_ena_3v),
	   .VBGP(bandgap_out),
	   .VSSA(VSS),
	   .VDDA(VDD3V3),
	   .VBGVTN()
	);

	wire real bias400n;

	/* Comparator */
	acmpc01_3v3 comparator (
	   .OUT(comp_out),
	   .EN(comp_ena_3v),
	   .IBN(bias400n),
	   .INN(comp_ninput),	// multiplexed
	   .INP(comp_pinput),	// multiplexed
	   .VDDA(VDD3V3),
	   .VSSA(VSS)
	);

	/* Level shift down */

	BU_3VX2 comp_out_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(comp_out),
	   .Q(comp_out_lv)
	);

	/* Level shift up */

	LS_3VX2 comp_ena_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(comp_ena),
	   .Q(comp_ena_3v)
	);

	/* Bias for comparator */
	acsoc01_3v3 comp_bias (
	   .EN(comp_ena_3v),
	   .VSSA(VSS),
	   .VDDA(VDD3V3),
	   .CS0_200N(bias400n),
	   .CS1_200N(bias400n),
	   .CS2_200N(),
	   .CS3_200N()
	);

	/* Crystal oscillator (5-12.5 MHz) */
	axtoc02_3v3 xtal (
	   .CLK(xtal_out),
	   .XI(XI),
	   .XO(XO),
	   .EN(spi_xtal_ena),
	   .GNDO(VSS),
	   .GNDR(VSS),
	   .VDD(VDD1V8),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3)
	);

	/* Level shift down (because xtal osc is 3V but PLL is 1.8V) */

	BU_3VX2 xtal_out_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(xtal_out),
	   .Q(xtal_out_buf)
	);

	/* Buffering of xtal_out clock */

	BUX4 BUX4_1 (
`ifdef LVS
		.vdd(VDD1V8),
		.gnd(VSS),
`endif
		.A(xtal_out_buf),
		.Q(xtal_out_lv)
	);

	/* SoC core uses xtal_in to drive ADC which has a max	*/
	/* clock rate of 2MHz.  Since crystal clock is		*/
	/* typically 10 to 12.5MHz, divide it by 8.		*/

	DFRX2 clock_div2 (
`ifdef LVS
	    .gnd(VSS),
	    .vdd(VDD1V8),
`endif
	    .D(xtal_div2_buf),
	    .C(xtal_out_lv),
	    .QN(xtal_div2_buf),
	    .Q(xtal_div2)
	);
	DFRX2 clock_div4 (
`ifdef LVS
	    .gnd(VSS),
	    .vdd(VDD1V8),
`endif
	    .D(xtal_div4_buf),
	    .C(xtal_div2),
	    .QN(xtal_div4_buf),
	    .Q(xtal_div4)
	);
	DFRX2 clock_div8 (
`ifdef LVS
	    .gnd(VSS),
	    .vdd(VDD1V8),
`endif
	    .D(xtal_div8_buf),
	    .C(xtal_div4),
	    .QN(xtal_div8_buf),
	    .Q(xtal_div8)
	);

	wire real bias10u, bias5u, pll_vco_in;

	/* Multiplexer for connecting VCO_IN			*/
	/* NOTE that this multiplexer is 3.3V.  It is the	*/
	/* responsibility of the hardware connected to the	*/
	/* comp_inp pin to limit VCO_IN to 1.2V max.		*/

        AMUX2_3V vco_in_mux (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .AIN1(VSS),
	   .AIN2(comp_inp),
	   .AOUT(pll_vco_in),
	   .SEL(spi_pll_vco_in_lv)
	);

	/* 8x clock multiplier PLL (NOTE: IP from A_CELLS_1V8) */
	apllc03_1v8 pll (
	   .VSSD(VSS),
	   .EN_VCO(spi_pll_vco_ena_lv),
	   .EN_CP(spi_pll_cp_ena_lv),
	   .B_VCO(bias5u),
	   .B_CP(bias10u),
	   .VSSA(VSS),
	   .VDDD(VDD1V8),
	   .VDDA(VDD1V8),
	   .VCO_IN(pll_vco_in),
	   .CLK(clk),		// output (fast) clock
	   .REF(xtal_out_lv),	// input (slow) clock
	   .B(spi_pll_trim_lv) 	// 4-bit trim
	);

	/* Buffering of PLL clock */

	BUX4 BUX4_0 (
`ifdef LVS
		.vdd(VDD1V8),
		.gnd(VSS),
`endif
		.A(clk),
		.Q(clk_buf)
	);

	BUX12 BUX12_1 (
`ifdef LVS
		.vdd(VDD1V8),
		.gnd(VSS),
`endif
		.A(clk_buf),
		.Q(pll_clk)
	);

	/* Biasing for PLL */
	acsoc04_1v8 pll_bias (
	   .EN(spi_pll_bias_ena_lv),
	   .VDDA(VDD1V8),
	   .VSSA(VSS),
	   .CS3_8u(bias10u),
	   .CS2_4u(bias5u),
	   .CS1_2u(bias10u),
	   .CS0_1u(bias5u)
	);

	/* Level shift up */

	LS_3VX2 rcosc_ena_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(rcosc_ena),
	   .Q(rcosc_ena_3v)
	);

	/* RC oscillator */
	arcoc01_3v3 rcosc (
	   .CLK(rcosc_out),
	   .EN(rcosc_ena_3v),
	   .VDDA(VDD3V3),
	   .VSSA(VSS)
	);

	/* Level shift down */

	BU_3VX2 rcosc_out_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(rcosc_out),
	   .Q(rcosc_out_lv)
	);

	/* Buffering for regulator enables.  This is mostly to		*/
	/* isolate an LVS error, but no harm comes of individually	*/
	/* buffering each regulator's enable lines.			*/

	BU_3VX2 reg_ena_buf0 (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .A(spi_reg_ena),
	   .Q(reg0_ena)
	);
	BU_3VX2 reg_ena_buf1 (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .A(spi_reg_ena),
	   .Q(reg1_ena)
	);
	IN_3VX2 reg_enb_buf0 (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .A(spi_reg_ena),
	   .Q(reg0_enb)
	);
	IN_3VX2 reg_enb_buf1 (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .A(spi_reg_ena),
	   .Q(reg1_enb)
	);

	/* reg1_ena/b has antenna taps due to the length of the wires */
	ANTENNACELLNP2_3V ANTENNACELLNP2_3V_0 (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .A(reg1_ena)
	);
	ANTENNACELLNP2_3V ANTENNACELLNP2_3V_1 (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD3V3),
`endif
	   .A(reg1_enb)
	);

	/* NOTE: iverilog does not like two devices connected in */
	/* parallel to one real-valued driver, so one device is	 */
	/* removed for simulation (hack solution).		 */

	/* 1.8V regulator (x2) */
	aregc01_3v3 regulator1 (
	   .OUT(VDD1V8),
	   .VIN3(VDD3V3),
	   .GNDO(VSS),
	   .EN(reg0_ena),
	   .GNDR(VSS),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3),
	   .VDD(VDD1V8),
	   .ENB(reg0_enb)
	);

`ifdef LVS
	aregc01_3v3 regulator2 (
	   .OUT(VDD1V8),
	   .VIN3(VDD3V3),
	   .GNDO(VSS),
	   .EN(reg1_ena),
	   .GNDR(VSS),
	   .VDDO(VDD3V3),
	   .VDDR(VDD3V3),
	   .VDD(VDD1V8),
	   .ENB(reg1_enb)
	);
`endif

	/* Power-on-reset */
	aporc02_3v3 por (
	   .POR(reset),
	   .PORB(resetn),
	   .VDDA(VDD3V3),
	   .VSSA(VSS)
	);

	/* Level shift down */

	BU_3VX2 por_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(reset),
	   .Q(reset_lv)
	);

	/* Level shift up */

	LS_3VX2 temp_level (
	   .VDD3V3(VDD3V3),
	   .VDD1V8(VDD1V8),
	   .VSSA(VSS),
	   .A(overtemp_ena),
	   .Q(overtemp_ena_3v)
	);

	/* Over-temperature alarm */
	atmpc01_3v3 temp (
	   .OVT(overtemp),
	   .EN(overtemp_ena_3v),
	   .VDDA(VDD3V3),
	   .VSSA(VSS)
	);

	/* Level shift down */

	BU_3VX2 overtemp_level (
`ifdef LVS
	   .gnd(VSS),
	   .vdd3(VDD1V8),
`endif
	   .A(overtemp),
	   .Q(overtemp_lv)
	);

endmodule	// ravenna
