/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
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

`ifdef PICORV32_V
`error "ravenna_soc_orig.v must be read before picorv32.v!"
`endif

/* Note:  Synthesize register memory from flops */
/* Inefficient, but not terribly so */

/* Also note:  To avoid having a hard macro in the place & route	*/
/* (method not finished yet in qflow), SRAM pins are brought out to	*/
/* the ravenna_soc I/O so that raven_soc.v itself is fully		*/
/* synthesizable and routable with qflow as-is.				*/

`define PICORV32_REGS ravenna_soc_regs

module ravenna_soc_orig (
	input pll_clk,
	input ext_clk,
	input ext_clk_sel,
	input ext_reset,
	input reset,

	// Pass-through mode from housekeeping SPI
	input  pass_thru,
	input  pass_thru_csb,
	input  pass_thru_sck,
	input  pass_thru_sdi,
	output pass_thru_sdo,

	// Main SRAM, including clk and resetn above
	output [3:0] ram_wenb,
	output [11:0] ram_addr,
	output [31:0] ram_wdata,
	input  [31:0] ram_rdata,

	// NVRAM, including resetn above
	output 		nvram_ena,
	output 		nvram_wen,
	output [7:0]	nvram_addr,
	output [31:0]	nvram_wdata,
	input  [31:0]	nvram_rdata,	
	output		nvram_clk,
	input		nvram_rdy,
	output 		nvram_hs,
	output 		nvram_hr,
	output 		nvram_mem_sel,
	output 		nvram_mem_all,
	// (To do:  Add additional control signals)

	// Memory mapped I/O signals
	output [15:0] gpio_out,
	input  [15:0] gpio_in,
	output [15:0] gpio_pullupb,
	output [15:0] gpio_pulldownb,
	output [15:0] gpio_outenb,

	output 	      adc0_ena,
	output 	      adc0_convert,
	input  [9:0]  adc0_data,
	input  	      adc0_done,
	output	      adc0_clk,
	output [1:0]  adc0_inputsrc,
	output 	      adc1_ena,
	output 	      adc1_convert,
	output	      adc1_clk,
	output [1:0]  adc1_inputsrc,
	input  [9:0]  adc1_data,
	input  	      adc1_done,

	output	      dac_ena,
	output [9:0]  dac_value,

	output	      analog_out_sel,	// Analog output select (DAC or bandgap)
	output	      opamp_ena,	// Op-amp enable for analog output
	output	      opamp_bias_ena,	// Op-amp bias enable for analog output
	output	      bg_ena,		// Bandgap enable

	output	      comp_ena,
	output [1:0]  comp_ninputsrc,
	output [1:0]  comp_pinputsrc,
	output	      rcosc_ena,

	output	      overtemp_ena,
	input	      overtemp,
	input	      rcosc_in,		// RC oscillator output
	input	      xtal_in,		// crystal oscillator output
	input	      comp_in,		// comparator output
	input	      spi_sck,

	input [7:0]   spi_ro_config,
	input 	      spi_ro_xtal_ena,
	input 	      spi_ro_reg_ena,
	input 	      spi_ro_pll_cp_ena,
	input 	      spi_ro_pll_vco_ena,
	input 	      spi_ro_pll_bias_ena,
	input [3:0]   spi_ro_pll_trim,

	input [11:0]  spi_ro_mfgr_id,
	input [7:0]   spi_ro_prod_id,
	input [3:0]   spi_ro_mask_rev,

	output ser_tx,
	input  ser_rx,

	output scl_pad_i,
	output scl_pad_o,
	output scl_padoeb,
	output sda_pad_i,
	output sda_pad_o,
	output sda_padoeb,

	output spi_master_sdo,
	output spi_master_csb,
	output spi_master_sck,
	input  spi_master_sdi,
	output spi_master_sdoenb,

	// IRQ
	input  irq_pin,		// dedicated IRQ pin
	input  irq_spi,		// IRQ from standalone SPI

	// trap
	output trap,

	// Flash memory control (SPI master)
	output flash_csb,
	output flash_clk,

	output flash_clk_oeb,
	output flash_csb_oeb,

	output flash_io0_oeb,
	output flash_io1_oeb,
	output flash_io2_oeb,
	output flash_io3_oeb,

	output flash_io0_do,
	output flash_io1_do,
	output flash_io2_do,
	output flash_io3_do,

	input  flash_io0_di,
	input  flash_io1_di,
	input  flash_io2_di,
	input  flash_io3_di
);
	/* parameter integer MEM_WORDS = 256; */
	/* Increase scratchpad memory to 4K words */
	parameter integer MEM_WORDS = 4096;
	parameter [31:0] STACKADDR = (4*MEM_WORDS);       // end of memory
	parameter [31:0] PROGADDR_RESET = 32'h 0010_0000; // 1 MB into flash

	wire	      resetn;
	wire	      clk;

	wire          iomem_valid;
	reg           iomem_ready;
	wire   [ 3:0] iomem_wstrb;
	wire   [31:0] iomem_addr;
	wire   [31:0] iomem_wdata;
	reg    [31:0] iomem_rdata;

	// memory-mapped I/O control registers

	wire   [15:0] gpio_pullupb;
	wire   [15:0] gpio_pulldownb;
	wire   [15:0] gpio_outenb;

	reg    [15:0] gpio;		// GPIO output data
	reg    [15:0] gpio_pu;		// GPIO pull-up enable
	reg    [15:0] gpio_pd;		// GPIO pull-down enable
	reg    [15:0] gpio_oeb;		// GPIO output enable (sense negative)
	reg 	      adc0_ena;		// ADC0 enable
	reg 	      adc0_convert;	// ADC0 convert
	reg    [1:0]  adc0_clksrc;	// ADC0 clock source
	reg    [1:0]  adc0_inputsrc;	// ADC0 input source
	reg 	      adc1_ena;		// ADC1 enable
	reg 	      adc1_convert;	// ADC1 convert
	reg    [1:0]  adc1_clksrc;	// ADC1 clock source
	reg    [1:0]  adc1_inputsrc;	// ADC1 input source
	reg	      dac_ena;		// DAC enable
	reg    [9:0]  dac_value;	// DAC output value
	reg	      comp_ena;		// Comparator enable
	reg    [1:0]  comp_ninputsrc;	// Comparator negative input source
	reg    [1:0]  comp_pinputsrc;	// Comparator positive input source
	reg	      rcosc_ena;	// RC oscillator enable
	reg	      overtemp_ena;	// Over-temperature alarm enable
	reg    [1:0]  comp_output_dest; // Comparator output destination
	reg    [1:0]  rcosc_output_dest; // RC oscillator output destination
	reg    [1:0]  overtemp_dest;	// Over-temperature alarm destination
	reg    [1:0]  pll_output_dest;	// PLL clock output destination
	reg    [1:0]  xtal_output_dest; // Crystal oscillator output destination
	reg    [1:0]  trap_output_dest; // Trap signal output destination
	reg    [1:0]  irq_7_inputsrc;	// IRQ 5 source
	reg    [1:0]  irq_8_inputsrc;	// IRQ 6 source
	reg	      analog_out_sel;	// Analog output select
 	reg	      opamp_ena;	// Analog output op-amp enable
 	reg	      opamp_bias_ena;	// Analog output op-amp bias enable
 	reg	      bg_ena;		// Bandgap enable
	wire	      adc0_clk;		// ADC0 clock (multiplexed)
	wire	      adc1_clk;		// ADC1 clock (multiplexed)

	wire          nvram_ena;	// NVRAM SRAM enable
	reg	      nvram_hs;		// NVRAM store
	reg 	      nvram_hr;		// NVRAM recall
	reg 	      nvram_mem_sel;	// NVRAM bank select
	reg 	      nvram_mem_all;	// NVRAM select all
 	reg  [4:0]    nvram_clkdiv;	// Clock divider for 4MHz NVRAM clock
	wire	      nvram_clk;	// Divided-down clock

	wire [3:0] ram_wenb;
	wire [11:0] ram_addr;
	wire [31:0] ram_wdata;

	wire [7:0]  nvram_addr;
	wire [31:0] nvram_wdata;

	// Reset assignment.  "reset" comes from POR, while "ext_reset"
	// comes from standalone SPI (and is normally zero unless
	// activated from the SPI).

	assign resetn = ~(reset | ext_reset);

	// Clock assignment
	assign clk = (ext_clk_sel == 1'b1) ? ext_clk : pll_clk;

	// ADC clock assignments
	
	assign adc0_clk = (adc0_clksrc == 2'b00) ? rcosc_in :
			  (adc0_clksrc == 2'b01) ? spi_sck :
			  (adc0_clksrc == 2'b10) ? xtal_in :
			  ext_clk;

	assign adc1_clk = (adc1_clksrc == 2'b00) ? rcosc_in :
			  (adc1_clksrc == 2'b01) ? spi_sck :
			  (adc1_clksrc == 2'b10) ? xtal_in :
			  ext_clk;

	// GPIO assignments

	assign gpio_out[0] = (comp_output_dest == 2'b01) ? comp_in : gpio[0];
	assign gpio_out[1] = (comp_output_dest == 2'b10) ? comp_in : gpio[1];
	assign gpio_out[2] = (rcosc_output_dest == 2'b01) ? rcosc_in : gpio[2];
	assign gpio_out[3] = (rcosc_output_dest == 2'b10) ? rcosc_in : gpio[3];
	assign gpio_out[4] = (rcosc_output_dest == 2'b11) ? rcosc_in : gpio[4];
	assign gpio_out[5] = (xtal_output_dest == 2'b01) ? xtal_in : gpio[5]; 
	assign gpio_out[6] = (xtal_output_dest == 2'b10) ? xtal_in : gpio[6]; 
	assign gpio_out[7] = (xtal_output_dest == 2'b11) ? xtal_in : gpio[7]; 
	assign gpio_out[8] = (pll_output_dest == 2'b01) ? pll_clk : gpio[8];
	assign gpio_out[9] = (pll_output_dest == 2'b10) ? clk : gpio[9];
	assign gpio_out[10] = (pll_output_dest == 2'b11) ? nvram_clk : gpio[10];
	assign gpio_out[11] = (trap_output_dest == 2'b01) ? trap : gpio[11];
	assign gpio_out[12] = (trap_output_dest == 2'b10) ? trap : gpio[12];
	assign gpio_out[13] = (trap_output_dest == 2'b11) ? trap : gpio[13];
	assign gpio_out[14] = (overtemp_dest == 2'b01) ? overtemp : gpio[14];
	assign gpio_out[15] = (overtemp_dest == 2'b10) ? overtemp : gpio[15];

	assign gpio_outenb[0] = ~resetn | ((comp_output_dest == 2'b00)  ? gpio_oeb[0] : 1'b0);
	assign gpio_outenb[1] = ~resetn | ((comp_output_dest == 2'b00)  ? gpio_oeb[1] : 1'b0);
	assign gpio_outenb[2] = ~resetn | ((rcosc_output_dest == 2'b00) ? gpio_oeb[2] : 1'b0); 
	assign gpio_outenb[3] = ~resetn | ((rcosc_output_dest == 2'b00) ? gpio_oeb[3] : 1'b0);
	assign gpio_outenb[4] = ~resetn | ((rcosc_output_dest == 2'b00) ? gpio_oeb[4] : 1'b0);
	assign gpio_outenb[5] = ~resetn | ((xtal_output_dest == 2'b00)  ? gpio_oeb[5] : 1'b0);
	assign gpio_outenb[6] = ~resetn | ((xtal_output_dest == 2'b00)  ? gpio_oeb[6] : 1'b0);
	assign gpio_outenb[7] = ~resetn | ((xtal_output_dest == 2'b00)  ? gpio_oeb[7] : 1'b0);
	assign gpio_outenb[8] = ~resetn | ((pll_output_dest == 2'b00)   ? gpio_oeb[8] : 1'b0);
	assign gpio_outenb[9] = ~resetn | ((pll_output_dest == 2'b00)   ? gpio_oeb[9] : 1'b0);
	assign gpio_outenb[10] = ~resetn | ((pll_output_dest == 2'b00)  ? gpio_oeb[10] : 1'b0);
	assign gpio_outenb[11] = ~resetn | ((trap_output_dest == 2'b00) ? gpio_oeb[11] : 1'b0);
	assign gpio_outenb[12] = ~resetn | ((trap_output_dest == 2'b00) ? gpio_oeb[12] : 1'b0);
	assign gpio_outenb[13] = ~resetn | ((trap_output_dest == 2'b00) ? gpio_oeb[13] : 1'b0);
	assign gpio_outenb[14] = ~resetn | ((overtemp_dest == 2'b00)    ? gpio_oeb[14] : 1'b0);
	assign gpio_outenb[15] = ~resetn | ((overtemp_dest == 2'b00)    ? gpio_oeb[15] : 1'b0);

	assign gpio_pullupb[0] = (comp_output_dest == 2'b00)  ? gpio_pu[0] : 1'b1;
	assign gpio_pullupb[1] = (comp_output_dest == 2'b00)  ? gpio_pu[1] : 1'b1;
	assign gpio_pullupb[2] = (rcosc_output_dest == 2'b00) ? gpio_pu[2] : 1'b1; 
	assign gpio_pullupb[3] = (rcosc_output_dest == 2'b00) ? gpio_pu[3] : 1'b1;
	assign gpio_pullupb[4] = (rcosc_output_dest == 2'b00) ? gpio_pu[4] : 1'b1;
	assign gpio_pullupb[5] = (xtal_output_dest == 2'b00)  ? gpio_pu[5] : 1'b1;
	assign gpio_pullupb[6] = (xtal_output_dest == 2'b00)  ? gpio_pu[6] : 1'b1;
	assign gpio_pullupb[7] = (xtal_output_dest == 2'b00)  ? gpio_pu[7] : 1'b1;
	assign gpio_pullupb[8] = (pll_output_dest == 2'b00)   ? gpio_pu[8] : 1'b1;
	assign gpio_pullupb[9] = (pll_output_dest == 2'b00)   ? gpio_pu[9] : 1'b1;
	assign gpio_pullupb[10] = (pll_output_dest == 2'b00)  ? gpio_pu[10] : 1'b1;
	assign gpio_pullupb[11] = (trap_output_dest == 2'b00) ? gpio_pu[11] : 1'b1;
	assign gpio_pullupb[12] = (trap_output_dest == 2'b00) ? gpio_pu[12] : 1'b1;
	assign gpio_pullupb[13] = (trap_output_dest == 2'b00) ? gpio_pu[13] : 1'b1;
	assign gpio_pullupb[14] = (overtemp_dest == 2'b00)    ? gpio_pu[14] : 1'b1;
	assign gpio_pullupb[15] = (overtemp_dest == 2'b00)    ? gpio_pu[15] : 1'b1;

	assign gpio_pulldownb[0] = (comp_output_dest == 2'b00)  ? gpio_pd[0] : 1'b1;
	assign gpio_pulldownb[1] = (comp_output_dest == 2'b00)  ? gpio_pd[1] : 1'b1;
	assign gpio_pulldownb[2] = (rcosc_output_dest == 2'b00) ? gpio_pd[2] : 1'b1; 
	assign gpio_pulldownb[3] = (rcosc_output_dest == 2'b00) ? gpio_pd[3] : 1'b1;
	assign gpio_pulldownb[4] = (rcosc_output_dest == 2'b00) ? gpio_pd[4] : 1'b1;
	assign gpio_pulldownb[5] = (xtal_output_dest == 2'b00)  ? gpio_pd[5] : 1'b1;
	assign gpio_pulldownb[6] = (xtal_output_dest == 2'b00)  ? gpio_pd[6] : 1'b1;
	assign gpio_pulldownb[7] = (xtal_output_dest == 2'b00)  ? gpio_pd[7] : 1'b1;
	assign gpio_pulldownb[8] = (pll_output_dest == 2'b00)   ? gpio_pd[8] : 1'b1;
	assign gpio_pulldownb[9] = (pll_output_dest == 2'b00)   ? gpio_pd[9] : 1'b1;
	assign gpio_pulldownb[10] = (pll_output_dest == 2'b00)  ? gpio_pd[10] : 1'b1;
	assign gpio_pulldownb[11] = (trap_output_dest == 2'b00) ? gpio_pd[11] : 1'b1;
	assign gpio_pulldownb[12] = (trap_output_dest == 2'b00) ? gpio_pd[12] : 1'b1;
	assign gpio_pulldownb[13] = (trap_output_dest == 2'b00) ? gpio_pd[13] : 1'b1;
	assign gpio_pulldownb[14] = (overtemp_dest == 2'b00)    ? gpio_pd[14] : 1'b1;
	assign gpio_pulldownb[15] = (overtemp_dest == 2'b00)    ? gpio_pd[15] : 1'b1;

	wire irq_7, irq_8;

	assign irq_7 = (irq_7_inputsrc == 2'b01) ? gpio_in[0] :
		       (irq_7_inputsrc == 2'b10) ? gpio_in[1] :
		       (irq_7_inputsrc == 2'b11) ? gpio_in[2] : 1'b0;
	assign irq_8 = (irq_8_inputsrc == 2'b01) ? gpio_in[3] :
		       (irq_8_inputsrc == 2'b10) ? gpio_in[4] :
		       (irq_8_inputsrc == 2'b11) ? gpio_in[5] : 1'b0;

	assign ram_wenb = (mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS) ?
		{~mem_wstrb[3], ~mem_wstrb[2], ~mem_wstrb[1], ~mem_wstrb[0]} : 4'hf;
        assign ram_addr = mem_addr[13:2];
	assign ram_wdata = mem_wdata;		// Just for naming conventions.

	assign nvram_wen = (mem_valid && !mem_ready &&
		(mem_addr[31:7] == 25'h0050000)) ?
		mem_wstrb[3] | mem_wstrb[2] | mem_wstrb[1] | mem_wstrb[0] : 4'h0;
	assign nvram_addr = mem_addr[9:2];
	assign nvram_wdata = mem_wdata;

	reg [31:0] irq;
	wire irq_stall = 0;	// Not implemented
	wire irq_uart;
	wire irq_i2c;
	wire irq_spi_master;

	always @* begin
		irq = 0;
		irq[3] = irq_stall;
		irq[4] = irq_uart;
		irq[5] = irq_pin;
		irq[6] = irq_spi;
		irq[7] = irq_7;
		irq[8] = irq_8;
		irq[9] = comp_output_dest[0] & comp_output_dest[1] & comp_in;
		irq[10] = overtemp_dest[0] & overtemp_dest[1] & overtemp;
		irq[11] = irq_i2c;
		irq[12] = irq_spi_master;
		irq[13] = timer0_irq_out;
	end

	wire mem_valid;
	wire mem_instr;
	wire mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	wire [31:0] mem_rdata;

	wire spimem_ready;
	wire [31:0] spimem_rdata;

	reg ram_ready;
	wire [31:0] ram_rdata;

	reg nvram_ready;
	wire [31:0] nvram_rdata;

	assign iomem_valid = mem_valid && (mem_addr[31:24] > 8'h 02);
	assign iomem_wstrb = mem_wstrb;
	assign iomem_addr = mem_addr;
	assign iomem_wdata = mem_wdata;

	wire spimemio_cfgreg_sel = mem_valid && (mem_addr == 32'h 0200_0000);
	wire [31:0] spimemio_cfgreg_do;

	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;
	wire        simpleuart_reg_dat_wait;

	wire        simplei2c_reg_cfg1_sel = mem_valid && (mem_addr == 32'h 0200_0010);
	wire [31:0] simplei2c_reg_cfg1_do;
	wire        simplei2c_reg_cfg2_sel = mem_valid && (mem_addr == 32'h 0200_0014);
	wire [31:0] simplei2c_reg_cfg2_do;

	wire        simplei2c_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0018);
	wire [31:0] simplei2c_reg_dat_do;
	wire        simplei2c_reg_dat_wait;

	wire        simplespi_reg_cfg_sel = mem_valid && (mem_addr == 32'h 0200_0020);
	wire [31:0] simplespi_reg_cfg_do;

	wire        simplespi_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0024);
	wire [31:0] simplespi_reg_dat_do;
	wire        simplespi_reg_dat_wait;

	// Note that NVRAM "CE" is essentially a read/write access pulse.
	assign nvram_ena = nvram_ready;

	assign mem_ready = (iomem_valid && iomem_ready) ||
		spimem_ready || ram_ready || nvram_ready ||
		spimemio_cfgreg_sel ||
		simpleuart_reg_div_sel ||
		(simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait) ||
		simplei2c_reg_cfg1_sel || simplei2c_reg_cfg2_sel ||
		(simplei2c_reg_dat_sel && !simplei2c_reg_dat_wait) ||
		simplespi_reg_cfg_sel ||
		(simplespi_reg_dat_sel && !simplespi_reg_dat_wait);

	assign mem_rdata = (iomem_valid && iomem_ready) ? iomem_rdata :
			spimem_ready ? spimem_rdata :
			ram_ready ? ram_rdata :
			nvram_ready ? nvram_rdata :
			spimemio_cfgreg_sel ? spimemio_cfgreg_do :
			simpleuart_reg_div_sel ? simpleuart_reg_div_do :
			simpleuart_reg_dat_sel ? simpleuart_reg_dat_do :
			simplei2c_reg_cfg1_sel ? simplei2c_reg_cfg1_do :
			simplei2c_reg_cfg2_sel ? simplei2c_reg_cfg2_do :
			simplei2c_reg_dat_sel ? simplei2c_reg_dat_do :
			simplespi_reg_cfg_sel ? simplespi_reg_cfg_do :
			simplespi_reg_dat_sel ? simplespi_reg_dat_do :
			32'h 0000_0000;

	picorv32 #(
		.STACKADDR(STACKADDR),
		.PROGADDR_RESET(PROGADDR_RESET),
		.PROGADDR_IRQ(32'h 0000_0000),
		.BARREL_SHIFTER(1),
		.COMPRESSED_ISA(1),
		.ENABLE_MUL(1),
		.ENABLE_DIV(1),
		.ENABLE_IRQ(1),
		.ENABLE_IRQ_QREGS(0)
	) cpu (
		.clk         (clk        ),
		.resetn      (resetn     ),
		.mem_valid   (mem_valid  ),
		.mem_instr   (mem_instr  ),
		.mem_ready   (mem_ready  ),
		.mem_addr    (mem_addr   ),
		.mem_wdata   (mem_wdata  ),
		.mem_wstrb   (mem_wstrb  ),
		.mem_rdata   (mem_rdata  ),
		.irq         (irq        ),
		.trap        (trap       )
	);

	spimemio spimemio (
		.clk    (clk),
		.resetn (resetn),
		.valid  (mem_valid && mem_addr >= 4*MEM_WORDS && mem_addr < 32'h 0200_0000),
		.ready  (spimem_ready),
		.addr   (mem_addr[23:0]),
		.rdata  (spimem_rdata),

		.pass_thru (pass_thru),
		.pass_thru_csb (pass_thru_csb),
		.pass_thru_sck (pass_thru_sck),
		.pass_thru_sdi (pass_thru_sdi),
		.pass_thru_sdo (pass_thru_sdo),

		.flash_csb    (flash_csb   ),
		.flash_clk    (flash_clk   ),

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
		.flash_io3_di (flash_io3_di),

		.cfgreg_we(spimemio_cfgreg_sel ? mem_wstrb : 4'b 0000),
		.cfgreg_di(mem_wdata),
		.cfgreg_do(spimemio_cfgreg_do)
	);

	simpleuart simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (ser_tx      ),
		.ser_rx      (ser_rx      ),

		.reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
		.reg_div_di  (mem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait),
		.irq_out     (irq_uart)
	);

	simple_i2c_master simplei2c (
		.clk	(clk	),
		.resetn	(resetn	),
		.reg_cfg1_we (simplei2c_reg_cfg1_sel ? mem_wstrb[2:0] : 3'b000),
		.reg_cfg1_di (mem_wdata),
		.reg_cfg1_do (simplei2c_reg_cfg1_do),
		.reg_cfg2_we (simplei2c_reg_cfg2_sel ? mem_wstrb[0] : 1'b0),
		.reg_cfg2_di (mem_wdata),
		.reg_cfg2_do (simplei2c_reg_cfg2_do),
		.reg_dat_we  (simplei2c_reg_dat_sel ? mem_wstrb[0] : 1'b0),
		.reg_dat_re  (simplei2c_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simplei2c_reg_dat_do),
		.reg_dat_wait(simplei2c_reg_dat_wait),
		.irq_o(irq_i2c),
		.scl_pad_i(scl_pad_i),
		.scl_pad_o(scl_pad_o),
		.scl_padoeb_o(scl_padoeb),
		.sda_pad_i(scl_pad_i),
		.sda_pad_o(scl_pad_o),
		.sda_padoeb_o(scl_padoeb)
	);

        simple_spi_master spi_master (
		.resetn(resetn),
		.clk(clk),
		.reg_cfg_we(simplespi_reg_cfg_sel ? mem_wstrb[1:0] : 2'b00),
		.reg_cfg_di(mem_wdata),
		.reg_cfg_do(simplespi_reg_cfg_do),
		.reg_dat_we(simplespi_reg_dat_sel ? mem_wstrb[0] : 1'b0),
		.reg_dat_re(simplespi_reg_dat_sel && !mem_wstrb),
		.reg_dat_di(mem_wdata),
		.reg_dat_do(simplespi_reg_dat_do),
		.reg_dat_wait(simplespi_reg_dat_wait),
		.irq_out(irq_spi_master),
		.sdi(spi_master_sdi),
		.csb(spi_master_csb),
		.sck(spi_master_sck),
		.sdo(spi_master_sdo),
		.sdoenb(spi_master_sdoenb)
	);

	always @(posedge clk) begin
		ram_ready <= mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS;
		nvram_ready <= mem_valid && !mem_ready &&
				(mem_addr[31:7] == 25'h0050000);
	end

	// PicoSoC memory mapped IP
	// 2 ADCs (1 multiplexed from internal signals, including core 1.8V VDD,
	//	DAC output, comparator input, external input)
	// 1 DAC
	// 1 comparator (1 end tied to DAC, other could be shared w/ADC input)
	// 1 RC oscillator (output can be tied to one or both ADC clocks)
	// 1 crystal oscillator (output to level-shift-down = 3V buffer powered at 1.8V)
	// 1 1.8V regulator (sets VDD on padframe)
	// 1 bandgap
	// 1 power-on-reset (POR)
	// 1 temperature alarm
	
	// NOTE: Signals affecting critical core functions are controlled through
	// an independent SPI having read-only access through the picorv32 core.
	// SPI pins are independent of picorv32 SPI master.  Signals controlled by
	// the SPI are:
	// 1) crystal oscillator enable (default on)
	// 2) 1.8V regulator enable (default on)
	// 3) bandgap enable (default on)
	// 4) picorv32 internal debug signals (TBD)
	// 5) additional picorv32 IRQ (TBD)
	// 6) PLL enables (default on)
	// 7) PLL trim (default TBD)
	// NOTE:  SPI should have a pass-through mode that configures SDO as a
	// copy of a chosen signal for as long as CSB is held low.  This can be
	// an SPI command, allows other internal signals to be passed to the
	// output and viewed, including the RC oscillator output, comparator output,
	// and other edge-based signals.

	// Memory map:
	// NOTE:

	// SPI master:	0x02000000	(control)
	// UART:	0x02000004-8	(clock, data)
	// I2C master:	0x02000010-f	(clock, data)
	// NVRAM:	0x02800000-1fc	(memory)
	// GPIO:	0x03000000	(in/out, pu/pd, data)
	// ADC0:	0x03000020
	// ADC1:	0x03000040
	// DAC:		0x03000060
	// comparator:	0x03000080
	// RC osc:	0x030000a0
	// SPI slave:	0x030000c0	(read-only)

	// Memory map details:
	// GPIO:	32 channels total.  
	//		addr 0x03000000		data (16 bits)
	//		addr 0x03000001		out (=1) or in (=0) (default 0)
	//		addr 0x03000002		pu (=1) or none (=0) (default 0)
	//		addr 0x03000003		pd (=1) or none (=0) (default 0)
	//		addr 0x03000004-f 	reserved (may be used for other pad I/O)
	//
	// ADC0:	addr 0x03000020		enable
	// 		addr 0x03000021		data (read-only)
	//      	addr 0x03000022		done (read-only)
	//		addr 0x03000023		start conversion
	//		addr 0x03000024		clock source (RC osc, SPI clk, xtal, core)
	//		addr 0x03000025		input source (core VDD, ext, DAC, comp in)
	//
	// ADC1:	addr 0x03000040		enable
	// 		addr 0x03000041		data (read-only)
	//      	addr 0x03000042		done (read-only)
	//		addr 0x03000043		start conversion
	//		addr 0x03000044		clock source (RC osc, SPI clk, xtal, core)
	//		addr 0x03000045		input source (bg, ext, I/O vdd, gnd)
	//
	// DAC:		addr 0x03000060		enable
	//     		addr 0x03000061		value
	//
	// comparator:  addr 0x03000080		enable
	//		addr 0x03000081		value
	//		addr 0x03000082		input source (DAC, bg, core VDD, ext)
	//		addr 0x03000083		output dest (ext gpio pin 0-1, IRQ, none)
	//
	// bandgap:	addr 0x03000090		enable
	//
	// RC osc:	addr 0x030000a0		enable
	//		addr 0x030000a1		output dest (ext gpio pin 2-4)
	//
	// SPI slave:	addr 0x030000c0		SPI configuration
	//		addr 0x030000c1		xtal osc, reg, bg enables
	// 		addr 0x030000c2		PLL enables, trim
	// 		addr 0x030000c3		manufacturer ID
	// 		addr 0x030000c4		product ID
	// 		addr 0x030000c5		product mask revision
	// Xtal mon:	addr 0x030000c6		xtal osc output dest (ext gpio pin 5-7)
	// PLL mon:	addr 0x030000c7		PLL output dest (ext gpio pin 8-10)
	// trap mon:	addr 0x030000c8		trap output dest (ext gpio pin 11-13)
	// IRQ7 src:	addr 0x030000c9		IRQ 7 source (ext gpio pin 0-3)
	// IRQ8 src:	addr 0x030000ca		IRQ 8 source (ext gpio pin 4-7)
	// Analog:	addr 0x030000cb		analog output select (DAC, bg)
	//
	// Overtemp:	addr 0x030000e0		over-temperature alarm enable
	//		addr 0x030000e1		over-temperature alarm data
	//		addr 0x030000e2		output dest (ext gpio pin 14-15, IRQ)
	//
	// NVRAM:	addr 0x02700000		program control
	//		addr 0x02700004		NVRAM 4MHz clock divider

	always @(posedge clk) begin
		if (!resetn) begin
			gpio <= 0;
			gpio_oeb <= 16'hffff;
			gpio_pu <= 0;
			gpio_pd <= 0;
			adc0_ena <= 0;
			adc0_convert <= 0;
			adc0_clksrc <= 0;
			adc0_inputsrc <= 0;
			adc1_ena <= 0;
			adc1_convert <= 0;
			adc1_clksrc <= 0;
			adc1_inputsrc <= 0;
			dac_ena <= 0;
			dac_value <= 0;
			comp_ena <= 0;
			comp_ninputsrc <= 0;
			comp_pinputsrc <= 0;
			rcosc_ena <= 0;
			comp_output_dest <= 0;
			rcosc_output_dest <= 0;
			overtemp_dest <= 0;
			overtemp_ena <= 0;
			pll_output_dest <= 0;
			xtal_output_dest <= 0;
			trap_output_dest <= 0;
			irq_7_inputsrc <= 0;
			irq_8_inputsrc <= 0;
			analog_out_sel <= 0;
			opamp_ena <= 0;
			opamp_bias_ena <= 0;
			bg_ena <= 0;
			nvram_hs <= 0;
			nvram_hr <= 0;
			nvram_mem_sel <= 0;
			nvram_mem_all <= 0;
			nvram_clkdiv <= 5'd24;	// master clock / (clkdiv + 1) = 4MHz

		end else begin
			iomem_ready <= 0;
			if (iomem_valid && !iomem_ready && iomem_addr[31:8] == 24'h030000) begin
				iomem_ready <= 1;
				case (iomem_addr[7:0])
				    8'h00: begin
					iomem_rdata <= {gpio_out, gpio_in};
					if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
					if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
				    end
				    8'h04: begin
					iomem_rdata <= {16'd0, gpio_oeb};
					if (iomem_wstrb[0]) gpio_oeb[ 7: 0] <= iomem_wdata[ 7: 0];
					if (iomem_wstrb[1]) gpio_oeb[15: 8] <= iomem_wdata[15: 8];
				    end
				    8'h08: begin
					iomem_rdata <= {16'd0, gpio_pu};
					if (iomem_wstrb[0]) gpio_pu[ 7: 0] <= iomem_wdata[ 7: 0];
					if (iomem_wstrb[1]) gpio_pu[15: 8] <= iomem_wdata[15: 8];
				    end
				    8'h0c: begin
					iomem_rdata <= {16'd0, gpio_pu};
					if (iomem_wstrb[0]) gpio_pd[ 7: 0] <= iomem_wdata[ 7: 0];
					if (iomem_wstrb[1]) gpio_pd[15: 8] <= iomem_wdata[15: 8];
				    end
				    8'h10: begin
					iomem_rdata <= {31'd0, adc0_ena};
					if (iomem_wstrb[0]) adc0_ena <= iomem_wdata[0];
				    end
				    8'h14: begin
					iomem_rdata <= {22'd0, adc0_data};
				    end
				    8'h18: begin
					iomem_rdata <= {31'd0, adc0_done};
				    end
				    8'h1c: begin
					iomem_rdata <= {31'd0, adc0_convert};
					if (iomem_wstrb[0]) adc0_convert <= iomem_wdata[0];
				    end
				    8'h20: begin
					iomem_rdata <= {30'd0, adc0_clksrc};
					if (iomem_wstrb[0]) adc0_clksrc <= iomem_wdata[1:0];
				    end
				    8'h24: begin
					iomem_rdata <= {30'd0, adc0_inputsrc};
					if (iomem_wstrb[0]) adc0_inputsrc <= iomem_wdata[1:0];
				    end
				    8'h30: begin
					iomem_rdata <= {31'd0, adc1_ena};
					if (iomem_wstrb[0]) adc1_ena <= iomem_wdata[0];
				    end
				    8'h34: begin
					iomem_rdata <= {22'd0, adc1_data};
				    end
				    8'h38: begin
					iomem_rdata <= {31'd0, adc1_done};
				    end
				    8'h3c: begin
					iomem_rdata <= {31'd0, adc1_convert};
					if (iomem_wstrb[0]) adc1_convert <= iomem_wdata[0];
				    end
				    8'h40: begin
					iomem_rdata <= {30'd0, adc1_clksrc};
					if (iomem_wstrb[0]) adc1_clksrc <= iomem_wdata[1:0];
				    end
				    8'h44: begin
					iomem_rdata <= {30'd0, adc1_inputsrc};
					if (iomem_wstrb[0]) adc1_inputsrc <= iomem_wdata[1:0];
				    end
				    8'h50: begin
					iomem_rdata <= {31'd0, dac_ena};
					if (iomem_wstrb[0]) dac_ena <= iomem_wdata[0];
				    end
				    8'h54: begin
					iomem_rdata <= {22'd0, dac_value};
					if (iomem_wstrb[0]) dac_value[7:0] <= iomem_wdata[7:0];
					if (iomem_wstrb[1]) dac_value[9:8] <= iomem_wdata[9:8];
				    end
				    8'h60: begin
					iomem_rdata <= {31'd0, comp_ena};
					if (iomem_wstrb[0]) comp_ena <= iomem_wdata[0];
				    end
				    8'h64: begin
					iomem_rdata <= {30'd0, comp_ninputsrc};
					if (iomem_wstrb[0]) comp_ninputsrc <= iomem_wdata[1:0];
				    end
				    8'h68: begin
					iomem_rdata <= {30'd0, comp_pinputsrc};
					if (iomem_wstrb[0]) comp_pinputsrc <= iomem_wdata[1:0];
				    end
				    8'h6c: begin
					iomem_rdata <= {30'd0, comp_output_dest};
					if (iomem_wstrb[0]) comp_output_dest <= iomem_wdata[1:0];
				    end
				    8'h70: begin
					iomem_rdata <= {31'd0, rcosc_ena};
					if (iomem_wstrb[0]) rcosc_ena <= iomem_wdata[0];
				    end
				    8'h74: begin
					iomem_rdata <= {30'd0, rcosc_output_dest};
					if (iomem_wstrb[0]) rcosc_output_dest <= iomem_wdata[1:0];
				    end
				    8'h80: begin
					iomem_rdata <= {24'd0, spi_ro_config};
				    end
				    8'h84: begin
					iomem_rdata <= {30'd0, spi_ro_xtal_ena, spi_ro_reg_ena};
				    end
				    8'h88: begin
					iomem_rdata <= {25'd0, spi_ro_pll_trim, spi_ro_pll_cp_ena, spi_ro_pll_vco_ena, spi_ro_pll_bias_ena};
				    end
				    8'h8c: begin
					iomem_rdata <= {20'd0, spi_ro_mfgr_id};
				    end
				    8'h90: begin
					iomem_rdata <= {24'd0, spi_ro_prod_id};
				    end
				    8'h94: begin
					iomem_rdata <= {28'd0, spi_ro_mask_rev};
				    end
				    8'h98: begin
					iomem_rdata <= {31'd0, ext_clk_sel};
				    end
				    8'ha0: begin
					iomem_rdata <= {30'd0, xtal_output_dest};
					if (iomem_wstrb[0]) xtal_output_dest <= iomem_wdata[1:0];
				    end
				    8'ha4: begin
					iomem_rdata <= {30'd0, pll_output_dest};
					if (iomem_wstrb[0]) pll_output_dest <= iomem_wdata[1:0];
				    end
				    8'ha8: begin
					iomem_rdata <= {30'd0, trap_output_dest};
					if (iomem_wstrb[0]) trap_output_dest <= iomem_wdata[1:0];
				    end
				    8'hb0: begin
					iomem_rdata <= {30'd0, irq_7_inputsrc};
					if (iomem_wstrb[0]) irq_7_inputsrc <= iomem_wdata[1:0];
				    end
				    8'hb4: begin
					iomem_rdata <= {30'd0, irq_8_inputsrc};
					if (iomem_wstrb[0]) irq_8_inputsrc <= iomem_wdata[1:0];
				    end
				    8'hc0: begin
					iomem_rdata <= {31'd0, analog_out_sel};
					if (iomem_wstrb[0]) analog_out_sel <= iomem_wdata[0];
				    end
				    8'hc4: begin
					iomem_rdata <= {31'd0, opamp_bias_ena};
					if (iomem_wstrb[0]) opamp_bias_ena <= iomem_wdata[0];
				    end
				    8'hc8: begin
					iomem_rdata <= {31'd0, opamp_ena};
					if (iomem_wstrb[0]) opamp_ena <= iomem_wdata[0];
				    end
				    8'hd0: begin
					iomem_rdata <= {31'd0, bg_ena};
					if (iomem_wstrb[0]) bg_ena <= iomem_wdata[0];
				    end
				    8'he0: begin
					iomem_rdata <= {31'd0, overtemp_ena};
					if (iomem_wstrb[0]) overtemp_ena <= iomem_wdata[0];
				    end
				    8'he4: begin
					iomem_rdata <= {31'd0, overtemp};
				    end
				    8'he8: begin
					iomem_rdata <= {30'd0, overtemp_dest};
					if (iomem_wstrb[0]) overtemp_dest <= iomem_wdata[1:0];
				    end
				    8'hf0: begin
					iomem_rdata <= {27'd0, nvram_rdy, nvram_hs, nvram_hr, nvram_mem_sel, nvram_mem_all};
					if (iomem_wstrb[0]) begin
					    nvram_hs <= iomem_wdata[3];
					    nvram_hr <= iomem_wdata[2];
					    nvram_mem_sel <= iomem_wdata[1];
					    nvram_mem_all <= iomem_wdata[0];
					end
				    end
				    8'hf4: begin
					iomem_rdata <= {27'd0, nvram_clkdiv};
					if (iomem_wstrb[0]) nvram_clkdiv <= iomem_wdata[4:0];
				end
			end
		end
	end

	/* NVRAM clock divider */
	clock_divider clockdiv (
	    .resetn(resetn),
	    .clkin(clk),
	    .clkout(nvram_clk),
	    .divval(nvram_clkdiv)
	);

XXX WIP XXX
	/* Counter-timer */
	counter_timer timer0 (
	    .resetn(resetn),
	    .clkin(clk),
	    .wstrb(),
	    .value_cur(timer0_cur),
	    .value_in(timer0_in),
	    .enable(timer0_enable),
	    .updown(timer0_updown),
	    .oneshot(timer0_oneshot),
	    .irq_ena(timer0_irq_ena).
	    .irq_out(timer0_irq_out)
	);

	/*
	ravenna_soc_mem #(.WORDS(MEM_WORDS)) picomem (
		.clk(clk),
		.ena(resetn),
		.wen((mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS) ? mem_wstrb : 4'b0),
		.addr(mem_addr[23:2]),
		.wdata(mem_wdata),
		.rdata(ram_rdata)
	);
	*/
endmodule

`include "picorv32.v"
`include "spimemio.v"
`include "simpleuart.v"
`include "simple_i2c_master.v"
`include "i2c_master_byte_ctrl.v"
`include "i2c_master_bit_ctrl.v"
`include "i2c_master_defines.v"
`include "simple_spi_master.v"

// Implementation note:
// Replace the following two modules with wrappers for your SRAM cells.

module ravenna_soc_regs (
	input clk, wen,
	input [5:0] waddr,
	input [5:0] raddr1,
	input [5:0] raddr2,
	input [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2
);
	reg [31:0] regs [0:31];

	always @(posedge clk)
		if (wen) regs[waddr[4:0]] <= wdata;

	assign rdata1 = regs[raddr1[4:0]];
	assign rdata2 = regs[raddr2[4:0]];
endmodule

/* Removing original ravenna_soc_mem and replacing it with  */
/* an external SRAM compiled on the X-Fab memory compiler */

/*
module ravenna_soc_mem #(
	parameter integer WORDS = 256
) (
	input clk,
	input ena,
	input [3:0] wen,
	input [21:0] addr,
	input [31:0] wdata,
	output reg [31:0] rdata
);
	reg [31:0] mem [0:WORDS-1];

	always @(posedge clk) begin
		if (ena == 1'b1) begin
			rdata <= mem[addr];
			if (wen[0]) mem[addr][ 7: 0] <= wdata[ 7: 0];
			if (wen[1]) mem[addr][15: 8] <= wdata[15: 8];
			if (wen[2]) mem[addr][23:16] <= wdata[23:16];
			if (wen[3]) mem[addr][31:24] <= wdata[31:24];
		end
	end
endmodule
*/

/* Full count 50% duty cycle clock divider:  divide by (divval + 1)	*/
/* Made from two half-counters, one on the clock positive edge and one	*/
/* on the clock negative edge.  On odd counts they overlap by 1/2 cycle	*/
/* so that the OR of the two counts always has a 50% duty cycle.	*/

/* Note that the NVRAM clock must be exactly 4MHz, and the maximum core	*/
/* clock is 100MHz, so the divider is typically 25 unless the core is	*/
/* being run slower.  Note that this implies that the NVRAM cannot be	*/
/* operated (for program/restore operations) at a core clock < 4MHz.	*/
/* But since this routine's minimum divide value is 2, then the core	*/
/* clock must be at least 8MHz.  At slower rates, the core can still be	*/
/* used as SRAM and might also do restore operations.			*/

module clock_divider (
    input resetn,
    input clkin,
    output clkout,
    input [4:0] divval
);
    reg  [4:0] cntpos;
    reg  [4:0] cntneg;
    reg	       negpart;
    reg	       pospart;
    wire clkout;

    always @(posedge clkin) begin
	if (resetn == 1'b0) begin
	    cntpos <= 5'd0;
	    pospart <= 1'b0;
	end else if (cntpos == divval) begin
	    cntpos <= 5'd0;
	end else begin
	    cntpos <= cntpos + 1;
	end

	if (cntpos > {1'b0, divval[4:1]}) begin
	   pospart <= 1'b1;
	end else begin
	   pospart <= 1'b0;
	end
    end

    always @(negedge clkin) begin
	if (resetn == 1'b0) begin
	    cntneg <= 5'd0;
	    negpart <= 1'b0;
	end else if (cntneg == divval) begin
	    cntneg <= 5'd0;
	end else begin
	    cntneg <= cntneg + 1;
	end

	if (cntneg > {1'b0, divval[4:1]}) begin
	   negpart <= 1'b1;
	end else begin
	   negpart <= 1'b0;
	end
    end

    assign clkout = pospart | (negpart & ~divval[0]);

endmodule

/* Simple 32-bit counter-timer for ravenna. */

module counter_timer (
    input resetn,
    input clkin,
    output [31:0] value_cur,
    input [31:0] value_in,
    input	wstrb,
    input	enable,
    input	updown,
    input	oneshot,
    input	irq_ena,
    output	irq_out
);

reg [31:0] value_cur;
reg [31:0] value_reset;
reg	   irq_out;

always @(posedge clkin or negedge resetn) begin
    if (resetn == 1'b0) begin
	value_cur <= 32'd0;	
	value_reset <= 32'd0;
	irq_out <= 32'd0;
    end else begin
	if (wstrb == 1'b1) begin
	    value_reset <= value_in;
	    if (updown == 1'b1) begin
		value_cur <= 32'd0;
	    end else begin
		value_cur <= value_in;
	    end
	end else begin
	    if (enable == 1'b1) begin
		if (updown == 1'b1) begin
		    if (value_cur == value_in) begin
			if (oneshot != 1'b1) begin
			    value_cur <= 32'd0;
			end
			if (irq_ena == 1'b1) begin
			    irq_out <= 1'b1;
			end
		    end else begin
			value_cur <= value_cur + 1;
			irq_out <= 1'b0;
		    end
		end else begin
		    if (value_cur == 32'd0) begin
			if (oneshot != 1'b1) begin
			    value_cur <= value_reset;
			end
			if (irq_ena == 1'b1) begin
			    irq_out <= 1'b1;
			end
		    end else begin
			value_cur <= value_cur - 1;
			irq_out <= 1'b0;
		    end
		end
	    end else begin
		irq_out <= 1'b0;
	    end
	end
    end

endmodule
