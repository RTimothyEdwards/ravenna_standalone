//************************************************************************/
// File             : XNVR_136X32P128_VD01.v
// Description      : Simple functional stand-in for X-Fab verilog model
//************************************************************************/

module XNVR_136X32P128_VD01 (BUSYNVC, CLKI, DOUT, DSO, MEM1_ENT, MEM2_ENT,
	RCLT, RDY, TM_NVCPI, TRIM, VSESTART, A, CE, CLK, CLK4,
	DIN, DRSTN, DSCLK, DSI, DUP, HR, HS, MEM_ALLC, MEM_SEL,
	PEIN, POR, TM_NVCP, VCORE, VSE1, VSE2, VSEBUSY, WE );

   output   [31:0]      DOUT;
   output   [15:0] 	TRIM;
   output   [3:0]	TM_NVCPI;
   output 		BUSYNVC, CLKI, MEM1_ENT, MEM2_ENT, RCLT;
   output		RDY, VSESTART, DSO;   
   input    [7:0]      	A;
   input    [31:0]      DIN;
   input    [3:0]	TM_NVCP;
   input		CE, CLK, HR, HS, MEM_ALLC, MEM_SEL, PEIN;
   input		POR, VCORE, VSE1, VSE2, VSEBUSY, WE, DSCLK, DRSTN, DSI; 
       
   input		CLK4;
   input 		DUP;

   reg	    [31:0] 	sram  [0:135];
   reg	    [31:0] 	nvram [0:135];
   reg	    [31:0]	DOUT;
   reg			RDY;

   reg	    [15:0]	TRIM;
   reg	    [3:0]	TM_NVCPI;
   reg			BUSYNVC;
   reg			CLKI;
   reg			MEM1_ENT;
   reg			MEM2_ENT;
   reg			RCLT;
   reg			VSESTART;
   reg			DSO;

   integer  i, b;

   initial begin
	for (i = 0; i < 136; i = i + 1)
	    sram[i] = 32'b0;
   end

   /* SRAM part */
   always @(posedge CLK or negedge CE) begin
	if (~CE) begin
	    RDY <= 0;
	    DOUT <= 32'b0;
	end else begin
	    RDY <= 1'b0;
	    if (WE) sram[A] <= DIN;
	    DOUT <= sram[A];
	end
    end

    /* NVRAM part (unimplemented) */
   always @(posedge CLK or negedge CE) begin
	if (~CE) begin
	    TRIM <= 16'b0;
	    TM_NVCPI <= 4'b0;
	    BUSYNVC <= 0;
	    CLKI <= CLK;
	    MEM1_ENT <= 0;
	    MEM2_ENT <= 0;
	    RCLT <= 0;
	    VSESTART <= 0;
	    DSO <= 0;
	    
	end else begin
	    CLKI <= ~CLKI;
	end
   end

endmodule
	
