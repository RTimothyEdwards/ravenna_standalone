//************************************************************************/
// File             : XCPF_136X32DP128_VD03.v
// Description      : Simple non-functional stand-in for X-Fab verilog model
//************************************************************************/

module XCPF_136X32DP128_VD03 (CLK4, CLK4M, VCORE, VSE1, VSE2,
	VSEBUSY, BUSYNVC, CLKI, MEM1_ENT, MEM2_ENT, NVREF_EXT,
	POR, RCLT, TM_NVCPI, TRIM, VSESTART );

   output                       CLK4, CLK4M, VCORE, VSE1, VSE2, VSEBUSY;
   
   input [15:0] 	TRIM;
   input [3:0]		TM_NVCPI;
   input                BUSYNVC, CLKI, MEM1_ENT, MEM2_ENT;
   input		POR, RCLT, VSESTART;

   // NOTE:  Replaced inout with input real.  This does not mimic behavior of
   // NVREF_EXT in test mode but allows it to check that the cell is connected
   // to ground during normal use.

   // inout			NVREF_EXT;
   input			NVREF_EXT;
   wire real NVREF_EXT;
   
   reg 				CLK4, CLK4M, VCORE, VSE1, VSE2, VSEBUSY;
  
   // convert NVREF_EXT to an internal digital value
   wire nvref_int;
   assign nvref_int = (NVREF_EXT < 0.5) ? 1'b0 : (NVREF_EXT > 2.0) ? 1'b1 : 1'bx;

   always @(posedge CLKI or posedge POR) begin
	if (POR) begin
	    CLK4 <= 0;
	    CLK4M <= 0;
	    VCORE <= 0;
	    VSE1 <= 0;
	    VSE2 <= 0;
	    VSEBUSY <= 0;
	end
   end
//--------------------------------------------------------------------------

endmodule
   
