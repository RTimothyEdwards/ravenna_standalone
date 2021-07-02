//************************************************************************/
// File             : XSPRAMBLP_4096X32_M8P.v
// Description      : Simple functional stand-in for X-Fab verilog model
//************************************************************************/

`timescale 1ns/10ps

module XSPRAMBLP_4096X32_M8P (Q, D, A, CLK, CEn, WEn, RDY);

output [31:0]	Q;		// RAM data output

input  [31:0]	D;		// RAM data input bus
input  [11:0]	A;		// RAM address bus
input		CLK; 		// RAM clock
input		CEn;		// RAM enable
input  [31:0]	WEn;		// RAM per-bit write enable, 0-active
output		RDY;		// Test output

reg   [31:0] mem [0:4096];
reg   [31:0] Q;
reg	     RDY;

integer i, b;

initial begin
    for (i = 0; i < 4096; i = i + 1)
	mem[i] = 32'b0;
end

always @(posedge CLK or posedge CEn) begin
    if (CEn) begin
	RDY <= 0;
	Q <= 32'b0;
    end else begin
	RDY <= 1;
        for (b = 0; b < 32; b = b + 1)
	    if (!WEn[b]) mem[A][b] <= D[b];
	Q <= mem[A];
    end
end
endmodule

