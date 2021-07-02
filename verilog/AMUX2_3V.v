// Analog 2-input, 1-output multiplexer
// with 1.8V digital select.
//
// NOTE:  This is a simple functional model only and captures
// none of the internal workings of the multiplexer, such
// as double-gating each input and connecting the middle node
// to VSSA on the unselected input.

module AMUX2_3V (
   input wire real VDD3V3,
   input wire real VDD1V8,
   input wire real VSSA,
   input wire real AIN1,
   input wire real AIN2,
   output wire real AOUT,
   input SEL
);
   wire SEL;
   real NaN;

   initial begin
      NaN = 0.0 / 0.0;
   end

   assign AOUT = (SEL == 1'b1) ? AIN2 :
		 (SEL == 1'b0) ? AIN1 :
		 NaN;

endmodule
