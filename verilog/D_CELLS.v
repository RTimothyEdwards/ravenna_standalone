//************************************************************************
// Simple functional stand-in for X-Fab verilog models for D_CELLS
// Process:  XH018
//************************************************************************

`timescale 1ns/10ps

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : BUX2
//   description : Buffer
//****************************************************************************

module BUX2 (A, Q);

   input     A;
   output    Q;

   // Function Q = A
   wire Q;
   assign Q = A;

endmodule

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : BUX4
//   description : Buffer
//****************************************************************************

module BUX4 (A, Q);

   input     A;
   output    Q;

   // Function Q = A
   wire Q;
   assign Q = A;

endmodule

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : BUX12
//   description : Buffer
//****************************************************************************

module BUX12 (A, Q);

   input     A;
   output    Q;

   // Function Q = A
   wire Q;
   assign Q = A;

endmodule

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : DFRX2
//   description : D-flipflop
//****************************************************************************

module DFRX2 (D, C, QN, Q);

   input     D;
   input     C;
   output    QN;
   output    Q;

   reg	Q;
   wire QN;

   assign QN = ~Q;
   
   // Function Q <= D
   always @(posedge C) begin
      Q <= D;
   end;

endmodule

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : INX2
//   description : Inverter
//****************************************************************************

module INX2 (A, Q);

   input     A;
   output    Q;

   // Function Q = !A
   wire Q;
   assign Q = !A;

endmodule

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : LOGIC0
//   description : Constant logic 0
//****************************************************************************

module LOGIC0 (Q);

   output    Q;

   // Function Q = 0
   wire Q;
   assign Q = 1'b0;

endmodule

//****************************************************************************
//   technology  : X-Fab XH018
//   module name : LOGIC1
//   description : Constant logic 1
//****************************************************************************

module LOGIC1 (Q);

   output    Q;

   // Function Q = 1
   wire Q;
   assign Q = 1'b1;

endmodule

//****************************************************************************

