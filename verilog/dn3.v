/* Simple verilog model for device dn3 for LVS purposes */

module dn3 #(
    parameter [ 0:0] area = 1.0
) (
        input real pos,
        input real neg
);

wire real pos, neg;

/* Not modeled, for LVS purposes only */

endmodule       // dn3
