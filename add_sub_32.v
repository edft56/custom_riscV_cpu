`timescale 1ns/1ns

module fullAdder1(  input x,
                    input y,
                    input c_in,
                    output sum,
                    output P_out,
                    output G_out);

    assign sum = x ^ y ^ c_in;
    assign P_out = x ^ y;
    assign G_out = x & y;

endmodule


module CLA4 (   input [3:0] P_in,G_in,
                input C_in,
                output [2:0] C_out,
                output P_out, G_out);

integer i;

    always @*
    begin
        for(i=0; i<4; i=i+1)
        begin
            P_out = P_out & P_in[i];
            G_out = 
        end
    end

endmodule