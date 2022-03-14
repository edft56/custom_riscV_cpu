`timescale 1ns/1ns

`include "../CLA_adder_32/cla_adder_32.v"


module mul32x32(input  [OPERAND_SIZE-1   : 0]      X,
                input  [OPERAND_SIZE-1   : 0]      Y,
                output [2*OPERAND_SIZE-1 : 0] Result
                );
    parameter OPERAND_SIZE = 32;

    wire [OPERAND_SIZE-3 : 0] stage_sum   [OPERAND_SIZE-2];
    wire [OPERAND_SIZE-2 : 0] stage_cout  [OPERAND_SIZE-2];
    wire [OPERAND_SIZE-1 : 0] partial_product [OPERAND_SIZE-1];

    assign Result[0] = partial_product[0][0];

    genvar i;
    generate
        for(i=0; i<OPERAND_SIZE-1; i=i+1) begin
            assign partial_product[i] = X & {32{Y[i]}};
        end
    endgenerate


    First_Stage S1( .X( partial_product[0][OPERAND_SIZE-1 : 1] ), 
                    .Y( partial_product[1][OPERAND_SIZE-2 : 0] ),
                    .Sum( {stage_sum[0],Result[1]} ),
                    .Cout( stage_cout[0] )
                    );

    generate
        for(i=0; i<OPERAND_SIZE-2; i=i+1) begin
            Second_Stage S2(.X( partial_product[2+i][OPERAND_SIZE-2 : 0] ), 
                            .Y( {partial_product[1+i][OPERAND_SIZE-1],stage_sum[i]} ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[1+i],Result[i+2]} ),
                            .Cout( stage_cout[1+i] )
                           );
        end
    endgenerate

    Third_Stage S3( .X( {1'b0, stage_cout[OPERAND_SIZE-2]} ),
                    .Y( {1'b0, partial_product[OPERAND_SIZE-1][OPERAND_SIZE-1], stage_sum[OPERAND_SIZE-2]} ),
                    .Sum( Result[2*OPERAND_SIZE-1 : OPERAND_SIZE] ),
                    .Cout()
                    );

endmodule

module First_Stage( input  [OPERAND_SIZE-2 : 0]    X, 
                    input  [OPERAND_SIZE-2 : 0]    Y, 
                    output [OPERAND_SIZE-2 : 0]  Sum, 
                    output [OPERAND_SIZE-2 : 0] Cout
                    );
	parameter OPERAND_SIZE = 32;

    genvar i;

    generate
        for(i=0; i<OPERAND_SIZE-2; i=i+1) begin
            HalfAdder1 HA0 (X[i],Y[i],Sum[i],Cout[i]);
        end
    endgenerate

endmodule

module Second_Stage(input  [OPERAND_SIZE-2 : 0]   X, 
                    input  [OPERAND_SIZE-2 : 0]   Y, 
                    input  [OPERAND_SIZE-2 : 0] Cin, 
                    output [OPERAND_SIZE-2 : 0] Sum, 
                    output [OPERAND_SIZE-2 : 0] Cout
                    );
    parameter OPERAND_SIZE = 32;

    genvar i;

    generate
        for(i=0; i<OPERAND_SIZE-2; i=i+1) begin
            FullAdder1 FA0 (X[i],Y[i],Cin[i],Sum[i],Cout[i]);
        end
    endgenerate

endmodule


module Third_Stage( input  [OPERAND_SIZE-1 : 0] X,
                    input  [OPERAND_SIZE-1 : 0] Y,
                    output [OPERAND_SIZE-1 : 0] Sum,
                    output                      Cout
                    ); // Carry Look-Ahead

parameter OPERAND_SIZE = 32;

	cla_adder_32 cla32( .x( X ), 
                        .y( Y ), 
                        .sub( 1'b0 ), 
                        .sum( Sum ), 
                        .c_out( Cout )
                        );

endmodule


module FullAdder1(X, Y, Cin, Sum, Cout);
output Sum, Cout;
input X, Y, Cin;

	assign Sum = X ^ Y ^ Cin;
	assign Cout = (X&&Y) + (X&&Cin) + (Y&&Cin);

endmodule 

module HalfAdder1(X, Y, Sum, Cout);
output Sum, Cout;
input X, Y;

	assign Sum = X ^ Y;
	assign Cout = (X&&Y);

endmodule 

