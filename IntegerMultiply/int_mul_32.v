`timescale 1ns/1ns

`include "../CLA_adder_32/cla_adder_32.v"


module mul32x32(input  [OPERAND_SIZE-1   : 0]      X,
                input  [OPERAND_SIZE-1   : 0]      Y,
                output [2*OPERAND_SIZE-1 : 0] Result
                );
    parameter OPERAND_SIZE = 32;

    wire [OPERAND_SIZE-3 : 0] stage_sum       [OPERAND_SIZE-2 : 0];
    wire [OPERAND_SIZE-2 : 0] stage_cout      [OPERAND_SIZE-2 : 0];
    wire [OPERAND_SIZE-1 : 0] partial_product [OPERAND_SIZE-1 : 0];

    assign Result[0] = partial_product[0][0];

    genvar i;
    generate
        for(i=0; i<OPERAND_SIZE; i=i+1) begin
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

module mul32x32_pipelined(  input                            clk,
                            input  [OPERAND_SIZE-1   : 0]      X,
                            input  [OPERAND_SIZE-1   : 0]      Y,
                            output reg [2*OPERAND_SIZE-1 : 0] Result
                            );
    parameter OPERAND_SIZE = 32;
                                              //Pipe Stages
    wire [OPERAND_SIZE-3   : 0] stage_sum       [2:0];
    wire [OPERAND_SIZE-2   : 0] stage_cout      [2:0];
    wire [OPERAND_SIZE-1   : 0] partial_product [2:0][OPERAND_SIZE-1 : 0];
    wire [2*OPERAND_SIZE-1 : 0] stage_result    [2:0];


    first_pipe_stage P1(.clk_i( clk ),
                        .X_i( X ),
                        .Y_i( Y ),
                        
                        .stage_partial_product_o(partial_product[0]),
                        .stage_sum_o(stage_sum[0]),
                        .stage_cout_o(stage_cout[0]),
                        .stage_result_o(stage_result[0])
                        );

    second_pipe_stage P2(.clk_i(),
                         .stage_partial_product_i(partial_product[0]),
                         .stage_sum_i(stage_sum[0]),
                         .stage_cout_i(stage_cout[0]),
                         .stage_result_i(stage_result[0]),

                         .stage_partial_product_o(partial_product[1]),
                         .stage_sum_o(stage_sum[1]),
                         .stage_cout_o(stage_cout[1]),
                         .stage_result_o(stage_result[1])
                        );
    
    second_pipe_stage P3(.clk_i(),
                         .stage_partial_product_i(partial_product[1]),
                         .stage_sum_i(stage_sum[1]),
                         .stage_cout_i(stage_cout[1]),
                         .stage_result_i(stage_result[1]),

                         .stage_partial_product_o(partial_product[2]),
                         .stage_sum_o(stage_sum[2]),
                         .stage_cout_o(stage_cout[2]),
                         .stage_result_o(stage_result[2])
                        );

    third_pipe_stage P4(.clk_i(),
                        .stage_partial_product_i(partial_product[2]),
                        .stage_sum_i(stage_sum[2]),
                        .stage_cout_i(stage_cout[2]),
                        .stage_result_i(stage_result[2]),

                        .stage_result_o(Result)
                        );


endmodule

module first_pipe_stage(input  clk_i,
                        input  [OPERAND_SIZE-1   : 0] X_i,
                        input  [OPERAND_SIZE-1   : 0] Y_i,
                        
                        output reg[OPERAND_SIZE-1   : 0] stage_partial_product_o [OPERAND_SIZE-1 : 0], //pray compiler optimizes away the extra registers
                        output reg[OPERAND_SIZE-3   : 0] stage_sum_o,
                        output reg[OPERAND_SIZE-2   : 0] stage_cout_o,
                        output reg[2*OPERAND_SIZE-1 : 0] stage_result_o //pray compiler optimizes away the extra registers
                        );
    parameter OPERAND_SIZE = 32;
    parameter CUR_SECOND_STAGES = 8;

    wire [OPERAND_SIZE-3      : 0] stage_sum       [CUR_SECOND_STAGES : 0];
    wire [OPERAND_SIZE-2      : 0] stage_cout      [CUR_SECOND_STAGES : 0];
    wire [OPERAND_SIZE-1      : 0] partial_product [OPERAND_SIZE-1 : 0];
    wire [2+CUR_SECOND_STAGES : 0] stage_result;

    assign stage_result[0] = partial_product[0][0];

    genvar i;
    generate
        for(i=0; i<OPERAND_SIZE; i=i+1) begin
            assign partial_product[i] = X_i & {OPERAND_SIZE{Y_i[i]}};
        end
    endgenerate


    First_Stage S1( .X( partial_product[0][OPERAND_SIZE-1 : 1] ), 
                    .Y( partial_product[1][OPERAND_SIZE-2 : 0] ),
                    .Sum( {stage_sum[0],stage_result[1]} ),
                    .Cout( stage_cout[0] )
                    );

    generate
        for(i=0; i<CUR_SECOND_STAGES; i=i+1) begin
            Second_Stage S2(.X( partial_product[2+i][OPERAND_SIZE-2 : 0] ), 
                            .Y( {partial_product[1+i][OPERAND_SIZE-1],stage_sum[i]} ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[1+i],stage_result[i+2]} ),
                            .Cout( stage_cout[1+i] )
                           );
        end
    endgenerate

    always @(negedge clk_i) begin
        stage_result_o[2+CUR_SECOND_STAGES : 0] <= stage_result;

        stage_sum_o             <= stage_sum[CUR_SECOND_STAGES];
        stage_cout_o            <= stage_cout[CUR_SECOND_STAGES];
        stage_partial_product_o <= partial_product;
    end
    
    
endmodule

module second_pipe_stage(   input clk_i,
                            input [OPERAND_SIZE-1    : 0] stage_partial_product_i [OPERAND_SIZE-1 : 0],
                            input [OPERAND_SIZE-3    : 0] stage_sum_i,
                            input [OPERAND_SIZE-2    : 0] stage_cout_i,
                            input [2*OPERAND_SIZE-1  : 0] stage_result_i,

                            output reg [OPERAND_SIZE-1   : 0] stage_partial_product_o [OPERAND_SIZE-1 : 0],
                            output reg [OPERAND_SIZE-3   : 0] stage_sum_o,
                            output reg [OPERAND_SIZE-2   : 0] stage_cout_o,
                            output reg [2*OPERAND_SIZE-1 : 0] stage_result_o
                        );
    parameter OPERAND_SIZE = 32;
    parameter RESULT_INDEX = 8; //how many bits of the result have already been computed
    parameter CUR_SECOND_STAGES = 8;

    wire [OPERAND_SIZE-3      : 0] stage_sum    [CUR_SECOND_STAGES-1 : 0];
    wire [OPERAND_SIZE-2      : 0] stage_cout   [CUR_SECOND_STAGES-1 : 0];
    wire [CUR_SECOND_STAGES-1 : 0] stage_result;

    Second_Stage S2(.X( stage_partial_product_i[RESULT_INDEX+1][OPERAND_SIZE-2 : 0] ), 
                    .Y( {stage_partial_product_i[RESULT_INDEX][OPERAND_SIZE-1], stage_sum_i} ),
                    .Cin( stage_cout_i ),
                    .Sum( {stage_sum[0], stage_result[0]} ),
                    .Cout( stage_cout[0] )
                    );

    genvar i;
    generate
        for(i=0; i<CUR_SECOND_STAGES-1; i=i+1) begin
            Second_Stage S2(.X( stage_partial_product_i[RESULT_INDEX+2 + i][OPERAND_SIZE-2 : 0] ), 
                            .Y( {stage_partial_product_i[RESULT_INDEX+1 + i][OPERAND_SIZE-1], stage_sum[i]} ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[1+i],stage_result[1 + i]} ),
                            .Cout( stage_cout[1+i] )
                           );
        end
    endgenerate

    always @(negedge clk_i) begin
        stage_result_o[RESULT_INDEX + CUR_SECOND_STAGES-1 : RESULT_INDEX] <= stage_result;

        stage_sum_o             <= stage_sum[CUR_SECOND_STAGES-1];
        stage_cout_o            <= stage_cout[CUR_SECOND_STAGES-1];
        stage_partial_product_o <= stage_partial_product_i;
    end
endmodule


module third_pipe_stage(    input clk_i,
                            input [OPERAND_SIZE-1    : 0] stage_partial_product_i [OPERAND_SIZE-1 : 0],
                            input [OPERAND_SIZE-3    : 0] stage_sum_i,
                            input [OPERAND_SIZE-2    : 0] stage_cout_i,
                            input [2*OPERAND_SIZE-1  : 0] stage_result_i,

                            output reg [2*OPERAND_SIZE-1 : 0] stage_result_o
                        );
    parameter OPERAND_SIZE = 32;
    parameter RESULT_INDEX = 8; //how many bits of the result have already been computed
    parameter CUR_SECOND_STAGES = 8;

    wire [OPERAND_SIZE-3           : 0] stage_sum    [CUR_SECOND_STAGES-1 : 0];
    wire [OPERAND_SIZE-2           : 0] stage_cout   [CUR_SECOND_STAGES-1 : 0];
    wire [CUR_SECOND_STAGES-1 + 32 : 0] stage_result;

    Second_Stage S2(.X( stage_partial_product_i[RESULT_INDEX+1][OPERAND_SIZE-2 : 0] ), 
                    .Y( {stage_partial_product_i[RESULT_INDEX][OPERAND_SIZE-1], stage_sum_i} ),
                    .Cin( stage_cout_i ),
                    .Sum( {stage_sum[0], stage_result[0]} ),
                    .Cout( stage_cout[0] )
                    );

    genvar i;
    generate
        for(i=0; i<CUR_SECOND_STAGES-1; i=i+1) begin
            Second_Stage S2(.X( stage_partial_product_i[RESULT_INDEX+2 + i][OPERAND_SIZE-2 : 0] ), 
                            .Y( {stage_partial_product_i[RESULT_INDEX+1 + i][OPERAND_SIZE-1], stage_sum[i]} ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[1+i],stage_result[1 + i]} ),
                            .Cout( stage_cout[1+i] )
                           );
        end
    endgenerate


    Third_Stage S3( .X( {1'b0, stage_cout[CUR_SECOND_STAGES-1]} ),
                    .Y( {1'b0, stage_partial_product_i[OPERAND_SIZE-1][OPERAND_SIZE-1], stage_sum[CUR_SECOND_STAGES-1]} ),
                    .Sum( stage_result[CUR_SECOND_STAGES-1 + 32 : CUR_SECOND_STAGES] ),
                    .Cout()
                    );


    always @(negedge clk_i) begin
        stage_result_o[2*OPERAND_SIZE-1 : RESULT_INDEX] <= stage_result;
    end
endmodule


module First_Stage( input  [OPERAND_SIZE-2 : 0]    X, 
                    input  [OPERAND_SIZE-2 : 0]    Y, 
                    output [OPERAND_SIZE-2 : 0]  Sum, 
                    output [OPERAND_SIZE-2 : 0] Cout
                    );
	parameter OPERAND_SIZE = 32;

    genvar i;

    generate
        for(i=0; i<OPERAND_SIZE-1; i=i+1) begin
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
        for(i=0; i<OPERAND_SIZE-1; i=i+1) begin
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

