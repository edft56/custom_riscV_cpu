`timescale 1ns/1ns



//pipelined
module int_mul_32(  input                                      clk,
                            input                                    start,
                            input                             signed_mul_i,
                            input      [OPERAND_SIZE-1   : 0]            X,
                            input      [OPERAND_SIZE-1   : 0]            Y,
                        
                            output                            result_rdy,
                            output     [2*OPERAND_SIZE-1 : 0] Result
                            );
    parameter OPERAND_SIZE = 32;
                                              //Pipe Stages
    wire [OPERAND_SIZE-2   : 0] stage_sum       [2:0];
    wire [OPERAND_SIZE-1   : 0] stage_cout      [2:0];
    wire [OPERAND_SIZE-1   : 0] partial_product [2:0][OPERAND_SIZE-1 : 0];
    wire [2*OPERAND_SIZE-1 : 0] stage_result    [2:0];
    wire                        signed_mul      [2:0];

    reg  [               2 : 0] pipe_state;


    first_pipe_stage #( .OPERAND_SIZE(OPERAND_SIZE),
                        .CUR_SECOND_STAGES(8)
                        )
                     P1(.signed_mul_i( signed_mul_i ),
                        .clk_i( clk ),
                        .X_i( X ),
                        .Y_i( Y ),
                        
                        .signed_mul_o( signed_mul[0] ),
                        .stage_partial_product_o(partial_product[0]),
                        .stage_sum_o(stage_sum[0]),
                        .stage_cout_o(stage_cout[0]),
                        .stage_result_o(stage_result[0])
                        );

    second_pipe_stage #(.OPERAND_SIZE(OPERAND_SIZE),
                        .CUR_SECOND_STAGES(9),
                        .RESULT_INDEX(10)
                        )
                      P2(.signed_mul_i( signed_mul[0] ),
                         .clk_i( clk ),
                         .stage_partial_product_i(partial_product[0]),
                         .stage_sum_i(stage_sum[0]),
                         .stage_cout_i(stage_cout[0]),
                         .stage_result_i(stage_result[0]),
                         
                         .signed_mul_o( signed_mul[1] ),
                         .stage_partial_product_o(partial_product[1]),
                         .stage_sum_o(stage_sum[1]),
                         .stage_cout_o(stage_cout[1]),
                         .stage_result_o(stage_result[1])
                        );
    
    second_pipe_stage #(.OPERAND_SIZE(OPERAND_SIZE),
                        .CUR_SECOND_STAGES(9),
                        .RESULT_INDEX(19)
                        )
                      P3(.signed_mul_i( signed_mul[1] ),
                         .clk_i( clk ),
                         .stage_partial_product_i(partial_product[1]),
                         .stage_sum_i(stage_sum[1]),
                         .stage_cout_i(stage_cout[1]),
                         .stage_result_i(stage_result[1]),

                         .signed_mul_o( signed_mul[2] ),
                         .stage_partial_product_o(partial_product[2]),
                         .stage_sum_o(stage_sum[2]),
                         .stage_cout_o(stage_cout[2]),
                         .stage_result_o(stage_result[2])
                        );

    third_pipe_stage #( .OPERAND_SIZE(OPERAND_SIZE),
                        .CUR_SECOND_STAGES(4),
                        .RESULT_INDEX(28)
                        )
                     P4(.signed_mul_i( signed_mul[2] ),
                        .clk_i( clk ),
                        .stage_partial_product_i(partial_product[2]),
                        .stage_sum_i(stage_sum[2]),
                        .stage_cout_i(stage_cout[2]),
                        .stage_result_i(stage_result[2]),

                        .stage_result_o(Result)
                        );

    
    assign result_rdy = pipe_state[0];

    always @(negedge clk)
    begin
        pipe_state[1:0] <= pipe_state[2:1];
        pipe_state[2]   <= (start) ? 1'b1 : 1'b0;
    end

endmodule

module first_pipe_stage(input  clk_i,
                        input  signed_mul_i,
                        input  [OPERAND_SIZE-1   : 0] X_i,
                        input  [OPERAND_SIZE-1   : 0] Y_i,
                        
                        output reg                       signed_mul_o,
                        output reg[OPERAND_SIZE-1   : 0] stage_partial_product_o [OPERAND_SIZE-1 : 0], //pray compiler optimizes away the extra registers
                        output reg[OPERAND_SIZE-2   : 0] stage_sum_o,
                        output reg[OPERAND_SIZE-1   : 0] stage_cout_o,
                        output reg[2*OPERAND_SIZE-1 : 0] stage_result_o //pray compiler optimizes away the extra registers
                        );
    parameter OPERAND_SIZE = 32;
    parameter CUR_SECOND_STAGES = 8;

    wire [OPERAND_SIZE-2        : 0] stage_sum       [CUR_SECOND_STAGES : 0];
    wire [OPERAND_SIZE-1        : 0] stage_cout      [CUR_SECOND_STAGES : 0];
    wire [OPERAND_SIZE-1        : 0] partial_product [OPERAND_SIZE-1 : 0];
    wire [2+CUR_SECOND_STAGES-1 : 0] stage_result;

    assign stage_result[0] = partial_product[0][0];

    genvar i;
    generate
        for(i=0; i<OPERAND_SIZE; i=i+1) begin
            assign partial_product[i] = X_i & {OPERAND_SIZE{Y_i[i]}};
        end
    endgenerate


    First_Stage #( .OPERAND_SIZE(OPERAND_SIZE) ) 
                S1( .signed_mul( signed_mul_i ),
                    .X( partial_product[0][OPERAND_SIZE-1 : 1] ), 
                    .Y( partial_product[1][OPERAND_SIZE-1 : 0] ),
                    .Sum( {stage_sum[0],stage_result[1]} ),
                    .Cout( stage_cout[0] )
                    );

    generate
        for(i=0; i<CUR_SECOND_STAGES; i=i+1) begin
            Second_Stage #( .OPERAND_SIZE(OPERAND_SIZE) ) 
                         S2(.signed_mul( signed_mul_i ),
                            .X( partial_product[2+i] ), 
                            .Y( stage_sum[i] ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[1+i],stage_result[i+2]} ),
                            .Cout( stage_cout[1+i] )
                           );
        end
    endgenerate

    always @(negedge clk_i) begin
        stage_result_o[2+CUR_SECOND_STAGES-1 : 0] <= stage_result;

        stage_sum_o             <= stage_sum[CUR_SECOND_STAGES];
        stage_cout_o            <= stage_cout[CUR_SECOND_STAGES];
        stage_partial_product_o <= partial_product;
        signed_mul_o            <= signed_mul_i;
    end
    
    
endmodule

module second_pipe_stage(   input clk_i,
                            input signed_mul_i,
                            input [OPERAND_SIZE-1    : 0] stage_partial_product_i [OPERAND_SIZE-1 : 0],
                            input [OPERAND_SIZE-2    : 0] stage_sum_i,
                            input [OPERAND_SIZE-1    : 0] stage_cout_i,
                            input [2*OPERAND_SIZE-1  : 0] stage_result_i,

                            output reg                        signed_mul_o,
                            output reg [OPERAND_SIZE-1   : 0] stage_partial_product_o [OPERAND_SIZE-1 : 0],
                            output reg [OPERAND_SIZE-2   : 0] stage_sum_o,
                            output reg [OPERAND_SIZE-1   : 0] stage_cout_o,
                            output reg [2*OPERAND_SIZE-1 : 0] stage_result_o
                        );
    parameter OPERAND_SIZE = 32;
    parameter RESULT_INDEX = 8; //how many bits of the result have already been computed
    parameter CUR_SECOND_STAGES = 8;

    wire [OPERAND_SIZE-2      : 0] stage_sum    [CUR_SECOND_STAGES : 0];
    wire [OPERAND_SIZE-1      : 0] stage_cout   [CUR_SECOND_STAGES : 0];
    wire [CUR_SECOND_STAGES-1 : 0] stage_result;

    assign stage_cout[0] = stage_cout_i;
    assign stage_sum[0]  = stage_sum_i;


    genvar i;
    generate
        for(i=0; i<CUR_SECOND_STAGES; i=i+1) begin
            Second_Stage #( .OPERAND_SIZE(OPERAND_SIZE) ) 
                         S2(.signed_mul( signed_mul_i ),
                            .X( stage_partial_product_i[RESULT_INDEX + i] ), 
                            .Y( stage_sum[i] ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[i+1],stage_result[i]} ),
                            .Cout( stage_cout[i+1] )
                           );
        end
    endgenerate

    always @(negedge clk_i) begin
        stage_result_o[RESULT_INDEX-1                     :            0] <= stage_result_i[RESULT_INDEX-1 : 0];
        stage_result_o[RESULT_INDEX + CUR_SECOND_STAGES-1 : RESULT_INDEX] <= stage_result;

        stage_sum_o             <= stage_sum[CUR_SECOND_STAGES];
        stage_cout_o            <= stage_cout[CUR_SECOND_STAGES];
        stage_partial_product_o <= stage_partial_product_i;
        signed_mul_o            <= signed_mul_i;
    end
endmodule


module third_pipe_stage(    input clk_i,
                            input signed_mul_i,
                            input [OPERAND_SIZE-1    : 0] stage_partial_product_i [OPERAND_SIZE-1 : 0],
                            input [OPERAND_SIZE-2    : 0] stage_sum_i,
                            input [OPERAND_SIZE-1    : 0] stage_cout_i,
                            input [2*OPERAND_SIZE-1  : 0] stage_result_i,

                            output reg [2*OPERAND_SIZE-1 : 0] stage_result_o
                        );
    parameter OPERAND_SIZE = 32;
    parameter RESULT_INDEX = 8; //how many bits of the result have already been computed
    parameter CUR_SECOND_STAGES = 8;

    wire [OPERAND_SIZE-2           : 0] stage_sum    [CUR_SECOND_STAGES : 0];
    wire [OPERAND_SIZE-1           : 0] stage_cout   [CUR_SECOND_STAGES : 0];
    wire [CUR_SECOND_STAGES-1 + 32 : 0] stage_result;

    assign stage_cout[0] = stage_cout_i;
    assign stage_sum[0]  = stage_sum_i;

    genvar i;
    generate
        for(i=0; i<CUR_SECOND_STAGES-1; i=i+1) begin
            Second_Stage #( .OPERAND_SIZE(OPERAND_SIZE) ) 
                         S2(.signed_mul( signed_mul_i ),
                            .X( stage_partial_product_i[RESULT_INDEX + i] ), 
                            .Y( stage_sum[i] ),
                            .Cin( stage_cout[i] ),
                            .Sum( {stage_sum[i+1],stage_result[i]} ),
                            .Cout( stage_cout[i+1] )
                           );
        end
    endgenerate
    

    Second_Stage #( .OPERAND_SIZE(OPERAND_SIZE) ) 
                 S2(.signed_mul( signed_mul_i ),
                    .X( (signed_mul_i) ? 
                        ~stage_partial_product_i[RESULT_INDEX + CUR_SECOND_STAGES -1] : //double negation on MSB so it's not inverted
                        stage_partial_product_i[RESULT_INDEX + CUR_SECOND_STAGES -1]
                      ), 
                    .Y( stage_sum[CUR_SECOND_STAGES-1] ),
                    .Cin( stage_cout[CUR_SECOND_STAGES-1] ),
                    .Sum( {stage_sum[CUR_SECOND_STAGES],stage_result[CUR_SECOND_STAGES-1]} ),
                    .Cout( stage_cout[CUR_SECOND_STAGES] )
                    );


    Third_Stage S3( .signed_mul( signed_mul_i ),
                    .X( stage_cout[CUR_SECOND_STAGES] ),
                    .Y( stage_sum[CUR_SECOND_STAGES] ),
                    .Sum( stage_result[CUR_SECOND_STAGES-1 + 32 : CUR_SECOND_STAGES] ),
                    .Cout()
                    );

    always @* begin
        stage_result_o[RESULT_INDEX-1   :            0] = stage_result_i[RESULT_INDEX-1 : 0];
        stage_result_o[2*OPERAND_SIZE-1 : RESULT_INDEX] = stage_result;
    end
endmodule


module First_Stage( input                       signed_mul,
                    input  [OPERAND_SIZE-2 : 0]          X, 
                    input  [OPERAND_SIZE-1 : 0]          Y, 
                    output [OPERAND_SIZE-1 : 0]        Sum, 
                    output [OPERAND_SIZE-1 : 0]       Cout
                    );
	parameter OPERAND_SIZE = 32;

    genvar i;

    generate
        for(i=0; i<OPERAND_SIZE-2; i=i+1) begin
            HalfAdder1 HA0 (X[i],Y[i],Sum[i],Cout[i]);
        end
    endgenerate

    HalfAdder1 HA1 ( (signed_mul) ? ~X[OPERAND_SIZE-2] : X[OPERAND_SIZE-2], Y[OPERAND_SIZE-2], Sum[OPERAND_SIZE-2], Cout[OPERAND_SIZE-2]);

    HalfAdder1 HA2 ( (signed_mul) ? 1'b1 : 1'b0 , (signed_mul) ? ~Y[OPERAND_SIZE-1] : Y[OPERAND_SIZE-1], Sum[OPERAND_SIZE-1], Cout[OPERAND_SIZE-1]);

endmodule

module Second_Stage(input                       signed_mul,
                    input  [OPERAND_SIZE-1 : 0]          X, // stage partial product
                    input  [OPERAND_SIZE-2 : 0]          Y, // prev stage sum
                    input  [OPERAND_SIZE-1 : 0]        Cin, // prev stage cout
                    output [OPERAND_SIZE-1 : 0]        Sum, 
                    output [OPERAND_SIZE-1 : 0]       Cout
                    );
    parameter OPERAND_SIZE = 32;

    genvar i;

    generate
        for(i=0; i<OPERAND_SIZE-1; i=i+1) begin
            FullAdder1 FA0 (X[i],Y[i],Cin[i],Sum[i],Cout[i]);
        end
    endgenerate

    HalfAdder1 HA0 ( (signed_mul) ? ~X[OPERAND_SIZE-1] : X[OPERAND_SIZE-1], Cin[OPERAND_SIZE-1], Sum[OPERAND_SIZE-1], Cout[OPERAND_SIZE-1]);

endmodule


module Third_Stage( input                       signed_mul,
                    input  [OPERAND_SIZE-1 : 0]          X, //prev stage cout
                    input  [OPERAND_SIZE-2 : 0]          Y, //prev stage sum
                    output [OPERAND_SIZE-1 : 0]        Sum,
                    output                            Cout
                    ); // Carry Look-Ahead

    parameter OPERAND_SIZE = 32;

	cla_adder_32 cla32( .x( X ), 
                        .y( {((signed_mul) ? 1'b1 : 1'b0), Y} ), 
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

