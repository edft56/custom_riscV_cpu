`timescale 1ns/1ns

module top(   input [31:0] x,
                            input [4:0] shift_by,
                            input left,
                            input arith,
                            output [31:0] out);

wire [31:0] stage16,stage8,stage4,stage2,stage1;

    wire [31:0] reverse1_out;

    wire replace_val = (arith) ? x[31] : 1'b0;

    wire [31:0] in1_sh16 = { {16{replace_val}} , reverse1_out[31:16] };
    wire [31:0] in1_sh8 = { {8{replace_val}} , stage16[31:8] };
    wire [31:0] in1_sh4 = { {4{replace_val}} , stage8[31:4] };
    wire [31:0] in1_sh2 = { {2{replace_val}} , stage4[31:2] };
    wire [31:0] in1_sh1 = { {1{replace_val}} , stage2[31:1] };

    genvar i;

    generate
        

        for(i=0; i<32; i=i+1) begin
            mux_2to1 reverse1(.in0( x[i] ),
                            .in1( x[31-i] ),
                            .sel( left ),
                            .out( reverse1_out[i] ));

            mux_2to1 shift16(.in0( reverse1_out[i] ),
                            .in1( in1_sh16[i] ),
                            .sel( shift_by[4] ),
                            .out( stage16[i] ));

            mux_2to1 shift8(.in0( stage16[i] ),
                            .in1( in1_sh8[i] ),
                            .sel( shift_by[3] ),
                            .out( stage8[i] ));

            mux_2to1 shift4(.in0( stage8[i] ),
                            .in1( in1_sh4[i] ),
                            .sel( shift_by[2] ),
                            .out( stage4[i] ));

            mux_2to1 shift2(.in0( stage4[i] ),
                            .in1( in1_sh2[i] ),
                            .sel( shift_by[1] ),
                            .out( stage2[i] ));

            mux_2to1 shift1(.in0( stage2[i] ),
                            .in1( in1_sh1[i] ),
                            .sel( shift_by[0] ),
                            .out( stage1[i] ));

            mux_2to1 reverse2(.in0( stage1[i] ),
                            .in1( stage1[31-i] ),
                            .sel( left ),
                            .out( out[i] ));
        end
    endgenerate


endmodule


module mux_2to1(input in0,
                input in1,
                input sel,
                output out);

    assign out = (sel) ? in1 : in0;

endmodule
