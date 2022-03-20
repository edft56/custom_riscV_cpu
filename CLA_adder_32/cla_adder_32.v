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

reg P_out_int;
reg [3:0] C_out_int;

    assign G_out = G_in[3] | (P_in[3]&G_in[2]) | (P_in[3]&P_in[2]&G_in[1]) | (P_in[3]&P_in[2]&P_in[1]&G_in[0]);
    assign P_out = P_out_int;
    assign C_out = C_out_int[3:1];

    always @*
    begin
        C_out_int[0] = C_in;
        P_out_int = P_in[0];
        for(i=1; i<4; i=i+1)
        begin
            P_out_int = P_out_int & P_in[i];
            C_out_int[i] = G_in[i-1] | (C_out_int[i-1] & P_in[i-1]);
        end
    end

endmodule


module claAdder4(   input [3:0] x,y,
                    input c_in,
                    output [3:0] sum,
                    output c_out,P_out,G_out);

    wire [3:0] P_int,G_int,C_int;
    wire       P_final, G_final;
    
    assign C_int[0] = c_in;
    assign c_out = (P_final & c_in) | G_final;
    assign P_out = P_final;
    assign G_out = G_final;

    CLA4 CLA0(  .P_in(P_int),
                .G_in(G_int),
                .C_in(c_in),
                .C_out(C_int[3:1]),
                .P_out(P_final),
                .G_out(G_final));

    genvar i;

    generate
        for(i=0; i<4; i=i+1) begin
            fullAdder1 FA0( .x(x[i]),
                            .y(y[i]),
                            .c_in(C_int[i]),
                            .sum(sum[i]),
                            .P_out(P_int[i]),
                            .G_out(G_int[i]));
        end
    endgenerate

endmodule

module CLA_8(G_in,P_in,C_in,G_out,P_out,C_out);
input [7:0] G_in,P_in;
input C_in;
output G_out,P_out;
output [6:0] C_out;

reg [7:0] C_out_int;
reg P_int;
integer i;

	assign C_out = C_out_int[7:1];
	assign P_out = P_int;

	assign G_out =  G_in[7] | (P_in[7] & G_in[6]) | (P_in[7] & P_in[6] & G_in[5]) 
                    | (P_in[7] & P_in[6] & P_in[5] & G_in[4]) 
				    | (P_in[7] & P_in[6] & P_in[5] & P_in[4] & G_in[3]) 
                    | (P_in[7] & P_in[6] & P_in[5] & P_in[4] & P_in[3] & G_in[2])
				    | (P_in[7] & P_in[6] & P_in[5] & P_in[4] & P_in[3] & P_in[2] & G_in[1])
                    | (P_in[7] & P_in[6] & P_in[5] & P_in[4] & P_in[3] & P_in[2] & P_in[1] & G_in[0]);

	always@*
	begin
		C_out_int[0] = C_in;
		P_int = P_in[0];
		for(i=1; i<8; i=i+1)
		begin
			P_int = P_int & P_in[i];
			C_out_int[i] = G_in[i-1] | (P_in[i-1] & C_out_int[i-1]);
		end
	end

endmodule


module cla_adder_32(  input [31:0] x,
                    input [31:0] y,
                    input sub,
                    output [31:0] sum,
                    output c_out
                    );

wire [7:0] C_int;
wire [7:0] P_int, G_int;
wire [31:0] y_final;
wire P_final, G_final;

    assign C_int[0] = sub;
    assign c_out = (P_final & sub) | G_final;

    assign y_final = y ^ {32{sub}};


    genvar i;

    generate
        for(i=0; i<8; i=i+1) begin
            claAdder4 CLA4_0(   .x( x[4*(i+1)-1 : 4*i] ),
                                .y( y_final[4*(i+1)-1 : 4*i] ),
                                .c_in( C_int[i] ),
                                .sum( sum[4*(i+1)-1 : 4*i] ),
                                .c_out( ),
                                .P_out( P_int[i] ),
                                .G_out( G_int[i] ));
        end
    endgenerate

    CLA_8 CLA8_0(   .G_in(G_int),
                    .P_in(P_int),
                    .C_in(sub),
                    .G_out(G_final),
                    .P_out(P_final),
                    .C_out(C_int[7:1]));

endmodule
