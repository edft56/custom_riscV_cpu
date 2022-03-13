`timescale 1ns/1ns

module mul8x8(X,Y,Result);
input [7:0] X,Y;
output [15:0] Result;

wire[5:0] stage_sum0,stage_sum1,stage_sum2,stage_sum3,stage_sum4,stage_sum5,stage_sum6;
wire[6:0] stage_Cout0,stage_Cout1,stage_Cout2,stage_Cout3,stage_Cout4,stage_Cout5,stage_Cout6;
wire[7:0] input1,input2,input3,input4,input5,input6,input7,input8;

	assign input1 = X & {8{Y[0]}};
	assign input2 = X & {8{Y[1]}};
	assign input3 = X & {8{Y[2]}};
	assign input4 = X & {8{Y[3]}};
	assign input5 = X & {8{Y[4]}};
	assign input6 = X & {8{Y[5]}};
	assign input7 = X & {8{Y[6]}};
	assign input8 = X & {8{Y[7]}};

	assign Result[0] = input1[0];

	First_Stage S0 (input1[7:1],input2[6:0],{stage_sum0,Result[1]},stage_Cout0);

	Second_Stage S1 ({input2[7],stage_sum0},input3[6:0],stage_Cout0,{stage_sum1,Result[2]},stage_Cout1);
	Second_Stage S2 ({input3[7],stage_sum1},input4[6:0],stage_Cout1,{stage_sum2,Result[3]},stage_Cout2);
	Second_Stage S3 ({input4[7],stage_sum2},input5[6:0],stage_Cout2,{stage_sum3,Result[4]},stage_Cout3);
	Second_Stage S4 ({input5[7],stage_sum3},input6[6:0],stage_Cout3,{stage_sum4,Result[5]},stage_Cout4);
	Second_Stage S5 ({input6[7],stage_sum4},input7[6:0],stage_Cout4,{stage_sum5,Result[6]},stage_Cout5);
	Second_Stage S6 ({input7[7],stage_sum5},input8[6:0],stage_Cout5,{stage_sum6,Result[7]},stage_Cout6);

	Third_Stage2 S7 ({input8[7],stage_sum6},stage_Cout6,Result[14:8],Result[15]);
endmodule

module mul8x8(  input  [31:0]      X,
                input  [31:0]      Y,
                output [63:0] Result
                );



endmodule

module First_Stage( input  [31:0]    X, 
                    input  [31:0]    Y, 
                    output [31:0]  Sum, 
                    output [31:0] Cout
                    );
	
    genvar i;

    generate
        for(i=0; i<31; i=i+1) begin
            HalfAdder1 HA0 (X[i],Y[i],Sum[i],Cout[i]);
        end
    endgenerate

endmodule

module Second_Stage(input  [31:0]   X, 
                    input  [31:0]   Y, 
                    input  [31:0] Cin, 
                    output [31:0] Sum, 
                    output [31:0] Cout
                    );
    
    genvar i;

    generate
        for(i=0; i<31; i=i+1) begin
            FullAdder1 FA0 (X[i],Y[i],Cin[i],Sum[i],Cout[i]);
        end
    endgenerate

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



module Third_Stage2(X,Y,Sum,Cout); // Carry Look-Ahead
input [6:0] X,Y;
output [6:0] Sum;
output Cout;

wire Pg,Gg;

	CLA_Adder_7 CLA7(X,Y,0,Sum,Pg,Gg);
	assign Cout = Gg;
endmodule

