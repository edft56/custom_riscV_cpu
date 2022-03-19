`timescale 1ns/1ns


module int_div_32(  input                        clk_i,
                    input                        load_i,
                    input   [OPERAND_SIZE-1 : 0] dividend_i,
                    input   [OPERAND_SIZE-1 : 0] divisor_i,

                    output  [OPERAND_SIZE-1 : 0] quotient_o,
                    output  [OPERAND_SIZE-1 : 0] remainder_o   
                    );
    parameter OPERAND_SIZE = 32;

    reg [OPERAND_SIZE-1 : 0] acc_q;
    reg [OPERAND_SIZE-1 : 0] dividend_q;
    reg [OPERAND_SIZE-1 : 0] divisor_q;
    reg                      state_q;
    reg [             4 : 0] cntr_q; 

    initial state_q = 0;

    wire [OPERAND_SIZE : 0] sub_result;

    assign sub_result  = {acc_q[OPERAND_SIZE-2:0],dividend_q[OPERAND_SIZE-1]} - divisor_q;
    assign quotient_o  = dividend_q;
    assign remainder_o = acc_q; 

    always @(negedge clk_i) begin
        case(state_q)
            1'b0: begin
                acc_q      <= 0;
                dividend_q <= (load_i) ? dividend_i : dividend_q;
                divisor_q  <= (load_i) ? divisor_i  : divisor_q;
                cntr_q     <= 0;
                state_q    <= (load_i) ? 1'b1       : state_q;
            end
            1'b1: begin
                acc_q      <= (sub_result[OPERAND_SIZE] == 0) ?      sub_result[OPERAND_SIZE-1 : 0] : { acc_q[OPERAND_SIZE-2 : 0], dividend_q[OPERAND_SIZE-1] };
                dividend_q <= (sub_result[OPERAND_SIZE] == 0) ? {dividend_q[OPERAND_SIZE-2:0],1'b1} :                       {dividend_q[OPERAND_SIZE-2:0],1'b0};
                divisor_q  <= divisor_q;
                cntr_q     <= cntr_q + 1;
                state_q    <= (cntr_q == OPERAND_SIZE-1) ? 1'b0 : state_q;
            end
            default: begin
                acc_q      <= 0;
                dividend_q <= 0;
                divisor_q  <= 0;
                cntr_q     <= 0;
                state_q    <= 0;
            end
        endcase
    end

endmodule
