`timescale 1ns/1ns


module int_div_32(  input                        clk_i,
                    input                        load_i,
                    input   [OPERAND_SIZE-1 : 0] dividend_i,
                    input   [OPERAND_SIZE-1 : 0] divisor_i,
                    input                        signed_i,

                    output  reg                  result_rdy,
                    output  [OPERAND_SIZE-1 : 0] quotient_o,
                    output  [OPERAND_SIZE-1 : 0] remainder_o   
                    );
    parameter OPERAND_SIZE = 32;

    reg [OPERAND_SIZE-1 : 0] acc_q;
    reg [OPERAND_SIZE-1 : 0] dividend_q;
    reg [OPERAND_SIZE-1 : 0] divisor_q;
    reg [             1 : 0] state_q;
    reg [             4 : 0] cntr_q; 
    reg                      sign_result;
    reg                      sign_dividend;
    reg                      signed_divsn;

    wire [OPERAND_SIZE-1 : 0] sub_result;


    initial state_q = 0;

    
    assign quotient_o  = dividend_q;
    assign remainder_o = acc_q; 


    cla_adder_32 cla32( .x( {acc_q[OPERAND_SIZE-2:0],dividend_q[OPERAND_SIZE-1]} ), 
                        .y( divisor_q ), 
                        .sub( (signed_i) ? ~divisor_q[OPERAND_SIZE-1] : 1'b1 ), 
                        .sum( sub_result[OPERAND_SIZE-1 : 0] ), 
                        .c_out( )
                        );

    //assign sub_result  = {acc_q[OPERAND_SIZE-2:0],dividend_q[OPERAND_SIZE-1]} - divisor_q;

    always @(negedge clk_i) begin
        case(state_q)
            2'b00: begin
                result_rdy    <= 0;
                signed_divsn  <= signed_i;
                sign_dividend <= dividend_i[OPERAND_SIZE-1];
                sign_result   <= (dividend_i[OPERAND_SIZE-1] ^ divisor_i[OPERAND_SIZE-1]);
                acc_q         <= 0;
                dividend_q    <= (load_i) ? ( (signed_i & (dividend_i[OPERAND_SIZE-1] == 1'b1)) ? ~dividend_i + 1 : dividend_i ) : dividend_q;
                divisor_q     <= (load_i) ? divisor_i  : divisor_q;
                cntr_q        <= 0;
                state_q       <= (load_i) ? 2'b01      : state_q;
            end
            2'b01: begin
                result_rdy    <= 0;
                signed_divsn  <= signed_divsn;
                sign_dividend <= sign_dividend;
                sign_result   <= sign_result;
                acc_q         <= (sub_result[OPERAND_SIZE-1] == 0) ?      sub_result[OPERAND_SIZE-1 : 0] : { acc_q[OPERAND_SIZE-2 : 0], dividend_q[OPERAND_SIZE-1] };
                dividend_q    <= (sub_result[OPERAND_SIZE-1] == 0) ? {dividend_q[OPERAND_SIZE-2:0],1'b1} :                       {dividend_q[OPERAND_SIZE-2:0],1'b0};
                divisor_q     <= divisor_q;
                cntr_q        <= cntr_q + 1;
                state_q       <= (cntr_q == OPERAND_SIZE-1) ? 2'b10 : state_q;
            end
            2'b10: begin
                result_rdy    <= 1;
                signed_divsn  <= signed_divsn;
                sign_dividend <= sign_dividend;
                sign_result   <= sign_result;
                acc_q         <= (signed_divsn & sign_dividend) ? ~acc_q+1      : acc_q;
                dividend_q    <= (signed_divsn & sign_result) ? ~dividend_q+1 : dividend_q;
                divisor_q     <= divisor_q;
                cntr_q        <= cntr_q;
                state_q       <= 2'b00;
            end
            default: begin
                result_rdy    <= 0;
                signed_divsn  <= 0;
                sign_dividend <= 0;
                sign_result   <= 0;
                acc_q         <= 0;
                dividend_q    <= 0;
                divisor_q     <= 0;
                cntr_q        <= 0;
                state_q       <= 0;
            end
        endcase
    end

endmodule
