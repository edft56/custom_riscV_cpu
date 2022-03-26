`timescale 1ns/1ns

`include "CLA_adder_32/cla_adder_32.v"
`include "barrel_shifter_32/barrel_shifter_32.v"
`include "IntegerMultiply/Signed/int_mul_32.v"
`include "IntegerDivide/int_div_32.v"

`define WL 31 //word length
`define IMEM_SIZE 8*1024*1024
`define DMEM_SIZE 8*1024*1024

module riscv_custom( input clk, input reset);
    wire [`WL:0]   PC_IF;
    wire [ 31:0]   fetched_instruction_IF;
    
    wire [`WL:0]   rd_data;
    wire [`WL:0]   rs1_data_DE;
    wire [ 4:0]    rd_idx_DE;
    wire [`WL:0]   rs2_data_DE;
    wire [ 4:0]    rs1_idx_DE;
    wire [ 4:0]    rs2_idx_DE;
    wire [`WL:0]   branch_PC_DE;
    wire           branch_instruction_DE;
    wire           jump_instruction_DE;
    wire [`WL:0]   immediate_data_DE;
    wire           data_mem_cs_DE;
    wire           data_mem_wr_DE;
    wire [ 2:0]    funct3_DE;
    wire [ 6:0]    opcode_DE;
    wire [`WL:0]   alu_result;
    wire           PC_alu_input_DE;
    wire           immediate_alu_input_DE;
    wire           alu_write_reg_DE;
    wire           immediate_reg_write_DE;
    wire           PC_reg_write_DE;
    wire [`WL:0]   PC_DE;
    wire [ 4:0]    alu_op_DE;

    
    wire [`WL:0]   rs2_data_EX;
    wire [ 4:0]    rs2_idx_EX;
    wire [ 4:0]    rd_idx_EX;
    wire [`WL:0]   ALU_result_EX;
    wire           take_branch;
    wire           data_mem_cs_EX;
    wire           data_mem_wr_EX;
    wire [ 6:0]    opcode_EX;
    wire [ 2:0]    funct3_EX;
    wire [`WL:0]   immediate_data_EX;
    wire [`WL:0]   PC_EX;
    wire           alu_write_reg_EX;
    wire           immediate_reg_write_EX;
    wire           PC_reg_write_EX;

    wire [ 4:0]    rd_idx_MEM;
    wire [`WL:0]   ALU_result_MEM;
    wire [`WL:0]   mem_result_MEM;
    wire [ 6:0]    opcode_MEM;
    wire [ 2:0]    funct3_MEM;
    wire [`WL:0]   immediate_data_MEM;
    wire [`WL:0]   PC_MEM;
    wire           alu_write_reg_MEM;
    wire           immediate_reg_write_MEM;
    wire           PC_reg_write_MEM;
    wire           mem_reg_write_MEM;

    wire           forward_rs1_MEM_EX;
    wire           forward_rs2_MEM_EX;
    wire           forward_rs1_WB_EX;
    wire           forward_rs2_WB_EX;
    wire           forward_WB_MEM;

    wire           interlock_stall;


    registerFile regFile(   .clk( clk ),
                            .write( alu_write_reg_MEM | immediate_reg_write_MEM | PC_reg_write_MEM | mem_reg_write_MEM ),
                            .rd_idx( rd_idx_MEM ),
                            .rd_data( rd_data ),
                            .rs1_idx( fetched_instruction_IF[19:15] ),
                            .rs2_idx( fetched_instruction_IF[24:20] ),

                            .rs1_data( rs1_data_DE ),
                            .rs2_data( rs2_data_DE )
                        );

    instructionFetch IF_stage0( .clk_i( clk ),
                                .branch_PC_i( branch_PC_DE ),
                                .jump_PC_i( alu_result ),
                                .take_branch_i( take_branch ),
                                .interlock_stall_i( interlock_stall ),
                                .reset_i( reset ),
                                .jump_instruction_DE ( jump_instruction_DE ),

                                .PC_IF_o( PC_IF ),
                                .fetched_instruction_IF_o( fetched_instruction_IF )
                                );

    
    decode DE_stage0(   .clk( clk ),
                        .interlock_stall( interlock_stall ),
                        .take_branch(take_branch),
                        .fetched_instruction_IF( fetched_instruction_IF ),
                        .PC_IF( PC_IF ),
                        
                        .funct3_DE( funct3_DE ),
                        .opcode_DE( opcode_DE ),
                        .rd_idx_DE( rd_idx_DE ),
                        .rs1_idx_DE( rs1_idx_DE ),
                        .rs2_idx_DE( rs2_idx_DE ),
                        .jump_instruction_DE( jump_instruction_DE ),
                        .branch_instruction_DE( branch_instruction_DE ),
                        .PC_alu_input_DE( PC_alu_input_DE ),
                        .immediate_alu_input_DE( immediate_alu_input_DE ),
                        .data_mem_cs_DE( data_mem_cs_DE ),
                        .data_mem_wr_DE( data_mem_wr_DE ),
                        .alu_write_reg_DE( alu_write_reg_DE ),
                        .immediate_reg_write_DE( immediate_reg_write_DE ),
                        .PC_reg_write_DE( PC_reg_write_DE ),
                        .immediate_data_DE( immediate_data_DE ),
                        .branch_PC_DE( branch_PC_DE ),
                        .PC_DE( PC_DE ),
                        .alu_op_DE( alu_op_DE )
                        );
    
    exec_stage exec_stage0( .clk( clk ),
                            .rs1_data_DE( rs1_data_DE ),
                            .rs2_data_DE( rs2_data_DE ),
                            .immediate_data_DE( immediate_data_DE ),
                            .PC_DE( PC_DE ),
                            .rd_idx_DE( rd_idx_DE ),
                            .rs2_idx_DE( rs2_idx_DE ),
                            .alu_op_DE( alu_op_DE ),
                            .opcode_DE( opcode_DE ),
                            .funct3_DE( funct3_DE ),
                            .forwarded_data_MEM( ALU_result_EX ),
                            .forwarded_data_WB( rd_data ),
                            .forward_rs1_MEM_EX( forward_rs1_MEM_EX ),
                            .forward_rs2_MEM_EX( forward_rs2_MEM_EX ),
                            .forward_rs1_WB_EX( forward_rs1_WB_EX ),
                            .forward_rs2_WB_EX( forward_rs2_WB_EX ), 
                            .data_mem_cs_DE( data_mem_cs_DE ),
                            .data_mem_wr_DE( data_mem_wr_DE ),
                            .branch_instruction_DE( branch_instruction_DE ),
                            .PC_alu_input_DE( PC_alu_input_DE ),
                            .immediate_alu_input_DE( immediate_alu_input_DE ),
                            .alu_write_reg_DE( alu_write_reg_DE ),
                            .immediate_reg_write_DE( immediate_reg_write_DE ),
                            .PC_reg_write_DE( PC_reg_write_DE ),

                            .alu_result_EX( ALU_result_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .immediate_data_EX( immediate_data_EX ),
                            .PC_EX( PC_EX ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .alu_write_reg_EX( alu_write_reg_EX ),
                            .immediate_reg_write_EX( immediate_reg_write_EX ),
                            .PC_reg_write_EX( PC_reg_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),
                            .rs2_idx_EX( rs2_idx_EX ),
                            .opcode_EX( opcode_EX ),
                            .funct3_EX( funct3_EX ),
                            .alu_result( alu_result ),
                            .take_branch( take_branch )
                            );


    mem_stage mem_stage0(   .clk( clk ),
                            .alu_result_EX( ALU_result_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .alu_write_reg_EX( alu_write_reg_EX ),
                            .immediate_reg_write_EX( immediate_reg_write_EX ),
                            .PC_reg_write_EX( PC_reg_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),
                            .opcode_EX( opcode_EX ),
                            .forward_WB_MEM( forward_WB_MEM ),
                            .forwarded_data_WB( rd_data ),
                            .immediate_data_EX( immediate_data_EX ),
                            .PC_EX( PC_EX ),
                            .funct3_EX( funct3_EX ),

                            .mem_result_MEM( mem_result_MEM ),
                            .alu_result_MEM( ALU_result_MEM ),
                            .PC_MEM( PC_MEM ),
                            .immediate_data_MEM( immediate_data_MEM ),
                            .alu_write_reg_MEM( alu_write_reg_MEM ),
                            .immediate_reg_write_MEM( immediate_reg_write_MEM ),
                            .PC_reg_write_MEM( PC_reg_write_MEM ),
                            .mem_reg_write_MEM( mem_reg_write_MEM ),
                            .rd_idx_MEM( rd_idx_MEM ),
                            .opcode_MEM( opcode_MEM ),
                            .funct3_MEM( funct3_MEM )
                        );


    wb_stage wb_stage0( .alu_data_MEM( ALU_result_MEM ),
                        .mem_data_MEM( mem_result_MEM ),
                        .PC_MEM( PC_MEM ),
                        .immediate_data_MEM( immediate_data_MEM ),
                        .funct3_MEM( funct3_MEM ),
                        .alu_write_reg_MEM( alu_write_reg_MEM ),
                        .immediate_reg_write_MEM( immediate_reg_write_MEM ),
                        .PC_reg_write_MEM( PC_reg_write_MEM ),
                        .mem_reg_write_MEM( mem_reg_write_MEM ),
                        
                        .rd_data( rd_data )
                    );

    control ctrl0(  .clk_i( clk ),
                    .fetched_instruction_IF_i( fetched_instruction_IF ),
                    .opcode_DE_i( opcode_DE ),
                    .opcode_EX_i( opcode_EX ),
                    .opcode_MEM_i( opcode_MEM ),
                    .rd_idx_DE_i( rd_idx_DE ),
                    .rd_idx_EX_i( rd_idx_EX ),
                    .rd_idx_MEM_i( rd_idx_MEM ),
                    .rs1_idx_DE_i( rs1_idx_DE ),
                    .rs2_idx_DE_i( rs2_idx_DE ),
                    .rs2_idx_EX_i( rs2_idx_EX ),
                    
                    .forward_rs1_MEM_EX_o( forward_rs1_MEM_EX ),
                    .forward_rs2_MEM_EX_o( forward_rs2_MEM_EX ),
                    .forward_rs1_WB_EX_o( forward_rs1_WB_EX ),
                    .forward_rs2_WB_EX_o( forward_rs2_WB_EX ),
                    .forward_WB_MEM_o( forward_WB_MEM ),
                    .interlock_stall_o( interlock_stall )
                );                

endmodule

module instructionFetch(input clk_i,
                        input [`WL:0]   branch_PC_i,
                        input [`WL:0]   jump_PC_i,
                        input           take_branch_i,
                        input           interlock_stall_i,
                        input           reset_i,
                        input           jump_instruction_DE,

                        output [`WL:0]  PC_IF_o,
                        output [ 31:0]  fetched_instruction_IF_o
                        );
    
    reg [`WL:0] PC_q;
    reg [ 31:0] instruction_buffer_q [1:0];
    reg [ 31:0] PC_buffer_q [1:0];
    reg         after_interlock_stall_q;
    
    wire [ 31:0] mem_instruction;

    wire [`WL:0] mem_addr = (jump_instruction_DE) ? {jump_PC_i[31:1],1'b0} : ( (take_branch_i) ? branch_PC_i : PC_q );


    memory #(
            .MEM_WORD_SIZE( 32 ),
            .MEM_SIZE_BYTES( `IMEM_SIZE ),
            .ADRESS_WIDTH( 23 ),
            .initialize( 1 ),
            .init_file("ass_bin.dat")
            )
        instr_mem
            (   
            .clk( clk_i ),
            .wr( 1'b0 ),
            .cs( 1'b1 & reset_i ),
            .addr( mem_addr >> 2 ),
            .data_in(),

            .data_out( mem_instruction )
            );

    assign fetched_instruction_IF_o = instruction_buffer_q[0];
    assign PC_IF_o = PC_buffer_q[0];

    always @(negedge clk_i)
    begin
        if(!reset_i)  begin   
            PC_q                    <= 0;
            after_interlock_stall_q <= 0;
            instruction_buffer_q[0] <= 0;
            instruction_buffer_q[1] <= 0;
        end
        else if (take_branch_i | jump_instruction_DE)  begin
            PC_q                    <= mem_addr + 4;
            after_interlock_stall_q <= 0;
            instruction_buffer_q[0] <= mem_instruction;
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0]          <= PC_q;
            PC_buffer_q[1]          <= PC_q;
        end
        else if (interlock_stall_i)  begin
            PC_q                    <= PC_q;
            after_interlock_stall_q <= 1;
            instruction_buffer_q[0] <= instruction_buffer_q[1];
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0]          <= PC_buffer_q[1];
            PC_buffer_q[1]          <= PC_q;
        end
        else if (after_interlock_stall_q) begin
            PC_q                    <= PC_q + 4;
            after_interlock_stall_q <= 0;
            instruction_buffer_q[0] <= instruction_buffer_q[1];
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0]          <= PC_buffer_q[1];
            PC_buffer_q[1]          <= PC_q;
        end
        else begin
            PC_q                    <= PC_q + 4;
            after_interlock_stall_q <= 0;
            instruction_buffer_q[0] <= mem_instruction;
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0]          <= PC_q;
            PC_buffer_q[1]          <= PC_q;
        end 
    end

endmodule

module decode(  input           clk,
                input           interlock_stall,
                input           take_branch,
                input [ 31:0]   fetched_instruction_IF,
                input [`WL:0]   PC_IF,
                
                output reg [  2:0]  funct3_DE,
                output reg [  6:0]  opcode_DE,
                output reg [  4:0]  rd_idx_DE, rs1_idx_DE, rs2_idx_DE,
                output reg          jump_instruction_DE,
                output reg          branch_instruction_DE,
                output reg          PC_alu_input_DE,
                output reg          immediate_alu_input_DE,
                output reg          data_mem_cs_DE, data_mem_wr_DE,
                output reg          alu_write_reg_DE,
                output reg          immediate_reg_write_DE,
                output reg          PC_reg_write_DE,
                output reg [`WL:0]  immediate_data_DE,
                output reg [`WL:0]  branch_PC_DE,
                output reg [`WL:0]  PC_DE,
                output reg [  4:0]  alu_op_DE
                );

    wire [  6:0] opcode;
    wire [`WL:0] branch_immediate_data;
    wire [  2:0] funct3;
    
    reg branch_jump_stall_timer_q;


                                        //Sign Extend                                                                                                     x2
    assign branch_immediate_data = {{20{fetched_instruction_IF[31]}},fetched_instruction_IF[7],fetched_instruction_IF[30:25],fetched_instruction_IF[11:8],1'b0};
    assign opcode                = (interlock_stall | take_branch | jump_instruction_DE | branch_jump_stall_timer_q) ? 7'b0000000 : fetched_instruction_IF[6:0];
    assign funct3                = fetched_instruction_IF[14:12];

    always @(negedge clk)
    begin
        funct3_DE                 <= funct3; 
        rd_idx_DE                 <= fetched_instruction_IF[11:7];
        rs1_idx_DE                <= fetched_instruction_IF[19:15];
        rs2_idx_DE                <= fetched_instruction_IF[24:20];
        opcode_DE                 <= opcode;
        branch_PC_DE              <= PC_IF + branch_immediate_data;
        PC_DE                     <= PC_IF;
        branch_jump_stall_timer_q <= (take_branch | jump_instruction_DE);


        case(opcode)
            7'b0000000: //pipeline stall
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b0;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= 5'd0; //dont care

                immediate_data_DE       <= 32'd0;
            end
            7'b1100011: //Branch
            begin
                branch_instruction_DE   <= 1'b1;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= {1'b0,funct3}; //add

                immediate_data_DE       <= branch_immediate_data;
            end
            7'b0010011: //ALU Immediate
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b1;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;
                                                                            //SLRI,SRAI
                alu_op_DE               <= {fetched_instruction_IF[30] & (funct3 == 3'b101), funct3};

                                            //SLLI,SRLI,SRAI
                immediate_data_DE       <= (funct3 == 3'b001 & funct3 == 3'b101) 
                                           ? {27'd0 , fetched_instruction_IF[24:20]} 
                                           : { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:20] };
            end
            7'b0000011: //LOADS
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b1;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= 5'd0; //add

                immediate_data_DE       <= { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:20]}; //sign extend
            end
            
            7'b0100011: //STORE
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b1;
                data_mem_wr_DE          <= 1'b1;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= 5'd0; //add

                immediate_data_DE       <= { {20{fetched_instruction_IF[31]}}, fetched_instruction_IF[31:25], fetched_instruction_IF[11:7] };
            end
            7'b0110011: //ALU RR
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b0;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b1;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= {fetched_instruction_IF[30],fetched_instruction_IF[25], funct3};

                immediate_data_DE       <= 32'd0;
            end
            7'b1101111: //JAL
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b1;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b1;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b1;

                alu_op_DE               <= 5'd0; //add

                immediate_data_DE       <= {{11{fetched_instruction_IF[31]}},fetched_instruction_IF[31],fetched_instruction_IF[19:12],fetched_instruction_IF[20],fetched_instruction_IF[30:21],1'b0};
            end
            7'b1100111: //JALR
                begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b1;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b1;

                alu_op_DE               <= 5'd0; //add

                immediate_data_DE       <= {{20{fetched_instruction_IF[31]}},fetched_instruction_IF[31:20]};
            end
            7'b0110111: //LUI
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b0;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b1;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= 5'd0; //dont care

                immediate_data_DE       <= {fetched_instruction_IF[31:12], 12'd0};
            end
            7'b0010111: //AUIPC
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b1;
                PC_alu_input_DE         <= 1'b1;
                alu_write_reg_DE        <= 1'b1;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= 5'd0; //add

                immediate_data_DE       <= {fetched_instruction_IF[31:12], 12'd0};
            end
            default:
            begin
                branch_instruction_DE   <= 1'b0;
                jump_instruction_DE     <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                immediate_alu_input_DE  <= 1'b0;
                PC_alu_input_DE         <= 1'b0;
                alu_write_reg_DE        <= 1'b0;
                immediate_reg_write_DE  <= 1'b0;
                PC_reg_write_DE         <= 1'b0;

                alu_op_DE               <= 5'd0; //dont care

                immediate_data_DE       <= 32'd0;
            end
        endcase

    end
endmodule

module exec_stage(  input clk,
                    input [`WL:0]   rs1_data_DE, rs2_data_DE, immediate_data_DE,
                    input [`WL:0]   PC_DE,
                    input [  4:0]   rd_idx_DE, rs2_idx_DE,
                    input [  3:0]   alu_op_DE,
                    input [  6:0]   opcode_DE,
                    input [  2:0]   funct3_DE,
                    input [`WL:0]   forwarded_data_MEM, forwarded_data_WB,
                    input           forward_rs1_MEM_EX, forward_rs2_MEM_EX, forward_rs1_WB_EX, forward_rs2_WB_EX,
                    input           data_mem_cs_DE, data_mem_wr_DE,
                    input           branch_instruction_DE,
                    input           PC_alu_input_DE,
                    input           immediate_alu_input_DE,
                    input           alu_write_reg_DE,
                    input           immediate_reg_write_DE,
                    input           PC_reg_write_DE,
                    
                    output reg [`WL:0]  alu_result_EX,
                    output reg [`WL:0]  rs2_data_EX,
                    output reg [`WL:0]  immediate_data_EX,
                    output reg [`WL:0]  PC_EX,
                    output reg          data_mem_cs_EX, data_mem_wr_EX,
                    output reg          alu_write_reg_EX,
                    output reg          immediate_reg_write_EX,
                    output reg          PC_reg_write_EX,
                    output reg [4:0]    rd_idx_EX, rs2_idx_EX,
                    output reg [6:0]    opcode_EX,
                    output reg [2:0]    funct3_EX,
                    
                    output     [`WL:0]  alu_result,
                    output              take_branch
                    );

    wire branch_cond;
    wire [`WL:0] alu_input2;
    wire [`WL:0] alu_input1;
    wire [`WL:0] forwarded_rs1_data, forwarded_rs2_data;

    assign forwarded_rs1_data = (forward_rs1_MEM_EX) ? forwarded_data_MEM : forwarded_data_WB;
    assign forwarded_rs2_data = (forward_rs2_MEM_EX) ? forwarded_data_MEM : forwarded_data_WB;

    assign alu_input1 = (forward_rs1_MEM_EX | forward_rs1_WB_EX) ? forwarded_rs1_data : ( (PC_alu_input_DE)        ? PC_DE             : rs1_data_DE );
    assign alu_input2 = (forward_rs2_MEM_EX | forward_rs2_WB_EX) ? forwarded_rs2_data : ( (immediate_alu_input_DE) ? immediate_data_DE : rs2_data_DE );

    alu alu_unit(   .in1(alu_input1),
                    .in2(alu_input2),
                    .op(alu_op_DE[2:0]),
                    .funct7_bit(alu_op_DE[3]),
                    .out(alu_result),
                    .take_branch(branch_cond));

    assign take_branch = (branch_instruction_DE & branch_cond);

    always @(negedge clk)
    begin
        alu_result_EX           <= alu_result;
        data_mem_cs_EX          <= data_mem_cs_DE;
        data_mem_wr_EX          <= data_mem_wr_DE;
        alu_write_reg_EX        <= alu_write_reg_DE;
        immediate_reg_write_EX  <= immediate_reg_write_DE;
        PC_reg_write_EX         <= PC_reg_write_DE;
        rs2_data_EX             <= rs2_data_DE;
        rd_idx_EX               <= rd_idx_DE;
        opcode_EX               <= opcode_DE;
        funct3_EX               <= funct3_DE;
        rs2_idx_EX              <= rs2_idx_DE;
        immediate_data_EX       <= immediate_data_DE;
        PC_EX                   <= PC_DE;
    end

endmodule


module mem_stage(   input           clk,
                    input [`WL:0]   alu_result_EX,
                    input [`WL:0]   rs2_data_EX,
                    input           data_mem_cs_EX, data_mem_wr_EX,
                    input           alu_write_reg_EX,
                    input           immediate_reg_write_EX,
                    input           PC_reg_write_EX,
                    input [  4:0]   rd_idx_EX,
                    input [  6:0]   opcode_EX,
                    input           forward_WB_MEM,
                    input [`WL:0]   forwarded_data_WB,
                    input [`WL:0]   immediate_data_EX,
                    input [  2:0]   funct3_EX,
                    input [`WL:0]   PC_EX,

                    output      [`WL:0] mem_result_MEM,
                    output reg  [`WL:0] alu_result_MEM,
                    output reg  [`WL:0] PC_MEM,
                    output reg  [`WL:0] immediate_data_MEM,
                    output              alu_write_reg_MEM,
                    output              immediate_reg_write_MEM,
                    output              PC_reg_write_MEM,
                    output              mem_reg_write_MEM,
                    output reg  [  4:0] rd_idx_MEM,
                    output reg  [  6:0] opcode_MEM,
                    output reg  [  2:0] funct3_MEM
                    );
    
    wire [`WL:0] mem_in = (forward_WB_MEM) ? forwarded_data_WB : rs2_data_EX;


    mmu mmu_0(  .clk(clk),
                .addr(alu_result_EX),
                .data_in( mem_in ),
                .wr_in(data_mem_wr_EX),
                .cs_in(data_mem_cs_EX),
                .size(funct3_EX[1:0]),
                
                .data_out(mem_result_MEM));

    always @(negedge clk)
    begin
        //mem_result_MEM    <= (data_mem_cs_EX & data_mem_wr_EX) ? mem_out : mem_result_MEM;
        alu_write_reg_MEM        <= alu_write_reg_EX;
        immediate_reg_write_MEM  <= immediate_reg_write_EX;
        PC_reg_write_MEM         <= PC_reg_write_EX;
        mem_reg_write_MEM        <= (data_mem_cs_EX & !data_mem_wr_EX);
        PC_MEM                   <= PC_EX;
        alu_result_MEM           <= alu_result_EX;
        rd_idx_MEM               <= rd_idx_EX;
        opcode_MEM               <= opcode_EX;
        immediate_data_MEM       <= immediate_data_EX;
        funct3_MEM               <= funct3_EX;
    end

endmodule

module wb_stage(input [`WL:0] alu_data_MEM,
                input [`WL:0] mem_data_MEM,
                input [`WL:0] PC_MEM,
                input [`WL:0] immediate_data_MEM,
                input [2:0] funct3_MEM,
                input alu_write_reg_MEM,
                input immediate_reg_write_MEM,
                input PC_reg_write_MEM,
                input mem_reg_write_MEM,
                
                output [`WL:0] rd_data
                );
    
    reg [`WL:0] rd_data_internal;

    wire [`WL:0] sign_extend_8 = { {24{mem_data_MEM[7]}} , mem_data_MEM[7:0] };
    wire [`WL:0] sign_extend_16 = { {16{mem_data_MEM[15]}} , mem_data_MEM[15:0] };

    wire [`WL:0] mem_data = (funct3_MEM == 3'b000) ? sign_extend_8 : ( (funct3_MEM == 3'b001) ? sign_extend_16 : mem_data_MEM );

    assign rd_data = rd_data_internal;

    always @* begin
        if      (alu_write_reg_MEM)         rd_data_internal = alu_data_MEM;
        else if (immediate_reg_write_MEM)   rd_data_internal = immediate_data_MEM;
        else if (PC_reg_write_MEM)          rd_data_internal = PC_MEM;
        else if (mem_reg_write_MEM)         rd_data_internal = mem_data;
        else                                rd_data_internal = alu_data_MEM; //dont care
    end

endmodule


module alu( input         clk,
            input [`WL:0] in1,
            input [`WL:0] in2,
            input [  4:0] op,

            output reg [`WL:0] out,
            output reg         take_branch
            );
    

    wire eq_int;
    wire [`WL:0] lt_int,ltu_int;
    wire [`WL:0] adder_out;
    wire [`WL:0] shifter_out;
    wire [ 63:0] mul_out;
    wire [`WL:0] quotient;
    wire [`WL:0] remainder;
    wire         div_result_rdy;

    assign eq_int = in1 == in2;
    assign lt_int = { 31'd0 , in1 < in2 };
    assign ltu_int = { 31'd0 , $signed(in1) < $signed(in2) };

    cla_adder_32 adder( .x(in1),
                        .y(in2),
                        .sub(op[4]),
                        .sum(adder_out),
                        .c_out()
                    );

    barrel_shifter_32 shift(.x(in1),
                            .shift_by(in2[4:0]), //may need to shift by values > 32
                            .left(~op[2]),
                            .arith(op[4]),
                            .out(shifter_out)
                            );

    mul32x32_pipelined mul0(.clk(clk),
                            .signed_mul_i( !(op[2:0] & 3'b011) ),
                            .X( in1 ),
                            .Y( in2 ),
                    
                            .Result(mul_out)
                            );

    int_div_32 div0(.clk_i(clk),
                    .load_i( op[3:2] & 2'b11 ),
                    .dividend_i( in1 ),
                    .divisor_i( in2 ),
                    .signed_i( {op[3:2],op[0]} & 3'b110 ),

                    .result_rdy( div_result_rdy ),
                    .quotient_o( quotient ),
                    .remainder_o( remainder )   
                    );


    always @*
    begin
        case( op[3:0] )
            4'b0000: out = adder_out;
            4'b0001: out = shifter_out;
            4'b0010: out = lt_int; 
            4'b0011: out = ltu_int; 
            4'b0100: out = in1 ^ in2;
            4'b0101: out = shifter_out;
            4'b0110: out = in1 | in2;
            4'b0111: out = in1 & in2;

            4'b1000: out = MUL;
            4'b1001: out = MULH;
            4'b1010: out = MULHSU;
            4'b1011: out = MULHU ;
            4'b1100: out = DIV;
            4'b1101: out = DIVU;
            4'b1110: out = REM;
            4'b1111: out = REMU;
            default: out = 0;
        endcase

        case( op[2:0] )
            3'b000: take_branch = eq_int;
            3'b001: take_branch = ~eq_int;
            3'b100: take_branch = lt_int[0];
            3'b101: take_branch = ~lt_int[0];
            3'b110: take_branch = ltu_int[0];
            3'b111: take_branch = ~ltu_int[0];
            default: take_branch = 0;
            
        endcase
    end

endmodule

//Assume addreses are 32-bit aligned
module mmu( input clk,
            input [`WL:0] addr,
            input [`WL:0] data_in,
            input wr_in,
            input cs_in,
            input [  1:0] size, //one hot 00 = 1 byte, 01 = 2 bytes, 10 = 4 bytes 

            output [`WL:0] data_out
            );

    wire wr[3:0];
    wire cs[3:0];

    assign wr[0] = wr_in;
    assign cs[0] = cs_in;

    assign wr[1] = wr_in & ( size[0] | size[1] );
    assign cs[1] = cs_in & ( size[0] | size[1] );

    assign wr[2] = wr_in & size[1];
    assign cs[2] = cs_in & size[1];
    assign wr[3] = wr_in & size[1];
    assign cs[3] = cs_in & size[1];

    genvar i;
    generate
        for(i=0; i<4; i=i+1) begin
            memory #(
                .MEM_WORD_SIZE( 8 ),
                .MEM_SIZE_BYTES( `DMEM_SIZE/4 ),
                .ADRESS_WIDTH( 21 )
                )
            bram0
                (   
                .clk( clk ),
                .wr( wr[i] ),
                .cs( cs[i] ),
                .addr( addr>>2 ),
                .data_in( data_in[8*(i+1)-1 : 8*i] ),

                .data_out( data_out[8*(i+1)-1 : 8*i] )
                );
        end
    endgenerate 

endmodule

module memory(  input clk,
                input wr,
                input cs,
                input [`WL:0] addr,
                input [MEM_WORD_SIZE-1:0] data_in,

                output reg [MEM_WORD_SIZE-1:0] data_out
                );

    parameter MEM_WORD_SIZE = 8; //in bits
    parameter MEM_SIZE_BYTES = 2*1024*1024; //in bytes
    parameter ADRESS_WIDTH = 21;
    parameter initialize = 0;
    parameter init_file = " ";

    reg [MEM_WORD_SIZE-1:0] RAM       [MEM_SIZE_BYTES-1:0];

    initial begin
        if(initialize) $readmemb(init_file, RAM);
    end

    
    always @(negedge clk)
    begin
        if(cs &  wr) RAM[addr[ADRESS_WIDTH-1:0]] <= data_in;
        if(cs & !wr) data_out <= RAM[addr[ADRESS_WIDTH-1:0]];
    end

endmodule

module registerFile(input clk,
                    input write,
                    input [4:0] rd_idx,
                    input [`WL:0] rd_data,
                    input [4:0] rs1_idx,
                    input [4:0] rs2_idx,

                    output reg [`WL:0] rs1_data,
                    output reg [`WL:0] rs2_data
                    );

    reg [`WL:0] registers [31:0];


    always @(posedge clk)
    begin
        registers[0] <= 0;
        if (write && rd_idx!=0'b0) registers[rd_idx] <= rd_data;
    end

    always @(negedge clk)
    begin
        rs1_data <= registers[rs1_idx];
        rs2_data <= registers[rs2_idx]; 
    end

endmodule

module control( input clk_i,
                input [31:0] fetched_instruction_IF_i,
                input [ 6:0] opcode_DE_i,
                input [ 6:0] opcode_EX_i,
                input [ 6:0] opcode_MEM_i,
                input [ 4:0] rd_idx_DE_i,
                input [ 4:0] rd_idx_EX_i,
                input [ 4:0] rd_idx_MEM_i,
                input [ 4:0] rs1_idx_DE_i,
                input [ 4:0] rs2_idx_DE_i,
                input [ 4:0] rs2_idx_EX_i,
                
                output forward_rs1_MEM_EX_o,
                output forward_rs2_MEM_EX_o,
                output forward_rs1_WB_EX_o,
                output forward_rs2_WB_EX_o,
                output forward_WB_MEM_o,
                output interlock_stall_o
                );


    forwarding_logic fwd_logic( .opcode_DE( opcode_DE_i ),
                                .opcode_EX( opcode_EX_i ),
                                .opcode_MEM( opcode_MEM_i ),
                                .rd_idx_EX( rd_idx_EX_i ),
                                .rd_idx_MEM( rd_idx_MEM_i ),
                                .rs1_idx_DE( rs1_idx_DE_i ),
                                .rs2_idx_DE( rs2_idx_DE_i ),
                                .rs2_idx_EX( rs2_idx_EX_i ),

                                .forward_rs1_MEM_EX( forward_rs1_MEM_EX_o ),
                                .forward_rs2_MEM_EX( forward_rs2_MEM_EX_o ),
                                .forward_rs1_WB_EX( forward_rs1_WB_EX_o ),
                                .forward_rs2_WB_EX( forward_rs2_WB_EX_o ),
                                .forward_WB_MEM( forward_WB_MEM_o )
                                );

    RAW_interlock_logic intl_logic( .opcode_DE( opcode_DE_i ),
                                .opcode_IF( fetched_instruction_IF_i[6:0] ),
                                .rs1_idx_IF( fetched_instruction_IF_i[19:15] ),
                                .rs2_idx_IF( fetched_instruction_IF_i[24:20] ),
                                .rd_idx_DE( rd_idx_DE_i ),

                                .stall_pipeline( interlock_stall_o )
                                );
  
endmodule

module forwarding_logic(input [6:0] opcode_DE,
                        input [6:0] opcode_EX,
                        input [6:0] opcode_MEM,
                        input [4:0] rd_idx_EX,
                        input [4:0] rd_idx_MEM,
                        input [4:0] rs1_idx_DE,
                        input [4:0] rs2_idx_DE,
                        input [4:0] rs2_idx_EX,

                        output forward_rs1_MEM_EX,
                        output forward_rs2_MEM_EX,
                        output forward_rs1_WB_EX,
                        output forward_rs2_WB_EX,
                        output forward_WB_MEM
                        );

    wire dst_op_cond_DE_rs1;
    wire dst_op_cond_DE_rs2;
    wire src_op_cond_EX;
    wire src_op_cond_MEM;
    wire dst_op_cond_EX;
    //                                  ALU_RR                      ALU_I                       LOAD                            STORE                   BRANCH
    assign src_op_cond_EX     = (opcode_EX == 7'b0110011)  | (opcode_EX == 7'b0010011);
    assign src_op_cond_MEM    = (opcode_MEM == 7'b0110011) | (opcode_MEM == 7'b0010011) | (opcode_MEM == 7'b0000011);
    assign dst_op_cond_DE_rs1 = (opcode_DE == 7'b0110011)  | (opcode_DE == 7'b0010011)  | (opcode_DE == 7'b0000011)  | (opcode_DE == 7'b0100011) | (opcode_DE == 7'b1100011);
    assign dst_op_cond_DE_rs2 = (opcode_DE == 7'b0110011);
    assign dst_op_cond_EX     =                                                                                        (opcode_EX == 7'b0100011);

    assign forward_rs1_MEM_EX = src_op_cond_EX  & dst_op_cond_DE_rs1 & (rd_idx_EX  == rs1_idx_DE);
    assign forward_rs2_MEM_EX = src_op_cond_EX  & dst_op_cond_DE_rs2 & (rd_idx_EX  == rs2_idx_DE);
    assign forward_rs1_WB_EX  = src_op_cond_MEM & dst_op_cond_DE_rs1 & (rd_idx_MEM == rs1_idx_DE);
    assign forward_rs2_WB_EX  = src_op_cond_MEM & dst_op_cond_DE_rs2 & (rd_idx_MEM == rs2_idx_DE);
    assign forward_WB_MEM     = dst_op_cond_EX  & (rs2_idx_EX == rd_idx_MEM); 

endmodule

module RAW_interlock_logic( input [6:0] opcode_DE,
                        input [6:0] opcode_IF,
                        input [4:0] rs1_idx_IF,
                        input [4:0] rs2_idx_IF,
                        input [4:0] rd_idx_DE,

                        output stall_pipeline
                        );

    wire opcode_cond_IF_rs1;
    wire opcode_cond_IF_rs2;
    wire opcode_cond_rs1;
    wire opcode_cond_rs2;
    wire rs1_idx_cond;
    wire rs2_idx_cond;

    //                                  ALU_RR                      ALU_I                       LOAD                            STORE                   BRANCH
    assign opcode_cond_IF_rs1 = (opcode_IF == 7'b0110011)  | (opcode_IF == 7'b0010011)  | (opcode_IF == 7'b0000011)  | (opcode_IF == 7'b0100011) | (opcode_IF == 7'b1100011); 
    assign opcode_cond_IF_rs2 = (opcode_IF == 7'b0110011)  |                                                                                       (opcode_IF == 7'b1100011);

    assign opcode_cond_rs1    = (opcode_DE == 7'b0000011) & opcode_cond_IF_rs1;
    assign opcode_cond_rs2    = (opcode_DE == 7'b0000011) & opcode_cond_IF_rs2;

    assign rs1_idx_cond       = rd_idx_DE == rs1_idx_IF;
    assign rs2_idx_cond       = rd_idx_DE == rs2_idx_IF;

    assign stall_pipeline     = ( opcode_cond_rs1 & rs1_idx_cond ) | ( opcode_cond_rs2  & rs2_idx_cond );

endmodule

