`timescale 1ns/1ns

`define WL 31 //word length


module riscv_custom( input clk, input reset);

    wire [`WL:0]   PC_IF;
    wire [ 31:0]   fetched_instruction_IF;
    //wire [`WL:0]   delayed_PC_IF; //there is a delay in reading from memory and writing to register, so this is a delayed pc so that pc matched with executing instr
    
    wire [ 4:0]    rs1_idx;
    wire [ 4:0]    rs2_idx;
    wire [`WL:0]   rd_data;
    wire [`WL:0]   rs1_data_DE;
    wire [ 4:0]    rd_idx_DE;
    wire [`WL:0]   rs2_data_DE;
    wire [ 4:0]    rs1_idx_DE;
    wire [ 4:0]    rs2_idx_DE;
    wire [`WL:0]   branch_PC_DE;
    wire           branch_instruction_DE;
    wire [`WL:0]   immediate_data_DE;
    wire           data_mem_cs_DE;
    wire           data_mem_wr_DE;
    wire           regFile_write_DE;
    wire [ 2:0]    funct3_DE;
    wire           funct7_bit5_DE;
    wire           immediate_instr_DE;
    wire [ 6:0]    opcode_DE;
    
    wire [`WL:0]   rs2_data_EX;
    wire [ 4:0]    rs2_idx_EX;
    wire [ 4:0]    rd_idx_EX;
    wire [`WL:0]   ALU_result_EX;
    wire           take_branch;
    wire           data_mem_cs_EX;
    wire           data_mem_wr_EX;
    wire           regFile_write_EX;
    wire [ 6:0]    opcode_EX;
    wire [ 2:0]    funct3_EX;
    wire [`WL:0]   immediate_data_EX;

    wire [ 4:0]    rd_idx_MEM;
    wire           regFile_write_MEM;
    wire [`WL:0]   ALU_result_MEM;
    wire [`WL:0]   mem_result_MEM;
    wire           data_mem_cs_MEM;
    wire           data_mem_wr_MEM;
    wire [ 6:0]    opcode_MEM;
    wire [ 2:0]    funct3_MEM;
    wire [`WL:0]   immediate_data_MEM;

    wire           forward_rs1_MEM_EX;
    wire           forward_rs2_MEM_EX;
    wire           forward_rs1_WB_EX;
    wire           forward_rs2_WB_EX;
    wire           forward_WB_MEM;

    wire           stall_pipeline;


    registerFile regFile(   .clk( clk ),
                            .write( regFile_write_MEM ),
                            .rd_idx( rd_idx_MEM ),
                            .rd_data( rd_data ),
                            .rs1_idx( rs1_idx ),
                            .rs2_idx( rs2_idx ),

                            .rs1_data( rs1_data_DE ),
                            .rs2_data( rs2_data_DE )
                        );

    instructionFetch IF_stage0( .clk_i( clk ),
                                .take_branch_i( take_branch ),
                                .branch_PC_i( branch_PC_DE ),
                                .stall_pipeline_i( stall_pipeline ),
                                .reset_i( reset ),

                                .PC_IF_o( PC_IF ),
                                .fetched_instruction_IF_o( fetched_instruction_IF )
                                );
    
    decode DE_stage0(   .clk( clk ),
                        .fetched_instruction_IF( fetched_instruction_IF ),
                        .delayed_PC_IF( PC_IF ),
                        .stall_pipeline( stall_pipeline ),
                        .take_branch(take_branch),

                        .rs1_idx( rs1_idx ),
                        .rs2_idx( rs2_idx ),
                        .rd_idx_DE( rd_idx_DE ),
                        .immediate_data_DE( immediate_data_DE ),
                        .branch_instruction_DE( branch_instruction_DE ),
                        .data_mem_cs_DE( data_mem_cs_DE ),
                        .data_mem_wr_DE( data_mem_wr_DE ),
                        .write_reg_DE( regFile_write_DE ),
                        .immediate_instr_DE( immediate_instr_DE ),
                        .funct3_DE( funct3_DE ),
                        .funct7_bit5_DE ( funct7_bit5_DE ),
                        .branch_PC_DE( branch_PC_DE ),
                        .opcode_DE( opcode_DE ),
                        .rs1_idx_DE( rs1_idx_DE ),
                        .rs2_idx_DE( rs2_idx_DE )
                        );
    

    exec_stage exec_stage0( .clk( clk ),
                            .rs1_data_DE( rs1_data_DE ),
                            .rs2_data_DE( rs2_data_DE ),
                            .immediate_data_DE( immediate_data_DE ),
                            .branch_instruction_DE( branch_instruction_DE ),
                            .immediate_instr_DE( immediate_instr_DE ),
                            .rd_idx_DE( rd_idx_DE ),
                            .funct3_DE( funct3_DE ),
                            .funct7_bit5_DE( funct7_bit5_DE ),
                            .data_mem_cs_DE( data_mem_cs_DE ),
                            .data_mem_wr_DE( data_mem_wr_DE ),
                            .write_reg_DE( regFile_write_DE ),
                            .opcode_DE( opcode_DE ),
                            .forwarded_data_MEM( ALU_result_EX ),
                            .forwarded_data_WB( rd_data ),
                            .forward_rs1_MEM_EX( forward_rs1_MEM_EX ),
                            .forward_rs2_MEM_EX( forward_rs2_MEM_EX ),
                            .forward_rs1_WB_EX( forward_rs1_WB_EX ),
                            .forward_rs2_WB_EX( forward_rs2_WB_EX ), 
                            .rs2_idx_DE( rs2_idx_DE ),

                            .alu_result_EX( ALU_result_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .take_branch( take_branch ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .write_reg_EX( regFile_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),
                            .opcode_EX( opcode_EX ),
                            .funct3_EX( funct3_EX ),
                            .rs2_idx_EX( rs2_idx_EX ),
                            .immediate_data_EX( immediate_data_EX )
                            );

    mem_stage mem_stage0(   .clk( clk ),
                            .alu_result_EX( ALU_result_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .write_reg_EX( regFile_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),
                            .opcode_EX( opcode_EX ),
                            .funct3_EX( funct3_EX ),
                            .forward_WB_MEM( forward_WB_MEM ),
                            .forwarded_data_WB( rd_data ),
                            .immediate_data_EX( immediate_data_EX ),

                            .mem_result_MEM( mem_result_MEM ),
                            .alu_result_MEM( ALU_result_MEM ),
                            .write_reg_MEM( regFile_write_MEM ),
                            .rd_idx_MEM( rd_idx_MEM ),
                            .data_mem_cs_MEM ( data_mem_cs_MEM ),
                            .data_mem_wr_MEM ( data_mem_wr_MEM ),
                            .opcode_MEM( opcode_MEM ),
                            .funct3_MEM( funct3_MEM ),
                            .immediate_data_MEM( immediate_data_MEM )
                        );

    wb_stage wb_stage0( .alu_data_MEM( ALU_result_MEM ),
                        .mem_data_MEM( mem_result_MEM ),
                        .data_mem_cs_MEM( data_mem_cs_MEM ),
                        .data_mem_wr_MEM( data_mem_wr_MEM ),
                        .funct3_MEM( funct3_MEM ),
                        .immediate_data_MEM( immediate_data_MEM ),
                        .opcode_MEM( opcode_MEM ),

                        .rd_data( rd_data )
                    );

    forwarding_logic fwd_logic( .opcode_DE( opcode_DE ),
                                .opcode_EX( opcode_EX ),
                                .opcode_MEM( opcode_MEM ),
                                .rd_idx_EX( rd_idx_EX ),
                                .rd_idx_MEM( rd_idx_MEM ),
                                .rs1_idx_DE( rs1_idx_DE ),
                                .rs2_idx_DE( rs2_idx_DE ),
                                .rs2_idx_EX( rs2_idx_EX ),

                                .forward_rs1_MEM_EX( forward_rs1_MEM_EX ),
                                .forward_rs2_MEM_EX( forward_rs2_MEM_EX ),
                                .forward_rs1_WB_EX( forward_rs1_WB_EX ),
                                .forward_rs2_WB_EX( forward_rs2_WB_EX ),
                                .forward_WB_MEM( forward_WB_MEM )
                                );

    interlock_logic intl_logic( .opcode_DE( opcode_DE ),
                                .opcode_IF( fetched_instruction_IF[6:0] ),
                                .rs1_idx_IF( fetched_instruction_IF[19:15] ),
                                .rs2_idx_IF( fetched_instruction_IF[24:20] ),
                                .rd_idx_DE( rd_idx_DE ),

                                .stall_pipeline( stall_pipeline )
                                );



endmodule




module instructionFetch(input clk_i,
                        input take_branch_i,
                        input [`WL:0] branch_PC_i,
                        input stall_pipeline_i,
                        input reset_i,

                        output [`WL:0] PC_IF_o,
                        output [ 31:0] fetched_instruction_IF_o
                        );
    
    reg [`WL:0] PC_q;
    reg [ 31:0] instruction_buffer_q [1:0];
    reg [ 31:0] PC_buffer_q [1:0];
    reg after_stall_q;
    
    wire [ 31:0] mem_instruction;


    memory #(
            .MEM_WORD_SIZE( 32 ),
            .MEM_SIZE_BYTES( 8*1024*1024 ),
            .ADRESS_WIDTH( 23 ),
            .initialize( 1 ),
            .init_file("ass_bin.dat")
            )
        instr_mem
            (   
            .clk( clk_i ),
            .wr( 1'b0 ),
            .cs( 1'b1 & reset_i ),
            .addr( (take_branch_i) ? branch_PC_i >> 2 : PC_q >> 2 ),
            .data_in(),

            .data_out( mem_instruction )
            );

    assign fetched_instruction_IF_o = instruction_buffer_q[0];
    assign PC_IF_o = PC_buffer_q[0];

    always @(negedge clk_i)
    begin
        if(!reset_i)  begin   
            PC_q <= 0;
            after_stall_q <= 0;
            instruction_buffer_q[0] <= 0;
            instruction_buffer_q[1] <= 0;
        end
        else if (take_branch_i)  begin
            PC_q <= branch_PC_i + 4;
            after_stall_q <= 0;
            instruction_buffer_q[0] <= mem_instruction;
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0] <= PC_q;
            PC_buffer_q[1] <= PC_q;
        end
        else if (stall_pipeline_i)  begin
            PC_q <= PC_q;
            after_stall_q <= 1;
            instruction_buffer_q[0] <= instruction_buffer_q[1];
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0] <= PC_buffer_q[1];
            PC_buffer_q[1] <= PC_q;
        end
        else if (after_stall_q) begin
            PC_q <= PC_q + 4;
            after_stall_q <= 0;
            instruction_buffer_q[0] <= instruction_buffer_q[1];
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0] <= PC_buffer_q[1];
            PC_buffer_q[1] <= PC_q;
        end
        else begin
            PC_q <= PC_q + 4;
            after_stall_q <= 0;
            instruction_buffer_q[0] <= mem_instruction;
            instruction_buffer_q[1] <= mem_instruction;
            PC_buffer_q[0] <= PC_q;
            PC_buffer_q[1] <= PC_q;
        end 
    end

endmodule

//R-type: 0110011(ALU RR), 0011011(Shifts-64), 0111011(ALU RR-64)
//I-type: 0010011(ALU I), 0000011(LOADS), 1100111(JALR), 0011011(ALU I-64), 
//S-type: 0100011(Store)
//B-type: 1100011(Branch) 
//U-type: 0110111(LUI), 0010111(AUIPC)
//J-type: 1101111(JAL)
module decode(  input clk,
                input [31:0] fetched_instruction_IF,
                input [`WL:0] delayed_PC_IF,
                input stall_pipeline,
                input take_branch,

                output [4:0] rs1_idx, rs2_idx,
                output reg [4:0] rd_idx_DE,
                output reg [`WL:0] immediate_data_DE,
                output reg branch_instruction_DE,
                output reg data_mem_cs_DE, data_mem_wr_DE,
                output reg write_reg_DE,
                output reg immediate_instr_DE,
                output reg [2:0] funct3_DE,
                output reg funct7_bit5_DE,
                output reg [`WL:0] branch_PC_DE,
                output reg [6:0] opcode_DE,
                output reg [4:0] rs1_idx_DE, rs2_idx_DE
                );

    wire [6:0] opcode = (stall_pipeline | (branch_taken_stall_timer_q != 1'b0) | take_branch) ? 7'b0000000 : fetched_instruction_IF[6:0];
    wire [6:0] funct7 = fetched_instruction_IF[31:25];
    wire [2:0] funct3 = fetched_instruction_IF[14:12];
    wire [`WL:0] add_to_PC_data;
    wire [`WL:0] PC_add_result;

    reg branch_taken_stall_timer_q;


    assign rs1_idx = fetched_instruction_IF[19:15];
    assign rs2_idx = fetched_instruction_IF[24:20];

                                                                                                    //Sign Extend                                                                                                     x2
    assign add_to_PC_data = (opcode == 7'b0010111) ? {fetched_instruction_IF[31:12], 12'd0} : {{20{fetched_instruction_IF[31]}},fetched_instruction_IF[7],fetched_instruction_IF[30:25],fetched_instruction_IF[11:8],1'b0};
    assign PC_add_result = delayed_PC_IF + add_to_PC_data;

    always @(negedge clk)
    begin
        funct3_DE <= funct3;
        funct7_bit5_DE <= funct7[5];
        rd_idx_DE <= fetched_instruction_IF[11:7]; 
        opcode_DE <= opcode;
        rs1_idx_DE <= rs1_idx;
        rs2_idx_DE <= rs2_idx;
        if (take_branch) branch_taken_stall_timer_q <= 1'b1;
        else branch_taken_stall_timer_q <= branch_taken_stall_timer_q >> 1;
                                        //Sign Extend                                                                                                   x2
        //branch_PC_DE <= delayed_PC_IF + {{20{fetched_instruction_IF[31]}},fetched_instruction_IF[7],fetched_instruction_IF[30:25],fetched_instruction_IF[11:8],1'b0}; 
        branch_PC_DE <= PC_add_result;

        case(opcode)
            7'b0000000: //pipeline stall
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b0;
                immediate_instr_DE      <= 1'b0;
            end
            7'b1100011: //Branch
            begin
                branch_instruction_DE   <= 1'b1;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b0;
                immediate_instr_DE      <= 1'b0;

                immediate_data_DE       <= {{20{fetched_instruction_IF[31]}},fetched_instruction_IF[7],fetched_instruction_IF[30:25],fetched_instruction_IF[11:8],1'b0};
            end
            7'b0010011: //ALU Immediate
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b1;
                immediate_instr_DE      <= 1'b1;

                                            //SLLI,SRLI,SRAI
                immediate_data_DE       <= (funct3 == 3'b001 & funct3 == 3'b101) ? {27'b0 , rs2_idx} : { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:20] };
            end
            7'b0000011: //LOADS
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b1;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b1;
                immediate_instr_DE      <= 1'b1;

                immediate_data_DE       <= { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:20]}; //sign extend
            end
            //7'b1100111: //JALR
            7'b0100011: //STORE
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b1;
                data_mem_wr_DE          <= 1'b1;
                write_reg_DE            <= 1'b0;
                immediate_instr_DE      <= 1'b1;

                immediate_data_DE       <= { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:25] , fetched_instruction_IF[11:7] };
            end
            7'b0110011: //ALU RR
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b1;
                immediate_instr_DE      <= 1'b0;
            end
            //7'b1101111: //JAL
            7'b0110111: //LUI
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b1;
                immediate_instr_DE      <= 1'b0;

                immediate_data_DE       <= {fetched_instruction_IF[31:12], 12'd0};
            end
            7'b0010111: //AUIPC
            begin
                branch_instruction_DE   <= 1'b0;
                data_mem_cs_DE          <= 1'b0;
                data_mem_wr_DE          <= 1'b0;
                write_reg_DE            <= 1'b1;
                immediate_instr_DE      <= 1'b0;

                immediate_data_DE       <= PC_add_result;
            end
            default:
            begin
                branch_instruction_DE   <= 0;
                immediate_data_DE       <= 0;
                data_mem_cs_DE          <= 0;
                data_mem_wr_DE          <= 0;
                write_reg_DE            <= 0;
                immediate_instr_DE      <= 0;
            end
        endcase

    end
endmodule


module exec_stage(  input clk,
                    input [`WL:0] rs1_data_DE, rs2_data_DE, immediate_data_DE,
                    input branch_instruction_DE, immediate_instr_DE,
                    input [4:0] rd_idx_DE, rs2_idx_DE,
                    input [2:0] funct3_DE,
                    input funct7_bit5_DE,
                    input data_mem_cs_DE, data_mem_wr_DE, write_reg_DE,
                    input [6:0] opcode_DE,
                    input [`WL:0] forwarded_data_MEM, forwarded_data_WB,
                    input forward_rs1_MEM_EX, forward_rs2_MEM_EX, forward_rs1_WB_EX, forward_rs2_WB_EX,

                    output reg [`WL:0] alu_result_EX,
                    output reg [`WL:0] rs2_data_EX,
                    output take_branch,
                    output reg data_mem_cs_EX, data_mem_wr_EX, write_reg_EX,
                    output reg [4:0] rd_idx_EX, rs2_idx_EX,
                    output reg [6:0] opcode_EX,
                    output reg [2:0] funct3_EX,
                    output reg [`WL:0] immediate_data_EX
                    );

    wire [`WL:0] alu_input2;
    wire [`WL:0] alu_input1;
    wire [`WL:0] forwarded_rs1_data, forwarded_rs2_data;

    assign forwarded_rs1_data = (forward_rs1_MEM_EX) ? forwarded_data_MEM : forwarded_data_WB;
    assign forwarded_rs2_data = (forward_rs2_MEM_EX) ? forwarded_data_MEM : forwarded_data_WB;

    assign alu_input1 = (forward_rs1_MEM_EX | forward_rs1_WB_EX) ? forwarded_rs1_data : rs1_data_DE;
    assign alu_input2 = (forward_rs2_MEM_EX | forward_rs2_WB_EX) ? forwarded_rs2_data : ( (branch_instruction_DE | immediate_instr_DE) ? immediate_data_DE : rs2_data_DE );

    alu alu_unit(   .clk(clk),
                    .in1(alu_input1),
                    .in2(alu_input2),
                    .op(funct3_DE),
                    .branch(branch_instruction_DE),
                    .funct7_bit( (immediate_instr_DE & funct3_DE==3'b0) ? 1'b0 : funct7_bit5_DE), //funct7 bit5 shouldnt be raised when instr is immediate and funct3 is 0 (could perform sub instead of addi)
                    .out(alu_result_EX),
                    .take_branch(take_branch));

    always @(negedge clk)
    begin
        data_mem_cs_EX          <= data_mem_cs_DE;
        data_mem_wr_EX          <= data_mem_wr_DE;
        write_reg_EX            <= write_reg_DE;
        rs2_data_EX             <= rs2_data_DE;
        rd_idx_EX               <= rd_idx_DE;
        opcode_EX               <= opcode_DE;
        funct3_EX               <= funct3_DE;
        rs2_idx_EX              <= rs2_idx_DE;
        immediate_data_EX       <= immediate_data_DE;
    end

endmodule

module mem_stage(   input clk,
                    input [`WL:0] alu_result_EX,
                    input [`WL:0] rs2_data_EX,
                    input data_mem_cs_EX, data_mem_wr_EX,
                    input write_reg_EX,
                    input [4:0] rd_idx_EX,
                    input [6:0] opcode_EX,
                    input [2:0] funct3_EX,
                    input forward_WB_MEM,
                    input [`WL:0] forwarded_data_WB,
                    input [`WL:0] immediate_data_EX,

                    output [`WL:0] mem_result_MEM,
                    output reg [`WL:0] alu_result_MEM,
                    output write_reg_MEM,
                    output reg [4:0] rd_idx_MEM,
                    output reg data_mem_cs_MEM, data_mem_wr_MEM,
                    output reg [6:0] opcode_MEM,
                    output reg [2:0] funct3_MEM,
                    output reg [`WL:0] immediate_data_MEM
                    );

    //wire [`WL:0] mem_out;
    wire [  1:0] load_store_size = (funct3_EX[1] == 1'b1) ? 2'b11 : ( (funct3_EX[0] == 1'b1) ? 2'b01 : 2'b00 );
    wire [`WL:0] mem_in = (forward_WB_MEM) ? forwarded_data_WB : rs2_data_EX;


    mmu mmu_0(  .clk(clk),
                .addr(alu_result_EX),
                .data_in( mem_in ),
                .wr_in(data_mem_wr_EX),
                .cs_in(data_mem_cs_EX),
                .size(load_store_size),
                
                .data_out(mem_result_MEM));

    always @(negedge clk)
    begin
        //mem_result_MEM    <= (data_mem_cs_EX & data_mem_wr_EX) ? mem_out : mem_result_MEM;
        write_reg_MEM            <= write_reg_EX;
        alu_result_MEM           <= alu_result_EX;
        rd_idx_MEM               <= rd_idx_EX;
        data_mem_cs_MEM          <= data_mem_cs_EX;
        data_mem_wr_MEM          <= data_mem_wr_EX;
        opcode_MEM               <= opcode_EX;
        funct3_MEM               <= funct3_EX;
        immediate_data_MEM       <= immediate_data_EX;
    end

endmodule

module wb_stage(input [`WL:0] alu_data_MEM,
                input [`WL:0] mem_data_MEM,
                input data_mem_cs_MEM, data_mem_wr_MEM,
                input [2:0] funct3_MEM,
                input [`WL:0] immediate_data_MEM,
                input [6:0] opcode_MEM,

                output [`WL:0] rd_data
                );

    wire [`WL:0] sign_extend_8 = { {24{mem_data_MEM[7]}} , mem_data_MEM[7:0] };
    wire [`WL:0] sign_extend_16 = { {16{mem_data_MEM[15]}} , mem_data_MEM[15:0] };

    wire [`WL:0] mem_data = (funct3_MEM == 3'b000) ? sign_extend_8 : ( (funct3_MEM == 3'b001) ? sign_extend_16 : mem_data_MEM );

                                //LUI                   AUIPC
    assign rd_data = (opcode_MEM == 7'b0110111 | opcode_MEM == 7'b0010111) ? immediate_data_MEM : ( (data_mem_cs_MEM & !data_mem_wr_MEM) ? mem_data : alu_data_MEM );

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

module interlock_logic( input [6:0] opcode_DE,
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


module alu( input clk,
            input [`WL:0] in1,
            input [`WL:0] in2,
            input [2:0] op,
            input branch,
            input funct7_bit,
            output reg [`WL:0] out,
            output reg take_branch
            );
    

    wire eq_int;
    wire [`WL:0] lt_int,ltu_int;
    wire [`WL:0] adder_out;
    wire [`WL:0] shifter_out;


    assign eq_int = in1 == in2;
    assign lt_int = { 31'd0 , in1 < in2 };
    assign ltu_int = { 31'd0 , $signed(in1) < $signed(in2) };

    cla_adder_32 adder(   .x(in1),
                        .y(in2),
                        .sub(funct7_bit),
                        .sum(adder_out),
                        .c_out()
                    );

    barrel_shifter_32 shift(.x(in1),
                            .shift_by(in2[4:0]), //may need to shift by values > 32
                            .left(~op[2]),
                            .arith(funct7_bit),
                            .out(shifter_out)
                            );


    always @(negedge clk)
    begin
        case( op )
            3'b000: out <= adder_out;
            3'b001: out <= shifter_out;
            3'b010: out <= lt_int; 
            3'b011: out <= ltu_int; 
            3'b100: out <= in1 ^ in2;
            3'b101: out <= shifter_out;
            3'b110: out <= in1 | in2;
            3'b111: out <= in1 & in2;
            default: out <= 0;
        endcase
    end

    always @* 
    begin
        case( {branch,op} )
            4'b1000: take_branch = eq_int;
            4'b1001: take_branch = ~eq_int;
            4'b1100: take_branch = lt_int[0];
            4'b1101: take_branch = ~lt_int[0];
            4'b1110: take_branch = ltu_int[0];
            4'b1111: take_branch = ~ltu_int[0];
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
            input [  1:0] size, //one hot inclusive 00 = 1 byte, 01 = 2 bytes, 11 = 4 bytes 

            output [`WL:0] data_out
            );

    wire wr[3:0];
    wire cs[3:0];

    assign wr[0] = wr_in;
    assign cs[0] = cs_in;
    assign wr[1] = wr_in & size[0];
    assign cs[1] = cs_in & size[0];
    assign wr[2] = wr_in & size[1];
    assign cs[2] = cs_in & size[1];
    assign wr[3] = wr[2];
    assign cs[3] = cs[2];

    genvar i;
    generate
        for(i=0; i<4; i=i+1) begin
            memory #(
                .MEM_WORD_SIZE( 8 ),
                .MEM_SIZE_BYTES( 2*1024*1024 ),
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

module shift_register(  input clk,
                        input [WIDTH-1:0] in,
                        input shift,
                        output [WIDTH-1:0] out);

    parameter WIDTH = 32;
    parameter DEPTH = 2;

    reg [WIDTH-1:0] registers [DEPTH-1:0];

    integer i;

    assign out = registers[DEPTH-1];
    assign registers[0] = in;

    always @(negedge clk)
    begin
        for(i=0; i<DEPTH; i=i+1)begin
            if(shift) registers[i+1] <= registers[i];
        end
    end
endmodule





