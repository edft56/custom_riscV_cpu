`timescale 1ns/1ns

`define WL 31 //word length


module riscv_core( input clk);

    wire [`WL:0]   PC_IF;
    wire [ 31:0]   fetched_instruction_IF;
    
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
    wire [ 4:0]    rd_idx_EX;
    wire [`WL:0]   ALU_result_EX;
    wire           take_branch_EX;
    wire           data_mem_cs_EX;
    wire           data_mem_wr_EX;
    wire           regFile_write_EX;
    wire [ 6:0]    opcode_EX;

    wire [ 4:0]    rd_idx_MEM;
    wire           regFile_write_MEM;
    wire [`WL:0]   ALU_result_MEM;
    wire [`WL:0]   mem_result_MEM;
    wire           data_mem_cs_MEM;
    wire           data_mem_wr_MEM;
    wire [ 6:0]    opcode_MEM;

    wire           forward_rs1_EX_DE;
    wire           forward_rs2_EX_DE;
    wire           forward_rs1_MEM_DE;
    wire           forward_rs2_MEM_DE;

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



    instructionFetch IF_stage0( .clk( clk ),
                                .take_branch( take_branch_EX ),
                                .PC( PC_IF ),
                                .branch_pc( branch_PC_DE ),
                                .stall_pipeline( stall_pipeline ),
                                .prev_instruction( fetched_instruction_IF ),

                                .PC_IF( PC_IF ),
                                .fetched_instruction_IF( fetched_instruction_IF )
                                );
    
    decode DE_stage0(   .clk( clk ),
                        .fetched_instruction_IF( fetched_instruction_IF ),
                        .PC_IF( PC_IF ),
                        .stall_pipeline( stall_pipeline ),

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
                            .op( funct3_DE ),
                            .funct7_bit5_DE( funct7_bit5_DE ),
                            .data_mem_cs_DE( data_mem_cs_DE ),
                            .data_mem_wr_DE( data_mem_wr_DE ),
                            .write_reg_DE( regFile_write_DE ),
                            .opcode_DE( opcode_DE ),
                            .forwarded_data_EX( ALU_result_EX ),
                            .forwarded_data_MEM( rd_data ),
                            .forward_rs1_EX_DE( forward_rs1_EX_DE ),
                            .forward_rs2_EX_DE( forward_rs2_EX_DE ),
                            .forward_rs1_MEM_DE( forward_rs1_MEM_DE ),
                            .forward_rs2_MEM_DE( forward_rs2_MEM_DE ), 

                            .alu_result_EX( ALU_result_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .take_branch_EX( take_branch_EX ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .write_reg_EX( regFile_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),
                            .opcode_EX( opcode_EX )
                            );

    mem_stage mem_stage0(   .clk( clk ),
                            .alu_result_EX( ALU_result_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .write_reg_EX( regFile_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),
                            .opcode_EX( opcode_EX ),

                            .mem_result_MEM( mem_result_MEM ),
                            .alu_result_MEM( ALU_result_MEM ),
                            .write_reg_MEM( regFile_write_MEM ),
                            .rd_idx_MEM( rd_idx_MEM ),
                            .data_mem_cs_MEM ( data_mem_cs_MEM ),
                            .data_mem_wr_MEM ( data_mem_wr_MEM ),
                            .opcode_MEM( opcode_MEM )
                        );

    wb_stage wb_stage0( .alu_data_MEM( ALU_result_MEM ),
                        .mem_data_MEM( mem_result_MEM ),
                        .data_mem_cs_MEM( data_mem_cs_MEM ),
                        .data_mem_wr_MEM( data_mem_wr_MEM ),

                        .rd_data( rd_data )
                    );

    forwarding_logic fwd_logic( .opcode_DE( opcode_DE ),
                                .opcode_EX( opcode_EX ),
                                .opcode_MEM( opcode_MEM ),
                                .rd_idx_EX( rd_idx_EX ),
                                .rd_idx_MEM( rd_idx_MEM ),
                                .rs1_idx_DE( rs1_idx_DE ),
                                .rs2_idx_DE( rs2_idx_DE ),

                                .forward_rs1_EX_DE( forward_rs1_EX_DE ),
                                .forward_rs2_EX_DE( forward_rs2_EX_DE ),
                                .forward_rs1_MEM_DE( forward_rs1_MEM_DE ),
                                .forward_rs2_MEM_DE( forward_rs2_MEM_DE ) 
                                );

    interlock_logic intl_logic( .opcode_DE( opcode_DE ),
                                .opcode_IF( fetched_instruction_IF[6:0] ),
                                .rs1_idx_DE( rs1_idx_DE ),
                                .rs2_idx_DE( rs2_idx_DE ),
                                .rd_idx_IF( fetched_instruction_IF[11:7] ),

                                .stall_pipeline( stall_pipeline )
                                );



endmodule


module instructionFetch(input clk,
                        input take_branch,
                        input [`WL:0] PC,
                        input [`WL:0] branch_pc,
                        input stall_pipeline,
                        input [`WL:0] prev_instruction,

                        output reg [`WL:0] PC_IF,
                        output reg [31:0] fetched_instruction_IF
                        );
    

    wire [31:0] mem_bus;
    wire [`WL:0] NPC ;

    memory instr_mem(   .clk(clk),
                        .wr(1'b0),
                        .cs(1'b1),
                        .addr(PC),
                        .mem_bus(mem_bus));

    assign NPC = (take_branch) ? branch_pc : PC + 4;

    always @(negedge clk)
    begin
        if(stall_pipeline) begin
            fetched_instruction_IF <= prev_instruction;
            PC_IF <= PC;
        end
        else begin
            fetched_instruction_IF <= mem_bus;
            PC_IF <= NPC;
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
                input [`WL:0] PC_IF,
                input stall_pipeline,

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

    wire [6:0] opcode = (stall_pipeline) ? 7'b0000000 : fetched_instruction_IF[6:0];
    wire [6:0] funct7 = fetched_instruction_IF[31:25];


    assign rs1_idx = fetched_instruction_IF[19:15];
    assign rs2_idx = fetched_instruction_IF[24:20];


    always @(negedge clk)
    begin
        funct3_DE <= fetched_instruction_IF[14:12];
        funct7_bit5_DE <= funct7[5];
        rd_idx_DE <= fetched_instruction_IF[11:7]; 
        opcode_DE <= opcode;
        rs1_idx_DE <= rs1_idx;
        rs2_idx_DE <= rs2_idx;

        branch_PC_DE <= PC_IF + {19'd0,fetched_instruction_IF[31],fetched_instruction_IF[7],fetched_instruction_IF[30:25],fetched_instruction_IF[11:8],1'b0};

        case(opcode)
            7'b0000000: //pipeline stall
            begin
                branch_instruction_DE <= 1'b0;
                data_mem_cs_DE <= 1'b0;
                data_mem_wr_DE <= 1'b0;
                write_reg_DE <= 1'b0;
            end
            7'b1100011: //Branch
            begin
                branch_instruction_DE <= 1'b1;
                immediate_data_DE <= {19'd0,fetched_instruction_IF[31],fetched_instruction_IF[7],fetched_instruction_IF[30:25],fetched_instruction_IF[11:8],1'b0};
            end
            7'b0010011: //ALU Immediate
            begin
                immediate_instr_DE <= 1'b1;
                immediate_data_DE <= (fetched_instruction_IF[14:12] == 3'b001 & fetched_instruction_IF[14:12] == 3'b101) ? { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:20] } : { {27{rs2_idx[4]}} , rs2_idx };
                write_reg_DE <= 1'b1;
            end
            7'b0000011: //LOADS
            begin
                data_mem_cs_DE <= 1'b1;
                data_mem_wr_DE <= 1'b0;
                immediate_data_DE <= { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:20]};
                write_reg_DE <= 1'b1;
            end
            //7'b1100111: //JALR
            7'b0100011: //STORE
            begin
                data_mem_cs_DE <= 1'b1;
                data_mem_wr_DE <= 1'b1;
                immediate_data_DE <= { {20{fetched_instruction_IF[31]}} , fetched_instruction_IF[31:25] , fetched_instruction_IF[11:7]  };
            end
            7'b0110011: //ALU RR
            begin
                write_reg_DE <= 1'b1;
            end
            //7'b1101111: //JAL
            //7'b0110111: //LUI
            //7'b0010111: //AUIPC
            default:
            begin
                branch_instruction_DE <= branch_instruction_DE;
                immediate_data_DE <= immediate_data_DE;
                data_mem_cs_DE <= data_mem_cs_DE;
                data_mem_wr_DE <= data_mem_wr_DE;
                write_reg_DE <= write_reg_DE;
                immediate_instr_DE <= immediate_instr_DE;
            end
        endcase

    end
endmodule


module exec_stage(  input clk,
                    input [`WL:0] rs1_data_DE, rs2_data_DE, immediate_data_DE,
                    input branch_instruction_DE, immediate_instr_DE,
                    input [4:0] rd_idx_DE,
                    input [2:0] op,
                    input funct7_bit5_DE,
                    input data_mem_cs_DE, data_mem_wr_DE, write_reg_DE,
                    input [6:0] opcode_DE,
                    input [`WL:0] forwarded_data_EX, forwarded_data_MEM,
                    input forward_rs1_EX_DE, forward_rs2_EX_DE, forward_rs1_MEM_DE, forward_rs2_MEM_DE, 

                    output reg [`WL:0] alu_result_EX,
                    output reg [`WL:0] rs2_data_EX,
                    output reg take_branch_EX,
                    output reg data_mem_cs_EX, data_mem_wr_EX, write_reg_EX,
                    output reg [4:0] rd_idx_EX,
                    output reg [6:0] opcode_EX
                    );

    wire [`WL:0] alu_input2;
    wire [`WL:0] alu_input1;
    wire [`WL:0] forwarded_rs1_data, forwarded_rs2_data;

    assign forwarded_rs1_data = (forward_rs1_EX_DE) ? forwarded_data_EX : forwarded_data_MEM;
    assign forwarded_rs2_data = (forward_rs2_EX_DE) ? forwarded_data_EX : forwarded_data_MEM;

    assign alu_input1 = (forward_rs1_EX_DE | forward_rs1_MEM_DE) ? forwarded_rs1_data : rs1_data_DE;
    assign alu_input2 = (forward_rs2_EX_DE | forward_rs2_MEM_DE) ? forwarded_rs2_data : ( (branch_instruction_DE | immediate_instr_DE) ? immediate_data_DE : rs2_data_DE );

    alu alu_unit(   .clk(clk),
                    .in1(alu_input1),
                    .in2(alu_input2),
                    .op(op),
                    .branch(branch_instruction_DE),
                    .funct7_bit(funct7_bit5_DE),
                    .out(alu_result_EX),
                    .take_branch(take_branch_EX));

    always @(negedge clk)
    begin
        data_mem_cs_EX <= data_mem_cs_DE;
        data_mem_wr_EX <= data_mem_wr_DE;
        write_reg_EX <= write_reg_DE;
        rs2_data_EX <= rs2_data_DE;
        rd_idx_EX <= rd_idx_DE;
        opcode_EX <= opcode_DE;
    end

endmodule

module mem_stage(   input clk,
                    input [`WL:0] alu_result_EX,
                    input [`WL:0] rs2_data_EX,
                    input data_mem_cs_EX, data_mem_wr_EX,
                    input write_reg_EX,
                    input [4:0] rd_idx_EX,
                    input [6:0] opcode_EX,

                    output reg [`WL:0] mem_result_MEM,
                    output reg [`WL:0] alu_result_MEM,
                    output write_reg_MEM,
                    output reg [4:0] rd_idx_MEM,
                    output reg data_mem_cs_MEM, data_mem_wr_MEM,
                    output reg [6:0] opcode_MEM
                    );

    wire [`WL:0] mem_bus;

    assign mem_bus = (data_mem_cs_EX & ~data_mem_wr_EX) ? rs2_data_EX : 32'bZ;

    memory mem_0(   .clk(clk),
                    .wr(data_mem_wr_EX),
                    .cs(data_mem_cs_EX),
                    .addr(alu_result_EX),
                    .mem_bus(mem_bus));

    always @(negedge clk)
    begin
        mem_result_MEM    <= (data_mem_cs_EX & data_mem_wr_EX) ? mem_bus : mem_result_MEM;
        write_reg_MEM     <= write_reg_EX;
        alu_result_MEM    <= alu_result_EX;
        rd_idx_MEM        <= rd_idx_EX;
        data_mem_cs_MEM   <= data_mem_cs_EX;
        data_mem_wr_MEM   <= data_mem_wr_EX;
        opcode_MEM        <= opcode_EX;
    end

endmodule

module wb_stage(input [`WL:0] alu_data_MEM,
                input [`WL:0] mem_data_MEM,
                input data_mem_cs_MEM, data_mem_wr_MEM,

                output [`WL:0] rd_data
                );
    
    assign rd_data = (data_mem_cs_MEM & data_mem_wr_MEM) ? mem_data_MEM : alu_data_MEM;

endmodule


module forwarding_logic(input [6:0] opcode_DE,
                        input [6:0] opcode_EX,
                        input [6:0] opcode_MEM,
                        input [4:0] rd_idx_EX,
                        input [4:0] rd_idx_MEM,
                        input [4:0] rs1_idx_DE,
                        input [4:0] rs2_idx_DE,

                        output forward_rs1_EX_DE,
                        output forward_rs2_EX_DE,
                        output forward_rs1_MEM_DE,
                        output forward_rs2_MEM_DE 
                        );

    wire opcode_cond_DE_rs1;
    wire opcode_cond_DE_rs2;
    wire opcode_cond_EX;
    wire opcode_cond_MEM;
    //                                  ALU_RR                      ALU_I                       LOAD                            STORE                   BRANCH
    assign opcode_cond_EX     = (opcode_EX == 7'b0110011)  | (opcode_EX == 7'b0010011);
    assign opcode_cond_MEM    = (opcode_MEM == 7'b0110011) | (opcode_MEM == 7'b0010011) | (opcode_MEM == 7'b0000011);
    assign opcode_cond_DE_rs1 = (opcode_DE == 7'b0110011)  | (opcode_DE == 7'b0010011)  | (opcode_DE == 7'b0000011)  | (opcode_DE == 7'b0100011) | (opcode_DE == 7'b1100011);
    assign opcode_cond_DE_rs2 = (opcode_DE == 7'b0110011);

    assign forward_rs1_EX_DE  = opcode_cond_EX  & opcode_cond_DE_rs1 & (rd_idx_EX  == rs1_idx_DE);
    assign forward_rs2_EX_DE  = opcode_cond_EX  & opcode_cond_DE_rs2 & (rd_idx_EX  == rs2_idx_DE);
    assign forward_rs1_MEM_DE = opcode_cond_MEM & opcode_cond_DE_rs1 & (rd_idx_MEM == rs1_idx_DE);
    assign forward_rs2_MEM_DE = opcode_cond_MEM & opcode_cond_DE_rs2 & (rd_idx_MEM == rs2_idx_DE);

endmodule

module interlock_logic( input [6:0] opcode_DE,
                        input [6:0] opcode_IF,
                        input [4:0] rs1_idx_DE,
                        input [4:0] rs2_idx_DE,
                        input [4:0] rd_idx_IF,

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

    assign rs1_idx_cond       = rd_idx_IF == rs1_idx_DE;
    assign rs2_idx_cond       = rd_idx_IF == rs2_idx_DE;

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
        case( {branch,op} )
            4'b0000: out <= adder_out;
            4'b0001: out <= shifter_out;
            4'b0010: out <= lt_int; 
            4'b0011: out <= ltu_int; 
            4'b0100: out <= in1 ^ in2;
            4'b0101: out <= shifter_out;
            4'b0110: out <= in1 | in2;
            4'b0111: out <= in1 & in2;

            4'b1000: take_branch <= eq_int;
            4'b1001: take_branch <= ~eq_int;
            4'b1100: take_branch <= lt_int[0];
            4'b1101: take_branch <= ~lt_int[0];
            4'b1110: take_branch <= ltu_int[0];
            4'b1111: take_branch <= ~ltu_int[0];

            default:
            begin
                out <= out;
                take_branch <= take_branch;
            end
        endcase

    end


endmodule


module memory(  input clk,
                input wr,
                input cs,
                input [`WL:0] addr,

                inout [`WL:0] mem_bus
                );

    reg [`WL:0] RAM [2048:0];
    reg [`WL:0] data_out;

    assign mem_bus = (cs & ~wr) ? data_out : 32'bZ;

    always @(negedge clk)
    begin
        if(cs & wr) RAM[addr[11:0]] <= mem_bus;
        data_out <= RAM[addr[11:0]];
    end

endmodule

/*module shift_register(  input clk,
                        input [31:0] in,
                        output reg [31:0] out);

    reg [31:0] registers [7:0];

    integer i;

    always @(negedge clk)
    begin
        out <= registers[7];
        registers[1] <= in;

        for(i=0; i<7; i=i+1)
        begin
            registers[i] <= registers[i-1];
        end
    end
endmodule*/





