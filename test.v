`timescale 1ns/1ns

`define WL 31 //word length


module riscv_core( input clk);

    wire [`WL:0]   PC;
    wire [`WL:0]   branch_PC;
    wire [ 31:0]   fetched_instruction;
    wire           branch;

    wire [ 4:0]    rs1_idx;
    wire [ 4:0]    rs2_idx;
    wire [ 4:0]    rd_idx_DE;
    wire [`WL:0]   rd_data;
    wire [`WL:0]   rs1_data;
    wire [`WL:0]   rs2_data_DE;
    

    wire [`WL:0]   immediate_data;
    wire [`WL:0]   rs2_data_EX;
    wire [ 4:0]    rd_idx_EX;
    wire           data_mem_cs_DE;
    wire           data_mem_wr_DE;
    wire           regFile_write_DE;
    wire           immediate_instr;
    wire [ 2:0]    funct3;
    wire           funct7_bit5;

    wire [`WL:0]   exec_out_EX;
    wire           take_branch;
    wire           data_mem_cs_EX;
    wire           data_mem_wr_EX;
    wire           regFile_write_EX;
    wire [ 4:0]    rd_idx_MEM;

    wire           regFile_write_MEM;
    wire [`WL:0]   exec_out_MEM;
    wire [`WL:0]   mem_data_out;
    wire           data_mem_cs_MEM;
    wire           data_mem_wr_MEM;

    registerFile regFile(   .clk( clk ),
                            .write( regFile_write_MEM ),
                            .rd_idx( rd_idx_MEM ),
                            .rd_data( rd_data ),
                            .rs1_idx( rs1_idx ),
                            .rs2_idx( rs2_idx ),

                            .rs1_data( rs1_data ),
                            .rs2_data( rs2_data_DE )
                            );



    instructionFetch IF_stage0( .clk( clk ),
                                .take_branch( take_branch ),
                                .pc( PC ),
                                .branch_pc( branch_PC ),

                                .npc( PC ),
                                .instruction( fetched_instruction )
                                );
    
    decode DE_stage0(   .clk( clk ),
                        .instruction( fetched_instruction ),
                        .PC( PC ),

                        .rs1_idx( rs1_idx ),
                        .rs2_idx( rs2_idx ),
                        .rd_idx_DE( rd_idx_DE ),
                        .immediate_data( immediate_data ),
                        .branch( branch ),
                        .data_mem_cs( data_mem_cs_DE ),
                        .data_mem_wr( data_mem_wr_DE ),
                        .write_reg( regFile_write_DE ),
                        .immediate_instr( immediate_instr ),
                        .funct3( funct3 ),
                        .funct7_bit ( funct7_bit5 ),
                        .branch_PC( branch_PC )
                        );
    

    exec_stage exec_stage0( .clk( clk ),
                            .rs1_data( rs1_data ),
                            .rs2_data_DE( rs2_data_DE ),
                            .immediate_data( immediate_data ),
                            .branch( branch ),
                            .immediate_instr( immediate_instr ),
                            .rd_idx_DE( rd_idx_DE ),
                            .op( funct3 ),
                            .funct7_bit( funct7_bit5 ),
                            .data_mem_cs_DE( data_mem_cs_DE ),
                            .data_mem_wr_DE( data_mem_wr_DE ),
                            .write_reg_DE( regFile_write_DE ),

                            .alu_out( exec_out_EX ),
                            .rs2_data_EX( rs2_data_EX ),
                            .take_branch( take_branch ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .write_reg_EX( regFile_write_EX ),
                            .rd_idx_EX( rd_idx_EX )
                            );

    mem_stage mem_stage0(   .clk( clk ),
                            .addr( exec_out_EX ),
                            .data_in( rs2_data_EX ),
                            .data_mem_cs_EX( data_mem_cs_EX ),
                            .data_mem_wr_EX( data_mem_wr_EX ),
                            .write_reg_EX( regFile_write_EX ),
                            .rd_idx_EX( rd_idx_EX ),

                            .mem_data_out( mem_data_out ),
                            .alu_data_out( exec_out_MEM ),
                            .write_reg_MEM( regFile_write_MEM ),
                            .rd_idx_MEM( rd_idx_MEM ),
                            .data_mem_cs_MEM ( data_mem_cs_MEM ),
                            .data_mem_wr_MEM ( data_mem_wr_MEM )
                        );

    wb_stage wb_stage0( .alu_data( exec_out_MEM ),
                        .mem_data( mem_data_out ),
                        .data_mem_cs_MEM( data_mem_cs_MEM ), .data_mem_wr_MEM( data_mem_wr_MEM ),
                        .rd_data( rd_data )
                    );

endmodule


module instructionFetch(input clk,
                        input take_branch,
                        input [`WL:0] pc,
                        input [`WL:0] branch_pc,

                        output reg [`WL:0] npc,
                        output reg [31:0] instruction
                        );
    

    wire [31:0] mem_bus;

    memory instr_mem(   .clk(clk),
                        .wr(1'b0),
                        .cs(1'b1),
                        .addr(pc),
                        .mem_bus(mem_bus));

    always @(negedge clk)
    begin
        instruction <= mem_bus;
        npc <= (take_branch) ? branch_pc : pc + 4;
    end

endmodule

//R-type: 0110011(ALU RR), 0011011(Shifts-64), 0111011(ALU RR-64)
//I-type: 0010011(ALU I), 0000011(LOADS), 1100111(JALR), 0011011(ALU I-64), 
//S-type: 0100011(Store)
//B-type: 1100011(Branch) 
//U-type: 0110111(LUI), 0010111(AUIPC)
//J-type: 1101111(JAL)
module decode(  input clk,
                input [31:0] instruction,
                input [`WL:0] PC,

                output [4:0] rs1_idx,rs2_idx,
                output reg [4:0] rd_idx_DE,
                output reg [`WL:0] immediate_data,
                output reg branch,
                output reg data_mem_cs,data_mem_wr,
                output reg write_reg,
                output reg immediate_instr,
                output reg [2:0] funct3,
                output reg funct7_bit,
                output reg [`WL:0] branch_PC
                );

    wire [6:0] opcode = instruction[6:0];
    wire [6:0] funct7 = instruction[31:25];


    assign rs1_idx = instruction[19:15];
    assign rs2_idx = instruction[24:20];


    always @(negedge clk)
    begin
        funct3 <= instruction[14:12];
        funct7_bit <= funct7[5];
        rd_idx_DE <= instruction[11:7]; 

        branch_PC <= PC + {19'd0,instruction[31],instruction[7],instruction[30:25],instruction[11:8],1'b0};

        case(opcode)
            7'b1100011: //Branch
            begin
                branch <= 1'b1;
                immediate_data <= {19'd0,instruction[31],instruction[7],instruction[30:25],instruction[11:8],1'b0};
            end
            7'b0010011: //ALU Immediate
            begin
                immediate_instr <= 1'b1;
                immediate_data <= (instruction[14:12] == 3'b001 & instruction[14:12] == 3'b101) ? { {20{instruction[31]}} , instruction[31:20] } : { {27{rs2_idx[4]}} , rs2_idx };
                write_reg <= 1'b1;
            end
            7'b0000011: //LOADS
            begin
                data_mem_cs <= 1'b1;
                data_mem_wr <= 1'b0;
                immediate_data <= { {20{instruction[31]}} , instruction[31:20]};
                write_reg <= 1'b1;
            end
            //7'b1100111: //JALR
            7'b0100011: //STORE
            begin
                data_mem_cs <= 1'b1;
                data_mem_wr <= 1'b1;
                immediate_data <= { {20{instruction[31]}} , instruction[31:25] , instruction[11:7]  };
            end
            7'b0110011: //ALU RR
            begin
                write_reg <= 1'b1;
            end
            //7'b1101111: //JAL
            //7'b0110111: //LUI
            //7'b0010111: //AUIPC
            default:
            begin
                branch <= branch;
                immediate_data <= immediate_data;
                data_mem_cs <= data_mem_cs;
                data_mem_wr <= data_mem_wr;
                write_reg <= write_reg;
                immediate_instr <= immediate_instr;
            end
        endcase

    end
endmodule


module exec_stage(  input clk,
                    input [`WL:0] rs1_data, rs2_data_DE, immediate_data,
                    input branch, immediate_instr,
                    input [4:0] rd_idx_DE,
                    input [2:0] op,
                    input funct7_bit,
                    input data_mem_cs_DE, data_mem_wr_DE, write_reg_DE,

                    output reg [`WL:0] alu_out,
                    output reg [`WL:0] rs2_data_EX,
                    output reg take_branch,
                    output reg data_mem_cs_EX, data_mem_wr_EX, write_reg_EX,
                    output reg [4:0] rd_idx_EX
                    );

    wire [`WL:0] alu_input2;

    assign alu_input2 = (branch | immediate_instr) ? immediate_data : rs2_data_DE;

    alu alu_unit(   .clk(clk),
                    .in1(rs1_data),
                    .in2(alu_input2),
                    .op(op),
                    .branch(branch),
                    .funct7_bit(funct7_bit),
                    .out(alu_out),
                    .take_branch(take_branch));

    always @(negedge clk)
    begin
        data_mem_cs_EX <= data_mem_cs_DE;
        data_mem_wr_EX <= data_mem_wr_DE;
        write_reg_EX <= write_reg_DE;
        rs2_data_EX <= rs2_data_DE;
        rd_idx_EX <= rd_idx_DE;
    end

endmodule

module mem_stage(   input clk,
                    input [`WL:0] addr,
                    input [`WL:0] data_in,
                    input data_mem_cs_EX, data_mem_wr_EX,
                    input write_reg_EX,
                    input [4:0] rd_idx_EX,

                    output reg [`WL:0] mem_data_out,
                    output reg [`WL:0] alu_data_out,
                    output write_reg_MEM,
                    output reg [4:0] rd_idx_MEM,
                    output reg data_mem_cs_MEM, data_mem_wr_MEM
                    );

    wire [`WL:0] mem_bus;

    assign mem_bus = (data_mem_cs_EX & ~data_mem_wr_EX) ? data_in : 32'bZ;

    memory mem_0(   .clk(clk),
                    .wr(data_mem_wr_EX),
                    .cs(data_mem_cs_EX),
                    .addr(addr),
                    .mem_bus(mem_bus));

    always @(negedge clk)
    begin
        mem_data_out <= (data_mem_cs_EX & data_mem_wr_EX) ? mem_bus : mem_data_out;
        write_reg_MEM <= write_reg_EX;
        alu_data_out <= data_in;
        rd_idx_MEM <= rd_idx_EX;
        data_mem_cs_MEM <= data_mem_cs_EX;
        data_mem_wr_MEM <= data_mem_wr_EX;
    end

endmodule

module wb_stage(input [`WL:0] alu_data,
                input [`WL:0] mem_data,
                input data_mem_cs_MEM, data_mem_wr_MEM,

                output [`WL:0] rd_data
                );
    
    assign rd_data = (data_mem_cs_MEM & data_mem_wr_MEM) ? mem_data : alu_data;

endmodule



module registerFile(clk,write,rd_idx,rd_data,rs1_idx,rs2_idx,rs1_data,rs2_data);
    input clk;
    input write;
    input [4:0] rd_idx,rs1_idx,rs2_idx;
    input [`WL:0] rd_data;
    output reg [`WL:0] rs1_data,rs2_data;

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
                            .shift_by(in2[4:0]),
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


module memory(clk,wr,cs,addr,mem_bus);
input clk;
input wr,cs;
input [`WL:0] addr; 
inout [`WL:0] mem_bus;

reg [`WL:0] RAM [2048:0];
reg [`WL:0] data_out;

    assign mem_bus = (cs & ~wr) ? data_out : 32'bZ;

    always @(negedge clk)
    begin
        if(cs & wr) RAM[addr[11:0]] <= mem_bus;
        data_out <= RAM[addr[11:0]];
    end

endmodule

module shift_register(  input clk,
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
endmodule





