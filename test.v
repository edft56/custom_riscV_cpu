`timescale 1ns/1ns

`define WL 31 //word length


module riscv_CPU();

    registerFile regFile(   clk,
                            write,
                            rd_idx,
                            rd_data,
                            rs1_idx,
                            rs2_idx,
                            rs1_data,
                            rs2_data);

    instructionFetch IF_stage0(clk,branch,pc,branch_pc,npc,instruction);
    
    decode DE_stage0(clk,instruction,rs1_data,rs2_data,immediate_data,branch,cs,wr,write_reg,immediate_instr,funct3);
    
    exec_stage exec_stage0( clk,
                rs1_data,rs2_data,immediate,
                branch,immediate_instr,
                op,
                funct7_bit,
                cs_DE,wr_DE,write_reg_DE,
                alu_out,
                take_branch,
                cs_EX,wr_EX,write_reg_EX
                );

    mem_stage mem_stage0(  clk,
                addr,
                data_in,
                cs,wr,
                write_reg_EX,
                mem_data_out,
                alu_data_out,
                write_reg_MEM
                );

    wb_stage wb_stage0(   clk,
                [`WL:0] alu_data,
                [`WL:0] mem_data,
                write_reg,cs,wr,
                [4:0] rd_idx
                );

endmodule


module instructionFetch(clk,branch,pc,branch_pc,npc,instruction);
input clk;
input [31:0] instr_mem [1024:0];
input [`WL:0] pc,branch_pc;
input branch;
output reg [`WL:0] npc;
output reg [31:0] instruction;

    wire [31:0] mem_bus;

    memory instr_mem(   .clk(clk),
                        .wr(1'b0),
                        .cs(1'b1),
                        .addr(pc),
                        .mem_bus(mem_bus));

    always @(negedge clk)
    begin
        instruction <= mem_bus[pc];
        npc <= (branch) ? branch_pc : pc + 4;
    end

endmodule

//R-type: 0110011(ALU RR), 0011011(Shifts-64), 0111011(ALU RR-64)
//I-type: 0010011(ALU I), 0000011(LOADS), 1100111(JALR), 0011011(ALU I-64), 
//S-type: 0100011(Store)
//B-type: 1100011(Branch) 
//U-type: 0110111(LUI), 0010111(AUIPC)
//J-type: 1101111(JAL)
module decode(clk,instruction,rs1_data,rs2_data,immediate_data,branch,cs,wr,write_reg,immediate_instr,funct3);
input clk;
input [31:0] instruction;
output reg [`WL:0] rs1_data,rs2_data;
output reg [31:0] immediate_data;
output reg branch,cs,wr,write_reg,immediate_instr;
output reg [2:0] funct3;

wire [6:0] opcode = instruction[6:0];
wire [6:0] funct7 = instruction[31:25];
wire [4:0] rd_idx = instruction[11:7];
wire [4:0] rs1_idx = instruction[19:15];
wire [4:0] rs2_idx = instruction[24:20];

    registerFile regFile (  .clk(clk), 
                            .write(0'b0), 
                            .rd_idx(instruction[11:7]), 
                            .rd_data(0'b0), 
                            .rs1_idx(instruction[19:15]), 
                            .rs2_idx(instruction[24:20]), 
                            .rs1_data(rs1_data), 
                            .rs2_data(rs2_data)
                            );

    always @(negedge clk)
    begin
        funct3 <= instruction[14:12];

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
                cs <= 1'b1;
                wr <= 1'b0;
                immediate_data <= { {20{instruction[31]}} , instruction[31:20]};
                write_reg <= 1'b1;
            end
            //7'b1100111: //JALR
            7'b0100011: //STORE
            begin
                cs <= 1'b1;
                wr <= 1'b1;
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
                cs <= cs;
                wr <= wr;
                write_reg <= write_reg;
                immediate_instr <= immediate_instr;
            end
        endcase

    end
endmodule


module exec_stage(  input clk,
                    input [`WL:0] rs1_data,rs2_data,immediate,
                    input branch,immediate_instr,
                    input [2:0] op,
                    input funct7_bit,
                    input cs_DE,wr_DE,write_reg_DE,
                    output reg [`WL:0] alu_out,
                    output reg take_branch,
                    output reg cs_EX,wr_EX,write_reg_EX
                    );

    wire [`WL:0] alu_input2;

    assign alu_input2 = (branch | immediate_instr) ? immediate : rs2_data;

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
        cs_EX <= cs_DE;
        wr_EX <= wr_DE;
        write_reg_EX <= write_reg_DE;
    end

endmodule

module mem_stage(   input clk,
                    input [`WL:0] addr,
                    input [`WL:0] data_in,
                    input cs,wr,
                    input write_reg_EX,
                    output reg [`WL:0] mem_data_out,
                    output reg [`WL:0] alu_data_out,
                    output write_reg_MEM
                    );

    wire [`WL:0] mem_bus;

    assign mem_bus = (cs & ~wr) ? data_in : 32'bZ;

    memory mem_0(   .clk(clk),
                    .wr(wr),
                    .cs(cs),
                    .addr(addr),
                    .mem_bus(mem_bus));

    always @(negedge clk)
    begin
        mem_data_out <= (cs & wr) ? mem_bus : mem_data_out;
        write_reg_MEM <= write_reg_EX;
        alu_data_out <= data_in;
    end

endmodule

module wb_stage(input clk,
                input [`WL:0] alu_data,
                input [`WL:0] mem_data,
                input write_reg,cs,wr,
                input [4:0] rd_idx
                );
    
    registerFile regFile_0( .clk(clk), 
                            .write(write_reg), 
                            .rd_idx(rd_idx), 
                            .rd_data( (cs & wr) ? mem_data : alu_data ), 
                            .rs1_idx(), 
                            .rs2_idx(), 
                            .rs1_data(), 
                            .rs2_data()
                            );

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


module branch_addr_comp(clk,pc,immediate_offset,branch_pc);
input clk;
input [`WL:0] pc,immediate_offset;
output reg [`WL:0] branch_pc;

    always @(negedge clk)
    begin
        branch_pc <= pc + immediate_offset;
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
                        output [31:0] out);

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

module write_back_stage(clk,sel_input,write,mem_input,alu_input,rd_idx);
input clk;
input sel_input,write;
input [`WL:0] mem_input,alu_input;
input [4:0] rd_idx;

    registerFile regFile (  .clk(clk), 
                            .write(write), 
                            .rd_idx(rd_idx), 
                            .rd_data((sel_input) ? mem_input : alu_input), 
                            .rs1_idx(), 
                            .rs2_idx(), 
                            .rs1_data(), 
                            .rs2_data()
                            );
    
endmodule




