`timescale 1ns/1ns

`define WL 31 //word length


module instructionFetch(clk,instr_mem,branch,pc,branch_pc,npc,instruction);
input clk;
input [31:0] instr_mem [1024:0];
input [`WL:0] pc,branch_pc;
input branch;
output reg [`WL:0] npc;
output reg [31:0] instruction;

    always @(negedge clk)
    begin
        instruction <= instr_mem[pc];
        npc <= (branch) ? branch_pc : pc + 4;
    end

endmodule

//R-type: 0110011(ALU RR), 0011011(Shifts-64), 0111011(ALU RR-64)
//I-type: 0010011(ALU I), 0000011(LOADS), 1100111(JALR), 0011011(ALU I-64), 
//S-type: 0100011(Store)
//B-type: 1100011(Branch) 
//U-type: 0110111(LUI), 0010111(AUIPC)
//J-type: 1101111(JAL)
module decode(clk,instruction,rs1_data,rs2_data,immediate,branch,branch_op,alu_op,cs,wr,write_reg);
input clk;
input [31:0] instruction;
output reg [`WL:0] rs1_data,rs2_data;
output reg [31:0] immediate;
output reg branch,cs,wr,write_reg;
output reg [2:0] branch_op;
output reg [3:0] alu_op;

wire [6:0] opcode = instruction[6:0];
wire [2:0] funct3 = instruction[14:12];
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
        case(opcode)
            7'b1100011: //Branch
            begin
                branch <= 1'b1;
                branch_op <= funct3;
                immediate <= {19'd0,instruction[31],instruction[7],instruction[30:25],instruction[11:8],1'b0};
            end
            7'b0010011: //ALU Immediate
            begin
                alu_op <= {funct7[5],funct3};
                immediate <= (funct3 == 001 & funct3 == 101) ? { {20{instruction[31]}} , instruction[31:20] } : { {27{rs2_idx[4]}} , rs2_idx };
                write_reg <= 1'b1;
            end
            7'b0000011: //LOADS
            begin
                cs <= 1'b1;
                wr <= 1'b0;
                immediate <= { {20{instruction[31]}} , instruction[31:20]};
                write_reg <= 1'b1;
            end
            //7'b1100111: //JALR
            7'b0100011: //STORE
            begin
                cs <= 1'b1;
                wr <= 1'b1;
                immediate <= { {20{instruction[31]}} , instruction[31:25] , instruction[11:7]  };
            end
            7'b0110011: //ALU RR
            begin
                alu_op <= {funct7[5],funct3};
                write_reg <= 1'b1;
            end
            //7'b1101111: //JAL
            //7'b0110111: //LUI
            //7'b0010111: //AUIPC
            default:
            begin
                branch <= branch;
                branch_op <= branch_op;
                immediate <= immediate;
                cs <= cs;
                wr <= wr;
                alu_op <= alu_op;
                write_reg <= write_reg;
            end
        endcase

    end
endmodule


module exec_stage(clk,rs1_data,rs2_data,immediate,branch,branch_op,alu_op,cs_DE,wr_DE,write_reg_DE);
input clk;
input [`WL:0] rs1_data,rs2_data,immediate;
input [3:0] alu_op;
input [2:0] branch_op;
input branch, cs_DE, wr_DE, write_reg_DE;


endmodule



module registerFile(clk,write,rd_idx,rd_data,rs1_idx,rs2_idx,rs1_data,rs2_data);
input clk;
input write;
input [4:0] rd_idx,rs1_idx,rs2_idx;
input [`WL:0] rd_data;
output reg [`WL:0] rs1_data,rs2_data;

reg [`WL:0] registers [31:0];

    always @(negedge clk)
    begin
        registers[0] <= 0;
        if (write && rd_idx!=0'b0) registers[rd_idx] <= rd_data;
        rs1_data <= registers[rs1_idx];
        rs2_data <= registers[rs2_idx];
        
    end

endmodule


module alu(clk, in1, in2, op, branch, branch_op, out, lt, ltu, take_branch);
//operations
//Shift Left Logical    0001
//Shift Right Logical   0101
//Shift Right Arithmetic    1101
//ADD   0000
//SUB   1000
//XOR   0100
//OR    0110
//AND   0111
//Set on Less Than S    0010
//Set on Less Than U    0011
input clk;
input [`WL:0] in1,in2;
input [3:0] op;
input branch;
input [2:0] branch_op;
output reg [`WL:0] out;
output reg lt,ltu;
output reg take_branch;

wire eq_int,lt_int,ltu_int;

    assign eq_int = in1 == in2;
    assign lt_int = in1 < in2;
    assign ltu_int = $signed(in1) < $signed(in2);

    always @(negedge clk)
    begin
        case(op)
            4'b0000: out <= in1 + in2;
            4'b1000: out <= in1 - in2;
            4'b0001: out <= in1 << in2;
            4'b0100: out <= in1 ^ in2;
            4'b0101: out <= in1 >> in2;
            4'b1101: out <= $signed(in1) >>> in2;
            4'b0110: out <= in1 | in2;
            4'b0111: out <= in1 & in2;
            default: out <= out;
        endcase

        case({branch,branch_op})
            4'b1000: take_branch <= eq_int;
            4'b1001: take_branch <= ~eq_int;
            4'b1100: take_branch <= lt_int;
            4'b1101: take_branch <= ~lt_int;
            4'b1110: take_branch <= ltu_int;
            4'b1111: take_branch <= ~ltu_int;
            default: take_branch <= take_branch;
        endcase

        lt <= lt_int;
        ltu <= ltu_int;
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




