#include <iostream>
#include <cstdint>
#include <string>
#include <vector>
#include <cmath>

std::vector<std::string> R_instr = {
		"add","sub", "sll", "slt",
		"sltu", "xor", "srl", 
		"sra", "or", "and"
        };

uint32_t generate_alu_rr_instr(){
    uint32_t opcode = 0b0110011;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t rs1_idx = rand() % 32;
    uint32_t rs2_idx = rand() % 32;
    uint32_t funct3 = rand() % 8;
    uint32_t funct7_val = (rand() % 2) ? 0b0100000 : 0;
    uint32_t funct7 = (funct3 == 0 || funct3 == 5) ? funct7_val : 0;
    uint32_t instruction = funct7 << 25 | rs2_idx << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_alu_imm_instr(){
    uint32_t opcode = 0b0010011;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t rs1_idx = rand() % 32;
    uint32_t funct3 = rand() % 8;
    int32_t immediate = ((int32_t)pow(2,11) - 1) - 2*(rand() % (int32_t)pow(2,11));
    uint32_t instruction = immediate << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_load_instr(){
    uint32_t opcode = 0b0000011;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t rs1_idx = rand() % 32;
    uint32_t funct3_values[5] = {0,1,2,4,5};
    uint32_t funct3 = funct3_values[rand() % 5];
    int32_t immediate = ((int32_t)pow(2,11) - 1) - 2*(rand() % (int32_t)pow(2,11));
    uint32_t instruction = immediate << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_jalr_instr(){
    uint32_t opcode = 0b1100111;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t rs1_idx = rand() % 32;
    uint32_t funct3 = 0;
    int32_t immediate = ((int32_t)pow(2,11) - 1) - 2*(rand() % (int32_t)pow(2,11));
    uint32_t instruction = immediate << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_store_instr(){
    uint32_t opcode = 0b0100011;
    uint32_t rs2_idx = rand() % 32; //no zeroes
    uint32_t rs1_idx = rand() % 32;
    uint32_t funct3 = rand() % 3;
    int32_t immediate = ((int32_t)pow(2,11) - 1) - 2*(rand() % (int32_t)pow(2,11));
    int32_t imm_field1 = immediate & 0x00000FE0;
    int32_t imm_field2 = immediate & 0x0000001F;
    uint32_t instruction = imm_field1 << 20 | rs1_idx << 15 | funct3 << 12 | imm_field2 << 7 | opcode;
    return instruction;
}


