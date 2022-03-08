#include <iostream>
#include <cstdint>
#include <cmath>



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
    int32_t immediate = (rand() % (int32_t)pow(2,12));
    uint32_t instruction = immediate << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_load_instr(uint32_t mem_range = (uint32_t)pow(2,12), uint32_t align = 4, bool base_zero = true){
    uint32_t opcode = 0b0000011;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t rs1_idx = (base_zero) ? 0 : rand() % 32;
    uint32_t funct3_values[5] = {0,1,2,4,5};
    uint32_t funct3 = funct3_values[rand() % 5];
    uint32_t immediate = (rand() % mem_range) & (align-1);
    uint32_t instruction = immediate << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_jalr_instr(uint32_t jump_range = (uint32_t)pow(2,12), bool base_zero = true){
    uint32_t opcode = 0b1100111;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t rs1_idx = (base_zero) ? 0 : rand() % 32;
    uint32_t funct3 = 0;
    uint32_t immediate = (rand() % jump_range) & 0x00000003; // makes sure its multiple of 4
    uint32_t instruction = immediate << 20 | rs1_idx << 15 | funct3 << 12 | rd_idx << 7 | opcode;
    return instruction;
}

uint32_t generate_store_instr(uint32_t mem_range = (uint32_t)pow(2,12), uint32_t align = 4, bool base_zero = true){
    uint32_t opcode = 0b0100011;
    uint32_t rs2_idx = rand() % 32;
    uint32_t rs1_idx = (base_zero) ? 0 : rand() % 32;
    uint32_t funct3 = rand() % 3;
    uint32_t immediate = (rand() % mem_range) & (align-1);
    uint32_t imm_field1 = immediate & 0x00000FE0;
    uint32_t imm_field2 = immediate & 0x0000001F;
    uint32_t instruction = imm_field1 << 20 | rs1_idx << 15 | funct3 << 12 | imm_field2 << 7 | opcode;
    return instruction;
}

uint32_t generate_lui_instr(){
    uint32_t opcode = 0b0110111;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    int32_t immediate = rand() % (int32_t)pow(2,20);

    uint32_t instruction = immediate << 7 | opcode;
    return instruction;
}

uint32_t generate_auipc_instr(){
    uint32_t opcode = 0b0010111;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    int32_t immediate = rand() % (int32_t)pow(2,20);

    uint32_t instruction = immediate << 7 | opcode;
    return instruction;
}

uint32_t generate_branch_instr(uint32_t branch_range = (uint32_t)pow(2,12)){
    uint32_t opcode = 0b1100011;
    uint32_t rs2_idx = rand() % 32;
    uint32_t rs1_idx = rand() % 32;
    uint32_t funct3_values[6] = {0,1,4,5,6,7};
    uint32_t funct3 = funct3_values[rand() % 6];
    uint32_t immediate = (rand() % branch_range) & 0x00000003; // makes sure its multiple of 4
    uint32_t imm_field1 = immediate & 0x00000800;
    uint32_t imm_field2 = immediate & 0x000003F0;
    uint32_t imm_field3 = immediate & 0x0000000F;
    uint32_t imm_field4 = immediate & 0x00000400;
    uint32_t final_immediate = imm_field1 << 31 | imm_field2 << 25 | imm_field3 << 8 | imm_field4 << 7;
    uint32_t instruction = final_immediate | rs2_idx << 20 | rs1_idx << 15 | funct3 << 12 | opcode;
    return instruction;
}

uint32_t generate_jal_instr(uint32_t jump_range = (uint32_t)pow(2,21)){
    uint32_t opcode = 0b1101111;
    uint32_t rd_idx = rand() % 31 + 1; //no zeroes
    uint32_t immediate = (rand() % (jump_range/2)) & 0x00000003; // makes sure its multiple of 4;
    uint32_t imm_field1 = immediate & 0x00080000;
    uint32_t imm_field2 = immediate & 0x000003FF;
    uint32_t imm_field3 = immediate & 0x00000400;
    uint32_t imm_field4 = immediate & 0x0007F800;
    uint32_t final_immediate = imm_field1 << 31 | imm_field2 << 21 | imm_field3 << 20 | imm_field4 << 12;
    uint32_t instruction = final_immediate | rd_idx << 7 | opcode;
    return instruction;
}



