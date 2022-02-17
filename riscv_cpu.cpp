#include <iostream>


uint32_t instruction_fetch(uint32_t* instruction_memory, uint32_t* PC){

    uint32_t instruction = instruction_memory[*PC];
    *PC = *PC + 4;

    return instruction;
}

void instruction_decode(uint32_t instruction, uint32_t* register_file){
    uint8_t opcode = instruction << 25;

    switch (opcode) {
        case 0b1100011: //branch

        case 0b0010011: //ALU immediate

        case 0b0000011: //load

        case 0b0100011: //store

        case 0b0110011: //ALU RR

        default:
    }
}

