#include <iostream>

class Branch{
    public:
        bool get_branch_cond(uint32_t rs1, uint32_t rs2, uint8_t branch_op){
            if( branch_op == 0) return rs1 == rs2;
            if( branch_op == 1) return rs1 != rs2;
            if( branch_op == 4) return rs1  < rs2;
            if( branch_op == 5) return rs1 >= rs2;
            if( branch_op == 6) return *(int32_t*)(&rs1) <  *(int32_t*)(&rs2);
            if( branch_op == 7) return *(int32_t*)(&rs1) >= *(int32_t*)(&rs2);
        }

        uint32_t get_branch_addr(uint32_t PC, uint32_t immediate_data){
            return PC + immediate_data;
        }
};

class ALU{
    public:
        uint32_t get_alu_result(uint32_t rs1, uint32_t rs2, uint8_t alu_op, bool func7_bit5){
            if( alu_op == 0) return (func7_bit5) ? rs1 - rs2 : rs1 + rs2;
            if( alu_op == 1) return rs1 << rs2;
            if( alu_op == 2) return rs1  < rs2;
            if( alu_op == 3) return *(int32_t*)(&rs1) <  *(int32_t*)(&rs2);
            if( alu_op == 4) return rs1 ^ rs2;
            if( alu_op == 5) return (func7_bit5) ? *(int32_t*)(&rs1) >> *(int32_t*)(&rs2) : rs1 >> rs2;
            if( alu_op == 6) return rs1 | rs2;
            if( alu_op == 7) return rs1 & rs2;
        }
};

class Ram{
    private:
        uint8_t* mem_ptr;

    public:
        Ram(uint32_t ram_size_in_bytes){
            mem_ptr = (uint8_t*) malloc(ram_size_in_bytes*sizeof(uint8_t));
        }


        template<class T>
        T load(uint32_t address){
            return reinterpret_cast<T*>mem_ptr[address/sizeof(T)];
        }

        template<class T>
        void store(uint32_t address, T data_to_store){
            reinterpret_cast<T*>mem_ptr[address/sizeof(T)] = data_to_store;
        }

        ~Ram(){
            free(mem_ptr);
        }
};

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

