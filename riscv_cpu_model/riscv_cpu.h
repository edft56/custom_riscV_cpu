#pragma once
#include <iostream>
#include <cstdint>
#include <string>

class Branch{
    public:
        bool get_branch_cond(uint32_t rs1, uint32_t rs2, uint8_t branch_op);
        uint32_t get_branch_addr(uint32_t PC, uint32_t immediate_data);
};

class ALU{
    public:
        uint32_t get_alu_result(uint32_t rs1, uint32_t rs2, uint8_t alu_op, bool func7_bit5);
};

class Ram{
    private:
        uint8_t* mem_ptr;
        uint32_t mem_size;
    public:
        Ram(uint32_t ram_size_in_bytes);
        
        void initialize_RAM(std::string filename);

        template<class T>
        T load(uint32_t address);

        template<class T>
        void store(uint32_t address, T data_to_store);

        ~Ram();
};

class RegisterFile{
    private:
        uint32_t reg_file[32]={0};

    public:
        uint32_t read(uint8_t reg_idx);

        void write(uint8_t reg_idx, uint32_t data_to_write);

        void print_all_registers();
};


class RiscvCore{
    public:
        Ram data_memory{8*1024*1024};
        Ram instruction_memory{8*1024*1024};

        RegisterFile reg_file;
        Branch branch_unit;
        ALU alu_unit;

        uint32_t PC=0;


        void handle_branch_instr(uint8_t rs1_idx, uint8_t rs2_idx, uint8_t funct3, uint32_t immediate_data);

        void handle_alu_instr(uint8_t rs1_idx, uint8_t rs2_idx, uint8_t rd_idx, uint8_t funct3, bool funct7_bit5);
    
        void handle_alu_imm_instr(uint8_t rs1_idx, uint32_t immediate_data, uint8_t rd_idx, uint8_t funct3, bool funct7_bit5);

        void handle_load_instr(uint32_t rs1_idx, uint32_t immediate_data, uint8_t rd_idx, uint8_t funct3);

        void handle_store_instr(uint32_t rs1_idx, uint32_t immediate_data, uint8_t rs2_idx, uint8_t funct3);

        void handle_jal(uint32_t rd_idx, uint32_t immediate_data);

        void handle_jalr(uint32_t rd_idx, uint32_t rs1_idx, uint32_t immediate_data);

        void handle_lui(uint32_t rd_idx, uint32_t immediate_data);

        void handle_auipc(uint32_t rd_idx, uint32_t immediate_data);

        uint32_t instruction_fetch();

        void instruction_decode_and_execute(uint32_t instruction);

    public:
        void issue_and_execute_instruction();

        void initialize_mems();
};