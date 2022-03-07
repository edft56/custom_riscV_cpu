#include "riscv_cpu.h"
#include <iostream>
#include <fstream>
#include <string>
#include <bitset>
#include <cstdint>


std::string reg_map [32] =   {
                                               "x0",  
                                                 "ra", 
                                                 "sp", 
                                                 "gp", 
                                                 "tp", 
                                                 "t0", 
                                                 "t1", 
                                                 "t2", 
                                                 "s0",  //x8
                                                   //x8
                                                 "s1", 
                                                 "a0", 
                                                 "a1", 
                                                 "a2", 
                                                 "a3", 
                                                 "a4", 
                                                 "a5", 
                                                 "a6", 
                                                 "a7", 
                                                 "s2", 
                                                 "s3", 
                                                 "s4", 
                                                 "s5", 
                                                 "s6", 
                                                 "s7", 
                                                 "s8", 
                                                 "s9", 
                                                 "s10",
                                                 "s11",
                                                 "t3", 
                                                 "t4", 
                                                 "t5", 
                                                 "t6", 
                                            };



bool     Branch::get_branch_cond(uint32_t rs1, uint32_t rs2, uint8_t branch_op){
            if( branch_op == 0) return rs1 == rs2;
            if( branch_op == 1) return rs1 != rs2;
            if( branch_op == 4) return rs1  < rs2;
            if( branch_op == 5) return rs1 >= rs2;
            if( branch_op == 6) return *(int32_t*)(&rs1) <  *(int32_t*)(&rs2);
            if( branch_op == 7) return *(int32_t*)(&rs1) >= *(int32_t*)(&rs2);
            std::cout<<"Unknown Branch op"<<std::endl;
            exit(1);
        }

uint32_t Branch::get_branch_addr(uint32_t PC, uint32_t immediate_data){
    return PC + immediate_data;
}


uint32_t ALU::get_alu_result(uint32_t rs1, uint32_t rs2, uint8_t alu_op, bool func7_bit5){
    if( alu_op == 0) return (func7_bit5) ? rs1 - rs2 : rs1 + rs2;
    if( alu_op == 1) return rs1 << rs2;
    if( alu_op == 2) return *(int32_t*)(&rs1) <  *(int32_t*)(&rs2);
    if( alu_op == 3) return rs1  < rs2; 
    if( alu_op == 4) return rs1 ^ rs2;
    if( alu_op == 5) return (func7_bit5) ? *(int32_t*)(&rs1) >> *(int32_t*)(&rs2) : rs1 >> rs2;
    if( alu_op == 6) return rs1 | rs2;
    if( alu_op == 7) return rs1 & rs2;
    std::cout<<"Unknown ALU op"<<std::endl;
    exit(1);
}


        Ram::Ram(uint32_t ram_size_in_bytes){
    mem_size = ram_size_in_bytes;
    mem_ptr = (uint8_t*) calloc(ram_size_in_bytes,sizeof(uint8_t));
}

void    Ram::initialize_RAM(std::string filename){
    std::ifstream init_file;
    init_file.open(filename, std::ios::binary | std::ios::ate);
    if (!init_file) { std::cout << "Unable to open Ram init file"<<"\n"; exit(1);}

    uint32_t size = init_file.tellg();
    if (size>mem_size) {std::cout<<"Init file too big for Ram size"<<"\n"; exit(1);}

    init_file.seekg(0);
    init_file.read( reinterpret_cast<char*>(mem_ptr), size );
    init_file.close();
}

template<class T>
T       Ram::load(uint32_t address){
    return reinterpret_cast<T*>(mem_ptr)[address/sizeof(T)];
}
template uint32_t Ram::load<uint32_t>(uint32_t);
template uint16_t Ram::load<uint16_t>(uint32_t);
template uint8_t  Ram::load<uint8_t>(uint32_t);

template<class T>
void    Ram::store(uint32_t address, T data_to_store){
    reinterpret_cast<T*>(mem_ptr)[address/sizeof(T)] = data_to_store;
}
template void Ram::store<uint32_t>(uint32_t,uint32_t);
template void Ram::store<uint16_t>(uint32_t,uint16_t);
template void Ram::store<uint8_t> (uint32_t, uint8_t);

        Ram::~Ram(){
    free(mem_ptr);
}



uint32_t RegisterFile::read(uint8_t reg_idx){
    return reg_file[reg_idx];
}

void     RegisterFile::write(uint8_t reg_idx, uint32_t data_to_write){
    if (reg_idx!=0) reg_file[reg_idx] = data_to_write;
}

void     RegisterFile::print_all_registers(){
    for(int i=0; i<32; i++){
        std::cout<<reg_map[i]<<" "<<reg_file[i]<<"\n";
    }
}



void     RiscvCore::handle_branch_instr(uint8_t rs1_idx, uint8_t rs2_idx, uint8_t funct3, uint32_t immediate_data){
    uint32_t rs1_data = reg_file.read(rs1_idx);
    uint32_t rs2_data = reg_file.read(rs2_idx);
    bool take_branch = branch_unit.get_branch_cond(rs1_data, rs2_data, funct3);
    if(take_branch) PC = branch_unit.get_branch_addr(PC,immediate_data);
}

void     RiscvCore::handle_alu_instr(uint8_t rs1_idx, uint8_t rs2_idx, uint8_t rd_idx, uint8_t funct3, bool funct7_bit5){
    uint32_t rs1_data = reg_file.read(rs1_idx);
    uint32_t rs2_data = reg_file.read(rs2_idx);
    uint32_t alu_result = alu_unit.get_alu_result(rs1_data,rs2_data,funct3,funct7_bit5);
    reg_file.write(rd_idx,alu_result);
}

void     RiscvCore::handle_alu_imm_instr(uint8_t rs1_idx, uint32_t immediate_data, uint8_t rd_idx, uint8_t funct3, bool funct7_bit5){
    uint32_t rs1_data = reg_file.read(rs1_idx);
    if (funct3==0) funct7_bit5 = false;
    uint32_t alu_result = alu_unit.get_alu_result(rs1_data,immediate_data,funct3,funct7_bit5);
    reg_file.write(rd_idx,alu_result);
}

void     RiscvCore::handle_load_instr(uint32_t rs1_idx, uint32_t immediate_data, uint8_t rd_idx, uint8_t funct3){
    uint32_t mem_data = 0;

    uint32_t rs1_data = reg_file.read(rs1_idx);
    uint32_t address = rs1_data + immediate_data;

    switch (funct3){
        case  0: //LB
            mem_data = data_memory.load<uint8_t>(address);
            mem_data = ( (mem_data & 0x80)==0 ) ? mem_data : mem_data | 0xFFFFFF00; //sign extend
            break;
        case  1: //LH
            mem_data = data_memory.load<uint16_t>(address);
            mem_data = ( (mem_data & 0x8000)==0 ) ? mem_data : mem_data | 0xFFFF0000; //sign extend
            break;
        case  2: //LW
            mem_data = data_memory.load<uint32_t>(address);
            break;
        case  4: //LBU
            mem_data = data_memory.load<uint8_t>(address);
            break;
        case  5: //LHU
            mem_data = data_memory.load<uint16_t>(address);
            break;
        default:
            std::cout<<"Wrong funct3 on Load instruction "<<funct3<<std::endl;
            exit(1);
    }
    
    reg_file.write(rd_idx,mem_data);
}

void     RiscvCore::handle_store_instr(uint32_t rs1_idx, uint32_t immediate_data, uint8_t rs2_idx, uint8_t funct3){
    uint32_t rs2_data = reg_file.read(rs2_idx);
    uint32_t rs1_data = reg_file.read(rs1_idx);

    uint32_t address = rs1_data + immediate_data;

    switch (funct3){
        case  0: 
            data_memory.store<uint8_t>(address,(uint8_t)rs2_data);
            break;
        case  1: 
            data_memory.store<uint16_t>(address,(uint16_t)rs2_data);
            break;
        case  2: 
            data_memory.store<uint32_t>(address,rs2_data);
            break;
        default:
            std::cout<<"Wrong funct3 on Store instruction"<<std::endl;
            exit(1);
    }
}

void     RiscvCore::handle_jal(uint32_t rd_idx, uint32_t immediate_data){
    reg_file.write(rd_idx, PC);
    PC = PC + immediate_data;
}

void     RiscvCore::handle_jalr(uint32_t rd_idx, uint32_t rs1_idx, uint32_t immediate_data){
    uint32_t rs1_data = reg_file.read(rs1_idx);
    reg_file.write(rd_idx, PC);
    PC = rs1_data + immediate_data;
}

void     RiscvCore::handle_lui(uint32_t rd_idx, uint32_t immediate_data){
    reg_file.write(rd_idx, immediate_data);
}

void     RiscvCore::handle_auipc(uint32_t rd_idx, uint32_t immediate_data){
    reg_file.write(rd_idx, immediate_data + PC);
}

uint32_t RiscvCore::instruction_fetch(){
    uint32_t instruction = instruction_memory.load<uint32_t>(PC);
    PC = PC + 4;
    return instruction;
}

void     RiscvCore::instruction_decode_and_execute(uint32_t instruction){
    uint8_t opcode = instruction & 0x7F;
    uint8_t funct3 = (instruction & 0x7000) >> 12;
    bool funct7_bit5 = ((instruction & 0x40000000) ) != 0;
    uint8_t rs1_idx = (instruction & 0xF8000) >> 15;
    uint8_t rs2_idx = (instruction & 0x1F00000) >> 20;
    uint8_t rd_idx = (instruction & 0xF80) >> 7;

    uint32_t immediate_data;

    switch (opcode) {
        case 0b01100011: //branch  
        {   
            std::cout<<std::bitset<32>(instruction)<<"\n";          
            uint32_t sign_extend = ( ( (instruction & 0x80000000) != 0 ) ? 0xFFFFF800 : 0x00000000 );
            immediate_data = ( sign_extend | ( ((instruction & 0x7E000000)>>21) | ((instruction & 0xF00)>>8) | ((instruction & 0x80)<<3) ) ) << 1;
            std::cout<<immediate_data<<"\n";
            handle_branch_instr(rs1_idx,rs2_idx,funct3,immediate_data);
            break;
        }
        case 0b00010011: //ALU immediate
        {
            uint32_t sign_extend_normal = ( ((instruction & 0x80000000) != 0) ? 0xFFFFF800 : 0x00000000 );

                                    //SLLI SRLI SRAI
            immediate_data = (funct3 == 1 || funct3 == 5) ? (instruction & 0x01F00000) >> 20  : sign_extend_normal | ((instruction & 0x7FF00000) >> 20);
            handle_alu_imm_instr(rs1_idx,immediate_data,rd_idx,funct3,funct7_bit5);
            break;
        }
        case 0b00000011: //load  
        {               
            uint32_t sign_extend = ( ((instruction & 0x80000000) != 0) ? 0xFFFFF800 : 0x00000000);
            immediate_data = sign_extend | ((instruction & 0x7FF00000)>>20);
            handle_load_instr(rs1_idx,immediate_data,rd_idx,funct3);
            break;
        }
        case 0b00100011: //store    
        {
            uint32_t sign_extend = ( ((instruction & 0x80000000) != 0) ? 0xFFFFF800 : 0x00000000);
            immediate_data = sign_extend | ((instruction & 0xFE000000)>>20) | ((instruction & 0xF80)>>7);
            handle_store_instr(rs1_idx,immediate_data,rs2_idx,funct3);
            break;
        }    
        case 0b00110011: //ALU RR
        {
            handle_alu_instr(rs1_idx,rs2_idx,rd_idx,funct3,funct7_bit5);
            break;
        }
        case 0b1101111: //JAL
        {
            uint32_t sign_extend = ( ((instruction & 0x80000000) != 0) ? 0xFFF00000 : 0x00000000);
            uint32_t imm_field1 = instruction & 0x80000000;
            uint32_t imm_field2 = instruction & 0x7FE00000;
            uint32_t imm_field3 = instruction & 0x00100000;
            uint32_t imm_field4 = instruction & 0x000FF000;
            uint32_t immediate_data = (imm_field1 | (imm_field2>>9) | (imm_field3<<2) | (imm_field4<<11)) >> 12;
            handle_jal(rd_idx, immediate_data);
            break;
        }
        case 0b1100111: //JALR
        {
            uint32_t sign_extend = ( ((instruction & 0x80000000) != 0) ? 0xFFFFF800 : 0x00000000);
            immediate_data = sign_extend | ((instruction & 0x7FF00000)>>20);
            handle_jalr(rd_idx, rs1_idx, immediate_data);
            break;
        }
        case 0b0110111: //LUI
        {
            immediate_data = instruction & 0xFFFFF000;
            handle_lui(rd_idx,immediate_data);
            break;
        }
        case 0b0010111: //AUIPC
        {
            immediate_data = instruction & 0xFFFFF000;
            handle_auipc(rd_idx,immediate_data);
            break;
        }
        default:
        {
            std::cout<<"Unknown opcode"<<std::endl;
            exit(1);
        }
    }
}

void     RiscvCore::issue_and_execute_instruction(){
    std::cout<<PC<<"\n";
    uint32_t instruction = instruction_fetch();
    instruction_decode_and_execute(instruction);
}

void     RiscvCore::initialize_mems(){
    instruction_memory.initialize_RAM("ass_bin.dat");
    data_memory.initialize_RAM("data_mem_data.dat");
}


int main(){
    RiscvCore cpu;
    cpu.initialize_mems();
    for(int i=0; i<28; i++){
        cpu.issue_and_execute_instruction();
    }
    cpu.reg_file.print_all_registers();
    return 0;
}



