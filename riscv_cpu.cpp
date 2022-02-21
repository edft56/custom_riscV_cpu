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
            std::cout<<"Unknown Branch op"<<std::endl;
            exit(1);
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
            std::cout<<"Unknown ALU op"<<std::endl;
            exit(1);
        }
};

class Ram{
    private:
        uint8_t* mem_ptr;

    public:
        Ram(uint32_t ram_size_in_bytes){
            mem_ptr = (uint8_t*) calloc(ram_size_in_bytes,sizeof(uint8_t));
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

class RegisterFile{
    private:
        uint32_t reg_file[32]={0};

    public:
        uint32_t read(uint8_t reg_idx){
            return reg_file[reg_idx];
        }

        void write(uint8_t reg_idx, uint32_t data_to_write){
            if (reg_idx!=0) reg_file[reg_idx] = data_to_write;
        }
};

class core{
    private:
        Ram data_memory{8*1024*1024}; //need to init
        Ram instruction_memory{8*1024*1024}; //need to init

        RegisterFile reg_file;
        Branch branch_unit;
        ALU alu_unit;

        uint32_t PC=0;


        void handle_branch_instr(uint8_t rs1_idx, uint8_t rs2_idx, uint8_t funct3, uint32_t& PC, uint32_t immediate_data){
            uint32_t rs1_data = reg_file.read(rs1_idx);
            uint32_t rs2_data = reg_file.read(rs2_idx);
            bool take_branch = branch_unit.get_branch_cond(rs1_data, rs2_data, funct3);
            if(take_branch) PC = branch_unit.get_branch_addr(PC,immediate_data);
        }

        void handle_alu_instr(uint8_t rs1_idx, uint8_t rs2_idx, uint8_t rd_idx, uint8_t funct3, bool funct7_bit5){
            uint32_t rs1_data = reg_file.read(rs1_idx);
            uint32_t rs2_data = reg_file.read(rs2_idx);
            uint32_t alu_result = alu_unit.get_alu_result(rs1_data,rs2_data,funct3,funct7_bit5);
            reg_file.write(rd_idx,alu_result);
        }

        void handle_alu_imm_instr(uint8_t rs1_idx, uint32_t immediate_data, uint8_t rd_idx, uint8_t funct3, bool funct7_bit5){
            uint32_t rs1_data = reg_file.read(rs1_idx);
            uint32_t alu_result = alu_unit.get_alu_result(rs1_data,immediate_data,funct3,funct7_bit5);
            reg_file.write(rd_idx,alu_result);
        }

        void handle_load_instr(uint32_t rs1_idx, uint32_t immediate_data, uint8_t rd_idx, uint8_t funct3){
            uint32_t mem_data = 0;

            uint32_t rs1_data = reg_file.read(rs1_idx);
            uint32_t address = rs1_data + immediate_data;

            switch (funct3){
                case  0: //LB
                    mem_data = data_memory.load<uint8_t>(address);
                    mem_data = ( (mem_data & 0x80)==0 ) ? mem_data : mem_data | 0xFFFFFF00; //sign extend
                case  1: //LH
                    mem_data = data_memory.load<uint16_t>(address);
                    mem_data = ( (mem_data & 0x8000)==0 ) ? mem_data : mem_data | 0xFFFF0000; //sign extend
                case  2: //LW
                    mem_data = data_memory.load<uint32_t>(address);
                case  4: //LBU
                    mem_data = data_memory.load<uint8_t>(address);
                case  5: //LHU
                    mem_data = data_memory.load<uint16_t>(address);
                default:
                    //throw error;
            }
            
            reg_file.write(rd_idx,mem_data);
        }

        void handle_store_instr(uint32_t rs1_idx, uint32_t immediate_data, uint8_t rs2_idx, uint8_t funct3){
            uint32_t rs2_data = reg_file.read(rs2_idx);
            uint32_t rs1_data = reg_file.read(rs1_idx);

            uint32_t address = rs1_data + immediate_data;

            switch (funct3){
                case  0: 
                    data_memory.store<uint8_t>(address,(uint8_t)rs2_data);
                case  1: 
                    data_memory.store<uint16_t>(address,(uint16_t)rs2_data);
                case  2: 
                    data_memory.store<uint32_t>(address,rs2_data);
                default:
                    //throw error;
            }
        }

        uint32_t instruction_fetch(){
            uint32_t instruction = instruction_memory.load<uint32_t>(PC);
            PC = PC + 4;
            return instruction;
        }

        void instruction_decode_and_execute(uint32_t instruction){
            uint8_t opcode = instruction && 0x7F;
            uint8_t funct3 = (instruction && 0x7000) >> 12;
            bool funct7_bit5 = ((instruction && 0x40000000) >> 30) == 1;
            uint8_t rs1_idx = (instruction && 0xF8000) >> 15;
            uint8_t rs2_idx = (instruction && 0x1F00000) >> 20;
            uint8_t rd_idx = (instruction && 0xF80) >> 7;

            uint32_t immediate_data;

            switch (opcode) {
                case 0b01100011: //branch               
                    uint32_t sign_extend = ( ( (instruction & 0x80000000)>>31==1 ) ? 0xFFFFF000 : 0x00000000 );
                    immediate_data = sign_extend | ((instruction & 0x7E000000)>>20) | ((instruction & 0xF00)>>7) | ((instruction & 0x80)<<4);
                    handle_branch_instr(rs1_idx,rs2_idx,funct3,PC,immediate_data);

                case 0b00010011: //ALU immediate
                    uint32_t sign_extend_normal = ( ((instruction & 0x80000000) >> 31 == 1) ? 0xFFFFF800 : 0x00000000 );

                                            //SLLI SRLI SRAI
                    immediate_data = (funct3==001 || funct3==101) ? (instruction & 0x01F00000) >> 20  : sign_extend_normal | ((instruction & 0xEFF00000) >> 20);
                    handle_alu_imm_instr(rs1_idx,immediate_data,rd_idx,funct3,funct7_bit5);

                case 0b00000011: //load                 
                    uint32_t sign_extend = ( ((instruction & 0x80000000) >> 31 == 1) ? 0xFFFFF800 : 0x00000000);
                    immediate_data = sign_extend | ((instruction & 0xEFF00000)>>20);
                    handle_load_instr(rs1_idx,immediate_data,rd_idx,funct3);

                case 0b00100011: //store    
                    uint32_t sign_extend = ( ((instruction & 0x80000000) >> 31 == 1) ? 0xFFFFF800 : 0x00000000);
                    immediate_data = sign_extend | ((instruction & 0xEE000000)>>20) | ((instruction & 0xF80)>>7);
                    handle_store_instr(rs1_idx,immediate_data,rs2_idx,funct3);
                    
                case 0b00110011: //ALU RR
                    handle_alu_instr(rs1_idx,rs2_idx,rd_idx,funct3,funct7_bit5);
                default:
            }
        }

    public:
        void issue_and_execute_instruction(){
            uint32_t instruction = instruction_fetch();
            instruction_decode_and_execute(instruction);
        }
};





