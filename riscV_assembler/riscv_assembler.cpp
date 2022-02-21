#include <iostream>
#include <string>
#include <map>
#include <sstream>
#include <fstream>
#include <ctype.h>
#include <vector>
#include <bitset>


std::map<std::string, uint32_t> reg_map =   {
                                                { "x0", 0   },
                                                { "ra", 1   },
                                                { "sp", 2   },
                                                { "gp", 3   },
                                                { "tp", 4   },
                                                { "t0", 5   },
                                                { "t1", 6   },
                                                { "t2", 7   },
                                                { "s0", 8   }, //x8
                                                { "fp", 8   }, //x8
                                                { "s1", 9   },
                                                { "a0", 10  },
                                                { "a1", 11  },
                                                { "a2", 12  },
                                                { "a3", 13  },
                                                { "a4", 14  },
                                                { "a5", 15  },
                                                { "a6", 16  },
                                                { "a7", 17  },
                                                { "s2", 18  },
                                                { "s3", 19  },
                                                { "s4", 20  },
                                                { "s5", 21  },
                                                { "s6", 22  },
                                                { "s7", 23  },
                                                { "s8", 24  },
                                                { "s9", 25  },
                                                { "s10", 26 },
                                                { "s11", 27 },
                                                { "t3", 28  },
                                                { "t4", 29  },
                                                { "t5", 30  },
                                                { "t6", 31  }
                                            };

std::map<std::string, uint32_t> instr_map =   {   // ALU RR
                                                { "add"  ,  0b00000000000000000000000000110011 }, 
                                                { "sub"  ,  0b01000000000000000000000000110011 },
                                                { "sll"  ,  0b00000000000000000001000000110011 },
                                                { "slt"  ,  0b00000000000000000010000000110011 },
                                                { "sltu" ,  0b00000000000000000011000000110011 },
                                                { "xor"  ,  0b00000000000000000100000000110011 },
                                                { "srl"  ,  0b00000000000000000101000000110011 },
                                                { "sra"  ,  0b01000000000000000101000000110011 },
                                                { "or"   ,  0b00000000000000000110000000110011 },
                                                { "and"  ,  0b00000000000000000111000000110011 },
                                                // ALU IMM
                                                { "addi" ,  0b00000000000000000000000000010011 },
                                                { "slli" ,  0b00000000000000000001000000010011 },
                                                { "slti" ,  0b00000000000000000010000000010011 },
                                                { "sltiu",  0b00000000000000000011000000010011 },
                                                { "xori" ,  0b00000000000000000100000000010011 },
                                                { "slri" ,  0b00000000000000000101000000010011 },
                                                { "srai" ,  0b01000000000000000101000000010011 },
                                                { "ori"  ,  0b00000000000000000110000000010011 },
                                                { "andi" ,  0b00000000000000000111000000010011 },
                                                // LOADS
                                                { "lb"   ,  0b00000000000000000000000000000011 },
                                                { "lh"   ,  0b00000000000000000001000000000011 },
                                                { "lw"   ,  0b00000000000000000010000000000011 },
                                                { "lbu"  ,  0b00000000000000000100000000000011 },
                                                { "lhu"  ,  0b00000000000000000101000000000011 },
                                                // STORES
                                                { "sw"   ,  0b00000000000000000010000000100011 },
                                                { "sb"   ,  0b00000000000000000000000000100011 },
                                                { "sh"   ,  0b00000000000000000001000000100011 },
                                                // BRANCHES
                                                { "beq"  ,  0b00000000000000000000000001100011 },
                                                { "bne"  ,  0b00000000000000000001000001100011 },
                                                { "blt"  ,  0b00000000000000000100000001100011 },
                                                { "bge"  ,  0b00000000000000000101000001100011 },
                                                { "bltu" ,  0b00000000000000000110000001100011 },
                                                { "bgeu" ,  0b00000000000000000111000001100011 }
                                            };

std::vector<std::string> R_instr = {
		"add","sub", "sll", 
		"sltu", "xor", "srl", 
		"sra", "or", "and",
		"addw", "subw", "sllw",
		"slrw", "sraw", "mul",
		"mulh", "mulu", "mulsu",
		"div", "divu", "rem",
		"remu"
        };

std::vector<std::string> I_instr = {
		"addi", "lb", "lw",
		"ld", "lbu", "lhu",
		"lwu", "fence", "fence.i", 
		"slli", "slti", "sltiu", 
		"xori", "slri", "srai",
		"ori", "andi", "addiw",
		"slliw", "srliw", "sraiw", 
		"jalr", "ecall", "ebreak", 
		"CSRRW", "CSRRS","CSRRC", 
		"CSRRWI", "CSRRSI", "CSRRCI" 
    };

std::vector<std::string> S_instr = {
		"sw", "sb", "sh", 
		"sd"
    };

std::vector<std::string> B_instr = {
		"beq", "bne", "blt", 
		"bge", "bltu", "bgeu"
    };

std::vector<std::string> U_instr = {"auipc", "lui"};

std::vector<std::string> UJ_instr = {"jal"};



int string_sep_elements(std::string string, char* sep_list,int sep_number,std::string* elements){
    bool element_found = false;
    int start_index = 0;
    int element_index = 0;
    int element_length = 0;

    for(int i=0; i<string.length(); i++){
        const char* cur_char = string.substr(i,1).c_str();

        if ( (std::isalpha(*cur_char) || std::isdigit(*cur_char) || *cur_char=='-') && element_found==false ){
            element_found = true;
            start_index = i;
        }

        if(element_found){
            bool sep_found = false;
            for(int k=0; k<sep_number; k++){
                if (*cur_char==sep_list[k]){
                    //std::cout<<*cur_char<<std::endl;
                    element_found=false;
                    elements[element_index] = string.substr(start_index, element_length);
                    element_index++;
                    element_length = 0;
                    sep_found = true;
                    break;
                }
            }
            if(i==string.length()-1 && !sep_found){
                element_found=false;
                elements[element_index] = string.substr(start_index, element_length+1);
                element_index++;
                element_length = 0;
            }
        }

        if(element_found) element_length++;

    }
    return element_index;
}

bool find_string_in_vector(std::vector<std::string>& vector, std::string str){
    for(int i=0; i<vector.size(); i++){
        if(vector[i]==str) return true;
    }
    return false;
}

uint32_t handle_R_type(std::string instruction_name ,std::string rest_of_instr){
    std::string elements_found[rest_of_instr.length()];

    char sep_list[3] = {' ', ',','\n'};

    int element_number = string_sep_elements(rest_of_instr,sep_list,3,elements_found);

    // for(int i=0; i<3; i++){
    //     std::cout<<elements_found[i]<<std::endl;
    // }

    if (element_number>3) {std::cout<<"Unknown ALU Imm Instruction"<<'\n'; exit(1);}
    else{
        uint32_t rd_idx = reg_map.at(elements_found[0]);
        uint32_t rs1_idx = reg_map.at(elements_found[1]);
        uint32_t rs2_idx = reg_map.at(elements_found[2]);

        uint32_t bin_instr = instr_map.at(instruction_name);

        return ( (bin_instr | (rd_idx << 7)) | (rs1_idx << 15) ) | (rs2_idx << 20);
    }
}

uint32_t handle_load(std::string instruction_name ,std::string rest_of_instr){
    std::string elements_found[rest_of_instr.length()];

    char sep_list[5] = {' ', ',','(',')','\n'};

    int element_number = string_sep_elements(rest_of_instr,sep_list,5,elements_found);

    // for(int i=0; i<3; i++){
    //     std::cout<<elements_found[i]<<std::endl;
    // }

    if (element_number>3) {std::cout<<"Unknown Load Instruction"<<'\n'; exit(1);}
    else{
        uint32_t rd_idx = reg_map.at(elements_found[0]);
        uint32_t rs1_idx = reg_map.at(elements_found[2]);
        int offset = stoi(elements_found[1]);
        if (abs(offset)>=2048) {std::cout<<"Load offset greater or equal than 2048"<<"\n"; exit(1);}
        uint32_t immediate_data = (*(uint32_t*)(&offset));

        uint32_t bin_instr = instr_map.at(instruction_name);

        return ((bin_instr | (rd_idx << 7)) | (rs1_idx << 15)) | (immediate_data<<20);
    }
    
}

uint32_t handle_S_type(std::string instruction_name ,std::string rest_of_instr){
    std::string elements_found[rest_of_instr.length()];

    char sep_list[5] = {' ', ',','(',')','\n'};

    int element_number = string_sep_elements(rest_of_instr,sep_list,5,elements_found);

    if (element_number>3) {std::cout<<"Unkown Store Instruction"<<'\n'; exit(1);}
    else{
        uint32_t rs2_idx = reg_map.at(elements_found[0]);
        uint32_t rs1_idx = reg_map.at(elements_found[2]);
        int offset = stoi(elements_found[1]);
        if (offset>=2048) {std::cout<<"Store offset greater or equal than 2048"<<"\n"; exit(1);}
        uint32_t immediate_data = (*(uint32_t*)(&offset));

        uint32_t immediate_field_1 = immediate_data & 0x1F;
        uint32_t immediate_field_2 = (immediate_data & 0xFFFFFFE0) >> 5;

        uint32_t bin_instr = instr_map.at(instruction_name);

        return ( ((bin_instr | (immediate_field_1 << 7)) | (rs1_idx << 15)) | (rs2_idx<<20) ) | (immediate_field_2<<25);
    }
    
}

uint32_t handle_alu_imm(std::string instruction_name ,std::string rest_of_instr){
    std::string elements_found[rest_of_instr.length()];

    char sep_list[3] = {' ', ',','\n'};

    int element_number = string_sep_elements(rest_of_instr,sep_list,3,elements_found);

    // for(int i=0; i<3; i++){
    //     std::cout<<elements_found[i]<<std::endl;
    // }

    if (element_number>3) {std::cout<<"Unkown ALU Imm Instruction"<<'\n'; exit(1);}
    else{
        uint32_t rd_idx = reg_map.at(elements_found[0]);
        uint32_t rs1_idx = reg_map.at(elements_found[1]);
        int32_t immediate_data = stoi(elements_found[2]); //signed
        if (abs(immediate_data)>=2048) {std::cout<<"Immediate greater or equal than 2048"<<"\n"; exit(1);}

        uint32_t bin_instr = instr_map.at(instruction_name);

        return ((bin_instr | (rd_idx << 7)) | (rs1_idx << 15)) | (immediate_data<<20);
    }
    
}

uint32_t handle_B_type(std::string instruction_name ,std::string rest_of_instr){
    std::string elements_found[rest_of_instr.length()];

    char sep_list[3] = {' ', ',','\n'};

    int element_number = string_sep_elements(rest_of_instr,sep_list,3,elements_found);

    // for(int i=0; i<3; i++){
    //     std::cout<<elements_found[i]<<std::endl;
    // }

    if (element_number>3) {std::cout<<"Unknown Branch Instruction"<<'\n'; exit(1);}
    else{
        uint32_t rs1_idx = reg_map.at(elements_found[0]);
        uint32_t rs2_idx = reg_map.at(elements_found[1]);
        int32_t immediate_data = stoi(elements_found[2]) * 2; //signed, branch offset is given in instruction forward or bwd so *4 gives bytes
        if (abs(immediate_data*4)>=4096) {std::cout<<"Branch offset greater or equal than 1024"<<"\n"; exit(1);}

        uint32_t immediate_field_1 = (((*(uint32_t*)(&immediate_data)) & 0x800) >> 11) | (((*(uint32_t*)(&immediate_data)) & 0x1E));
        uint32_t immediate_field_2 = (((*(uint32_t*)(&immediate_data)) & 0x1000) >> 6) | (((*(uint32_t*)(&immediate_data)) & 0x7E0) >> 5);
        
        // std::cout<<std::bitset<32>((((*(uint32_t*)(&immediate_data)) & 0x800) >> 11))<<std::endl;
        // std::cout<<std::bitset<32>(((*(uint32_t*)(&immediate_data)) & 0x1E))<<std::endl;
        // std::cout<<std::bitset<32>(immediate_field_1)<<std::endl;
        // std::cout<<std::bitset<32>(immediate_field_2)<<std::endl;

        uint32_t bin_instr = instr_map.at(instruction_name);

        return (((bin_instr | (immediate_field_1 << 7)) | (rs1_idx << 15)) | (rs2_idx<<20)) | (immediate_field_2<<25);
    }
    
}

uint32_t handle_I_type(std::string instruction_name ,std::string rest_of_instr){

    if (instruction_name=="lb" || instruction_name=="lw" || instruction_name=="lb" || instruction_name=="lbu" || instruction_name=="lhu"){
        return handle_load(instruction_name, rest_of_instr); 
    }
    else{
        return handle_alu_imm(instruction_name, rest_of_instr); 
    }
}


void assemble(std::string filename){
    std::ifstream code_file(filename);

    if (!code_file) { std::cout << "Unable to open file"<<"\n"; exit(1);}



    std::string line;
    while (std::getline(code_file, line)){
        if(line.length()==0) continue;
        uint32_t binary_instruction;

        int instr_start =-1;
        int instr_length = 0;
        std::string instruction_name;

        //std::cout<<line<<std::endl;
        for(int i=0; i<line.length(); i++){
            const char* cur_char = line.substr(i,1).c_str();
            //std::cout<<*cur_char<<std::endl;

            if (std::isalpha(*cur_char) && instr_start==-1){
                instr_start = i;
            }
            
            if (!std::isalpha(*cur_char) && instr_start!=-1){
                instruction_name = line.substr(instr_start,instr_length);
                //std::cout<<instruction_name<<" "<<instr_length<<std::endl;
                //std::cout<<line.substr(i,line.length()-instr_length)<<std::endl;
                
                if ( find_string_in_vector(R_instr,instruction_name) ) binary_instruction = handle_R_type(instruction_name,line.substr(i,line.length()-instr_length));
                else if ( find_string_in_vector(I_instr,instruction_name) ) binary_instruction = handle_I_type(instruction_name,line.substr(i,line.length()-instr_length));
                else if ( find_string_in_vector(S_instr,instruction_name) ) binary_instruction = handle_S_type(instruction_name,line.substr(i,line.length()-instr_length));
                else if ( find_string_in_vector(B_instr,instruction_name) ) binary_instruction = handle_B_type(instruction_name,line.substr(i,line.length()-instr_length));
                //else if ( find_string_in_vector(U_instr,instruction_name) ) binary_instruction = handle_U_type(instruction_name,line.substr(i,line.length()));
                //else if ( find_string_in_vector(UJ_instr,instruction_name) ) binary_instruction = handle_UJ_type(instruction_name,line.substr(i,line.length()));
                else {std::cout<<"Unknown instruction"<<"\n"; exit(1);}
                break;
            }
            instr_length = (instr_start!=-1) ? instr_length + 1 : instr_length;
        }

        std::cout<<std::bitset<32>(binary_instruction)<<"\n";

    }
}

int main(){
    assemble("test_assembly.txt");
    return 0;
}

