#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vcla_adder_32.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>

constexpr bool trace = false;


int main(int argc, char** argv, char** env) {
    Vcla_adder_32* top = new Vcla_adder_32;

    vluint64_t time = 0;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }


    //top->CLK = 0;
    bool correct = true;
    uint64_t test_times = 100000000;

    for(uint64_t i=0; i<test_times; i++){
        //top->CLK = 0;

        int64_t x_sim = (int64_t)( (int64_t)rand() % (uint64_t)pow(2,32) );
        int64_t y_sim = (int64_t)( (int64_t)rand() % (uint64_t)pow(2,32) );
        bool sub_sim = bool(test_times%2==0);
        
        top-> x = (uint32_t)x_sim;
        top-> y = (uint32_t)y_sim;
        top-> sub = sub_sim;

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);
        
        int64_t result;
        if (sub_sim){
            result = x_sim - y_sim;
        }
        else{
            result = x_sim + y_sim;
        }

        //std::cout<<(((uint32_t)(top->sum) & 0x80000000) != 0)<<"\n";
        
        bool sign_extend = x_sim<0 | y_sim<0 | (sub_sim & y_sim>=0);

        uint64_t verilog_result = (uint64_t)(top->sum);
        if(sign_extend) verilog_result = (uint64_t)( (((uint32_t)(top->sum) & 0x80000000) != 0) ? 0xFFFFFFFF00000000 : 0x0000000000000000) | verilog_result;

        //if(i<100) std::cout<<std::bitset<32>(x_sim)<<" "<<std::bitset<32>(y_sim)<<" "<<" "<<result<<" "<<verilog_result<<" "<<std::bitset<64>(result)<<"  "<<std::bitset<64>(verilog_result)<<"\n";

        if( result != verilog_result ) {correct = false; break;}

        if (trace) time++;
    }
    //00111001000100000010000110100001
    //

    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();

    return 0;
}

