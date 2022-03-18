#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
//#include "Vmul32x32.h"
#include "Vmul32x32_pipelined.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>

constexpr bool trace = false;

// void test_non_pipelined(){
//     Vmul32x32* top = new Vmul32x32;

//     vluint64_t time = 0;

//     Verilated::traceEverOn(true);
//     VerilatedVcdC* tfp = new VerilatedVcdC;
//     if (trace){
//         top->trace(tfp, 99);  // Trace 99 levels of hierarchy
//         tfp->open("top_sim.vcd");
//     }

//     //top->CLK = 0;
//     bool correct = true;
//     uint32_t test_times = 20000000;

//     for(int i=0; i<test_times; i++){
//         //top->CLK = 0;

//         uint64_t x_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
//         uint64_t y_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
        
//         top-> X = (uint32_t)x_sim;
//         top-> Y = (uint32_t)y_sim;

//         top->eval();            // Evaluate model
//         if (trace) tfp->dump(time);
        
//         uint64_t result;
//         result = x_sim * y_sim;

//         //if(i<10)std::cout<<x_sim<<" "<<y_sim<<" "<<result<<" "<<std::bitset<64>(result)<<"  "<<std::bitset<64>(uint64_t(top->Result))<<"\n";

//         if( result != uint64_t(top->Result) ) {correct = false; break;}

//         if (trace) time++;
//     }


//     if(!correct) std::cout<<"Not Correct\n";
//     else std::cout<<"Correct\n";

//     top->final();               // Done simulating
    
//     delete top;

//     if (trace)tfp->close();

//     return correct
// }

void test_pipelined(){
    Vmul32x32_pipelined* top = new Vmul32x32_pipelined;

    vluint64_t time = 0;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }

    bool correct = true;
    uint32_t test_times = 100000;

    uint64_t result[5] = {0};

    for(int i=0; i<test_times; i++){
        top->clk = 0;

        uint64_t x_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
        uint64_t y_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
        
        top-> X = (uint32_t)x_sim;
        top-> Y = (uint32_t)y_sim;

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);

        top->clk = 1;
        top->eval(); 
        if (trace) tfp->dump(time);
        
        
        for(int j=0; j<4; j++){
            result[4-j] = result[4-j-1];
        }
        result[0] = x_sim * y_sim;

        if(i<10) std::cout<<x_sim<<" "<<y_sim<<" "<<result[4]<<" "<<std::bitset<64>(result[4])<<"  "<<std::bitset<64>(uint64_t(top->Result))<<"\n";

        if( i>3 && result[4] != uint64_t(top->Result) ) {correct = false; break;}

        if (trace) time++;
    }


    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();
}

int main(int argc, char** argv, char** env) {
    //test_non_pipelined();
    test_pipelined();

    return 0;
}

