#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vmul32x32.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>

constexpr bool trace = false;


int main(int argc, char** argv, char** env) {
    Vmul32x32* top = new Vmul32x32;

    vluint64_t time = 0;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }


    //top->CLK = 0;
    bool correct = true;
    uint32_t test_times = 20000000;

    for(int i=0; i<test_times; i++){
        //top->CLK = 0;

        uint32_t x_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
        uint32_t y_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
        
        top-> X = x_sim;
        top-> Y = y_sim;

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);
        
        uint64_t result;
        result = x_sim * y_sim;

        std::cout<<x_sim<<" "<<y_sim<<" "<<std::bitset<64>(result)<<"  "<<std::bitset<64>(uint64_t(top->Result))<<"\n";

        if( result != uint64_t(top->Result) ) {correct = false; break;}

        if (trace) time++;
    }

    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();

    return 0;
}

