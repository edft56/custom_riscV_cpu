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
    uint32_t test_times = 20000000;

    for(int i=0; i<test_times; i++){
        //top->CLK = 0;

        int32_t x_sim = int32_t( rand() % uint64_t(pow(2,32)) );
        int32_t y_sim = int32_t( rand() % uint64_t(pow(2,32)) );
        bool sub_sim = bool(i % (test_times/2));
        
        top-> x = x_sim;
        top-> y = y_sim;
        top-> sub = sub_sim;

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);
        
        int32_t result;
        if (sub_sim){
            result = x_sim - y_sim;
        }
        else{
            result = x_sim + y_sim;
        }

        //if(i<100) std::cout<<x_sim<<" "<<y_sim<<" "<<result<<"  "<<int32_t(top->sum)<<"\n";

        if( result != int32_t(top->sum) ) {correct = false; break;}

        if (trace) time++;
    }

    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();

    return 0;
}

