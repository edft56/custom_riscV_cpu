#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vint_div_32.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>

constexpr bool trace = false;



void test(){
    Vint_div_32* top = new Vint_div_32;

    vluint64_t time = 0;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }

    bool correct = true;
    uint64_t test_times = (uint64_t)(100000000)*33;

    uint64_t x_sim;
    uint64_t y_sim;
    uint32_t quotient=0;
    uint32_t remainder=0;

    for(uint64_t i=0; i<test_times; i++){
        top->clk_i = 0;

        

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);

        top->clk_i = 1;
        top->eval(); 
        if (trace) tfp->dump(time);

        if(i%33==0){
            x_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
            y_sim = uint32_t( rand() % uint64_t(pow(2,32)) );
            top-> dividend_i = (uint32_t)x_sim;
            top-> divisor_i = (uint32_t)y_sim;
            top-> load_i = (i%33==0) ? 1 : 0;
        
            if( (uint32_t)top->quotient_o != quotient || (uint32_t)top->remainder_o!= remainder ) {correct = false; break;}
            quotient = x_sim/y_sim;
            remainder = x_sim%y_sim;
            //std::cout<<x_sim<<" "<<y_sim<<" "<<quotient<<" "<<remainder<<"  "<<(uint32_t)top->quotient_o<<"  "<<(uint32_t)top->remainder_o<<"\n";
        }

        

        if (trace) time++;
    }


    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();
}

int main(){
    test();
    return 0;
}
