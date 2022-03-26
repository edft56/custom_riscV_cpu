#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vint_div_32.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>
#include <random>

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
    uint64_t test_times = (uint64_t)(1000000)*34;

    int32_t x_sim;
    int32_t y_sim;
    int32_t quotient=0;
    int32_t remainder=0;

    std::random_device rseed;
    std::mt19937 rng(rseed());
    std::uniform_int_distribution<int> dist(-pow(2,31),pow(2,31)-1);

    for(uint64_t i=0; i<test_times; i++){
        top->clk_i = 0;

        

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);

        top->clk_i = 1;
        top->eval(); 
        if (trace) tfp->dump(time);

        if(i%34==0){
            bool signed_comp = rand()%2==0;
            x_sim = (signed_comp) ? dist(rng) : std::labs(dist(rng));
            y_sim = (signed_comp) ? dist(rng) : std::labs(dist(rng));
            top-> dividend_i = (int32_t)x_sim;
            top-> divisor_i = (int32_t)y_sim;
            top-> signed_i = signed_comp;
            top-> load_i = (i%34==0) ? 1 : 0;
        
            if( (i>34) && ( (int32_t)top->quotient_o != quotient || (int32_t)top->remainder_o!= remainder || !(bool)top->result_rdy ) ) {correct = false; break;}

            //if(i<100*34)std::cout<<x_sim<<" "<<y_sim<<" "<<quotient<<" "<<remainder<<"  "<<(int32_t)top->quotient_o<<"  "<<(int32_t)top->remainder_o<<"\n";
            quotient = x_sim/y_sim;
            remainder = x_sim%y_sim;
        }

        if (trace) time++;
    }
    //std::cout<<x_sim<<" "<<y_sim<<" "<<quotient<<" "<<remainder<<"  "<<(int32_t)top->quotient_o<<"  "<<(int32_t)top->remainder_o<<"\n";


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
