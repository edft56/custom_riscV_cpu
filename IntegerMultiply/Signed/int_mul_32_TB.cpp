#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vmul32x32_pipelined.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>
#include <random>

constexpr bool trace = false;



void test_pipelined(){
    Vmul32x32_pipelined* top = new Vmul32x32_pipelined;

    vluint64_t time = 0;

    std::random_device rseed;
    std::mt19937 rng(rseed());
    std::uniform_int_distribution<int> dist(-pow(2,30)-1, pow(2,30)-1);

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }

    bool correct = true;
    uint32_t test_times = 100000;

    int64_t result[5] = {0};

    top-> signed_mul_i = 0;
    top-> X = 0;
    top-> Y = 0;

    for(uint32_t i=0; i<test_times; i++){
        top->clk = 0;
        
        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);
        if (trace) time++;


        bool signed_or_not = i%2==0;
        
        int64_t x_sim = dist(rng);
        int64_t y_sim = dist(rng);
        x_sim = (signed_or_not) ? x_sim : std::labs(x_sim);
        y_sim = (signed_or_not) ? y_sim : std::labs(y_sim);
        
        top-> signed_mul_i = signed_or_not;
        top-> X = (int32_t)x_sim;
        top-> Y = (int32_t)y_sim;


        top->clk = 1;
        top->eval(); 
        if (trace) tfp->dump(time);
        if (trace) time++;
        
        for(int j=0; j<4; j++){
            result[4-j] = result[4-j-1];
        }
        result[0] = x_sim * y_sim;

        //if(i<10) std::cout<<x_sim<<" "<<y_sim<<" "<<result[4]<<" "<<std::bitset<64>(result[4])<<"  "<<std::bitset<64>((uint64_t)top->Result)<<"\n";

        if( bool(top->result_rdy) && (result[4] != int64_t(top->Result))  ) {correct = false; break;}

    }


    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();
}

int main(int argc, char** argv, char** env) {
    test_pipelined();

    return 0;
}
