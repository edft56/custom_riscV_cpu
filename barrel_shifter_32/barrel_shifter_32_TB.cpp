#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vtop.h"      // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include <cmath>
#include <bitset>

constexpr bool trace = false;


int main(int argc, char** argv, char** env) {
    Vtop* top = new Vtop;

    vluint64_t time = 0;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }


    //top->CLK = 0;
    bool correct = true;
    int32_t test_times = 10000000;

    srand(3);

    for(int i=0; i<test_times; i++){
        //top->CLK = 0;
        

        uint32_t x_sim = uint32_t( rand() % uint(pow(2,32)) );
        uint32_t shift_sim = uint32_t( rand() % uint(pow(2,5)) );
        bool left = bool( rand() % 2);
        bool arith;

        if(!left) {
            arith = bool( rand() % 2);
        }
        else{
            arith = false;
        }
        
        top-> x = x_sim;
        top-> shift_by = shift_sim;
        top-> left = left;
        top-> arith = arith;

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time);
        
        uint32_t result;
        if (left){
            result = (uint32_t)(x_sim << shift_sim);
        }
        else{
            if(arith){
                result = (uint32_t)(int32_t(x_sim) >> shift_sim);
            }
            else{
                result = x_sim >> shift_sim;
            }
            
        }


        if( result != uint32_t(top->out) ) {
            correct = false; 
            std::cout<<i<<" "<<x_sim<<" "<<shift_sim<<" "<<left<<" "<<arith<<" "<<result<<" "<<uint32_t(top->out)<<"\n";
            break;
        }

        if (trace) time++;
    }

    if(!correct) std::cout<<"Not Correct\n";
    else std::cout<<"Correct\n";

    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();

    return 0;
}

