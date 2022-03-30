#include <verilated.h>          // Defines common routines
#include <iostream>             // Need std::cout
#include "Vriscv_custom.h"               // From Verilating "top.v"
#include "verilated_vcd_c.h"

#include "../riscv_assembler/riscv_assembler.h"

#include <cmath>
#include <bitset>

const bool trace = true;

void simulate_DUT(){
    Vriscv_custom* top = new Vriscv_custom;

    vluint64_t sim_time = 70;

    
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    if (trace){
        top->trace(tfp, 99);  // Trace 99 levels of hierarchy
        tfp->open("top_sim.vcd");
    }


    vluint64_t time = 0;
    top->clk = 0;
    top->reset = 0; //active low
    
    for(uint i=0; i<sim_time; i++){ //
        
        top->clk = 0;
        top->reset = (time>1) ? 1 : 0; 

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time*2);

        top->clk = 1;

        top->eval();            // Evaluate model
        if (trace) tfp->dump(time*2 + 1);
        
        time++;
    }

    
    top->final();               // Done simulating
    
    delete top;

    if (trace)tfp->close();
}



int main() {
    //assemble("test_assembly2.txt","ass_bin.dat",true,false);
    assemble("test_M.txt","ass_bin.dat",true,false);
    simulate_DUT();    

    return 0;
}