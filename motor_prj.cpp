#include <iostream>
#include "unit_test.h"
#include "motor_simulation.h"
#include "motor_simulation_test.h"
#include "motor_control_test.h"


int main() 
{


    Run_MotorSimulation_Tests();
    Run_MotorControl_Tests();

    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}



