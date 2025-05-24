#include <iostream>
#include "unit_test.h"
#include "motor_simulation.h"
#include "motor_simulation_test.h"

int add(int a, int b) {
    return a + b;
}

int main() {
    // Set log file for test results
    UT_SetLogFile("testresults.txt");

    std::cout << "Hello, World!" << std::endl;

    UT_CheckInRange(5, 1, 10, "Five is in range 1-10");
    UT_CheckTrue("Addition check", add(2, 3) == 5);

    Run_MotorTests();

    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}



