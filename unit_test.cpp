#include "unit_test.h"
#include <iostream>

bool UT_CheckInRange(int value, int center, int tolerance, const std::string& text) {
    bool passed = (value >= center - tolerance && value <= center + tolerance);
    std::cout << text << ": value = " << value << ", center = " << center;
    if (passed) {
        std::cout << " [PASS]" << std::endl;
    } else {
        std::cout << " [FAIL]" << std::endl;
    }
    return passed;
}

bool UT_CheckTrue(const std::string& text, bool condition) {
    if (condition) {
        std::cout << text << " passed" << std::endl;
    } else {
        std::cout << text << " failed" << std::endl;
    }
    return condition;
}