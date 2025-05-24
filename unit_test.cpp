#include "unit_test.h"
#include <iostream>

bool UT_CheckInRange_impl(int value, int center, int tolerance, const std::string& text, const char* file, int line) {
    bool passed = (value >= center - tolerance && value <= center + tolerance);
    std::cout << file << ":" << line << " | " << text << ": value = " << value << ", center = " << center;
    if (passed) {
        std::cout << " [PASS]" << std::endl;
    } else {
        std::cout << " [FAIL]" << std::endl;
    }
    return passed;
}

bool UT_CheckTrue_impl(const std::string& text, bool condition, const char* file, int line) {
    std::cout << file << ":" << line << " | " << text;
    if (condition) {
        std::cout << " [PASS]" << std::endl;
    } else {
        std::cout << " [FAIL]" << std::endl;
    }
    return condition;
}