#include <iostream>
#include "utils.h"

void printIfInRange(int value, int min, int max, const std::string& text) {
    if (value >= min && value <= max) {
        std::cout << "in range: " << text << std::endl;
    }
}

void checkAndPrint(const std::string& text, bool condition) {
    if (condition) {
        std::cout << "Check passed: " << text << std::endl;
    } else {
        std::cout << "Check failed: " << text << std::endl;
    }
}