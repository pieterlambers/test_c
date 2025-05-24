#include "unit_test.h"
#include <iostream>
#include <fstream>

static std::ofstream ut_log_file;

void UT_SetLogFile(const std::string& filename) {
    if (ut_log_file.is_open()) {
        ut_log_file.close();
    }
    ut_log_file.open(filename, std::ios::out | std::ios::app);
}

static void ut_log(const std::string& msg) {
    std::cout << msg;
    if (ut_log_file.is_open()) {
        ut_log_file << msg;
        ut_log_file.flush();
    }
}

bool UT_CheckInRange_impl(int value, int center, int tolerance, const std::string& text, const char* file, int line) {
    bool passed = (value >= center - tolerance && value <= center + tolerance);
    std::string msg = std::string(file) + ":" + std::to_string(line) + " | " + text +
        ": value = " + std::to_string(value) + ", center = " + std::to_string(center) +
        (passed ? " [PASS]\n" : " [FAIL]\n");
    ut_log(msg);
    return passed;
}

bool UT_CheckTrue_impl(const std::string& text, bool condition, const char* file, int line) {
    std::string msg = std::string(file) + ":" + std::to_string(line) + " | " + text +
        (condition ? " [PASS]\n" : " [FAIL]\n");
    ut_log(msg);
    return condition;
}