#pragma once
#include <string>

void UT_SetLogFile(const std::string& filename);

void UT_SetTestNumber(int number);
void UT_TestInfo(const std::string& text);

bool UT_CheckInRange_impl(int value, int center, int tolerance, const std::string& text, const char* file, int line);
bool UT_CheckTrue_impl(const std::string& text, bool condition, const char* file, int line);

#define UT_CheckInRange(value, center, tolerance, text) \
    UT_CheckInRange_impl(value, center, tolerance, text, __FILE__, __LINE__)

#define UT_CheckTrue(text, condition) \
    UT_CheckTrue_impl(text, condition, __FILE__, __LINE__)