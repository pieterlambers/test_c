#pragma once
#include <string>

bool UT_CheckInRange(int value, int center, int tolerance, const std::string& text);
bool UT_CheckTrue(const std::string& text, bool condition);