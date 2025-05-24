#include <iostream>
#include "utils.h"

int add(int a, int b) {
    return a + b;
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    printIfInRange(5, 1, 10, "Five is in range 1-10");
    checkAndPrint("Addition check", add(2, 3) == 5);

    return 0;
}



