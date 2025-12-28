#include <iostream>
#include <vector>

#include "MyTest.h"


void test1() {
    lizi::funcTest([](const int x) {
        std::cout << x << std::endl;
    }, 100);
}

void test2() {
    const lizi::MyTest* test = new lizi::MyTest();
    test->printAge();
    delete test;

    const lizi::MyTest test2(5);
    test2.printAge();
}

int main() {
    test2();


    return 0;
}
