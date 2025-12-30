#include <iostream>

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
    test2("hello test2");
}

void test3() {
    const lizi::MyTest* p_test = new lizi::MyTestChild(5);
    p_test->printAge();

    std::cout << typeid(*p_test).name() << std::endl;
    auto test_child = dynamic_cast<const lizi::MyTestChild*>(p_test);
    std::cout << typeid(*test_child).name() << std::endl;

    delete p_test, test_child;
}

void test4() {

}

int main() {
    test4();

    return 0;
}
