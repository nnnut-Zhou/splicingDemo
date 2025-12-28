//
// Created by root on 2025/12/27.
//

#include "MyTest.h"

#include <iostream>
#include <ostream>

namespace lizi {
    MyTest::MyTest(const int age_) {
        std::cout << "mytest is created." << std::endl;
        this->age_ = age_;
    }

    void MyTest::printAge() const {
        std::cout << "age is " << age_ << std::endl;
    }

    void MyTest::printAge() {
        age_++;
        std::cout << "age is " << age_ << std::endl;
    }

    void test(int a) {
        std::cout << "Hello World" << std::endl
                << a << std::endl;
    }

    void funcTest(func_type func, int n) {
        func(n);
    }
} // lizi
