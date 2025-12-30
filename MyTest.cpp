//
// Created by root on 2025/12/27.
//

#include "MyTest.h"

#include <iostream>
#include <ostream>

namespace lizi {
    MyTest::MyTest(const int age_) : age(age_) {
        std::cout << "mytest is created." << std::endl;
    }

    MyTest::MyTest() {
        std::cout << "mytest is created." << std::endl;
    }

    MyTest::~MyTest() {
        std::cout << "mytest is destroyed." << std::endl;
    }

    void MyTest::operator()(const std::string& s) const {
        std::cout << s << std::endl;
    }

    void MyTest::printAge() const {
        std::cout << "age is " << age << std::endl;
    }

    void MyTest::printAge() {
        age++;
        std::cout << "age is " << age << std::endl;
    }

    MyTestChild::MyTestChild(const int num_) : MyTest(3), num(num_) {
        std::cout << "mytestchild is created." << std::endl;
    }

    MyTestChild::~MyTestChild() {
        std::cout << "mytestchild is destroyed." << std::endl;
    }

    void MyTestChild::printNum() const {
        std::cout << num << std::endl;
    }

    void test(const int a) {
        std::cout << "Hello World" << std::endl
                << a << std::endl;
    }

    void funcTest(const func_type& func, const int n) {
        func(n);
    }
} // lizi
