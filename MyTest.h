//
// Created by root on 2025/12/27.
//

#ifndef TESTSPLICING_MYTEST_H
#define TESTSPLICING_MYTEST_H
#include <functional>

namespace lizi {
    using func_type = std::function<void(int)>;

    class MyTest {
    public:
        MyTest(const int age_);

        MyTest() = default;

        MyTest(const MyTest&) = delete;

        ~MyTest() = default;

        /**
         * Assignment operator
         */
        MyTest& operator=(const MyTest&) = delete;


        /**
         * Use constant object and its constant function
         */
        void printAge() const;

        /**
         * These two member functions with the same name can be overloaded.
         * For constant object, constant function takes precedence,
         * and for regular object, regular function takes precedence.
         */
        void printAge();

    private:
        int age_ = 0;
    };

    void test(int);

    /**
     * Use function pointer as parameter
     */
    void funcTest(func_type, int);
} // lizi

#endif //TESTSPLICING_MYTEST_H
