//
// Created by root on 2025/12/27.
//

#ifndef TESTSPLICING_MYTEST_H
#define TESTSPLICING_MYTEST_H
#include <functional>
#include <string>

namespace lizi {
    using func_type = std::function<void(int)>;

    class MyTest {
    public:
        explicit MyTest(int age_);

        MyTest();

        MyTest(const MyTest&) = delete;

        MyTest(MyTest&&) noexcept ;

        virtual ~MyTest();

        /**
         * Assignment operator
         */
        MyTest& operator=(const MyTest&) = delete;

        MyTest& operator=(MyTest&& my_test) noexcept ;

        void operator()(const std::string& s) const;


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
        int age = 0;
    };

    class MyTestChild : public MyTest {
    public:
        explicit MyTestChild(int num_);

        MyTestChild() = default;

        ~MyTestChild() override;

        void printNum() const;
    private:
        int num = 0;
    };

    void test(int);

    /**
     * Use function pointer as parameter
     */
    void funcTest(const func_type&, int);
} // lizi

#endif //TESTSPLICING_MYTEST_H
