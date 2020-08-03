// https://stackoverflow.com/questions/16455029/how-to-implement-an-initializer-list-for-user-defined-type-analogus-to-stdve

#include <vector>
#include <iostream>
#include <initializer_list>

using std::cout;
using std::endl;

/**
 * Learn how to use std::initializer_list
 * https://stackoverflow.com/questions/16455029/how-to-implement-an-initializer-list-for-user-defined-type-analogus-to-stdve
 * https://blog.csdn.net/wangshubo1989/article/details/49622871
 */
template <typename T>
class foo {
public:
    foo(std::initializer_list<T> init) 
      : vec(init)
    { }

    void print() {
        for (const auto& v : vec) {
            cout<<v<<endl;
        }
    }

private:
    std::vector<T> vec;
};

int main()
{
    foo<int> f = {1, 2, 3, 4, 5};
    f.print();

    return 0;
}