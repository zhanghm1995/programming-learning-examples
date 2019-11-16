/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年8月12日
* Copyright    :
* Descriptoin  : Learn how to use different function definition in C++
* References   :
======================================================================*/

#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <functional>
#include <memory>

using namespace std;

//声明一个模板
typedef std::function<int(int)> Functional;


//normal function
int TestFunc(int a)
{
    return a;
}

//lambda expression
auto lambda = [](int a)->int{return a;};

//functor仿函数
class Functor
{
public:
    int operator() (int a)
    {
        return a;
    }
};


//类的成员函数和类的静态成员函数
class CTest
{
public:
    int Func(int a)
    {
        return a;
    }
    static int SFunc(int a)
    {
        return a;
    }
};


int main(int argc, char* argv[])
{
    //封装普通函数
    Functional obj = TestFunc;
    int res = obj(0);
    cout << "normal function : " << res << endl;

    //封装lambda表达式
    obj = lambda;
    res = obj(1);
    cout << "lambda expression : " << res << endl;

    //封装仿函数
    Functor functorObj;
    obj = functorObj;
    res = obj(2);
    cout << "functor : " << res << endl;

    //封装类的成员函数和static成员函数
    CTest t;
    obj = std::bind(&CTest::Func, &t, std::placeholders::_1);
    res = obj(3);
    cout << "member function : " << res << endl;

    obj = CTest::SFunc;
    res = obj(4);
    cout << "static member function : " << res << endl;

    return 0;
}
