/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年8月8日
* Copyright    :
* Descriptoin  : Learning how to use Exception in C++
* References   :
======================================================================*/

#include <iostream>
// #undef NDEBUG
#include <cassert>

using std::cout;
using std::endl;

int main() {
    int a = 1;
    int b = 0;
    int c;

    try {
        // c = a / b;
        throw std::logic_error("fffffffffffffff");
    } catch(...) {
        cout<<"Not valid"<<endl;
    }

    assert(a == 10);
    
    // cout<<c<<endl;
}