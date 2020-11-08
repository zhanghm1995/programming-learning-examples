/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年11月8日
* Copyright    :
* Descriptoin  : Learn how to logging in C++
* References   :
======================================================================*/

// https://github.com/amrayn/easyloggingpp
// https://github.com/google/glog

#include <iostream>
#undef NDEBUG
using namespace std;

int main(int argc, char* argv[])
{

#ifdef NDEBUG
    printf("define NDEBUG\n");
#else
    printf("not define NDEBUG\n");
#endif

    return 0;
}