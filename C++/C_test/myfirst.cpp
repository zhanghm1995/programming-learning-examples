/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月15日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include "myfirst.h"
#include <iostream>
#include <typeinfo>

template <typename T> void
print_typeof (const T& x)
{
  std::cout<<typeid(x).name()<<std::endl;
}

template void print_typeof<double>(const double&);
template void print_typeof<int>(const int&);
