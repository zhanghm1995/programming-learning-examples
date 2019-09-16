/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年1月10日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <iostream>
#include "myfirst.h"
//#include "myfirst.cpp"

template <typename T>
class Base
{
public:
  void print(){
    printf("hhh in base\n");
  }
};

template <typename T>
class Derived : public Base<T>
{
public:
  void foo() {
    Base<T>::print();
  }
};


int main()
{
  double ice = 3.0;
  int c = 3;
  print_typeof(c);
  return 0;
}
