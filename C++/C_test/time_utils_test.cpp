/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月13日
* Copyright    :
* Descriptoin  : The time related operation in C++
* References   :
======================================================================*/

#include "time_utils.h"


void TestElapsedTime() {
  uint32_t cnt1 = 10e8;
  uint32_t sum = 0;
  Timer timer;
  timer.Start("Elapsed time test");
  for (uint32_t i = 0; i < cnt1; ++i) {
    sum += i;
  }
  timer.Elapsed();
}

int main(int argc, char **argv) {
  std::cout<<GetCurrentTime()<<std::endl;

  std::cout<<"================"<<std::endl;
  TestElapsedTime();

  std::cout<<"=================="<<std::endl;
  std::string format("%Y-%m-%d-%H-%M-%S");
  std::string date_time = GetDateTime(format);
  std::cout<<date_time<<std::endl;

  return 0;
}