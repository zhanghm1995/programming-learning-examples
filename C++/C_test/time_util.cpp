/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月13日
* Copyright    :
* Descriptoin  : The time related operation in C++
* References   :
======================================================================*/

#include <sys/time.h>
#include <iostream>
#include <chrono>

/**
 * @brief Class to calculate the elapsed time of some process
 **/
class Timer {
 public:
  Timer() { tic(); }
  void tic() { _start = std::chrono::system_clock::now(); }

  double toc(bool reset = false) {
    auto time = std::chrono::system_clock::now();
    std::chrono::duration<double, std::milli> dur = time - _start;
    if (reset) {
      tic();
    }
    return dur.count();
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> _start;
};

void TestElapsedTime() {
  uint32_t cnt1 = 10e8;
  uint32_t sum = 0;
  Timer timer;
  for (uint32_t i = 0; i < cnt1; ++i) {
    sum += i;
  }
  std::cout<<"Elapsed time is: "<<timer.toc()<<"ms"<<std::endl;
}

/**
 * @brief Get the microsecond of current time
 **/
double GetCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const double timestamp =
      static_cast<double>(tv.tv_sec * 1000000 + tv.tv_usec);
  return timestamp / 1000000;
}

/**
 * @brief Get date time according to specified format
 * @param format Time represenstation format
 */ 
std::string GetDateTime(const std::string& format) {
  time_t curr_time;
  time(&curr_time);
  tm* curr_tm = localtime(&curr_time);

  char buffer[50];
  std::strftime(buffer, sizeof(buffer), format.c_str(), curr_tm);
  return buffer;
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