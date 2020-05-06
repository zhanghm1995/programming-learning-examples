/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-23 21:29:44
 * @References: 
 * @Description: Time related operations in C++
 */

#include <sys/time.h>
#include <iostream>
#include <chrono>
#include <string>

/**
 * @brief Class to calculate the elapsed time of some process
 *                Two work pattern, and you can control whether cout or not
 **/
class Timer {
public:
  Timer() = default;
  explicit Timer(const std::string& comment) {
    comment_ = comment;
    tic();
  }

  void Start(const std::string& comment) {
    comment_ = comment;
    tic();
  }

  double Elapsed(bool verbose = true) {
    double ret = toc();
    if (verbose) {
      printf("[TIMER] %s elapsed time is %.4fms\n", comment_.c_str(), ret);
    }
    return ret;
  }

private:
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
  std::string comment_;
  std::chrono::time_point<std::chrono::system_clock> _start;
};

/**
 * @brief Get the microsecond of current time
 **/
inline double GetCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  const double timestamp =
      static_cast<double>(tv.tv_sec * 1000000 + tv.tv_usec);
  return timestamp / 1000000;
}

/**
 * @brief Get date time according to specified format
 * @param format Time representation format
 */ 
inline std::string GetDateTime(const std::string& format) {
  time_t curr_time;
  time(&curr_time);
  tm* curr_tm = localtime(&curr_time);

  char buffer[50];
  std::strftime(buffer, sizeof(buffer), format.c_str(), curr_tm);
  return buffer;
}
