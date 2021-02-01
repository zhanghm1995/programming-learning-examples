/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年12月14日
* Copyright    :
* Descriptoin  : Useful performance compuation time statistics for functions and operations 
* References   : Adapted from https://github.com/ApolloAuto/apollo/blob/master/modules/common/util/perf_util.h
======================================================================*/
#pragma once

#include <string>
#include <iostream>
#include <chrono>

#if defined(__GNUC__) || defined(__GNUG__)
#define AFUNC __PRETTY_FUNCTION__
#elif defined(__clang__)
#define AFUNC __PRETTY_FUNCTION__
#else
#define AFUNC __func__
#endif

// How to Use:
// 1)  Use PERF_FUNCTION to compute time cost of function execution as follows:
//      void MyFunc() {
//          PERF_FUNCION();
//          // do somethings.
//      }
//  Console log:
//  >>>>>>>>>>>>>>>>>>
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER MyFunc elapsed time: 100 ms
//  >>>>>>>>>>>>>>>>>>
//
//  2) Use PERF_BLOCK_START/END to compute time cost of block execution.
//      void MyFunc() {
//          // xxx1
//          PERF_BLOCK_START();
//
//          // do xx2
//
//          PERF_BLOCK_END("xx2");
//
//          // do xx3
//
//          PERF_BLOCK_END("xx3");
//      }
//
//  Console log:
//  >>>>>>>>>>>>>>>
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER xx2 elapsed time: 100 ms
//  I0615 15:49:30.756429 12748 timer.cpp:31] TIMER xx3 elapsed time: 200 ms
//  >>>>>>>>>>>>>>>
namespace cpp_common {
namespace util {

std::string function_signature(const std::string& func_name,
                               const std::string& indicator = "");
#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
  static void CleanUp() {                                                 \
    auto instance = Instance(false);                                      \
    if (instance != nullptr) {                                            \
      CallShutdown(instance);                                             \
    }                                                                     \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)

class Timer {
public:
  Timer() = default;

  explicit Timer(const std::string& comment) {
    comment_ = comment;
    tic();
  }

  void Start(const std::string& comment="") {
    comment_ = comment;
    tic();
  }

  double End(const std::string& msg, bool verbose = true) {
    double ret = toc();
    if (verbose) {
    //   printf("[TIMER] %s elapsed time is %.4fms\n", comment_.c_str(), ret);
      std::cout << "TIMER " << msg << " elapsed_time: " << ret << " ms"<<std::endl;
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

  DISALLOW_COPY_AND_ASSIGN(Timer);
};

class TimerWrapper {
 public:
  explicit TimerWrapper(const std::string& msg) : msg_(msg) { timer_.Start(); }

  ~TimerWrapper() { timer_.End(msg_); }

 private:
  Timer timer_;
  std::string msg_;

  DISALLOW_COPY_AND_ASSIGN(TimerWrapper);
};

}  // namespace util
}  // namespace cpp_common

#define ENABLE_PERF

#define UNUSED(param) (void)param

#if defined(ENABLE_PERF)
#define PERF_FUNCTION()                               \
  cpp_common::util::TimerWrapper _timer_wrapper_( \
      cpp_common::util::function_signature(AFUNC))
#define PERF_FUNCTION_WITH_NAME(func_name) \
  cpp_common::util::TimerWrapper _timer_wrapper_(func_name)
#define PERF_FUNCTION_WITH_INDICATOR(indicator)       \
  cpp_common::util::TimerWrapper _timer_wrapper_( \
      cpp_common::util::function_signature(AFUNC, indicator))
#define PERF_BLOCK_START()             \
  cpp_common::util::Timer _timer_; \
  _timer_.Start()
#define PERF_BLOCK_END(msg) _timer_.End(msg)
#define PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  _timer_.End(indicator+"_"+msg)
#else
#define PERF_FUNCTION()
#define PERF_FUNCTION_WITH_NAME(func_name) UNUSED(func_name);
#define PERF_FUNCTION_WITH_INDICATOR(indicator) UNUSED(indicator);
#define PERF_BLOCK_START()
#define PERF_BLOCK_END(msg) UNUSED(msg);
#define PERF_BLOCK_END_WITH_INDICATOR(indicator, msg) \
  {                                                   \
    UNUSED(indicator);                                \
    UNUSED(msg);                                      \
  }
#endif  // ENABLE_PERF
