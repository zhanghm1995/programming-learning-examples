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
#include <sstream>

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

/**
 * @brief 学习如何获得ns级的时间戳
 */ 
void ComputeTime() {
  std::chrono::system_clock::time_point nowTp = std::chrono::system_clock::now();
  std::chrono::system_clock::duration tp = nowTp.time_since_epoch();
  std::cout << "====" << nowTp.time_since_epoch().count() << "ns" << std::endl;
  std::cout<<"----"<<std::chrono::duration_cast<std::chrono::seconds>(tp).count()<<std::endl;
  tp -= std::chrono::duration_cast<std::chrono::seconds>(tp);
  std::cout<<"____"<<tp.count()<<std::endl; // ns部分

  //to_time_t方法把时间点转化为time_t
  time_t tt = std::chrono::system_clock::to_time_t(nowTp);
  //使用ctime转化为字符串
  std::cout << ctime(&tt)<<std::endl;
  //把time_t转化为tm
  tm *localTm =  localtime(&tt);
//  cout << "year=" << localTm->tm_year << endl;
//  cout << "month=" << localTm->tm_mon << endl;
//  cout << "day=" << localTm->tm_mday << endl;
//  cout << "hour=" << localTm->tm_hour << endl;
//  cout << "minute=" << localTm->tm_min << endl;
//  cout << "second=" << localTm->tm_sec << endl;
  
  //tm格式化输出
  char buffer[25];
  char format[] = "%Y-%m-%d %H:%M:%S";
  strftime(buffer, sizeof(buffer), format, localTm);

  char buffer_ns[20];
  sprintf(buffer_ns, "%09ld", tp.count());
  std::stringstream ss;
  ss<<buffer<<"."<<buffer_ns;
  std::cout << "============== "<<ss.str() << std::endl;
}

std::tm* Gettm(int64_t timestamp)
{
    int64_t milli = timestamp+ (int64_t)8*60*60*1000;//此处转化为东八区北京时间，如果是其它时区需要按需求修改
    auto mTime = std::chrono::milliseconds(milli);
    auto tp=std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm* now = std::gmtime(&tt);
    printf("%4d年%02d月%02d日 %02d:%02d:%02d\n",now->tm_year+1900,now->tm_mon+1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
   return now;
}

void TestChrono() {
  using std::cout;
  using std::endl;
  cout << "millisecond : ";
  cout << std::chrono::milliseconds::period::num << "/" << std::chrono::milliseconds::period::den << "s" <<endl;

  cout << "nanosecond : ";
  cout << std::chrono::nanoseconds::period::num << "/" << std::chrono::nanoseconds::period::den << "s" <<endl;

  std::chrono::hours hours(1);
  cout << hours.count() << "h" <<  endl;

  std::chrono::nanoseconds nanos(1000000000);
  cout << nanos.count() << "ns" <<  endl;
}