/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年8月11日
* Copyright    :
* Descriptoin  : Unit test for converting rosbag to kitti dataset format
*                Unit test: timestamp format save
* References   :
======================================================================*/
#include <iostream>
#include <map>
#include <string>
#include <ctime>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>
#include <boost/format.hpp>
using namespace boost::gregorian;
using namespace boost::posix_time;
using namespace std;

// 2011-09-26 13:04:32.345808896

#define DTTMFMT "%Y-%m-%d %H:%M:%S "
#define DTTMSZ 25
static char *getDtTm (char *buff) {
    time_t t = time (0);
    strftime (buff, DTTMSZ, DTTMFMT, localtime (&t));
    return buff;
}

std::tm* gettm(int64 timestamp)
{
    int64 milli = timestamp+ (int64)8*60*60*1000;//此处转化为东八区北京时间，如果是其它时区需要按需求修改
    auto mTime = std::chrono::milliseconds(milli);
    auto tp=std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds>(mTime);
    auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm* now = std::gmtime(&tt);
    printf("%4d年%02d月%02d日 %02d:%02d:%02d\n",now->tm_year+1900,now->tm_mon+1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
   return now;
}

std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto tmp=std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    std::time_t timestamp = tmp.count();
    //std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
    return timestamp;
}

#if 0
int main(void) {
    char buff[DTTMSZ];
    fstream filestr;
    filestr.open ("test.txt", fstream::out|fstream::app);

    // And this is how you call it:
    filestr << getDtTm (buff) << "Your message goes here" << std::endl;

    filestr.close();

    return 0;
}
#endif

int main()
{
  cout << "millisecond : ";
  cout << std::chrono::milliseconds::period::num << "/" << std::chrono::milliseconds::period::den << "s" <<endl;

  cout << "nanosecond : ";
  cout << std::chrono::nanoseconds::period::num << "/" << std::chrono::nanoseconds::period::den << "s" <<endl;

  chrono::hours hours(1);
  cout << hours.count() << "h" <<  endl;

  chrono::nanoseconds nanos(1000000000);
  cout << nanos.count() << "ns" <<  endl;

  cout<<"-------------"<<endl;
  while(1) {
  chrono::system_clock::time_point nowTp = chrono::system_clock::now();
//  chrono::time_point<chrono::system_clock, chrono::nanoseconds> tp = chrono::time_point_cast<chrono::nanoseconds>(nowTp);
  chrono::system_clock::duration tp = nowTp.time_since_epoch();
  cout << "====" << nowTp.time_since_epoch().count() << "ns" << endl;
  cout<<"----"<<chrono::duration_cast<chrono::seconds>(tp).count()<<endl;
  tp -= chrono::duration_cast<chrono::seconds>(tp);
  cout<<"____"<<tp.count()<<endl;
  //to_time_t方法把时间点转化为time_t
  time_t tt = chrono::system_clock::to_time_t(nowTp);
  //使用ctime转化为字符串
  cout << ctime(&tt);
  //把time_t转化为tm
  tm *localTm =  localtime(&tt);
//  cout << "year=" << localTm->tm_year << endl;
//  cout << "month=" << localTm->tm_mon << endl;
//  cout << "day=" << localTm->tm_mday << endl;
//  cout << "hour=" << localTm->tm_hour << endl;
//  cout << "minute=" << localTm->tm_min << endl;
//  cout << "second=" << localTm->tm_sec << endl;
  //tm格式化输出
  char buffer[20];
  char format[] = "%Y-%m-%d %H:%M:%S";
  strftime(buffer, sizeof(buffer), format, localTm);

  string nanosec_format = "%|010|";
  stringstream ss;
  ss<<buffer<<"."<<(boost::format(nanosec_format) % tp.count()).str();
  cout << "=============="<<ss.str() << std::endl;
  }

  return 0;
}
