/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年7月6日
* Copyright    :
* Descriptoin  : The class type related learning in C++
* References   :
======================================================================*/

#include <string>
#include <iostream>

using namespace std;

class Example {
public:
  std::string GetString() const
  {
    std::string str("zhanghaiming");
    return str;
  }

  int GetId() 
  {
    return id_;
  }

private:
  int id_ = 12;
};

int main(int argc, char** argv)
{
  Example exam1;
  const std::string& str = exam1.GetString();
  cout<<str<<endl;
  // const Example& const_exam = exam1;
  // const_exam.GetId();
  
  
  return 0;
}