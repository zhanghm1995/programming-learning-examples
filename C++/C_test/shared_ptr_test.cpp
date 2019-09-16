/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年8月29日
* Copyright    :
* Description  :
* References   :
======================================================================*/

#include <memory>
#include <vector>
#include <iostream>

using namespace std;
enum MoveProperty {
  MOVE_MOVING,
  MOVE_STATIC,
  MOVE_UNKOWNE,
  MOVE_CROSSING
};

struct RadarSupplement {
  MoveProperty move_property = MOVE_UNKOWNE;
};
typedef std::shared_ptr<RadarSupplement> RadarSupplementPtr;

struct Object {
  mutable int id = 0;
  int num[30];
  RadarSupplementPtr radar_ptr = nullptr;

//  void Increase()
//  {
//    cout<<"non const"<<endl;
//  }
  const int& GetId() const
  {
    cout<<"const"<<endl;
    return id;
  }

  int& GetId()
  {
    return id;
  }
};

std::ostream& operator << (std::ostream& s, const Object& p)
{
  for (int i = 0; i < 30; ++i) {
    s<<p.num[i]<<std::endl;
  }
  s<<p.radar_ptr->move_property<<endl;
  return (s);
}

#ifdef DEBUG
#define LOG(format, ...) fprintf(stdout, "Hello " format "\n", ##__VA_ARGS__)
#else
#define LOG(format, ...)
#endif

#define myprintf(format, ...) printf("Hello" format "\n", ##__VA_ARGS__)

int main(int argc, char** argv)
{
  Object obj, obj1;
  for (int i = 0; i < 30; ++i) {
    obj.num[i] = i * 3;
  }
  obj.radar_ptr.reset(new RadarSupplement());
  obj.radar_ptr->move_property = MOVE_MOVING;

  std::cout<<obj<<endl;

  cout<<"=++++++++++++++"<<endl;
  obj1 = obj;
  cout<<obj1<<endl;

  obj1.radar_ptr->move_property = MOVE_CROSSING;
  cout<<obj<<endl;
  cout<<obj1<<endl;

  cout<<"################"<<endl;
  const Object obj_const = obj;
  const int& i = obj_const.GetId();
  cout<<obj_const.id<<endl;
  Object objj;
  objj.GetId() = 3;
  int& jj = objj.GetId();
  jj = 10;
  cout<<objj.GetId()<<endl;

  cout<<"-------------------"<<endl;
  LOG("zhanghaiming");
  int age = 26;
  LOG("zhanghaiming %d", age);
//  printf("zhanghaiming");

  myprintf("zhanghaiming");
  myprintf("zhanghaiming %d", age);
  return 0;
}

