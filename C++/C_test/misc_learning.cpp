/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-12 21:34:01
 * @LastEditTime: 2020-04-21 21:21:43
 * @References: 
 * @Description: 
 */

#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::endl;

struct Point {
  int x;
  int y;
};

// Overload the << operator so we can using std::cout to print directly
std::ostream& operator<<(std::ostream& output, const Point& a) {
  return output << "(" << a.x << "," << a.y << ")";
}

void CustomDefinedCout() {
  Point pt;
  pt.x = 10;
  pt.y = 20;
  cout<<pt<<endl;
}

/**
 * @brief Use std::fill to initialize the C++ array with the same value
 */ 
void FillArray() {
  int int_array[20] = {-1};
  
  for (int i = 0; i < 20; ++i) {
    cout<<int_array[i]<<endl;
  }
  std::cout<<"++++++++++++++++++"<<endl;
  std::fill(int_array, int_array + 20, -1);
  for (int i = 0; i < 20; ++i) {
    cout<<int_array[i]<<endl;
  }
}

int main() {
  CustomDefinedCout();
  FillArray();
  return 0;
}