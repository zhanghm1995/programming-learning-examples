/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-12 21:34:01
 * @LastEditTime: 2020-05-04 21:10:25
 * @References: 
 * @Description: 
 */

#include <iostream>
#include <string>
#include <cmath>
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

// https://stackoverflow.com/questions/16691207/c-c-nan-constant-literal
void TestNaN() {
   cout<<"============TestNaN=================="<<endl;
  std::vector<double> vec = {NAN, INFINITY, 10};
  for (const auto& v : vec) {
    cout<<"this is "<<v<<endl;
    if (std::isnan(v)) {
    cout<<"Is NaN"<<endl;
    } else {
      cout<<"Not NaN"<<endl;
    }
    if (std::isnormal(v)) {
      cout<<"isnormal"<<endl;
    }
  }

  /////////////
  cout<<std::sqrt(-30)<<endl;
}

int main() {
  cout<<"============CustomDefinedCout=================="<<endl;
  CustomDefinedCout();
  cout<<"============FillArray=================="<<endl;
  FillArray();

  TestNaN();
  return 0;
}