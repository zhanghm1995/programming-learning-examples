/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-12 21:34:01
 * @LastEditTime: 2020-04-12 21:42:52
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

int main() {
    CustomDefinedCout();
    

  return 0;
}