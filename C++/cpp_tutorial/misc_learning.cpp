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

/**
 * @brief Wrap a long text to multi lines string
 */ 
std::string WrapText(const std::string& str_input, const int length)
{
    if (str_input.size() <= length) {
        return str_input;
    }

    std::string ret;
    int pos = 0;
    while (pos < (str_input.size() - length)) {
        ret += str_input.substr(pos, length);
        ret += "\n";
        pos += length;
    }

    ret += str_input.substr(pos);
    
    return ret;
}

std::vector<std::string> WrapText2(const std::string& str_input, const int length)
{
    std::vector<std::string> ret;
    if (str_input.size() <= length) {
        ret.push_back(str_input);
        return ret;
    }

    std::string str;
    int pos = 0;
    const int temp_length = static_cast<int>(str_input.size()) - length;
    while (pos < temp_length) {
        ret.push_back(str_input.substr(pos, length));
        pos += length;
    }

    ret.push_back(str_input.substr(pos));

    return ret;
}

void TestPrintf()
{
  // print multiple \\ characters
  printf("50 \\\\\n\\hline\n");

  // more flexible one
  char buf[1024]; 
  sprintf(buf, "%d score and %d years ago", 4, 7);
  std::string str(buf);
  sprintf(buf, " It's will increase to %d", 10);
  str += std::string(buf);

  cout << str<<" "<<str.length() <<endl;
}

int main() {
  cout<<"============CustomDefinedCout=================="<<endl;
  CustomDefinedCout();
  cout<<"============FillArray=================="<<endl;
  FillArray();

  TestNaN();

  const int LENGTH = 10;
  cout<<std::abs(-19.34)<<endl;

  TestPrintf();
  return 0;
}