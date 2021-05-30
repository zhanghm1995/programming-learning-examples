/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019-10-27
* Copyright    :
* Descriptoin  : Learn all about the usages of string in CPP, including 
                               std::string or the char[] and char*.
* References   : https://www.geeksforgeeks.org/program-generate-random-alphabets/
                               https://ykj373998035.gitbooks.io/c-string-/content/2stringde_fen_ge_3001_ti_636228_lei_si_string__spli.html
======================================================================*/

#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <cctype> // toupper, tolower
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <iomanip> // std::setw, std::setfill
#include <iterator>

#include "string_utils.h"

using std::cout;
using std::endl;

/**
 * @brief Convert the string to lower case or upper case
 **/
void ConvertStrCase() {
  std::string str("AaBb");
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  std::cout << "Lower case string is: " << str << std::endl;  // aabb
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  std::cout << "Upper case string is: " << str << std::endl;  // AABB
}

char InvertCase(char c) {
  return std::islower(c) ? std::toupper(c) : std::tolower(c);
}

/**
 * @brief Invert charactor case, that is from uppercase to lowercase and vice versa
 * @ref https://stackoverflow.com/questions/7426049/converting-from-upper-to-lower-case-and-vice-versa
 */ 
void InvertStrCase() {
  std::string str("AaBb"), result;
  std::transform(str.begin(), str.end(), std::back_inserter(result), InvertCase);
  cout<<result<<endl;
}

void CompareString() {
  std::string str("CaR"),  str1("car");
  // strcasecmp is defined in <cstring>
  // it can compare string without case sensitivity
  // References: http://c.biancheng.net/cpp/html/159.html
  if (!strcasecmp(str.c_str(), str1.c_str()))
    std::cout << "Two string is equal" << std::endl;
  else
    std::cout << "Two string is not equal" << std::endl;

  std::vector<std::string> person({"person", "people", "pedestrian"});
  std::string input("Pedestrian");
  auto it = std::find_if(person.begin(), person.end(), [&](std::string& obj) {
    return !strcasecmp(obj.c_str(), input.c_str());
  });

  if (it != person.end()) {
   std:: cout << "is person" << std::endl;
  } else {
    std::cout << "not person" << std::endl;
  }
}

/**
 * @brief Convert "10" to "00010" format
 */ 
template <typename DataType>
std::string FillFrontZero(const DataType input, const int num)
{
  std::stringstream ss;
  ss<<std::setw(num)<<std::setfill('0')<<input;
  return ss.str();
}

/**
 * @brief Convert string to other data and vice versa
 * @ref https://en.cppreference.com/w/cpp/string/basic_string/stof
 */ 
void ConvertStringData()
{
  
}

std::string TestConcatStrVec(const std::vector<std::string>& v, const std::string& delim)
{
  std::string s = JoinStr(v, delim.c_str());
  // std::string s;
  // std::for_each(v.begin(), v.end(), [&](const std::string &piece){ s += piece; });
  return s;
}

int main() {
  std::cout<<"======EndWith============"<<std::endl;
  if (EndWith("image.jpg", "jpg")) {
    std::cout<<"Yes, End with jpg"<<std::endl;
  } else {
    std::cout<<"No"<<std::endl;
  }

  std::vector<std::string> img_pats({"jpg", "bmp", "png"});
  if (EndWith("image.bmp",  img_pats)) {
    std::cout<<"Yes, End with image suffix"<<std::endl;
  } else {
    std::cout<<"No, Not End with image suffix"<<std::endl;
  }

  std::cout<<"========ConvertStrCase=========="<<std::endl;
  ConvertStrCase();

  std::cout<<"========InvertStrCase=========="<<std::endl;
  InvertStrCase();

  std::cout<<"========CompareString=========="<<std::endl;
  CompareString();

  std::cout<<"========Trim string=========="<<std::endl;
  std::string str = "    zhanghai ming \n";
  std::string str_trim = ltrim(str);
  std::cout<<str_trim<<std::endl;
  str_trim = rtrim(str);
  std::cout<<str_trim<<std::endl;

  std::cout<<"============ExpandString=============="<<std::endl;
  std::string out = FillFrontZero(10, 4);
  std::cout<<out<<std::endl;
  out = FillFrontZero("100", 4);
  std::cout<<out<<std::endl;

  cout<<"===================Join string vector===================="<<endl;
  std::vector<std::string> str_vec = {"zhang", "hai", "ming"};
  std::string str_joined = TestConcatStrVec(str_vec, "_");
  cout<<str_joined<<endl;

  std::cout<<"========Split string=========="<<std::endl;
  std::vector<std::string> result;
  SplitString("zhang hai ming", ' ', &result);
  std::copy(result.begin(), result.end(), std::ostream_iterator<std::string>(std::cout, ","));
  std::cout<<std::endl;
  /// 分割数字字符串
  std::string line_str("49.011212804408 8.4228850417969 112.83492279053 0.022447 1e-05 -1.2219096732051 -3.3256321640686 "
                                         "1.1384311814592 3.5147680214713 0.037625160413037 -0.03878884255623");
  result.clear();
  Split(line_str, result, " ");

  std::copy(result.begin(), result.end(), std::ostream_iterator<std::string>(std::cout, ","));
  std::cout<<std::endl;

  // 转为数字vector
  std::vector<double> result_double(result.size()); // 需要先分配存储空间
  std::transform(result.begin(), result.end(), result_double.begin(), [](const std::string& val) { return std::stod(val); });

  std::copy(result_double.begin(), result_double.end(), std::ostream_iterator<double>(std::cout, ","));
  std::cout<<std::endl;

  // 清空stringstream内容 https://stackoverflow.com/questions/20731/how-do-you-clear-a-stringstream-variable
  std::stringstream ss;
  ss.str("");

  return 0;
}