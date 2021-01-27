/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019-10-27
* Copyright    :
* Descriptoin  : Learn all about the usages of string in CPP, including 
                               std::string or the char[] and char*.
* References   : https://www.geeksforgeeks.org/program-generate-random-alphabets/
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
  if (it != person.end())
   std:: cout << "is person" << std::endl;
  else
    std::cout << "not person" << std::endl;
}

template <typename DataType>
std::string FillFrontZero(const DataType input, const int num)
{
  std::stringstream ss;
  ss<<std::setw(num)<<std::setfill('0')<<input;
  return ss.str();
}

/**
 * @brief Convert string to other data and vice versa
 */ 
void ConvertStringData()
{
  
}

/*! note: delimiter cannot contain NUL characters
  * https://stackoverflow.com/questions/5288396/c-ostream-out-manipulation/5289170#5289170
  */
template <typename Range, typename Value = typename Range::value_type>
std::string JoinStr(Range const& elements, const char *const delimiter) {
    std::ostringstream os;
    auto b = begin(elements), e = end(elements);

    if (b != e) {
        std::copy(b, prev(e), std::ostream_iterator<Value>(os, delimiter));
        b = prev(e);
    }
    if (b != e) {
        os << *b;
    }

    return os.str();
}

std::string ConcatStrVec(const std::vector<std::string>& v, const std::string& delim)
{
  std::string s = JoinStr(v, delim.c_str());
  // std::string s;
  // std::for_each(v.begin(), v.end(), [&](const std::string &piece){ s += piece; });
  return s;
}

int main() {
  std::cout<<"=================="<<std::endl;
  std::vector<std::string> result;
  SplitString("zhang hai ming", ' ', &result);
  for (auto o : result) {
      std::cout<<o<<std::endl;
  }

  std::cout<<"=================="<<std::endl;
  if (EndWith("image.jpg", ".jpg")) {
    std::cout<<"Yes"<<std::endl;
  } else {
    std::cout<<"No"<<std::endl;
  }

  std::cout<<"========ConvertStrCase=========="<<std::endl;
  ConvertStrCase();

  std::cout<<"========InvertStrCase=========="<<std::endl;
  InvertStrCase();

  std::cout<<"========CompareString=========="<<std::endl;
  CompareString();

  std::cout<<"=================="<<std::endl;
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

  cout<<"===================ConcatStrVec===================="<<endl;
  std::vector<std::string> str_vec = {"zhang", "hai", "ming"};
  std::string str_joined = ConcatStrVec(str_vec, "_");
  cout<<str_joined<<endl;
  return 0;
}