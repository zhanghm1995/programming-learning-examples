/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019-10-27
* Copyright    :
* Descriptoin  : Learn all about the usages of string in CPP, including 
                               std::string or the char[] and char*.
* References   :
======================================================================*/

#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "utils/string_utils.h"

using namespace std;

void test_string_find(const string &str)
{
    unordered_map<char, int> len_count_map;
    char current_char = 'a';
    int current_pos = 0;
    int len = str.length();

    while (current_pos < len)
    {
        current_char = str[current_pos];
        int next_pos = str.find_first_not_of(current_char, current_pos);
        if (next_pos == -1)
        {
            break;
        }

        int substr_len = next_pos - current_pos;
        if (!len_count_map[current_char])
        {
            len_count_map[current_char] = substr_len;
        }
        else
        {
            len_count_map[current_char] = max(len_count_map[current_char], substr_len);
        }
        cout << "current_char " << current_char << " " << substr_len << endl;
        current_pos = next_pos;
    }
    //最后一段单独判断
    int substr_len = len - current_pos;
    cout << "substr_len " << substr_len << endl;
    if (!len_count_map[current_char])
    {
        len_count_map[current_char] = substr_len;
    }
    else
    {
        len_count_map[current_char] = max(len_count_map[current_char], substr_len);
    }

    cout << "len_count_map " << len_count_map.size() << endl;
    for (auto m : len_count_map)
    {
        cout << m.first << " " << m.second << endl;
    }
}

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


int main()
{
  string str("zhanghaiming");
  test_string_find(str);

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

  std::cout<<"=================="<<std::endl;
  ConvertStrCase();

  std::cout<<"=================="<<std::endl;
  str = "    zhanghai ming \n";
  std::string str_trim = ltrim(str);
  std::cout<<str_trim<<std::endl;
  str_trim = rtrim(str);
  std::cout<<str_trim<<std::endl;
  return 0;
}