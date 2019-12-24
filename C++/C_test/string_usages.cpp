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
    string str("AaBb");
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    cout<<"Lower case string is: "<<str<<endl; // aabb
    std:transform(str.begin(), str.end(), str.begin(), ::toupper);
    cout<<"Upper case string is: "<<str<<endl; // AABB
}

void CompareString()
{
    string str("CaR"), str1("car");
    // strcasecmp is defined in <cstring>
    // it can compare string without case sensitivity
    // References: http://c.biancheng.net/cpp/html/159.html
    if (!strcasecmp(str.c_str(), str1.c_str()))
        cout << "Two string is equal" << endl;
    else
        cout << "Two string is not equal" << endl;

    vector<string> person({"person", "people", "pedestrian"});
    string input("Pedestrian");
    auto it = std::find_if(person.begin(), person.end(), [&](string &obj) {
        return !strcasecmp(obj.c_str(), input.c_str());
    });
    if (it != person.end())
        cout << "is person" << endl;
    else
        std::cout << "not person" << std::endl;
}

/**
 * @brief split string by one character
 * @param [in]: the string you want to split
 * @param [in]: the character
 * @param [out]: result strings after exploded by character
 * @return: the number of elements splitted in the given str
 **/
int SplitString(const std::string& str, char ch, std::vector<std::string>* result) {
    std::stringstream ss(str);
    std::string segment;
    int count = 0;
    while (std::getline(ss, segment, ch)) {
    result->push_back(segment);
    ++count;
    }
    return count;
}

/**
 * @brief Check if a string ends with a pattern.
 * @param ori The original string. To see if it ends with a specified pattern.
 * @param pat The target pattern. To see if the original string ends with it.
 * @return Whether the original string ends with the specified pattern.
 */
inline bool EndWith(const std::string& ori, const std::string& pat) {
  return std::equal(pat.rbegin(), pat.rend(), ori.rbegin());
}
inline bool StartWith(const std::string& ori, const std::string& pat) {
  return std::equal(pat.begin(), pat.end(), ori.begin());
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
  return 0;
}