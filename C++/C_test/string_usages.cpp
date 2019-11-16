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

int main()
{
    string str("zhanghaiming");
    test_string_find(str);
    return 0;
}