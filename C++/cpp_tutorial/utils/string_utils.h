/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-10 22:29:01
 * @LastEditTime: 2020-04-12 21:10:59
 * @References: 
 * @Description: Collect all string related operations for handy usages
 */

#pragma once

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iterator>

/**
 * @brief split string by one character
 * @param [in]: the string you want to split
 * @param [in]: the character
 * @param [out]: result strings after exploded by character
 * @return: the number of elements splitted in the given str
 **/
inline int SplitString(const std::string& str, char ch, std::vector<std::string>* result) {
  std::stringstream ss(str);
  std::string segment;
  int count = 0;
  while (std::getline(ss, segment, ch)) {
    result->push_back(segment);
    ++count;
  }
  return count;
}

inline void Split(const std::string& input_str, std::vector<std::string>& output, const char* delim) {
  int pos = 0;
  int npos = 0;
  int regexlen = strlen(delim);
  while ((npos = input_str.find(delim, pos)) != -1) {
    std::string tmp = input_str.substr(pos, npos - pos);
    output.push_back(tmp);
    pos = npos + regexlen;
  }
  output.push_back(input_str.substr(pos, input_str.length() - pos));
}

/**
 * @brief Join elements in stl::Container with delimiter
 * ! note: delimiter cannot contain NUL characters! note: delimiter cannot contain NUL characters* ! note: delimiter cannot contain NUL characters
 * https://stackoverflow.com/questions/5288396/c-ostream-out-manipulation/5289170#5289170
 */
template <typename Range, typename Value = typename Range::value_type>
inline std::string JoinStr(const Range& elements, const char *const delimiter) {
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

/**
 * @brief Trim the left whitespace, \n and so on charactors in a string
 * @ref http://www.martinbroadhurst.com/how-to-trim-a-stdstring.html
 */
inline std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ") {
  str.erase(0, str.find_first_not_of(chars));
  return str;
}

inline std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ") {
  str.erase(str.find_last_not_of(chars) + 1);
  return str;
}

inline std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ") {
  return ltrim(rtrim(str, chars), chars);
}

/**
 * @brief Check if a string starts with a pattern.
 * @param ori The original string. To see if it starts with a specified pattern.
 * @param pat The target pattern. To see if the original string starts with it.
 * @return Whether the original string starts with the specified pattern.
 */
inline bool StartWith(const std::string& ori, const std::string& pat) {
  return std::equal(pat.begin(), pat.end(), ori.begin());
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

inline bool EndWith(const std::string& ori, const std::vector<std::string>& pat_vec )
{
  return std::any_of(pat_vec.begin(), pat_vec.end(), [&](const std::string& pat){ return EndWith(ori, pat); });
}