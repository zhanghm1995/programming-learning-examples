/**
 * @file read_csv_file.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-03-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include "utils/string_utils.h"

int main(int argc, char **argv) 
{
  std::string g_oxts_record_file("../data/log_info.csv");
  std::ifstream fin(g_oxts_record_file);
  // skip the first line
  std::string line;
  getline(fin, line);
  while (getline(fin, line)) {
    std::vector<std::string> line_content;
    Split(line, line_content, ",");
    std::copy(line_content.begin(), line_content.end(), std::ostream_iterator<std::string>(std::cout, " "));
    std::cout<<"size "<<line_content.size()<<std::endl;
    std::cout<<std::endl;

    // if need to transform the string data to value, plz uncomment the following lines
    // std::vector<double> oxts_vec(line_content.size());
    // std::transform(line_content.begin(), line_content.end(), oxts_vec.begin(),
    //                [](const std::string &val) { return std::stod(val); });

  }
}
