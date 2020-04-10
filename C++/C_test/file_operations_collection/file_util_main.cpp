/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月14日
* Copyright    :
* Descriptoin  : Using C++ to do file related operation
* References   :
======================================================================*/

#include <string>
#include <iostream>
#include <vector>
#include <map>

#include <Eigen/Dense>

#include "file_util.h"
#include "utils/string_utils.h"

/**
 * @brief Get the system environment variable
 */ 
inline std::string GetEnv(const std::string& var_name,
                          const std::string& default_value = "") {
  const char* var = std::getenv(var_name.c_str());
  if (var == nullptr) {
    std::cout << "Environment variable [" << var_name << "] not set, fallback to "
          << default_value;
    return default_value;
  }
  return std::string(var);
}

void LoadFile2Map(const std::string& calib_file_path) {
    std::ifstream file(calib_file_path);
    std::string line;

    std::map<std::string, std::vector<std::string> > calib_content;
    while (std::getline(file, line)) {
        std::cout<<line<<std::endl;
        std::vector<std::string> line_content;
        Split(rtrim(line), line_content, " ");
        std::cout<<line_content.size()<<std::endl;
        std::vector<std::string> calib_param(line_content.begin() + 1, line_content.end());
        calib_content[line_content[0]] = calib_param;
    }

    auto P2_temp = calib_content["P2:"];
    std::vector<float> P2(P2_temp.size());
    std::transform(P2_temp.begin(), P2_temp.end(), P2.begin(), [](const std::string& val) {
      return std::atof(val.c_str()); });

    
    Eigen::MatrixXf P2_ = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(P2.data());
    std::cout<<P2_<<std::endl;
}

int main(int argc, char **argv)
{
    std::string env_name = GetEnv("HOME");
    std::cout<<env_name<<std::endl;

    env_name = GetEnv("PYTHONPATH");
    std::cout<<env_name<<std::endl;

    std::cout<<"===================="<<std::endl;
    if (file_util::PathExists("/home/zhanghm/readme.txt")) {
        std::cout<<"Path exists"<<std::endl;
    } else {
        std::cout<<"Path not exists"<<std::endl;
    }

    std::cout<<"===================="<<std::endl;
    if (file_util::DirectoryExists("/home/zhanghm/Coding")) {
        std::cout<<"Directory exists"<<std::endl;
    } else {
        std::cout<<"Directory not exists"<<std::endl;
    }

    std::cout<<"===================="<<std::endl;
    std::string calib_file_path = "../data/0000.txt";
    LoadFile2Map(calib_file_path);

    return 0;
}