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

    std::map<std::string, std::vector<float> > calib_content;
    while (std::getline(file, line)) {
        std::cout<<line<<std::endl;
        std::vector<std::string> line_content;
        Split(rtrim(line), line_content, " ");
        std::vector<std::string> calib_param_string(line_content.begin() + 1, line_content.end());
        std::vector<float> calib_param(calib_param_string.size());
        std::transform(calib_param_string.begin(), calib_param_string.end(), calib_param.begin(), [](const std::string& val) {
            return std::atof(val.c_str()); });
        calib_content[line_content[0]] = calib_param;
    }

    auto P2_temp = calib_content["P2:"];
    Eigen::MatrixXf P2 = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(P2_temp.data());
    std::cout<<P2<<std::endl;

    auto R0_Rect_temp = calib_content["R_rect"];
    Eigen::MatrixXf R0_Rect = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> >(R0_Rect_temp.data());
    std::cout<<R0_Rect<<std::endl;
}

int main(int argc, char **argv)
{
    std::string env_name = GetEnv("HOME");
    std::cout<<env_name<<std::endl;

    env_name = GetEnv("PYTHONPATH");
    std::cout<<env_name<<std::endl;

    std::cout<<"===================="<<std::endl;
    // std::string file_path = "/home/zhanghm/readme.txt";
    std::string file_path = "/home/zhanghm/Temp/0000.txt";
    // std::string file_path = "/home/zhanghm";
    if (file_util::PathExists(file_path)) {
        std::cout<<"file_path "<<file_path<<" exists"<<std::endl;
    } else {
        std::cout<<"file_path "<<file_path<<" not exists"<<std::endl;
    }

    std::cout<<"===================="<<std::endl;
    // std::string dir_path = "/home/zhanghm/Coding";
    std::string dir_path = "/home/zhanghm/Temp/0000.txt";
    if (file_util::DirectoryExists(dir_path)) {
        std::cout<<"dir_path "<<dir_path<<"  exists"<<std::endl;
    } else {
        std::cout<<"dir_path "<<dir_path<<" not exists"<<std::endl;
    }

    std::cout<<"===================="<<std::endl;
    std::string calib_file_path = "../data/0000.txt";
    LoadFile2Map(calib_file_path);

    return 0;
}