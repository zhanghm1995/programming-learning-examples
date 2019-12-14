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

#include "file_util.h"

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

    return 0;
}