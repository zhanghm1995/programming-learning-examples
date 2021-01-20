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
#include "utils/strnatcomp.h"

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

/**
 * @brief
 * @ref https://github.com/Amerge/natsort
 */ 
void TestListSubPaths()
{
    std::cout<<"=============TestListSubPaths==============="<<std::endl;
    // std::string dir_path = "/media/zhanghm/Data/Datasets/KITTI/tracking/training/oxts";
    std::string dir_path = "/media/zhanghm/Data/Datasets/KITTI/tracking/training/velodyne/0000";

    const auto name_vec = file_util::ListSubPaths(dir_path);

    for (const auto& name : name_vec) {
        std::cout<<name<<std::endl;
    }

    /// 如果需要对文件名进行排序
    std::cout<<"-------------------------只对不带后缀的数字文件名排序-------------------------"<<std::endl;
    const auto file_name_str_vec = file_util::ListSubPaths(dir_path, true);
    std::vector<int64_t> file_name_vec;
    std::transform(file_name_str_vec.begin(), file_name_str_vec.end(),
                                    std::back_inserter(file_name_vec), [](const std::string& str) { return std::stol(str); });

    std::sort(file_name_vec.begin(), file_name_vec.end());

    for (const auto& name : file_name_vec) {
        std::cout<<name<<std::endl;
    }

    std::cout<<"-------------------------对常规文件名自然排序-------------------------"<<std::endl;
    auto name_vec2 = file_util::ListSubPaths(dir_path);
    std::sort(name_vec2.begin(), name_vec2.end(), cpp_common::compareNat);

    for (const auto& name : name_vec2) {
        std::cout<<name<<std::endl;
    }
}

void TestJoinPath()
{
    std::cout<<"=============TestJoinPath==============="<<std::endl;
    std::string path1("/home/zhanghm");
    std::string path2("/dataset");
    auto path = file_util::JoinPath(path1, path2);
    std::cout << path<<std::endl;
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

    TestListSubPaths();

    TestJoinPath();

    return 0;
}