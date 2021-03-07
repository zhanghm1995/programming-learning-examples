#pragma once
/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月14日
* Copyright    :
* Descriptoin  : Using C++ to do file related operation
* References   : Apollo_Auto 5.0 cyber/common/file.h
======================================================================*/

#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace file_util {

/**
 * @brief Join the file path like /a/b and c/d.txt to /a/b/c/d.txt
 * @param path1 
 * @param path2 
 * @return std::string 
 */
std::string JoinPath(const std::string& path1, const std::string& path2);

/**
 * @brief Check if the path exists, including file path and directory path
 * @param path a file name, such as /a/b/c.txt, this function can also check a directory path exist or not
 * @return If the path exists.
 */
bool PathExists(const std::string &path);

/**
 * @brief Check if the directory specified by directory_path exists
 *               and is indeed a directory. If it is not a directory, although it exist, it will return false
 * @param directory_path Directory path.
 * @return If the directory specified by directory_path exists
 *         and is indeed a directory.
 */
bool DirectoryExists(const std::string &directory_path);

std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                                                                const bool remove_extension = false);

} // namespace file_util