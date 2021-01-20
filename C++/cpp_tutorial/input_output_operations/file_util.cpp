#include "file_util.h"
#include <string.h>
#include <iostream>

namespace file_util {

bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool DirectoryExists(const std::string &directory_path) {
  struct stat info;
  return stat(directory_path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                                                                const bool remove_extension) {
    std::vector<std::string> result;
    DIR *directory = opendir(directory_path.c_str());
    if (directory == nullptr) {
      std::cout << "Cannot open directory " << directory_path;
      return result;
    }

    struct dirent *entry;
    while ((entry = readdir(directory)) != nullptr) {
        // Skip "." and "..".
      if (strcmp(entry->d_name, ".") != 0 &&
          strcmp(entry->d_name, "..") != 0) {
          std::string name(entry->d_name);
          std::string file_name;
          size_t end = name.find_last_of(".");

          if (end != std::string::npos) {
            const auto len = (end != std::string::npos) ? end - 0 : end;
            file_name = name.substr(0, len);
          }
        result.emplace_back(file_name);
      }
    }
    closedir(directory);
    return result;
}

}