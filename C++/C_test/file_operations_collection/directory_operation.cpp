/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月2日
* Copyright    :
* Descriptoin  : Learn how to use native C++ to process directory and files related demands
* References   :
======================================================================*/

#include <dirent.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

/**
 * @brief This function will find all files in specified directories, including hidden files
 *                Assume no subdirectory in dir_name folder
 **/  
int ListFilesInDirectory(const std::string& dir_name) {
    DIR* dir;
    struct dirent* ent;

    int total_files = 0;
    if ((dir = opendir(dir_name.c_str())) != NULL) {
        // print all the files and directories within directory
        while ((ent = readdir(dir)) != NULL) {
            printf("%s\n", ent->d_name);

            // skip . & ..
            if (strlen(ent->d_name) > 2) {
                ++total_files;
            }
        }
        closedir(dir);
        printf("Total %d files in %s\n", total_files, dir_name.c_str());
    } else {
        // cannot open the directory
        perror("Failed to open directory");
        return -1;
    }
    return total_files;
}

void CountImageNumInDirectory(const std::string& dir_name)
{
    const std::vector<std::string> exts = {".jpg", ".jpeg", ".png", ".bmp", ".JPG", ".JPEG", ".PNG", ".BMP"};

     DIR* dir;
    struct dirent* ent;

    int total_num = 0;
    if ((dir = opendir(dir_name.c_str())) != NULL) {
        // print all the files and directories within directory
        while ((ent = readdir(dir)) != NULL) {
            std::string name(ent->d_name);
            size_t lDotPos = name.find_last_of(".");
            
            if (lDotPos != std::string::npos) {
                std::string ext = name.substr(lDotPos);
                if (std::find(exts.begin(), exts.end(), ext) != exts.end()) {
                    ++total_num;
                    printf("%s\n", name.c_str());
                }
            }
        }
        closedir(dir);
        printf("\nTotal %d images in %s\n", total_num, dir_name.c_str());
    } else {
        // cannot open the directory
        perror("Failed to open directory");
        return;
    }
}

int main(int argc, char **argv)
{
    int files_num = ListFilesInDirectory("/home/zhanghm/Temp/test");
    std::cout<<"There are "<<files_num<<" files"<<std::endl;

    std::cout<<"===================="<<std::endl;
    CountImageNumInDirectory("/home/zhanghm/Test");

    return 0;
}