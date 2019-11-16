/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年8月12日
* Copyright    :
* Descriptoin  :
* References   : Learn how to do file and directory related process by using C++
======================================================================*/


#include <iostream>
#include <dirent.h>
#include <string.h>
#include <vector>
#include <string>
#include <algorithm>
using std::string;

int main()
{
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> exts = {".jpg", ".jpeg", ".png", ".bmp", ".JPG", ".JPEG", ".PNG", ".BMP"};

  if ((dir = opendir ("/home/zhanghm/Test/C_test/1/")) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      string name(ent->d_name);
      size_t lDotPos = name.find_last_of(".");
      if (lDotPos != string::npos)
      {
        string ext = name.substr(lDotPos);//find the file name's extension name
        if (find(exts.begin(), exts.end(), ext) != exts.end())
        {
          printf("%s\n",name.c_str());
        }
      }
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    return EXIT_FAILURE;
  }
}
