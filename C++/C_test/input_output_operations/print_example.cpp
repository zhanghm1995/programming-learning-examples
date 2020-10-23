/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年10月23日
* Copyright    :
* Descriptoin  : Learn console print functions in C++
* References   :
======================================================================*/

#include <iostream>

using std::cout;
using std::endl;

/**
 * @brief Print output in terminal with color
 */ 
void PrintWithColor() {
  #define KNRM  "\x1B[0m"
  #define KRED  "\x1B[1;31m"
  #define KGRN  "\x1B[1;32m"
  #define KYEL  "\x1B[1;33m"
  #define KBLU  "\x1B[1;34m"
  #define KMAG  "\x1B[1;35m"
  #define KCYN  "\x1B[1;36m"
  #define KWHT  "\x1B[37m"

  printf("%sred\n", KRED);
  printf("%sgreen\n", KGRN);
  printf("%syellow\n", KYEL);
  printf("%sblue\n", KBLU);
  printf("%smagenta\n", KMAG);
  printf("%scyan\n", KCYN);
  printf("%swhite\n", KWHT);
  printf("%snormal\n", KNRM);

  printf(KRED "red\n" KNRM);
  printf(KYEL "yellow\n" KNRM);

  printf("normal\n");
}

int main()
{
    PrintWithColor();
}