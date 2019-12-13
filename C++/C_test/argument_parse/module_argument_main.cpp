/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年12月13日
* Copyright    :
* Descriptoin  : Learn how to parse command line arguments using pure C
* References   : Apollo_Auto
======================================================================*/

#include "module_argument.h"

/**
 * Usage: ./module_argument_main -d A.dag -d B.dag
 *  or  ./module_argument_main -h
 */
int main(int argc, char** argv) {
  ModuleArgument module_args;
  module_args.ParseArgument(argc, argv);
  return 0;
}