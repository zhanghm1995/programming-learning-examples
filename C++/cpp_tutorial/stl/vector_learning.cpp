/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2020年7月6日
* Copyright    :
* Descriptoin : Learn how to use yaml-cpp library to parse your customized yaml type
* References  : https://www.acodersjourney.com/6-tips-supercharge-cpp-11-vector-performance/
======================================================================*/

#include <vector>
#include <iostream>

#include "stl_util.h"

using std::cout;
using std::endl;

/**
 * @brief vector片段截取操作
 */ 
void ProcessPartVector()
{
    cout << "=========Begin ProcessPartVector=========" << endl;
    std::vector<int> int_vec{1,2,3,4,5,6,7,8,9};
    stl_util::PrintSTLContainer(int_vec);

    // 截取前5个数
    std::vector<int>::const_iterator first1 = int_vec.begin();
    std::vector<int>::const_iterator last1  = int_vec.begin() + 5;
    std::vector<int> cut1_vector(first1, last1);
    stl_util::PrintSTLContainer(cut1_vector);

    // 截取后5个数
    std::vector<int>::const_iterator first2 = int_vec.end() - 5;
    std::vector<int>::const_iterator last2  = int_vec.end();
    std::vector<int> cut2_vector(first2, last2);
    stl_util::PrintSTLContainer(cut2_vector);

    cout << "=========End ProcessPartVector=========" << endl;
}

int main(int argc, char** argv)
{
    ProcessPartVector();
}