/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-19 11:42:40
 * @LastEditTime: 2020-04-19 12:05:41
 * @References: 
 * @Description: 演示include_directories指令在多文件夹项目中用法
 */

#include <iostream>

#include "math.h"
#include "utils/utils.h"

using std::cout;
using std::endl;

int main() {
    double degree = 45;
    double radian = deg2rad(degree);

    cout<<"radian = "<<radian<<endl;
    cout<<"degree = "<<rad2deg(radian)<<endl;

    Student stu("Marry", 25);
    cout<<stu.name()<<" is "<<stu.age()<<" years old!"<<endl;
    return 0;
} 