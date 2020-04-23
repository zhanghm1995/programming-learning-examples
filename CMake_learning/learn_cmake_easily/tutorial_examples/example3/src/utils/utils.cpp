/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-19 11:59:06
 * @LastEditTime: 2020-04-19 12:02:53
 * @References: 
 * @Description: 
 */

#include "utils/utils.h"

Student::Student(std::string name, int age) {
    name_ = name;
    age_ = age;
}

std::string Student::name() const {
    return name_;
}

int Student::age() const {
    return age_;
}