/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-19 11:58:54
 * @LastEditTime: 2020-04-19 12:02:16
 * @References: 
 * @Description: 
 */

#include <string>

class Student {
public:
    Student(std::string name, int age);

    std::string name() const;

    int age() const;

private:
    std::string name_;
    int age_;
};