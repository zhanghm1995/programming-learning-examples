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
 * @brief 
 * @ref https://blog.csdn.net/qq_38149046/article/details/81181248
 */
void TestInsert()
{
    cout << "=========Begin TestInsert=========" << endl;
    /// 用法一: 在Vector的指定位置插入值
    std::vector<int> v = {1, 2, 3};
    v.insert(v.begin(),8);//在最前面插入新元素。  
    v.insert(v.begin()+2,1);//在迭代器中第二个元素前插入新元素  
    v.insert(v.end(),3);//在向量末尾追加新元素。

    v.insert(v.end(),4,1);//在尾部插入4个1
 
	int a[] = {1,2,3,4};
	v.insert(v.end(),a[1],a[3]);//在尾部插入a[1]个a[3]
 
    std::vector<int>::iterator it;  
    for(it=v.begin(); it!=v.end();it++)   { 
        cout<<*it<<" ";  
    } 
    cout<<endl;

    /// 用法二: 可将两个vector相连接
    std::vector<int> int_vec1 = {3, 4, 5};
    std::vector<int> int_vec2 = {7, 8, 9};
    int_vec1.insert(int_vec1.begin(), int_vec2.begin(), int_vec2.end());
    cout<<"在vector前面插入另一个vector"<<endl;
    stl_util::PrintSTLContainer(int_vec1);
    int_vec1.insert(int_vec1.end(), int_vec2.begin(), int_vec2.end());
    cout<<"在vector后面插入另一个vector"<<endl;
    stl_util::PrintSTLContainer(int_vec1);

    cout << "=========End TestInsert=========" << endl;
}
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

    TestInsert();
}