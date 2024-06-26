/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年7月6日
* Copyright    :
* Descriptoin  : The basic usages of STL containers
* References   :
======================================================================*/

#include <iostream>
#include <unordered_map>
#include <set>
#include <map>
#include <list>
#include <vector>
#include <fstream>
#include <string>
#include <string.h>
#include <algorithm>
#include <Eigen/Dense>

#include "stl_util.h"

using namespace Eigen;
using namespace std;

/**
 * @brief Learn how to use std::set
 * //set是 key结构的,是关联式容器，对插入的元素自动排序，默认是升序
 **/
void test_set()
{
  cout << "=========Begin test std::set=========*" << endl;
  std::set<int> Myset;
  cout << "Test insert() size() iterator functions:" << endl;
  Myset.insert(5);
  Myset.insert(3);
  Myset.insert(1);
  Myset.insert(7);
  Myset.insert(3);
  cout << Myset.size() << endl;

  std::set<int>::iterator it;
  for (it = Myset.begin(); it != Myset.end(); ++it) {
    cout << *it << " ";
  }
  cout << endl;

  cout << "Test find(), erase(), clear(), empty() functions:" << endl;
  it = Myset.find(5);
  Myset.erase(it, Myset.end()); //删除Myset的it到end之间的元素
  stl_util::PrintSTLContainer(Myset);

  cout << "Use find or count function to check the element existence:" << endl;
  if (Myset.find(5) == Myset.end()) {
    cout << "Cann't find specific element in set" << endl;
  } else {
    cout << "We find specific element in set" << endl;
  }
  if (Myset.count(3)) {
    cout << "We find specific element in set" << endl;
  }

  cout << Myset.empty() << endl;
  cout << "=========End test std::set=========" << endl;
}

bool badValue(int a)
{
  return (a % 2 == 0);
}

void test_set_delete()
{
  std::set<int> Myset = {1, 2, 3, 4, 5, 6, 7, 8};

  cout << "Method 1:" << endl;
  std::set<int> SetCase1 = Myset;
  for (auto i = SetCase1.begin(); i != SetCase1.end();) {
    if (badValue(*i)) {
      i = SetCase1.erase(i);
    } else {
      ++i;
    }
  }
  stl_util::PrintSTLContainer(SetCase1);

  // Note: this method may incorrect,
  //       but I don't know why the result is right!
  cout << "Method 2:" << endl;
  std::set<int> SetCase2 = Myset;
  for (auto it = SetCase2.begin(); it != SetCase2.end(); ++it) {
    if (badValue(*it)) {
      SetCase2.erase(it);
    }
  }
  stl_util::PrintSTLContainer(SetCase2);

  // https://stackoverflow.com/questions/2874441/deleting-elements-from-stl-set-while-iterating
  cout << "Method 3:" << endl;
  std::set<int> SetCase3 = Myset;
  auto it_case3 = SetCase3.begin();
  while (it_case3 != SetCase3.end()) {
    auto curr_it = it_case3++;
    if (badValue(*curr_it)) {
      SetCase3.erase(curr_it);
    }
  }
  stl_util::PrintSTLContainer(SetCase2);
}

void test_map()
{
  cout << "=========Begin test std::map=========" << endl;
  std::map<string, int> Mymap;
  cout << "Test insert(), size(), begin() , clear( ):" << endl;
  // The elements in map will be sorted automaticly
  Mymap.insert(std::make_pair("vector", 1)); //插入 key-value
  Mymap.insert(pair<string, int>("map", 4));
  Mymap.insert(pair<string, int>("set", 3));
  Mymap.insert(pair<string, int>("list", 2));
  Mymap["add"] = 11;

  cout << "Mymap size is " << Mymap.size() << endl;
  std::map<string, int>::const_iterator it;
  for (it = Mymap.begin(); it != Mymap.end(); ++it) {
    cout << it->first << "~~" << it->second << endl;
  }

  cout << "Test [], find(), count(), clear() , size() , empty():" << endl;
  cout << Mymap["map"] << endl;
  cout << "add new element " << Mymap["map2"] << endl; //这样会自动加入一个map2
  Mymap["map3"] = 10;
  for (it = Mymap.begin(); it != Mymap.end(); ++it) {
    cout << it->first << "~~" << it->second << endl;
  }

  cout << endl;
  it = Mymap.find("list"); //find() 查找一个元素
  if (it != Mymap.end()) { // Must check, otherwise will cause segmentation fault
    cout <<  it->first << " : " << it->second << endl;
  }

  cout << Mymap.count("add1") << endl; //count() 返回指定元素出现的次数

  cout << "Test erase():" << endl;
  Mymap.erase("vector");
  for (it = Mymap.begin(); it != Mymap.end(); ++it) {
    cout << it->first << "~~" << it->second << endl;
  }
  Mymap.clear();
  cout << Mymap.empty() << endl;
  cout << "=========End test std::map=========" << endl;
}

void test_unordered_map()
{
  typedef std::unordered_map<std::string, std::string> stringmap;
  stringmap second({{"apple", "red"}, {"lemon", "yellow"}}); // init list
  cout << second.empty() << endl;
  cout << second.size() << endl;
  cout << second["lemon"] << endl;
}

void test_list()
{
  std::list<int> intList = {2, 3, 1, 5, 7, 3};
  for (auto &o : intList) {
    cout << o << endl;
  }
  intList.remove(3);
  cout << "===========" << endl;
  for (auto &o : intList) {
    cout << o << endl;
  }
}

void test_vector()
{
  std::vector<int> intVec;
  // https://blog.csdn.net/theusProme/article/details/53420031
  int temp = intVec.back(); // will cause undefined behavior
  cout << temp << endl;
}

//////////////////////////////////Learn how to erase elements in STL containers///////////////////////////
// Ref: https://blog.csdn.net/sixdaycoder/article/details/81510877

// Learn how to erase the elements in vector, the same operation for deque, string and queue
void EraseVec()
{
  std::vector<int> vec{1,2,3,4,5,6,5,7};
  vec.erase(std::remove(vec.begin(), vec.end(), 5), vec.end());
}

void EraseVecIf()
{
  std::vector<int> vec{1,2,3,4,5,6,5,7};
  vec.erase(std::remove_if(vec.begin(), vec.end(), [](int elem) {
    return elem % 2 == 0; }), vec.end());
}

void EraseVecLoop()
{
  std::vector<int> vec{ 1, 2, 3, 4, 5, 5, 6, 7 }; 

  for (std::vector<int>::iterator it = vec.begin(); it != vec.end(); /*do nothing*/) {
    if (*it % 2 == 0) {
      it = vec.erase(it);
    } else {
      it++;
    }
  }
}

// Learn how to erase element for std::list
void EraseList()
{
  std::list<int> ls{ 1, 2, 3, 4, 5, 5, 6, 7 };

  ls.remove(5);
  ls.remove_if([](int elem) { return elem % 2 == 0; });
}

void EraseListLoop()
{
  std::list<int> ls{ 1, 2, 3, 4, 5, 5, 6, 7 };

  for (std::list<int>::iterator it = ls.begin(); it != ls.end(); ) {
    if (*it % 2 == 0) {
      it = ls.erase(it);
    } else {
      it++;
    }
  }
}

// Learn how to erase elements for std::map and std::unordered_map
void EraseMap()
{
  std::map<int, int> m;
  m.insert(std::make_pair(1, 1));
  m.insert(std::make_pair(2, 2));
  m.insert(std::make_pair(3, 3));
  m.insert(std::make_pair(4, 4));
  m.insert(std::make_pair(5, 5));
  m.insert(std::make_pair(6, 6));
  m.insert(std::make_pair(7, 7));
  std::map<int,int>::iterator it = m.find(5);
  if (it != m.end()) {
    m.erase(it);
  }
}

void EraseMapLoop()
{
  std::map<int, int> m;

  m.insert(std::make_pair(1, 1));
  m.insert(std::make_pair(2, 2));
  m.insert(std::make_pair(3, 3));
  m.insert(std::make_pair(4, 4));
  m.insert(std::make_pair(5, 5));
  m.insert(std::make_pair(6, 6));
  m.insert(std::make_pair(7, 7));

  //2.right
  for (std::map<int, int>::iterator it = m.begin(); it != m.end(); ) {
    if (it->second % 2 == 0) {
      //it是被删除的迭代器,it++的副作用可以获得it指向的下一个元素
      // m.erase(it++); // or use this one, both ok
       it = m.erase(it);
    } else {
      it++;
    }
  }
}

// Learn how to erase elements for std::set
void EraseSet()
{
  std::set<int> s;

  s.insert(1);
  s.insert(2);
  s.insert(3);
  s.insert(4);
  s.insert(5);
  s.insert(6);

  s.erase(5);
}
//////////////////////////////////Learn how to erase elements in STL containers///////////////////////////

int main(int argc, char **argv) {
  test_set();
  test_set_delete();
  test_map();
  // test_vector();
  return 0;
}
