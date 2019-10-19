/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年7月6日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <iostream>
#include <unordered_map>
#include <set>
#include <map>
#include <list>
#include <fstream>
#include <string>
#include <string.h>
#include <algorithm>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

void test_set()
{
  //set是 key结构的,是关联式容器，对插入的元素自动排序，默认是升序
  set<int> Myset;
  set<int> ::iterator it;
  cout << "insert() , size() , begin() , end() " << endl;
  Myset.insert(5);
  Myset.insert(3);
  Myset.insert(1);
  Myset.insert(7);
  Myset.insert(3);
  cout << Myset.size() << endl;
  cout << Myset.max_size() << endl;
  for (it = Myset.begin(); it != Myset.end(); ++it)
  {
    cout << "   " << *it;
  }
  cout << endl;

  cout << " find(),  erase() ,  clear()   , empty() 测试：" << endl;
  it = Myset.find(5);
  Myset.erase(it, Myset.end());   //删除Myset的it到end之间的元素
  for (it = Myset.begin(); it != Myset.end(); ++it)
  {
    cout << "   " << *it;
  }
  cout << endl;

  cout << Myset.empty() << endl;
}

void test_map()
{
  map<string, int> Mymap;
  cout << "insert()  , begin() , clear( ) 测试:" << endl;
  Mymap.insert(std::make_pair("vector", 1));  //插入 key-value
  Mymap.insert(pair<string ,int>("map", 4));
  Mymap.insert(pair<string ,int>("set", 3));
  Mymap.insert(pair<string ,int>("list", 2));
  Mymap["add"] = 11;
  map<string, int>::const_iterator  it;
  for (it = Mymap.begin(); it != Mymap.end(); ++it)
  {
    cout << it->first << "~~" << it->second<< endl;
  }
  cout << "[] , find() ,  count(), clear() , size() , empty() " << endl;
  cout << Mymap["map"] << endl;

  it = Mymap.find("list");  //find() 查找一个元素
  cout << (*it).first << " : " << (*it).second<< endl;
  cout << "size "<<Mymap.size() << endl;

  cout << Mymap.count("add1") << endl;//count() 返回指定元素出现的次数
  cout<<"add new element "<<Mymap["map2"]<<endl;//这样会自动加入一个map2
  cout << "=========="<<Mymap.size() << endl;

  for (it = Mymap.begin(); it != Mymap.end(); ++it)
  {
    cout << it->first << "~~" << it->second<< endl;  //!!!这里编译不通过，肯定是你没有包含string头文件！
  }
  cout<<"erase test--------------"<<endl;
  Mymap.erase("vector");
  for (it = Mymap.begin(); it != Mymap.end(); ++it)
  {
    cout << it->first << "~~" << it->second<< endl;  //!!!这里编译不通过，肯定是你没有包含string头文件！
  }
  Mymap.clear();
  cout<< Mymap.empty() << endl;
}

bool badValue(int a)
{
  return (a > 5);
}

void test_map_delete()
{
  set<int> Myset = {1, 2, 3, 4, 5, 6, 7, 8};
  for (auto i = Myset.begin(); i != Myset.end();) {
    if (badValue(*i)) {
      i = Myset.erase(i);
    } else {
      ++i;
    }
  }
  for (auto o : Myset) {
    cout<<o<<endl;
  }
}

void test_unordered_map()
{
  typedef std::unordered_map<std::string,std::string> stringmap;
  stringmap second({{"apple","red"}, {"lemon","yellow"}});       // init list
  cout<<second.empty()<<endl;
  cout<<second.size()<<endl;
  cout<<second["lemon"]<<endl;
}

void test_string_find(const string& str)
{
  unordered_map<char,int> len_count_map;
  char current_char = 'a';
  int current_pos = 0;
  int len = str.length();

  while(current_pos < len){
    current_char = str[current_pos];
    int next_pos = str.find_first_not_of(current_char,current_pos);
    if(next_pos == -1){
      break;
    }

    int substr_len = next_pos - current_pos;
    if(!len_count_map[current_char]){
      len_count_map[current_char] = substr_len;
    }
    else{
      len_count_map[current_char] = max(len_count_map[current_char],substr_len);
    }
    cout<<"current_char "<<current_char<<" "<<substr_len<<endl;
    current_pos = next_pos;
  }
  //最后一段单独判断
  int substr_len = len - current_pos;
  cout<<"substr_len "<<substr_len<<endl;
  if(!len_count_map[current_char]){
    len_count_map[current_char] = substr_len;
  }
  else{
    len_count_map[current_char] = max(len_count_map[current_char],substr_len);
  }

  cout<<"len_count_map "<<len_count_map.size()<<endl;
  for(auto m:len_count_map){
    cout<<m.first<<" "<<m.second<<endl;
  }
}

void test_find_max(const vector<int>& height){
  cout<<"-----------test_find_max---------"<<endl;
  //1)找到最高士兵的位置
  auto it_max = max_element(height.begin(),height.end());
  int pos = distance(height.begin(),it_max);
  int sum = 0;//能看到的士兵数量
  while(pos!=0){
    sum+=1;
    it_max = max_element(height.begin(),it_max);
    pos = distance(height.begin(),it_max);
  }
  sum+=1;
  cout<<sum<<endl;
}

void test_string_sort(vector<string>& str_vec){
  cout<<"-----------test_string_sort---------"<<endl;
  for(const auto e:str_vec){
    cout<<e<<endl;
  }
  cout<<"-----------after sort---------"<<endl;
  sort(str_vec.begin(),str_vec.end());
  for(const auto e:str_vec){
      cout<<e<<endl;
    }

}

void set_test(){
  std::set<int> myset;
  std::set<int>::iterator it;

  // set some initial values:
  for (int i=1; i<=5; i++) myset.insert(i*10);    // set: 10 20 30 40 50

  it=myset.find(20);
  myset.erase (it);
  myset.erase (myset.find(60));

  std::cout << "myset contains:";
  for (it=myset.begin(); it!=myset.end(); ++it)
    std::cout << ' ' << *it;
  std::cout << '\n';
}


void test_list()
{
  std::list<int> intList = {2,3,1,5,7,3};
  for (auto& o : intList) {
    cout<<o<<endl;
  }
  intList.remove(3);
  cout<<"=-=========="<<endl;
  for (auto& o : intList) {
      cout<<o<<endl;
    }
}


void test_vector()
{
  std::vector<int> intVec;
  // https://blog.csdn.net/theusProme/article/details/53420031
  int temp = intVec.back(); // will cause undefined behavior
  cout<<temp<<endl;
}

struct BaseTest
{
  BaseTest(){}
  BaseTest(int num, float height):num(num),height(height){}
  int num;
  float height;
};

template <typename T>
T mymax(const T a, const T b)
{
  return a > b ? a : b;
}

//template <>
//string mymax(const string a, const string b)
//{
//  return a.length() > b.length()? a : b;
//}

void delete1(vector<int>& vec)
{
  for(auto it = vec.begin(); it!=vec.end();){
    if(*it % 2 == 0)
      it = vec.erase(it);
    else
      ++it;
  }
}

void delete2(vector<int>& vec)
{
  for(int i = 0; i < vec.size(); ++i){
    if(vec[i]%2 == 0){
      vec.erase(vec.begin() + i);
      --i;
    }
  }
}

vector<string> person({"person", "people", "pedestrian"});

using namespace std;
int main(int argc, char **argv)
{
  // test_vector();

  string str("CaR"), str1("car");

  if (!strcasecmp(str.c_str(), str1.c_str()))
    cout<<"equal"<<endl;
  else
    cout<<"not equal"<<endl;

  cout<<endl;
  string input("Pedestrian");
  auto it = std::find_if(person.begin(), person.end(), [&](string& obj){
    return !strcasecmp(obj.c_str(), input.c_str());
  });
  if(it != person.end())
    cout<<"is person"<<endl;
  else
    std::cout<<"not person"<<std::endl;

  cout<<"============="<<endl;
  test_map_delete();
  return 0;
}

