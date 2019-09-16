/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年8月2日
* Copyright    :
* Description  :
* References   :
======================================================================*/

#include <iostream>
#include <algorithm>
#include <string>
#include <map>
#include <functional>
using namespace std;

struct Student
{
  Student(string name, float height, int age):
    name_(name),
    height_(height),
    age_(age)
  {
  }

  float height_;
  string name_;
  int age_;
};

std::ostream& operator << (std::ostream& strm, const Student& myIn)
{
  strm << "Student " << myIn.name_ <<  " has height "<<myIn.height_;
  return strm;
}

void TestSTLObject()
{
    std::string str = "Hello";
    std::vector<std::string> v;

    // uses the push_back(const T&) overload, which means
    // we'll incur the cost of copying str
    v.push_back(str);
    std::cout << "After copy, str is \"" << str << "\"\n";

    // uses the rvalue reference push_back(T&&) overload,
    // which means no strings will be copied; instead, the contents
    // of str will be moved into the vector.  This is less
    // expensive, but also means str might now be empty.
    v.push_back(std::move(str));
    str = "eee";
    std::cout << "After move, str is \"" << str << "\"\n";

    std::cout << "The contents of the vector are \"" << v[0]
                                         << "\", \"" << v[1] << "\"\n";
}

#include <vector>
#include <string>
#include <iostream>

struct President
{
    std::string name;
    std::string country;
    int year;

    President(std::string p_name, std::string p_country, int p_year)
        : name(std::move(p_name)), country(std::move(p_country)), year(p_year)
    {
        std::cout << "I am being constructed.\n";
    }

    ~President()
    {
      std::cout << "~ I am being deconstructor.\n";
    }
    President(const President& other)
        : name(std::move(other.name)), country(std::move(other.country)), year(other.year)
    {
        std::cout << "I am being copy constructed.\n";
    }
    President(President&& other)
        : name(std::move(other.name)), country(std::move(other.country)), year(other.year)
    {
        std::cout << "I am being moved.\n";
    }
    President& operator=(const President& other);
};

int main()
{
  vector<int> ddd;
  ddd.push_back(10);
    std::vector<President> elections;
    std::cout << "emplace_back:\n";
    elections.emplace_back("Nelson Mandela", "South Africa", 1994); //没有类的创建

    std::vector<President> reElections;
    std::cout << "\npush_back:\n";
    reElections.push_back(President("Franklin Delano Roosevelt", "the USA", 1936));

    std::vector<President> anotherElections;
    std::cout << "\nanother push_back:\n";
    President Frank("Franklin Delano Roosevelt", "the USA", 1937);
    anotherElections.push_back(Frank);

    std::vector<President> Elections3;
    std::cout << "\nanother push_back 3:\n";
    President Frank3("Franklin Delano Roosevelt", "the USA", 1937);
    Elections3.push_back(std::move(Frank));
    cout<<endl;
}

#if 0
int main()
{
  constexpr int a = 10;
  Student A("zhanghaiming", 170.0, 24);
  cout<<A<<endl;

  int c = 30;
  int&& d = std::move(c);
  cout<<d<<" "<<c<<endl;
  d = 12;
  cout<<d<<" "<<c<<endl;
  c = 22;
  cout<<d<<" "<<c<<endl;

  TestSTLObject();


  vector<string> names;
  names.emplace_back("zhanghaiming");

}
#endif
