#pragma once

#include <iostream>
#include <string>

namespace foo_bar {

void PrintString(const std::string& str) {
  std::cout<<str<<std::endl;
}

class Student {
public:
  Student(const std::string& name, int age) :
    name_(name), age_(age) {}

  std::string Name() const;  

private:
  std::string name_;
  int age_;
};

}