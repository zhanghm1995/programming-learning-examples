/**
 * To demostrate the google test for C++ class type
 * 
 **/
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

class ExampleClass {
public:
    ExampleClass(const std::string& name, const double height):
    name_(name),
    height_(height)
    {
        vec_ = {1, 3, 7, 8, 10, 7, 39, 26};
    }

    bool IsNumberIn(const int num)
    {
        auto it = std::find(vec_.begin(), vec_.end(), num);
        return it != vec_.end();
    }

    std::string GetName() const
    {
        return name_;
    }

    double height() const { return height_; }

    std::vector<int> GetVector() const
    {
        return vec_;
    }

    int GetMaxInVector()
    {
        std::sort(vec_.begin(), vec_.end());
        return vec_.back();
    }
private:
    std::string name_;
    std::vector<int> vec_;
    double height_;
};