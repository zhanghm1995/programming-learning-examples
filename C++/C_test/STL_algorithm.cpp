/**
 * The usage of STL algorithm to process the STL containers
 * 
 **/

#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

template <typename T>
std::ostream& operator << (std::ostream& s, const std::vector<T>& vec)
{
  for (const auto& o : vec) {
      s<<o<<" ";
  }
  return (s);
}

/**
 * @brief std::copy and std::merge function usage
 **/ 
void CopyFunction()
{
    /// std::copy function usage
    std::vector<int> a = {1, 2, 3, 4, 5};
    std::vector<int> b = {6, 7, 8, 9, 10};
    std::vector<int> c;
    std::copy(a.begin(), a.end(), std::back_inserter(c));
    std::copy(b.begin(), b.end(), std::back_inserter(c));
    std::cout<<c<<std::endl;

    /// last process can be implemented by std::merge
    // need allocate size, otherwise segmentation fault!
    std::vector<int> c1(a.size() + b.size()); 
    std::merge(a.begin(), a.end(), b.begin(), b.end(), c1.begin());
    std::cout<<c1<<std::endl;
}

int main()
{
    CopyFunction();

}