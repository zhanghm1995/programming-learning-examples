/**
 * The usage of STL algorithm to process the STL containers
 * 
 **/

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

/**
 * @brief Overload the operator<< function for std::cout std::vector
 * 
 */
template <typename T>
std::ostream &operator<<(std::ostream &s, const std::vector<T> &vec) {
  for (const auto &o : vec) {
    s << o << " ";
  }
  return (s);
}

/**
 * http://www.java2s.com/Tutorial/Cpp/0260__template/templatefunctiontoprintelementsofanSTLcontainer.htm
 * @brief This print function is valid for std::set, std::vector, std::list
 *                it not worked for std::map
 **/
template <typename T>
void PrintSTLContainer(T const &container) {
  typename T::const_iterator pos;                   // iterator to iterate over coll
  typename T::const_iterator end(container.end());  // end position

  for (pos = container.begin(); pos != end; ++pos) {
    std::cout << *pos << ' ';
  }
  std::cout << std::endl;
}

/**
 * @brief std::copy and std::merge function usage
 **/
void CopyFunction() {
  cout << "=========Begin test CopyFunction=========" << endl;
  /// std::copy function usage
  std::vector<int> a = {1, 2, 3, 4, 5};
  std::vector<int> b = {6, 7, 8, 9, 10};
  std::vector<int> c;
  std::copy(a.begin(), a.end(), std::back_inserter(c));
  std::copy(b.begin(), b.end(), std::back_inserter(c));
  std::cout << c << std::endl;

  /// above process can be implemented by std::merge
  // need allocate size in advance, otherwise segmentation fault!
  std::vector<int> c1(a.size() + b.size());
  std::merge(a.begin(), a.end(), b.begin(), b.end(), c1.begin());
  std::cout << c1 << std::endl;

  cout << "=========End test CopyFunction=========" << endl;
}

void PrintFunction() {
  std::vector<float> a = {1.67, 2.0, 3.5};
  std::cout << a << std::endl;
  PrintSTLContainer(a);
}

void MaxMinFunction() {
  cout << "=========Begin test MaxMinFunction=========" << endl;
  std::vector<int> height = {1, 6, 4, 5, 10, 8, 9};
  //1)找到最高士兵的位置
  auto it_max = max_element(height.begin(), height.end());
  int pos = distance(height.begin(), it_max);
  int sum = 0;  //能看到的士兵数量
  while (pos != 0) {
    sum += 1;
    it_max = max_element(height.begin(), it_max);
    pos = distance(height.begin(), it_max);
  }
  sum += 1;
  cout << sum << endl;

  cout << "=========End test MaxMinFunction=========" << endl;
}

struct Object {
    Object(int id, std::string name) :
    id(id), name(name) {}

    int id;
    std::string name;
};

/**
 * @brief Test the find algorithm in STL
 */ 
void FindFunction() {
    cout<<"*************test find***********************"<<endl;
    std::vector<int> int_vec;
    int_vec.push_back(1);
    int_vec.push_back(3);
    int_vec.push_back(10);

    auto int_it = std::find(int_vec.begin(), int_vec.end(), 31);
    if (int_it != int_vec.end()) {
        cout<<"Find value in int_vec"<<endl;
    } else {
        cout<<"Cann't find value in int_vec"<<endl;
    }

    cout<<"*************test find_if***********************"<<endl;
    std::vector<Object> obj_vec;
    Object obj1(10, "car"), obj2(20, "people");
    obj_vec.push_back(obj1);
    obj_vec.push_back(obj2);

    int need_id = 10;
    auto it = std::find_if(obj_vec.begin(), obj_vec.end(), [&](Object obj) {
        return (obj.id == need_id);
    });
    if (it != obj_vec.end()) {
        cout<<"Find object in vector"<<endl;
    } else {
        cout<<"Cann't find object in vector"<<endl;
    }
}

int main() {
  CopyFunction();
  PrintFunction();
  MaxMinFunction();

  cout<<"================FindFunction==================="<<endl;
  FindFunction();
}