/**
 * The usage of STL algorithm to process the STL containers
 * Refrences:
 * https://blog.csdn.net/Robin__Chou/article/details/53204970
 * 
 **/

#include <algorithm>
#include <iostream>
#include <vector>
#include <array>
#include <list>
#include <numeric> // std::accumulate
#include <iterator> // std::ostream_iterator

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
 * @ref https://www.fluentcpp.com/2018/03/30/is-stdfor_each-obsolete/
 */ 
void ForEachFunction() {
  cout << "=========Begin test ForEachFunction=========" << endl;
  std::vector<int> numbers = {1,2,3,4,5};
  std::for_each(std::begin(numbers), std::end(numbers), [](int& number) {
    ++number;
  });
  PrintSTLContainer(numbers);
  cout << "=========End test ForEachFunction=========" << endl;
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
  // need allocate size in advance, otherwise will cause segmentation fault!
  std::vector<int> c1(a.size() + b.size());
  std::merge(a.begin(), a.end(), b.begin(), b.end(), c1.begin());
  std::cout << c1 << std::endl;

  // 使用std::copy实现快速打印
  int arr[10];
  std::fill(arr, arr+10, 6);
  std::copy(arr, arr+10, std::ostream_iterator<int>(std::cout, " "));
  cout<<endl;

  cout << "=========End test CopyFunction=========" << endl;
}

/**
 * @brief std::transform函数能够在复制容器元素同时进行变换
 **/
void TransformFunction() {
   cout << "=========Begin test TransformFunction=========" << endl;
  auto increase = [](int score) -> int {
    if (score > 30 && score < 60) {
      score = 60;
    }
    return score;
  };

  std::vector<int> score_vec = {45, 34, 79, 59, 93};
  std::transform(score_vec.begin(), score_vec.end(), score_vec.begin(), increase);
  cout<<"score_vec: "<<endl;
  PrintSTLContainer(score_vec);

  // 如果用另一个向量存储结果,最好配合std::back_inserter使用,否则需要事先分配该向量的大小
  std::vector<int> score_vec2;
  std::transform(score_vec.begin(), score_vec.end(), std::back_inserter(score_vec2), increase);
  cout<<"score_vec2: "<<endl;
  PrintSTLContainer(score_vec);
  cout << "=========End test TransformFunction=========" << endl;
}

void PrintFunction() {
  std::vector<float> a = {1.67, 2.0, 3.5};
  std::cout << a << std::endl;
  PrintSTLContainer(a);
}

struct Object {
  Object(int id, std::string name) : id(id), name(name) {}

  int id;
  std::string name;
};

void SortFunction() {
  cout << "=========Begin test SortFunction=========" << endl;
  /// ------------------------------字符串默认排序
  std::vector<std::string> str_vec = {"Jiawei", "Zhanghaiming", "Chensisi", "Lifei", "Wangchao"};
  std::sort(str_vec.begin(), str_vec.end());
  PrintSTLContainer(str_vec);

  /// -------------------------------数值默认排序
  std::vector<float> float_vec = {10.2, 3.2, 49.1, 38, 1.9};
  std::sort(float_vec.begin(), float_vec.end());
  std::reverse(float_vec.begin(), float_vec.end());
  PrintSTLContainer(float_vec);

  /// -----------------------------指定排序规则
  Object obj1(10, "zhang san"), obj2(27, "li si"), obj3(8, "wang wu");
  std::vector<Object> obj_vec = {obj1, obj2, obj3};
  std::sort(obj_vec.begin(), obj_vec.end(), [](const Object& obj1, const Object& obj2) {
    return obj1.id > obj2.id;
  });
  for (const auto& obj : obj_vec) {
    cout<<obj.name<<" "<<obj.id<<endl;
  }
  cout << "=========End test SortFunction=========" << endl;
}

void MaxMinFunction() {
  cout << "=========Begin test MaxMinFunction=========" << endl;
  /// --------------------------查找默认最大值
  std::vector<int> height = {1, 6, 4, 5, 10, 8, 9};
  //1)找到最高士兵的位置
  auto it_max = std::max_element(height.begin(), height.end());
  int pos = std::distance(height.begin(), it_max);
  int sum = 0;  //能看到的士兵数量

  while (pos != 0) {
  sum += 1;
  it_max = std::max_element(height.begin(), it_max);
  pos = std::distance(height.begin(), it_max);
  }
  sum += 1;
  cout << sum << endl;

  /// --------------------------利用lambda表示式指定条件
  Object obj1(10, "zhang san"), obj2(27, "li si"), obj3(8, "wang wu");
  std::vector<Object> obj_vec = {obj1, obj2, obj3};
  auto it_min = std::min_element(obj_vec.begin(), obj_vec.end(), [](const Object obj1, Object obj2) {
    return obj1.id < obj2.id;
  });

  cout<<"[std::min_element] it_min is "<<it_min->name<<endl;

  /// --------------------------同时查找最大最小
  auto it_min_max = std::minmax_element(obj_vec.begin(), obj_vec.end(), [](const Object obj1, Object obj2) {
    return obj1.id < obj2.id;
  });

  cout<<"min_element is "<<it_min_max.first->name<<" max_element is "<<it_min_max.second->name<<endl;
  cout << "=========End test MaxMinFunction=========" << endl;
}

/**
 * @brief Test the std::find  and std::find_if algorithm in STL
 */ 
void FindFunction() {
  cout << "=========Begin test FindFunction=========" << endl;
  cout<<"*************test find***********************"<<endl;
  std::vector<int> int_vec;
  int_vec.push_back(1);
  int_vec.push_back(3);
  int_vec.push_back(10);
  int_vec.push_back(3);

  // -------------------------查找具体的值
  auto int_it = std::find(int_vec.begin(), int_vec.end(), 3);
  if (int_it != int_vec.end()) {
    cout<<"Find "<< *int_it<<" in int_vec index: "<<std::distance(int_vec.begin(), int_it)<<endl;
  } else {
    cout<<"Cann't find value in int_vec"<<endl;
  }

  // -------------------------查找满足条件的元素
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

  // -------------------------查找所有满足条件的元素
  auto isPass = [](int n) -> bool {
    return n >= 60;
  };

  vector<int> vecScore = {72, 54, 87};
  vector<int>::iterator it2 = vecScore.begin();
  do {
    it2 = std::find_if(it2, vecScore.end(), isPass);
    if (vecScore.end() != it2) {
      cout<<"find pass "<<*it2<<endl;
      ++it2;
    } else {
      break;
    }
  } while(true);

  // 或者使用以下方法
  // it = vecScore.begin();
  // while (it != vecScore.end()) {
  //   it = find_if(it, vecScore.end(), isPass);
  //   cout<<"find pass "<<*it<<endl;
  //   ++it;
  // }

  cout << "=========Begin test FindFunction=========" << endl;
}

void CountFunction() {
  cout << "=========Begin test CountFunction=========" << endl;
  std::vector<int> int_vec = {10, 12, 34, 10, 2, 3, 10, 22, 10};
  int count_num = std::count(int_vec.begin(), int_vec.end(), 10);
  cout<<"count_num: "<<count_num<<endl;

  // Test std::count_if
  Object obj1(10, "zhang san"), obj2(27, "li si"), obj3(8, "wang wu"), obj4(10, "wang wu");
  std::vector<Object> obj_vec = {obj1, obj2, obj3, obj4};

  int count_num2 = std::count_if(obj_vec.begin(), obj_vec.end(), [](const Object& obj) {
    return obj.id == 10;
  });
  cout<<"count_num2: "<<count_num2<<endl;

  cout << "=========End test CountFunction=========" << endl;
}

/**
 * @brief 利用迭代器定义能够接受不同类型容器的函数进行处理
 */ 
template <typename InputIterator>
void ProcessContainer(InputIterator first, InputIterator end) {
  for (auto i = first; i != end; ++i) {
    ++i->id;
  }
}

void TestProcessContainerFunction() {
  cout << "=========Begin test TestProcessContainerFunction=========" << endl;
  Object obj1(10, "zhang san"), obj2(27, "li si"), obj3(8, "wang wu");
  std::vector<Object> obj_vec = {obj1, obj2, obj3};
  std::list<Object> obj_list = {obj1, obj2, obj3};

  ProcessContainer(obj_vec.begin(), obj_vec.end());
  cout<<"obj_vec is "<<endl;
  for (const auto& obj : obj_vec) {
    cout<<obj.name<<" "<<obj.id<<std::endl;
  }

  cout<<endl;
  ProcessContainer(obj_list.begin(), obj_list.end());
  cout<<"obj_list is "<<endl;
  for (const auto& obj : obj_list) {
    cout<<obj.name<<" "<<obj.id<<std::endl;
  }
  cout << "=========End test TestProcessContainerFunction=========" << endl;
}

void AccumulateFunction() {
  cout << "=========End test AccumulateFunction=========" << endl;
  std::vector<Object> obj_vec = {Object(1, "zhang san"), Object(2, "Li si"), Object(4, "Wang chao")};

  // 需要包含头文件numeric
  // 从0开始累加
  int id_sum = std::accumulate(obj_vec.begin(), obj_vec.end(), 0, [](int a, const Object& b) {
    return a + b.id;
  });

  cout<<"id sum is: "<<id_sum<<endl;
  cout << "=========End test AccumulateFunction=========" << endl;
}

void TestContainerCondition() {
  cout << "=========Begin test TestContainerCondition=========" << endl;
  // ---------------------------- std::all_of------------------------------------
  // 所有元素都满足指定条件为真
  // http://www.cplusplus.com/reference/algorithm/all_of/
  std::vector<int> vec_int = {3,5,7,11,13,17,19,23};
  if (std::all_of(vec_int.begin(), vec_int.end(), [](int i) { return i % 2 ; })) {
    cout<<"All the elements are odd numbers."<<endl;
  }

  // !!容器元素为空时,默认返回真,注意!
  std::vector<int> vec_int2;
  if (std::all_of(vec_int2.begin(), vec_int2.end(), [](int i) { return i % 2 ; })) {
    cout<<"All the elements are odd numbers."<<endl;
  }

  // ---------------------------std::none_of--------------------------------
  // 所有元素都不满足指定条件为真
  // http://www.cplusplus.com/reference/algorithm/none_of/
  std::array<int, 8> foo = {1,2,4,8,16,32,64,128};
  if (std::none_of(foo.begin(), foo.end(), [](int i){return i<0;})) {
    std::cout << "There are no negative elements in the range.\n";
  }

  // ------------------------std::any_of-------------------------------------
  // 只要有一个元素满足指定条件为真
  // http://www.cplusplus.com/reference/algorithm/any_of/
  std::array<int,7> foo2 = {0,1,-1,3,-3,5,-5};
  if ( std::any_of(foo2.begin(), foo2.end(), [](int i){return i<0;}) ) {
    std::cout << "There are negative elements in the range.\n";
  }

  cout << "=========End test TestContainerCondition=========" << endl;
}

int main() {
  ForEachFunction();

  CopyFunction();
  TransformFunction();

  SortFunction();

  PrintFunction();

  MaxMinFunction();

  FindFunction();

  CountFunction();

  TestProcessContainerFunction();

  AccumulateFunction();

  TestContainerCondition();
}