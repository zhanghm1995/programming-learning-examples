/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年8月10日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/


#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <list>
using std::cout;
using std::endl;

bool isPass(int n)
{
  return n >= 60;
}


// idea from boost any but make it more simple and don't use type_info.
class Any {
 public:
  Any() : content_(NULL) {}

  template <typename ValueType>
  explicit Any(const ValueType &value)
      : content_(new Holder<ValueType>(value)) {}

  Any(const Any &other)
      : content_(other.content_ ? other.content_->Clone() : nullptr) {}

  ~Any() { delete content_; }

  template <typename ValueType>
  ValueType *AnyCast() {
    return content_ ? &(static_cast<Holder<ValueType> *>(content_)->held_)
                    : nullptr;
  }

 private:
  class PlaceHolder {
   public:
    virtual ~PlaceHolder() {}
    virtual PlaceHolder *Clone() const = 0;
  };

  template <typename ValueType>
  class Holder : public PlaceHolder {
   public:
    explicit Holder(const ValueType &value) : held_(value) {}
    virtual ~Holder() {}
    virtual PlaceHolder *Clone() const { return new Holder(held_); }

    ValueType held_;
  };

  PlaceHolder *content_;
};

class ObjectFactory {
 public:
  ObjectFactory() {}
  virtual ~ObjectFactory() {}
  virtual Any NewInstance() { return Any(); }
  ObjectFactory(const ObjectFactory &) = delete;
  ObjectFactory &operator=(const ObjectFactory &) = delete;

 private:
};

typedef std::map<std::string, ObjectFactory *> FactoryMap;
typedef std::map<std::string, FactoryMap> BaseClassMap;
BaseClassMap &GlobalFactoryMap();

bool GetRegisteredClasses(
    const std::string &base_class_name,
    std::vector<std::string> *registered_derived_classes_names);


#define PERCEPTION_REGISTER_REGISTERER(base_class)                    \
  class base_class##Registerer {                                      \
   public:                                                            \
    static base_class *GetInstanceByName(const ::std::string &name) { \
      FactoryMap &map =                                               \
          GlobalFactoryMap()[#base_class];                            \
      FactoryMap::iterator iter = map.find(name);                     \
      if (iter == map.end()) {                                        \
        for (auto c : map) {                                          \
          std::cout << "Instance:" << c.first;                           \
        }                                                             \
        std::cout << "Get instance " << name << " failed.";              \
        return nullptr;                                                  \
      }                                                               \
      Any object = iter->second->NewInstance();                       \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
  };


class BaseMultiTargetTracker {
 public:
  BaseMultiTargetTracker() = default;

  virtual ~BaseMultiTargetTracker() = default;

  // @brief: track segmented objects, and estimate motion
  // @param [in]: options
  // @param [in/out]: tracked object
  // tracked objects should be filled, required,
  virtual std::string Name() const = 0;
};  // class BaseMultiTargetTracker

PERCEPTION_REGISTER_REGISTERER(BaseMultiTargetTracker);

using std::vector;
class ObjectList {
public:
  void push_back(const int num)
  {
    ++cnt_;
    num_.push_back(num);
  }
private:
  int cnt_ = 0;
  vector<int> num_;
};
int main()
{
  std::vector<int> vecScore = {72, 54, 87};


  return 0;
}
