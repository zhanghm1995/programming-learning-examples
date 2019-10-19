#include "macro_define.h"
#define MaxLength 4.98

using VectInt = std::vector<int>;

class Example1 {
public:
    Example1()
    {
        length_ = MaxLength;
        vec_ = {1, 2, 3, 4, 5};
    }

    double GetLength() const { return length_; }
private:
    double length_;
    VectInt vec_;
};