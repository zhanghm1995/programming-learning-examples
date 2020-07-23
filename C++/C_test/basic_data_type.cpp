#include <typeinfo>
#include <iostream>
#include <limits>
#include <cfloat>

using std::cout;
using std::endl;

void UseTypeInfo() {
    cout << "=========Begin test UseTypeInfo=========" << endl;
    bool bool_var;
    char char_var;
    int int_var;
    unsigned int uint_var;
    long long_var;
    unsigned long ulong_var;
    long long longlong_var;
    unsigned long long ulonglong_var;
    float float_var;
    double double_var;

    cout<<typeid(bool_var).name()<<endl;
    cout<<typeid(char_var).name()<<endl;
    cout<<typeid(int_var).name()<<endl;
    cout<<typeid(uint_var).name()<<endl;
    cout<<typeid(long_var).name()<<endl;
    cout<<typeid(ulong_var).name()<<endl;
    cout<<typeid(longlong_var).name()<<endl;
    cout<<typeid(ulonglong_var).name()<<endl;
    cout<<typeid(float_var).name()<<endl;
    cout<<typeid(double_var).name()<<endl;
    cout << "=========End test UseTypeInfo=========" << endl;
}

void UseSizeOf() {
    cout << "=========Begin test UseSizeOf=========" << endl;
    cout<<"bool byte size: "<<sizeof(bool)<<endl;
    cout<<"char byte size: "<<sizeof(char)<<endl;
    cout<<"short int byte size: "<<sizeof(short int)<<endl;
    cout<<"unsigned short int byte size: "<<sizeof(unsigned short int)<<endl;
    cout<<"int byte size: "<<sizeof(int)<<endl;
    cout<<"unsigned int byte size: "<<sizeof(unsigned int)<<endl;
    cout<<"long int byte size: "<<sizeof(long int)<<endl;
    cout<<"long long int byte size: "<<sizeof(long long int)<<endl;
    cout<<"float byte size: "<<sizeof(float)<<endl;
    cout<<"double byte size: "<<sizeof(double)<<endl;
    cout << "=========Begin test UseSizeOf=========" << endl;
}

void UseNumericalLimit() {
    cout << "=========Begin test UseNumericalLimit=========" << endl;
    cout<<"uint16_t min: "<< std::numeric_limits<uint16_t>::min()<<endl;
    cout<<"uint16_t max: "<< std::numeric_limits<uint16_t>::max()<<endl;

    cout<<"short min: "<< std::numeric_limits<short>::min()<<endl;
    cout<<"short max: "<< std::numeric_limits<short>::max()<<endl;

    int int_min = std::numeric_limits<int>::min();
    cout<<"int min: "<<int_min<<endl;
    int int_max = std::numeric_limits<int>::max();
    cout<<"int max: "<<int_max<<endl;

    unsigned int unsigned_int_min = std::numeric_limits<unsigned int>::min();
    cout<<"unsigned int min: "<<unsigned_int_min<<endl;
    unsigned int unsigned_int_max = std::numeric_limits<unsigned int>::max();
    cout<<"unsigned int max: "<<unsigned_int_max<<endl;

    float float_min = std::numeric_limits<float>::min();
    cout<<"float min: "<<float_min<<endl;
    float float_max = std::numeric_limits<float>::max();
    cout<<"float max: "<<float_max<<endl;

    double double_min = std::numeric_limits<double>::min();
    cout<<"double min: "<<double_min<<endl;
    double double_max = std::numeric_limits<double>::max();
    cout<<"double max: "<<double_max<<endl;

    double_min = DBL_MIN;
    cout<<double_min<<endl;
    double_max = DBL_MAX;
    cout<<double_max<<endl;
    
    cout << "=========Begin test UseNumericalLimit=========" << endl;
}

void MiscTest() {
    cout << "=========Begin test MiscTest=========" << endl;
    auto x = -10U;
    cout<<x<<endl;

    cout<<typeid(0).name()<<endl;
    cout<<typeid(-10U).name()<<endl;
    cout<<typeid(2147483647).name()<<endl;
    cout<<typeid(2147483648).name()<<endl;
    cout<<"10L: "<<typeid(10L).name()<<endl;
    cout<<"10U: "<<typeid(10U).name()<<endl;

    cout<<typeid(10.0).name()<<endl;
    cout<<typeid(10.0F).name()<<endl;
    cout << "=========Begin test MiscTest=========" << endl;
}

// int main() {
//     UseTypeInfo();

//     UseSizeOf();

//     UseNumericalLimit();

//     MiscTest();
//     return 0;
// }


#include <cmath>
int main() {
    cout<<"0: "<<typeid(0).name()<<endl;
    cout<<"-10: "<<typeid(-10).name()<<endl;
    cout<<"2147483647: "<<typeid(2147483647).name()<<endl;
    cout<<"2147483648: "<<typeid(2147483648).name()<<endl;
    cout<<"10L: "<<typeid(10L).name()<<endl;
    cout<<"10U: "<<typeid(10U).name()<<endl;

    cout<<"10.0: "<<typeid(10.0).name()<<endl;
    cout<<"10.0F: "<<typeid(10.0F).name()<<endl;

    cout<<-std::pow(2, 128)<<endl;
    cout<<std::pow(2, 128)<<endl;

    cout<<"----"<<endl;
    cout<<std::numeric_limits<double>::lowest()<<endl;
    cout<<std::numeric_limits<double>::max()<<endl;
    cout<<std::pow(2, 1024)<<endl;
}