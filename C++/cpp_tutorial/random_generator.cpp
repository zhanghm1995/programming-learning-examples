/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-08-23 22:42:37
 * @References: 
 * @Description: Learn how to use random number in C++
 */

#include <iostream>
#include <random>
#include <ctime>
#include <map>
#include <iomanip>

using std::cout;
using std::endl;

void TestRandomGenerator() {
    std::random_device rd;
    std::default_random_engine rngl(rd());
    std::uniform_int_distribution<int> UniDist(1, 10);
    for (int i = 0; i < 20; ++i) {
        cout<<UniDist(rngl)<<endl;;
    }

    cout<<"==============="<<endl;
    for (int n = 0; n < 20; ++n) {
        cout<<rngl()<<endl;
    }
}

void NormalDistribution() {
    std::random_device rd;
    std::mt19937 gen(rd());
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(0,2);
    // std::uniform_int_distribution<> d(1,10);
 
    std::map<int, int> hist;
    for (int n=0; n<10000; ++n) {
        ++hist[std::round(d(gen))];
    }
    for (auto p : hist) {
        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
                  << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    }
}

void TestDistribution() {
    cout<<"================Begin TestDistribution===================="<<endl;
    std::random_device rd;  // 将用于为随机数引擎获得种子
    std::mt19937 gen(rd()); // 以播种标准 mersenne_twister_engine

    std::uniform_int_distribution<int> UniDist(1, 10);
    for (int i = 0; i < 20; ++i) {
        cout<<UniDist(gen)<<endl;;
    }

    std::normal_distribution<float> NormDist(1, 30);
    for (int i = 0; i < 50; ++i) {
        cout<<NormDist(gen)<<endl;;
    }
    cout<<"================End TestDistribution===================="<<endl;
}

/**
 * @brief 产生指定int范围内的随机数,不包括b
 */ 
void GenerateIntRand(int a, int b) {
    cout<<"================Begin GenerateIntRand===================="<<endl;
    const int max_temp = b - a;
    
    /// 产生int类型随机数
    for (int i = 0; i < 20; ++i) {
        auto r = (std::rand() % max_temp) + a;
        cout<<r<<endl;
    }
    cout<<"================End GenerateIntRand===================="<<endl;
}

int main() {
    GenerateIntRand(8, 20);

    TestDistribution();

    TestRandomGenerator();

    NormalDistribution();

    return 0;
}