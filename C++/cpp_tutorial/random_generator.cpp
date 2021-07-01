/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-08-23 22:42:37
 * @References: https://rextester.com/discussion/CAMZJ34875/Random-values-and-probability-distribution
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

void TestDiscreteDistribution() {
    const int nrolls = 100000; // number of experiments
    const int nstars = 100; //maximum number of stars to distribute
    std::random_device rd;
    std::default_random_engine generator(rd());
    
    /// 分配相同权重情形
    // std::vector<int> weights(10, 1);

    /// 分配不同权重情形
    std::vector<int> weights = {2, 2, 1, 1, 2, 2, 1, 1, 2, 2};

    std::discrete_distribution<int> distributions{std::begin(weights), std::end(weights)};


    int p[10] = {};

    for (int i = 0; i < nrolls; ++i) {
        int number = distributions(generator);
        ++p[number];
    }

    std::cout<<"a discrete_distribution:"<<std::endl;
    for (int i = 0; i < 10; ++i) {
        std::cout<<i<<": "<<std::string(p[i]*nstars/nrolls, '*')<<std::endl;
    }
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

/**
 * @brief Generate a random real type value between a and b.
 * 
 * @param a 
 * @param b 
 * @return double 
 */
double GenerateRandomValue(double a, double b) {
    std::random_device rd;
    std::default_random_engine rng{rd()};

    std::uniform_real_distribution<> real_dist{a, b};
    return real_dist(rng);
}

int main() {
    GenerateIntRand(8, 20);

    TestDistribution();

    TestRandomGenerator();

    NormalDistribution();

    TestDiscreteDistribution();

    return 0;
}