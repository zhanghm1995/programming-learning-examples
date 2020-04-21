/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-21 21:33:06
 * @LastEditTime: 2020-04-21 21:55:06
 * @References: 
 * @Description: Learn Eigen basic usages
 */

#include <iostream>
#include <Eigen/Dense>

using std::cout;
using std::endl;

void CreateAndInitializeMatrix() {
  /// Default initialization
  Eigen::Matrix3f a; // random value
  Eigen::VectorXf b(5); // all 0 value
  cout<<"a=\n"<<a<<endl;
  cout<<"b=\n"<<b<<endl;

  /// Simple initialization
  Eigen::Vector2d a1(5.0, 6.0);
  Eigen::Vector3d a2(5.0, 6.0, 7.0);
  Eigen::Vector4d a3(5.0, 6.0, 7.0, 8.0);
  cout<<"a3=\n"<<a3<<endl;

  /// Comma initialization
  Eigen::Matrix3f m;
  m << 1, 2, 3, 
             4, 5, 6, 
             7, 8, 9;
  cout<<"m=\n"<<m<<endl;

  Eigen::MatrixXd m1(3, 3); // must specify the size, otherwise cause segmentation fault
  m1 <<1,2,3,
              4,5,6,
              7,8,9;
  /// Special matrix initialization
  Eigen::MatrixXf M1 = Eigen::MatrixXf::Zero(3,4);
  Eigen::MatrixXf M2 = Eigen::MatrixXf::Ones(3,3);
  Eigen::MatrixXi M3 = Eigen::MatrixXi::Identity(3,3);
  Eigen::MatrixXf M4 = Eigen::Matrix3f::Random();
  cout<<"M3=\n"<<M3<<endl;
  Eigen::Vector3f V1 = Eigen::Vector3f::Ones();
}

int main() {
  CreateAndInitializeMatrix();
  return 0;
}