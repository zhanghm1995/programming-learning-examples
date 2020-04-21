/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-21 21:33:06
 * @LastEditTime: 2020-04-21 22:32:06
 * @References: 
 * @Description: Learn Eigen basic usages
 */

#include <iostream>
#include <Eigen/Dense>

using std::cout;
using std::endl;


void BasicDataType() {
  /// Default vector is a column vector, use Eigen::RowVectorXf if you want row vector
  Eigen::VectorXf v1(3);
  v1<<1,2,3;
  cout<<"v1=\n"<<v1<<endl;
  // typedef Eigen::Matrix<float, 1, -1> Eigen::RowVectorXf
  Eigen::RowVectorXf v2(3);
  v2<<1,2,3;
  cout<<"v2=\n"<<v2<<endl;

  /// Eigen matrix is stored in Column-Major
  Eigen::MatrixXf M1(3, 3); // Column major storage
  M1 << 1, 2, 3,
               4, 5, 6,
               7, 8, 9;
  // Convert Column-major storage matrix to Row-major storage matrix
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor> M2(M1);
  cout<<"M2=\n"<<M2<<endl;
}

/**
 * @brief Learn the matrix and vector initialization
 * @ref https://blog.csdn.net/zhanghm1995/article/details/80709455
 *            https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html
 */ 
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
  cout<<"=============CreateAndInitializeMatrix================="<<endl;
  CreateAndInitializeMatrix();
  cout<<"=============BasicDataType================="<<endl;
  BasicDataType();
  return 0;
}