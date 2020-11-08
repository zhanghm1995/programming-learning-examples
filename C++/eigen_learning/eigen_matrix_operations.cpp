/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-11 08:34:02
 * @LastEditTime: 2020-04-22 22:41:00
 * @References: Learn some operations on Eigen matrix
 * @Description: 
 */
// C++
#include <iostream>
#include <vector>
// Eigen
#include <Eigen/Dense>

using std::cout;
using std::endl;

/**
 * @brief Using << to stack two matrix, note that the VectorXf is column vector by default
 *                Eigen choose to stack two matrix horizontally or vertically according to the output matrix size
 * @ref https://stackoverflow.com/questions/21496157/eigen-how-to-concatenate-matrix-along-a-specific-dimension
 */ 
void StackTwoMatrix() {
  // Eigen::MatrixXf A(3,  4);
  // A<< 1, 3, 3, 2, 
  //          3, 4, 5, 1, 
  //          12, 34, 12, 3;

  Eigen::Vector3f A(1,2,3); // column vector
  Eigen::MatrixXf B = Eigen::MatrixXf::Ones(1, A.cols());

  Eigen::MatrixXf C(A.rows() + B.rows(), A.cols());
  C << A,
            B;
  std::cout<<C<<std::endl;     
}

/**
 * @brief Show how to reshape matrix so we can get different size matrix
 * @ref https://eigen.tuxfamily.org/dox/group__TutorialReshapeSlicing.html
 */ 
Eigen::MatrixXf ReshapeMatrix() {
  /// 将矩阵转为向量
  Eigen::MatrixXf M1(3, 3); // Column major storage
  M1 << 1, 2, 3,
               4, 5, 6,
               7, 8, 9;
  Eigen::Map<Eigen::RowVectorXf> v1(M1.data(), M1.size());
  cout<<v1<<endl;
  
  // Convert Column-major storage matrix to Row-major storage matrix
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor> M2(M1);
  cout<<M2<<endl;
  Eigen::Map<Eigen::RowVectorXf> v2(M2.data(), M2.size());
  cout << "v2:" << endl << v2 << endl;

  /// From vector to matrix
  Eigen::VectorXf dd(8);
  dd << 1, 2, 3, 4, 5, 6, 7, 8;
  cout<<dd<<endl;
  Eigen::Map<Eigen::Matrix<float, 4, 2, Eigen::RowMajor> > M3(dd.data());
  cout<<"M3 = "<<M3<<endl;
  return M3;
}

void OperateMatrixRowsAndCols() {
  /// 1) 矩阵每一行除以最后一行操作
  Eigen::MatrixXf M1(3, 3); // Column major storage
  M1 << 4, 5, 6,
               7, 8, 9,
               1, 2, 3;
  M1.array().rowwise() /= M1.row(2).array();
  cout<<M1.topRows(2)<<endl;

  /// 2) 矩阵统一加上某个向量值(broadcasting操作)
  cout<<"-----"<<endl;
  cout<<M1<<endl;
  Eigen::VectorXf tr(3);//此处必须是Vector类型
  tr << 1, 3, 5;

  Eigen::MatrixXf temp = M1.colwise() + tr;
  cout<<temp<<endl;
}

/**
 * @brief
 * @ref https://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
 */ 
void OperateBlock() {
  /// 1) 将矩阵转为齐次矩阵
  Eigen::Matrix4f temp2 = Eigen::Matrix4f::Zero();
  temp2(3, 3) = 1.0;
  Eigen::MatrixXf M1(3, 3); // Column major storage
  M1 << 4, 5, 6,
               7, 8, 9,
               1, 2, 3;
  temp2.topLeftCorner(3, 3) = M1;
  cout<<temp2<<endl;
}

/**
 * @brief Learn how to copy matrix deeply or shallow
 * @ref https://stackoverflow.com/questions/29231798/eigen-how-to-make-a-deep-copy-of-a-matrix?rq=1
 *            https://stackoverflow.com/questions/51018326/an-error-occured-using-eigenmap-after-pass-by-const-reference
 */ 
void DeepCopyMatrix() {
  using namespace Eigen;
  /// 尽量不要使用auto来进行赋值,可能会出现浅拷贝问题
  typedef Matrix<double,Dynamic,Dynamic,RowMajor> MyMatrix;
  double a[] = {1,2,3,4};
  auto M = Map<MyMatrix>(a, 2, 2);
  auto G = M;
  MatrixXd g = M; // 不用auto是深拷贝
  G(0,0) = 0;
  std::cout << M << "\n" << std::endl;
  std::cout << G << "\n" << std::endl;
  std::cout << g << "\n" << std::endl;

  /// Map的不同使用方式会出现内存共享的问题
  double my_floats[] = { 32,71,12,45,26,80,53,33 };
  std::vector<double> myvector(my_floats, my_floats + 8); // 深拷贝,myvector和数组无关
  Map<MyMatrix> MyM1(myvector.data(), 2, 4); // 浅拷贝,改变myvector会影响到MyM1矩阵
  cout<<"MyM1 = \n"<<MyM1<<endl;
  myvector[0] = 10;
  cout<<"MyM1 = \n"<<MyM1<<endl;

  Eigen::MatrixXd MyM2 = Map<MyMatrix>(myvector.data(), 2, 4); // 深拷贝,改变myvector不会影响到MyM2矩阵
  cout<<"MyM2 = \n"<<MyM2<<endl;
  myvector[1] = 30;
  cout<<"MyM2 = \n"<<MyM2<<endl;

  Eigen::MatrixXd MyM3 = MyMatrix::Map(myvector.data(), 2, 4); // 深拷贝,改变myvector不会影响到MyM3矩阵
  cout<<"MyM3 = \n"<<MyM3<<endl;
  myvector[2] = 50;
  cout<<"MyM3 = \n"<<MyM3<<endl;
}

int main() {
  cout<<"=========StackTwoMatrix============"<<endl;
  StackTwoMatrix();
  cout<<"=========ReshapeMatrix============"<<endl;
  Eigen::MatrixXf matrix = ReshapeMatrix();
  cout<<"matrix = "<<matrix<<endl;
  cout<<"=========OperateMatrixRowsAndCols============"<<endl;
  OperateMatrixRowsAndCols();
  cout<<"=========OperateBlock============"<<endl;
  OperateBlock();
  cout<<"=========DeepCopyMatrix============"<<endl;
  DeepCopyMatrix();
  cout<<"====================="<<endl;
  return 0;
}