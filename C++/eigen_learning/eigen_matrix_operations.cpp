/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-11 08:34:02
 * @LastEditTime: 2020-04-11 16:20:25
 * @References: 
 * @Description: 
 */
// C++
#include <iostream>
// Eigen
#include <Eigen/Dense>

using std::cout, std::endl;

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
  /// 1) 矩阵每一行除以最后一行
  Eigen::MatrixXf M1(3, 3); // Column major storage
  M1 << 4, 5, 6,
               7, 8, 9,
               1, 2, 3;
  M1.array().rowwise() /= M1.row(2).array();
  cout<<M1.topRows(2)<<endl;
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

int main() {
  cout<<"====================="<<endl;
  StackTwoMatrix();
  cout<<"====================="<<endl;
  Eigen::MatrixXf matrix = ReshapeMatrix();
  cout<<"matrix = "<<matrix<<endl;
  cout<<"====================="<<endl;
  OperateMatrixRowsAndCols();
  cout<<"====================="<<endl;
  OperateBlock();
  return 0;
}