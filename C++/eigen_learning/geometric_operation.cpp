/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年8月11日
* Copyright    :
* Descriptoin  : Geometric operations in Eigen
* References   : http://eigen.tuxfamily.org/dox/group__Geometry__Module.html#ga1f220045efa302626c287088b63b6ba9
======================================================================*/

#include <iostream>
#include <Eigen/Dense>
using namespace std;

using std::cout;
using std::endl;

/**
 * @brief Learn how to do rigid body transformation in Eigen
 */ 
void TransformRigidCoordinate() {
  Eigen::Translation3d translation(1.2, 3.4, 5.7);
  Eigen::Quaterniond rotation(1.0, 0, 0, 0);
  Eigen::Affine3d trans = translation * rotation;

   Eigen::Matrix4d pose = trans.matrix();
  cout<<pose<<endl;
}

int main()
{
  Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d (0,0,1));
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();// 虽然称为3d，实质上是4＊4的矩阵　　齐次坐标
  T.prerotate (rotation_vector);                                        // 按照rotation_vector进行旋转
  cout << "Transform matrix = \n" << T.matrix() <<endl;
  T.pretranslate(Eigen::Vector3d (1, 3, 4));               // 把平移向量设成(1,3,4)
  cout << "Transform matrix = \n" << T.matrix() <<endl;


  // 用变换矩阵进行坐标变换
  Eigen::Vector3d v(1, 3, 4);
  Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t

  T.translate ( Eigen::Vector3d ( 1,3,4 ) );               // 把平移向量设成(1,3,4)
  cout << "Transform matrix = \n" << T.matrix() <<endl;
  cout<<"After Isometry3d tranformed = "<<v_transformed.transpose()<<endl;

  cout<<"=========TransformRigidCoordinate=============="<<endl;
  TransformRigidCoordinate();
}