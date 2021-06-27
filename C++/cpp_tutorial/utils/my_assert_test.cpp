/*
zhanghm@zhanghm-PC:build$ ./Mat_basics 
OpenCV Error: Assertion failed (coefs.channels() == 1) in ProcessMat, file /home/zhanghm/Programming/programming-learning-examples/OpenCV_learning/Mat_basics.cpp, line 17
terminate called after throwing an instance of 'cv::Exception'
  what():  /home/zhanghm/Programming/programming-learning-examples/OpenCV_learning/Mat_basics.cpp:17: error: (-215) coefs.channels() == 1 in function ProcessMat

已放弃 (核心已转储)
*/

#include "my_assert.h"

int main() {
  int a = 10;
  Assert(a == 0);
}