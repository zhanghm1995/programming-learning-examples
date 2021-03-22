/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-26 22:59:58
 * @References: 
 * @Description:  Some math related calculation
 */

#include <iostream>
#include <cmath>
#include <vector>

using std::cout;
using std::endl;

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

template <typename DataType>
DataType deg2rad(const DataType degree)
{
    return degree * static_cast<DataType>(3.1415926535897932384 / 180.0);
}

/**
 * @brief Determine two angle direction by calculate the cross product
 */ 
bool Inverse(double theta1, double theta2) {
    double temp = cos(theta1) * sin(theta2) - sin(theta1) * cos(theta2);
    return temp < 0;
}

double SortTheta() {
    double x = 10, y = -10;
    double angle = std::atan2(y, x);
    double angle_deg = rad2deg(angle);

    double x2 = 10, y2 = 10;
    double angle2 = std::atan2(y2, x2);
    double angle2_deg = rad2deg(angle2);

    // Whether the same directioin from minimum angle to maximum angle
    if (Inverse(std::min(angle, angle2), std::max(angle, angle2))) {
        cout<<"Inverse"<<endl;
    } else {
        cout<<"Not Inverse"<<endl;
    }
}

/**
 * @brief Test std::atan2 function usage
 */ 
void TestAtan()
{
  std::vector<double> points_x = {10, 6, -1, -0.4, -10};
  std::vector<double> points_y = {0, 2, 8, -3, -0.5};

  for (int i = 0; i < points_y.size(); ++i) {
      auto x = points_x[i];
      auto y = points_y[i];
      auto tempb = (std::atan2(y, x)) * 180.0 / M_PI;
      printf("angle is %.3f\n", tempb);
  }
}

/**
 * @brief Get the Rainbow Color RGB value form a scalar value
 *                 refer to the sensor_msgs::PointCloud2 visualization scheme in Rviz software
 * @param value Should be in [0, 1]
 * @param color 
 */
static void getRainbowColor(float value, float* color)
{
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if ( !(i&1) ) f = 1 - f; // if i is even
  float n = 1 - f;

  if      (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
  else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
  else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
  else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
  else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;
}

void Range2Color(float min, float max, float val)
{
    auto diff = max - min;
    auto value = 1.0 - (val - min) / diff;

    float color[3];
    getRainbowColor(value, color);
    printf("rgb:[%f,%f,%f]\n", color[0], color[1], color[2]);
}

void TestColor()
{
    cout<<"-=============Begin TestColor==============="<<endl;
    std::vector<float> value_vec;
    value_vec.reserve(1000);
    for (float i = 0; i <= 96;) {
        value_vec.push_back(i);
        i += 0.5;
    }

    for (const auto& val : value_vec) {
        printf("val:%f ", val);
        Range2Color(0, 96, val);
    }
    cout<<"-=============End TestColor==============="<<endl;
}

int main() {
    SortTheta();

    double angle = 90.0;
    double angle_rad = deg2rad(angle);
    cout<<"angle_rad "<<angle_rad<<endl;

    TestColor();

    return 0;
}