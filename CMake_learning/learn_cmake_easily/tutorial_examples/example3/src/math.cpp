/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-19 11:42:33
 * @LastEditTime: 2020-04-19 11:57:51
 * @References: 
 * @Description: 
 */

#include "math.h"

double deg2rad(double degree) {
    return degree * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}