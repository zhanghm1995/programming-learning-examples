#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace std;
using namespace cv;
// 创建滑动条

#define WINDOW_NAME "name" //定义一个宏，相当于是一个固定的名字
// 全局变量声明
const int g_nMaxAlaphaValue=100;//alapha的最大的值
int g_nAlaphaValueSlider;//滑动条对应的变量
double g_dAlaphaValue;
double g_dBetaValue;
//声明存储图像的变量
Mat g_srcImage1;
Mat g_srcImage2;
Mat g_dstImage;



//--------响应滑动条的回调函数
void on_Trackbar(int,void*)
{
    //求出当前alapha值相对于最大值的比例
    g_dAlaphaValue=(double)g_nAlaphaValueSlider/g_nMaxAlaphaValue;
    //则beta值为1减去alapha的值
    g_dBetaValue=(1.0-g_dAlaphaValue);

    //根据alapha和beta值进行线性混合
    addWeighted(g_srcImage1,g_dAlaphaValue,g_srcImage2,g_dBetaValue,0.0,g_dstImage);

    //显示效果图
    imshow(WINDOW_NAME,g_dstImage);

}

//---------主函数
int main(int argc, char *argv[])
{
    //加载图像
   g_srcImage1 = imread("../data/tracker_bar_1.jpg");
   cv::Mat srcImage2 = imread("../data/tracker_bar_2.jpg");
   std::cout<<"hhhhh"<<std::endl;
   cv::resize(srcImage2, g_srcImage2, g_srcImage1.size());


   //设置滑动条初值为70
   g_nAlaphaValueSlider=60;

   //创建窗体
   namedWindow(WINDOW_NAME,1);

   //在创建的窗体中创建一个滑动条控件
   char TrackbarName[50];
   sprintf(TrackbarName,"name%d",g_nMaxAlaphaValue);               //字符串格式化
   createTrackbar(TrackbarName,WINDOW_NAME,&g_nAlaphaValueSlider,g_nMaxAlaphaValue,on_Trackbar);

   //结果在回调函数中显示
   on_Trackbar(g_nAlaphaValueSlider,0);

   while(1)
   {
       int key=cvWaitKey(10);
   if (key==27)
   {
       break;
   }
   }
   return(0);
}