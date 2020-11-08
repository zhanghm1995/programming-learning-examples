//功能：用轨迹条来控制两幅图像的Alpha混合
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <iostream> 
using namespace cv;
using namespace std;
 
#define WINDOW_NAME "【带有滑动条的两图像融合】"	//为窗口标题定义的宏
 
//----------【全局变量声明部分】----------
const int g_nMaxAlphaValue = 100;	//Alpha最大值
int g_nAlphaNow;  //滑动条此刻对应的值
double g_dAlphaValue;	//Alpha值
double g_dBetaValue;	//Beta值
Mat g_srcImage1;	//源图像1
Mat g_srcImage2;	//源图像2	
Mat g_dstImage;		//输出图像
 
//-------------【全局函数声明部分】----------------
void on_TrackBarChange(int, void*);	//响应滑动条的回调函数
 
//-------------------【主程序】-----------------
int main(int argc, char** argv)
{
	//【1】读取原始图像（两幅图像尺寸需相同）并检查图像是否读取成功  
	g_srcImage1 = imread("/home/zhanghm/图片/1.jpg");
	cv::Mat srcImage2 = imread("/home/zhanghm/图片/2.jpg");
    cv::resize(srcImage2, g_srcImage2, g_srcImage1.size());
	if (!g_srcImage1.data)
	{
		cout << "读取第一幅图片错误，请重新输入正确路径！\n";
		return -1;
	}
	if (!g_srcImage2.data)
	{
		cout << "读取第二幅图片错误，请重新输入正确路径！\n";
		return -1;
	}
 
	//【2】设置滑动条初始值为50
	g_nAlphaNow = 50;
	//【3】创建窗体
	namedWindow(WINDOW_NAME, 1);//参数一：窗体名称。参数二：0表示窗口尺寸可手动调整；1表示窗口尺寸为图像尺寸大小，且固定不可调。
	//【4】在创建的窗体中创建一个滑动条控件
	createTrackbar("透明值", WINDOW_NAME, &g_nAlphaNow, g_nMaxAlphaValue, on_TrackBarChange);
	//【5】结果在回调函数中显示
	on_TrackBarChange(g_nAlphaNow, 0);
	//【6】保持等待状态，按任意键退出
	waitKey(0);
	return 0;
}
 
//-------------【响应滑动条的回调函数】----------------
void on_TrackBarChange(int, void*)
{
	//求出当前alpha值相对于最大值的比例
	g_dAlphaValue = (double)g_nAlphaNow / g_nMaxAlphaValue;
	//则beta值为1减去alpha值
	g_dBetaValue = (1.0 - g_dAlphaValue);
	//根据alpha和beta值进行线性混合
	addWeighted(g_srcImage1, g_dAlphaValue, g_srcImage2, g_dBetaValue, 0.0, g_dstImage);
	//显示效果图
	imshow(WINDOW_NAME, g_dstImage);
}