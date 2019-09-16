#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

char window_name[30] = "HSV Segemtation";
Mat src;

static void onMouse( int event, int x, int y, int f, void* ){
  Mat image=src.clone();
  Vec3b rgb=image.at<Vec3b>(y,x);
  int B=rgb.val[0];
  int G=rgb.val[1];
  int R=rgb.val[2];

  Mat HSV;
  Mat RGB=image(Rect(x,y,1,1));
  cvtColor(RGB, HSV,CV_BGR2HSV);

  Vec3b hsv=HSV.at<Vec3b>(0,0);
  int H=hsv.val[0];
  int S=hsv.val[1];
  int V=hsv.val[2];

  char name[30];
  sprintf(name,"B=%d",B);
  putText(image,name, Point(150,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

  sprintf(name,"G=%d",G);
  putText(image,name, Point(150,80) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

  sprintf(name,"R=%d",R);
  putText(image,name, Point(150,120) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

  sprintf(name,"H=%d",H);
  putText(image,name, Point(25,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

  sprintf(name,"S=%d",S);
  putText(image,name, Point(25,80) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

  sprintf(name,"V=%d",V);
  putText(image,name, Point(25,120) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

  sprintf(name,"X=%d",x);
  putText(image,name, Point(25,300) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

  sprintf(name,"Y=%d",y);
  putText(image,name, Point(25,340) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

  //imwrite("hsv.jpg",image);
  imshow( window_name, image );
}

#if 1
int main(){
//  src = imread("bgr.jpg",1);
//  imshow(window_name,src);
//  setMouseCallback( window_name, onMouse, 0 );

  cv::Mat img_src(1000, 800, CV_8UC3, cv::Scalar(125, 125));
  cv::namedWindow("image_src",  CV_WINDOW_OPENGL);
  std::cout << cv::getBuildInformation() << std::endl;
  while (true) {
    imshow("image_src", img_src);
    int key = cvWaitKey(5);
    if (key == 'q') {
      break;
    }
  }
}
#endif
