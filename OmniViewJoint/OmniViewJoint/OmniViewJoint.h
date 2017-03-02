#pragma once
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;
#define SCREEN_W 320
#define SCREEN_H 480

#define SCAR_W 80
#define SCAR_H 320

#define NEW_MATRIX 0

vector<Point2f> calibPoints;
Mat imgShow;
Point pMerge[4][6];
Mat trans_Matrix = Mat::zeros(3, 3, CV_8U);

Mat imgCar;
Mat imgFront;   // 矫正
Mat imgRight1;
Mat imgRight2;
Mat imgBack1;
Mat imgLeft1;
Mat imgLeft2;
Mat imgBack2;

Mat disFront;   // 矫正
Mat disRight1;
Mat disRight2;
Mat disBack1;
Mat disLeft1;
Mat disLeft2;
Mat disBack2;

Mat proFront;  // 投影
Mat proRight1;
Mat proRight2;
Mat proBack1;
Mat proLeft1;
Mat proLeft2;
Mat proBack2;

Mat maskFront;   //掩膜
Mat maskRight1;
Mat maskRight2;
Mat maskBack1;
Mat maskLeft1;
Mat maskLeft2;
Mat maskBack2;
Mat maskCar;

// 扩展的mask
Mat maskETDFront;
Mat maskETDRight1;
Mat maskETDRight2;
Mat maskETDBack1;
Mat maskETDLeft1;
Mat maskETDLeft2;
Mat maskETDBack2;
Mat maskETDCar;

//缩减的mask
Mat maskCUTFront;
Mat maskCUTRight1;
Mat maskCUTRight2;
Mat maskCUTBack1;
Mat maskCUTLeft1;
Mat maskCUTLeft2;
Mat maskCUTBack2;
Mat maskCUTCar;

Mat omniView(Size(SCREEN_W, SCREEN_H), CV_8UC3, Scalar(0, 0, 0));

Mat mapx;
Mat mapy;