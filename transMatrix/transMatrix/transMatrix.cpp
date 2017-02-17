// transMatrix.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv.hpp"

using namespace cv;
using namespace std;

int main()
{
	vector<Point2f> imgPts(9);
	vector<Point2f> objPts(9);

	imgPts[0] = Point2f(442, 531);
	imgPts[1] = Point2f(633, 538);
	imgPts[2] = Point2f(827, 545);
	imgPts[3] = Point2f(382, 596);
	imgPts[4] = Point2f(630, 606);
	imgPts[5] = Point2f(879, 614);
	imgPts[6] = Point2f(320, 663);
	imgPts[7] = Point2f(625, 679);
	imgPts[8] = Point2f(933, 691);

	objPts[0] = Point2f(280, 1280 - 700);
	objPts[1] = Point2f(430, 1280 - 700);
	objPts[2] = Point2f(580, 1280 - 700);
	objPts[3] = Point2f(280, 980 - 700);
	objPts[4] = Point2f(430, 980 - 700);
	objPts[5] = Point2f(580, 980 - 700);
	objPts[6] = Point2f(280, 780 - 700);
	objPts[7] = Point2f(430, 780 - 700);
	objPts[8] = Point2f(580, 780 - 700);
	
	Mat transMatrix = findHomography(imgPts, objPts);
	Mat frame = imread("3S520-7.8-9.8-12.8-1.5.jpg");
	Mat trans;
	warpPerspective(frame, trans, transMatrix, Size(860, 2000));
	resize(trans, trans, Size(430, 1000));
	imshow("1", trans);
	waitKey();
	return 0;
}

