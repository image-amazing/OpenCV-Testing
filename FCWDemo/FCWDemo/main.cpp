#include <iostream>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include "OCamCalib.h"

using namespace std;
using namespace cv;

void getImage(Mat& frameFV, Mat& frameBV, Mat& table)
{
	int* pTable;

	for (unsigned i = 1; i < table.rows; i++)
	{
		pTable = table.ptr<int>(i);

		int xBV = pTable[0];
		int yBV = pTable[1];
		int xFV = pTable[2];
		int yFV = pTable[3];

		uchar rValue = frameFV.at<uchar>(yFV, 3 * xFV);
		uchar gValue = frameFV.at<uchar>(yFV, 3 * xFV + 1);
		uchar bValue = frameFV.at<uchar>(yFV, 3 * xFV + 2);

		frameBV.at<uchar>(yBV, 3 * xBV) = rValue;
		frameBV.at<uchar>(yBV, 3 * xBV + 1) = gValue;
		frameBV.at<uchar>(yBV, 3 * xBV + 2) = bValue;

	}
}

void seamRefineFromTables(Mat& frameFV1, Mat& frameFV2, Mat& frameBV, Mat& tableSeam)
{
	int* pTable;

	for (unsigned i = 1; i < tableSeam.rows; i++)
	{
		pTable = tableSeam.ptr<int>(i);

		int xBV = pTable[0];
		int yBV = pTable[1];
		int xFV1 = pTable[2];
		int yFV1 = pTable[3];
		int mu1 = pTable[4];
		int xFV2 = pTable[5];
		int yFV2 = pTable[6];
		int mu2 = pTable[7];

		frameBV.at<uchar>(yBV, xBV * 3) = mu1 * frameFV1.at<uchar>(yFV1, xFV1 * 3) / 100 + mu2 * frameFV2.at<uchar>(yFV2, xFV2 * 3) / 100;
		frameBV.at<uchar>(yBV, xBV * 3 + 1) = mu1 * frameFV1.at<uchar>(yFV1, xFV1 * 3 + 1) / 100 + mu2 * frameFV2.at<uchar>(yFV2, xFV2 * 3 + 1) / 100;
		frameBV.at<uchar>(yBV, xBV * 3 + 2) = mu1 * frameFV1.at<uchar>(yFV1, xFV1 * 3 + 2) / 100 + mu2 * frameFV2.at<uchar>(yFV2, xFV2 * 3 + 2) / 100;
	}
}

int main()
{
	///////////////////////////
	//       6，验证查找表
	///////////////////////////
	Mat frame1 = imread("frame1.bmp");
	Mat frame2 = imread("frame2.bmp");
	Mat frame3 = imread("frame3.bmp");
	Mat frame4 = imread("frame4.bmp");

	Mat tableFront;
	Mat tableBack;
	Mat tableLeft;
	Mat tableRight;

	FileStorage fs("tables.yml", FileStorage::READ);
	fs["Table Front"] >> tableFront;
	fs["Table Back"] >> tableBack;
	fs["Table Left"] >> tableLeft;
	fs["Table Right"] >> tableRight;
	fs.release();

	Mat omniViewCheck(Size(SCREEN_W, SCREEN_H), CV_8UC3, Scalar(0, 0, 0));
	getImage(frame1, omniViewCheck, tableFront);
	getImage(frame2, omniViewCheck, tableBack);
	getImage(frame3, omniViewCheck, tableLeft);
	getImage(frame4, omniViewCheck, tableRight);

	Mat tableSeam1;
	Mat tableSeam2;
	Mat tableSeam3;
	Mat tableSeam4;

	FileStorage fs2("tablesSeam.yml", FileStorage::READ);
	fs2["Seam1"] >> tableSeam1;
	fs2["Seam2"] >> tableSeam2;
	fs2["Seam3"] >> tableSeam3;
	fs2["Seam4"] >> tableSeam4;
	seamRefineFromTables(frame1, frame3, omniViewCheck, tableSeam1);	//右在前，左在后
	seamRefineFromTables(frame4, frame1, omniViewCheck, tableSeam2);
	seamRefineFromTables(frame2, frame3, omniViewCheck, tableSeam3);
	seamRefineFromTables(frame4, frame2, omniViewCheck, tableSeam4);

	imshow("OmniView", omniViewCheck);
	waitKey();

	return 0;

}