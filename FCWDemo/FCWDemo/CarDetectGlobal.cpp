///***********************************************************************************************************************
//...............................苏州清研微视电子科技有限公司    ©版权所有   未经许可 不得使用...............................
//文件名：carDetectGlobal.cpp
//主要功能：
//作者：
//起草时间：
//修订记录：
//备注：
//***********************************************************************************************************************/
//#include <stdio.h>
////#include "carDetect.h"
////#include "Ldw.h"
//#include <queue>
//#include <windows.h>
//#include "Fcw1.h"
//
//#pragma comment(lib, "winmm.lib")
//
//#define CAL_SPEED_NUMBER 10
//#define VIDEO_NAME "20170215121258.avi"
////#define VIDEO_NAME "2016-09-29-11-49-07.avi"
////#define VIDEO_NAME "2.avi"
//
//#define USE_RADAR 0
//#define USE_IMAGE 1
//
//inline void GetPointInLine(Vec4f line, int y, Point& point)
//{
//	if (line[0] * line[1] == 0)
//	{
//		point.x = 0;
//		point.y = 0;
//	}
//
//	point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
//	point.y = y;
//}
//
//void PlotSingleLine(Mat& frame, Vec4f& plotLine, double dL)
//{
//	Point p1, p2;
//
//	GetPointInLine(plotLine, frame.rows * 0.5, p1);
//	GetPointInLine(plotLine, frame.rows, p2);
//	line(frame, p1, p2, Scalar(255, 255, 0), 10);
//}
//
//
//float calculateSafeDist(unsigned short Speed)
//{
//	float iCarRange = 0;
//	float iManRange = 0;
//	float iManTime = 1.f;
//	float iCarU = 0.8f;
//	float iSpeed = Speed / 10.f;
//
//	iCarRange = (float)((iSpeed*iSpeed) / (3.6 * 2 * 9.8 * 3.6) / iCarU);
//	iManRange = iSpeed * iManTime / 3.6f;
//
//	return (iCarRange + iManRange);
//}
//
//int main()
//{
//	//// System Init
//	//VideoCapture capture(0);
//	//capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//	//capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
//
//	VideoCapture capture;
//	if (!capture.open(VIDEO_NAME))
//	{
//		cout << "Cannot open the video." << endl;
//		return -1;
//	}
//	CascadeClassifier cascade1;
//	if (!cascade1.load("cascade_995_50.xml"))
//	{
//		cout << "Cannot load the cascade." << endl;
//		return -1;
//	}
//
//	CascadeClassifier cascade2;
//	if (!cascade2.load("cascade_995_50.xml"))
//	{
//		cout << "Cannot load the cascade." << endl;
//		return -1;
//	}
//
//	Ldw cLdw;
//	cLdw.InitMatrixData();
//
//	Fcw cFcw;
//	cFcw.pLdw = &cLdw;
//
//	Mat frame;
//	int frameNum = 0;
//	double t;
//	//CAR_TARGET car, carPre;
//
//	Mat framePlot;
//	unsigned char* buf = (unsigned char*)malloc(4000 * 4000);
//
//	int c = 0;
//
//	//CAR_TARGET curTarget;
//	//CAR_TARGET preTarget;
//	Mat greyFrame;
//	//preTarget.flag = 0;
//	char saveName[400];
//	int size = 0;
//	char num_name[20];
//	unsigned short radar_speed = 0;
//	float radar_dist = 0;
//	float safeDist = 0;
//
//	float img_speed = 0;
//	float img_dist = 0;
//
//	unsigned int WarnFlag = 0;
//	string save_path = "D:\\dataSet\\";
//	string save_name;
//	//curTarget.flag = 0;
//	t = (double)getTickCount();
//
//	Mat smallFrame;
//	while (c != 27)
//	{
//		capture >> frame;
//
//		frameNum++;
//		//sprintf(num_name, "%.6d", frameNum);
//
//		save_name = save_path + "Img_" + num_name + ".jpg";
//
//		if (!frame.empty())
//		{
//			cvtColor(frame, greyFrame, CV_RGB2GRAY);
//			cout << "Frame No. " << frameNum << endl;
//
//			////LDW
//			//cLdw.frame = greyFrame;
//			//cLdw.Detect();
//			//PlotSingleLine(frame, cLdw.leftLane.curLane, 300.0);
//			//PlotSingleLine(frame, cLdw.rightLane.curLane, 300.0);
//
//			//FCW
//			//forwardColWarning(frame, preTarget, cascade1, cascade2, cLdw.trans_Matrix, curTarget, cLdw, buf);
//			cFcw.Detect();
//		}
//
//		imshow("result", frame);
//		//imwrite(save_name, frame);
//
//		c = cvWaitKey(5);
//		if (c == 27) break;
//	}
//
//	t = (double)getTickCount() - t;
//	t = t / ((double)getTickFrequency() / 1000.);
//	cout << endl << "Time Used:" << t << endl << endl;
//
//	free(buf);
//
//	return 0;
//}
//
