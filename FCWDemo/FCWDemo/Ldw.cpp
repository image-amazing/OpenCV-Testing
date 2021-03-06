#include "Ldw.h"

static uchar frameBuff[1280 * 720 * 2];
static uchar sFrameBuff[1280 * 720 * 2];
uchar leftMaskBuff[1280 * 720]; 
uchar rightMaskBuff[1280 * 720];

static stLanePoints lanePoints0[30000];
static stLanePoints lanePoints1[10000];

#define TEST_CAR 1
Ldw::Ldw()
{
	frame = Mat::zeros(720, 1280, CV_8U);
	frame.data = frameBuff;

	smallFrame = Mat::zeros(180, 320, CV_8U);
	smallFrame.data = sFrameBuff;

	cascade.load(LANE_CASCADE_NAME);

	// 用于左侧车道线的基本数据
	leftMask = Mat::zeros(720, 1280, CV_8U);
	leftMask.data = leftMaskBuff;

	rightMask = Mat::zeros(720, 1280, CV_8U);
	rightMask.data = rightMaskBuff;

	leftLaneRoi = Rect(0, 90, 160, 90);
	rightLaneRoi = Rect(160, 90, 160, 90);

	leftLane.tracked = 0;
	rightLane.tracked = 0;

	pos1 = 0;
	pos0 = 0;

	//stThres = { 305, 360, 555, 495, 525, 335, 300, 500 };     // 大车
	//stThres = { 340, 400, 520, 460, 490, 370, 300, 500};  // 小车
	stThres = {330, 390, 530, 470, 500, 360, 300, 500};
}

int Ldw::InitMaskAndRoi(Rect roi, Mat& mask, Rect& cannyRoi, stLane& line)
{
	uchar* pMask;
	// 如果上一帧检测到，则进行跟踪
	if (line.tracked != 0 )
	{
		// 跟踪
		Point2f tmPoint;
		Point leftTarget, rightTarget;
		bool flag = true;
		for (int i = 360; i < 720; i++)
		{
			pMask = mask.ptr<uchar>(i);

			if (line.curLane[0] * line.curLane[1] == 0)
				continue;

			tmPoint.x = (i - line.curLane[3]) / line.curLane[1] * line.curLane[0] + line.curLane[2];
			tmPoint.y = i*1.0f;

			for (int j = (int)tmPoint.x - 40; j < (int)tmPoint.x + 40; j++)
			{
				if (j >= 0 && j < 1280)
				{
					pMask[j] = 255;

					/* 生成roi区域 */
					if (flag)
					{
						leftTarget = Point(j, i);
						rightTarget = Point(j, i);
						flag = false;
					}

					if (leftTarget.x > j)
						leftTarget = Point(j, i);

					if (rightTarget.x < j)
						rightTarget = Point(j, i);
				}
			}
		}

		// 根据找到的目标，标记出CANNY的最大区域
		cannyRoi.x = min(leftTarget.x, rightTarget.x);
		cannyRoi.y = min(rightTarget.y, leftTarget.y);
		cannyRoi.width = abs(rightTarget.x - leftTarget.x);
		cannyRoi.height = 360;

		return true;
	}
	else
	{
		// 缩小4倍
		resize(frame, smallFrame, Size(320, 180), INTER_LINEAR);

		// 进行LBP车道线检测
		cascade.detectMultiScale(smallFrame(roi), laneTargets, 1.4, 1, 0, Size(15, 15), Size(30, 30));
		if (laneTargets.size() > 0)
		{
			// 提取mask，计算canny与梯度区域
			Rect tmRect;

			Rect leftTarget, rightTarget;
			for (int i = 0; i < laneTargets.size(); i++)
			{
				tmRect.x = (laneTargets[i].x + roi.x) * 4;
				tmRect.y = (laneTargets[i].y + roi.y) * 4;
				tmRect.width = laneTargets[i].width * 4;
				tmRect.height = laneTargets[i].height * 4;

				// 标记mask区域
				for (int j = tmRect.y; j < tmRect.y + tmRect.height; j++)
				{
					pMask = mask.ptr<uchar>(j);
					memset(pMask + tmRect.x, 0xFF, tmRect.width);
				}

				// 确定左侧ROI区域
				if (i == 0)
				{
					leftTarget = tmRect;
					rightTarget = tmRect;
				}
				else
				{
					// 最左侧目标
					if (leftTarget.x > tmRect.x)
						leftTarget = tmRect;

					// 最右侧的目标
					if (rightTarget.x < tmRect.x)
						rightTarget = tmRect;
				}

				// 根据找到的目标，标记出CANNY的最大区域
				cannyRoi.x = min(leftTarget.x, rightTarget.x);
				cannyRoi.y = min(rightTarget.y, leftTarget.y);
				cannyRoi.width = abs(rightTarget.x - leftTarget.x) + max(leftTarget.width, rightTarget.width);
				cannyRoi.height = abs(leftTarget.y - rightTarget.y) + max(leftTarget.height, rightTarget.height);
			}

			return true;
		}
		else
		{
			memset(&line, 0, sizeof(line));
			return false;
		}
	}
}

/***************************************************************************************************
函数名：ADS_LaneDetect_Test_Show_point
主要功能：// 测试函数 将当前边缘点 显示到图test_img上
*		temp_point   -I   当前边缘点
*       number       -I   边缘点个数
备注：无返回 VOID
***************************************************************************************************/
void Ldw::ADS_LaneDetect_Test_Show_point()
{
	int i = 0;
	int j = 0;
	int width = 0;
	int height = 0;
	Mat test_frame;

	test_frame = Mat::zeros(frame.rows, frame.cols, CV_8U);
	height = test_frame.rows;
	width = test_frame.cols;
	uchar* temp_ptr = nullptr;

	for (int n = 0; n < pos0; n++)
	{
		j = (int)lanePoints0[n].edgePoint.y;
		i = (int)lanePoints0[n].edgePoint.x;

		temp_ptr = test_frame.ptr<uchar>(j);
		temp_ptr[i] = 255;
	}
	imshow("point", test_frame);
	cvWaitKey(5);
	//imwrite(save_path, test_img);
}

void Ldw::InitEdgePoints(Rect roi, Mat mask)
{
	AsertROI(roi);
	Mat element(3, 3, CV_8U, Scalar(1));
	Mat cannyFrame, edge, sCanny, sFrame;
	Rect tmp(roi.x / 2, roi.y / 2, roi.width / 2, roi.height / 2);
	resize(frame, sFrame, Size(640, 360), INTER_LINEAR);
	Canny(sFrame(tmp), edge, 30, 90, 3, false);
	resize(edge, sCanny, Size(roi.width, roi.height), INTER_LINEAR);
	dilate(sCanny, cannyFrame, element);

	// 取CANNY边缘上的点
	unsigned char a00, a01, a02;
	unsigned char a10, a11, a12;
	unsigned char a20, a21, a22;


	uchar *pCannyPtr, *pMask;
	stLanePoints stPoint;
	uchar* pFrame1, *pFrame2, *pFrame3;

	for (int j = 1; j < roi.height - 1; j++)
	{
		pCannyPtr = cannyFrame.ptr<uchar>(j);		// canny 图像指针
		pMask = mask.ptr(j + roi.y);
		pFrame1 = frame.ptr<uchar>(j + roi.y);
		pFrame2 = frame.ptr<uchar>(j + roi.y - 1);
		pFrame3 = frame.ptr<uchar>(j + roi.y + 1);
		double ux, uy;

		//pFrame1 = frame.ptr(434);
		//pFrame2 = frame.ptr(433);
		//pFrame3 = frame.ptr(435);

		for (int i = 1; i < roi.width - 1; i++)
		{
			
			if (pCannyPtr[i] > 100 && pMask[i + roi.x] == 255 /*&& pFrame1[i + roi.x] > 150*/)
			{
				stPoint.edgePoint.x = i + roi.x;
				stPoint.edgePoint.y = j + roi.y;
	
				a00 = pFrame2[i + roi.x - 1];
				a01 = pFrame1[i + roi.x - 1];
				a02 = pFrame3[i + roi.x - 1];
				a10 = pFrame2[i + roi.x];
				a11 = pFrame1[i + roi.x];
				a12 = pFrame3[i + roi.x];
				a20 = pFrame2[i + roi.x + 1];
				a21 = pFrame1[i + roi.x + 1];
				a22 = pFrame3[i + roi.x + 1];

				ux = a20 * (1) + a21 * (2) + a22 * (1)
					+ (a00 * (-1) + a01 * (-2) + a02 * (-1));
				uy = a02 * (1) + a12 * (2) + a22 * (1)
					+ a00 * (-1) + a10 * (-2) + a20 * (-1);

				stPoint.grad = atan2(uy, ux);
				//stPoint.gray = a11;

				if (ux * uy != 0)
				{
					lanePoints0[pos0++] = stPoint;
				}
		
			}
			/*else
			{
				pCannyPtr[i] = 0;
			}*/
		}
	}
	//imshow("canny", cannyFrame);
	//cvWaitKey(5);
	//ADS_LaneDetect_Test_Show_point();
}

void Ldw::InitMainDirectPoints(void)
{
	int interVal = 0;
	float gHist[360] = { 0.0f };
	vector<stLanePoints>::iterator it;

	memset(gHist, 0, sizeof(gHist));
	if (pos0 > 0)
	{
		// 第一次确认梯度最大的方向
		for (int i = 0; i < pos0; i++)
		{
			interVal = int(lanePoints0[i].grad / (CV_PI*0.1111) + 9.0f);
			lanePoints0[i].interVal = interVal;
			gHist[interVal] += 1;
		}

		float maxHist = -1.0f;
		int maxDirect = 0;
		int start = 0;
		int end = 0;

		if (pre_main_direct > 0)
		{
			start = 9;
			end = 17;
		}
		else if (pre_main_direct < 0)
		{
			start = 0;
			end = 9;
		}
		else
		{
			start = 0;
			end = 17;
		}

		for (int i = 9; i < 17; i++)
		{
			if (i == 2 || i == 12)
			{
				i += 3;
			}
			if (gHist[i] > maxHist)
			{
				maxHist = gHist[i];
				maxDirect = i;
			}
		}
//		printf("%f-%d\n", maxHist, maxDirect);

		// 提取属于最大梯度方向上的点
		for (int i = 0; i < pos0; i++)
		{
			if (abs(lanePoints0[i].interVal - maxDirect) < 2)
			{
				lanePoints1[pos1++] = lanePoints0[i];
			}
		}

		// 第二次确认主方向的精确值
		pos0 = 0;
		float maxNumHist = -1.0f;
		int maxNumDirect = -1;
		float direct = 0.0f;

		memset(gHist, 0, sizeof(gHist));
		if (pos1 > 0)
		{
			for (int i = 0; i < pos1; i++)
			{
				interVal = int(lanePoints1[i].grad / (CV_PI*0.01111)) + 90;
				lanePoints1[i].interVal = interVal;
				gHist[interVal] += 1;
			}

			for (int i = 0; i < 180; i++)
			{
				if (gHist[i] > maxNumHist)
				{
					maxNumHist = gHist[i];
					maxNumDirect = i;
				}
			}
			direct = (CV_PI*0.01111) * (maxNumDirect - 90);
		}

		for (int i = 0; i < pos1; i++)
		{
			if (abs(lanePoints1[i].interVal - maxNumDirect) < 2)
			{
				lanePoints0[pos0++] = lanePoints1[i];
			}
		}
		//printf("size=%d\n", lanePoints.size());
		pre_main_direct = direct;
		//printf("%f\n", direct);
	}
	//ADS_LaneDetect_Test_Show_point();

}

void Ldw::FitLine(stLane& lane)
{
	vector<Point> line_point;
	//if (pos0 > 50)
	//{

	//}
	for (int i = 0; i < pos0; i += 2)
	{
		line_point.push_back(lanePoints0[i].edgePoint);
		//printf("%d \n", lanePoints0[i].gray);
	}
	fitLine(line_point, lane.curLane, CV_DIST_HUBER, 0, 0.01, 0.01);
	float theta = atan(lane.curLane[1] / lane.curLane[0]);

	lane.curTheta = theta;
	pos0 = 0; 
	pos1 = 0;
//	printf("theta=%f\n", theta);
}

//void Ldw::FitLane(stLane& lane)
//{
//	vector<Point> line_point;
//
//	int step = lanePointsPos / 40;
//	if (step < 1)
//		step = 1;
//
//	for (uint i = 0; i < lanePointsPos; i += step)
//	{
//		line_point.push_back(lanePoints[i].edgePoint);
//	}
//	fitLine(line_point, lane.curLane, CV_DIST_HUBER, 0, 0.01, 0.01);
//
//	float theta = atan(lane.curLane[1] / lane.curLane[0]);
//	lane.curTheta = theta;
//
//	lanePointsPos = 0;
//	tmpLanePointsPos = 0;
//}

void Ldw::LaneDet(Rect roi, Mat& mask, Rect cannyRoi, vector<stLanePoints> &lanePoints, 
	              stLane& lane)
{
	/* 初始化左线的MASK与CANNY区域 */
	if (InitMaskAndRoi(roi, mask, cannyRoi, lane))
	{
		InitEdgePoints(cannyRoi, mask);
		InitMainDirectPoints();
	}
	if (pos0 > 10)
	{
		FitLine(lane);
		Lane2BvLane(lane);
		lane.tracked = true;
	}
	else
	{
		lane.tracked = false;
	}

	if (lane.tracked)
	{
		lane.preLane = lane.curLane;
		//lane.preTheta = lane.curTheta;
	}
}


void Ldw::Lane2BvLane(stLane& lane)
{
	Point2f point = { 0 };
	vector<Point2f> be_point;
	vector<Point2f> af_point;
	Vec4f addLane;
	// 计算左线的鸟瞰线

	point.x = (0 - lane.curLane[3]) / lane.curLane[1] * lane.curLane[0] + lane.curLane[2];
	point.y = 0;
	be_point.push_back(point);
	point.x = lane.curLane[2];
	point.y = lane.curLane[3];
	be_point.push_back(point);

	perspectiveTransform(be_point, af_point, trans_Matrix);

	float dist = sqrt((af_point[0].x - af_point[1].x)*(af_point[0].x - af_point[1].x)
		+ (af_point[0].y - af_point[1].y)*(af_point[0].y - af_point[1].y));

	addLane[0] = (af_point[0].x - af_point[1].x) / dist;
	addLane[1] = (af_point[0].y - af_point[1].y) / dist;
	addLane[2] = af_point[0].x;
	addLane[3] = af_point[0].y;

	if (addLane[0] < 0)
	{
		addLane[0] = -addLane[0];
		addLane[1] = -addLane[1];
	}

	lane.bvLane = addLane;
	lane.bvTheta = atan(addLane[1] / addLane[0]);

	GetPointInLine(lane.bvLane, 0, lane.bvEnd);
}

bool Ldw::CheckLane(stLane& leftLine, stLane& rightLine)
{
	float thres = CV_PI / 12.0f;


	int leftFlag = 0, rightFlag = 0;

	/*leftLine.preTheta = leftLine.curTheta;
	rightLine.preTheta = rightLine.curTheta;*/

	// 先判断左线
	if (leftLine.curLane[0] == 0)
	{
		memset(&leftLine, 0, sizeof(leftLine));
		leftLine.tracked = false;
		leftFlag = true;
	}
	// 左线与上一帧线进行比较
	if (leftLine.tracked)
	{
		// 如果有前一帧的线
		printf("\n %f \n", abs(leftLine.preTheta) - abs(leftLine.curTheta));
		if (leftLine.preTheta != 0)
		{
			if (abs(abs(leftLine.preTheta) - abs(leftLine.curTheta)) > thres
				|| abs(leftLine.curTheta) < thres)
			{
				memset(&leftLine, 0, sizeof(leftLine));
				leftLine.tracked = false;
				leftFlag = true;
			}
		}

		if (leftLine.preBvEnd.x != 0)
		{
			// 判断endx 是否有效
			if (leftLine.bvEnd.x > stThres.Max_left || abs(leftLine.bvEnd.x - leftLine.preBvEnd.x) > 20)
			{
				memset(&leftLine, 0, sizeof(leftLine));
				leftLine.tracked = false;
				leftFlag = true;
			}
		}
		
	}
	else
	{
		memset(&leftLine, 0, sizeof(leftLine));
		leftFlag = true;
	}

	// 再判断右线
	if (rightLine.curLane[0] == 0)
	{
		memset(&rightLine, 0, sizeof(rightLine));
		rightLine.tracked = false;
		rightFlag = true;
	}
	if (rightLine.tracked)
	{
		if (rightLine.preTheta != 0)
		{
			if (abs(rightLine.preTheta - rightLine.curTheta) > thres
				|| abs(rightLine.curTheta) < thres)
			{
				memset(&rightLine, 0, sizeof(rightLine));
				rightLine.tracked = false;
				rightFlag = true;
			}
		}
		if (rightLine.preBvEnd.x != 0)
		{
			if (rightLine.bvEnd.x < stThres.Min_right || abs(rightLine.bvEnd.x - rightLine.preBvEnd.x) > 20)
			{
				memset(&rightLine, 0, sizeof(rightLine));
				rightLine.tracked = false;
				rightFlag = true;
			}
		}
	}
	else
	{
		rightFlag = true;
		memset(&rightLine, 0, sizeof(rightLine));
	}


//	printf("left:%f,%f,%f,%d\n", leftLine.preTheta, leftLine.curTheta, leftLine.bvTheta, leftLine.bvEnd.x );
//	printf("right:%f,%f,%f,%d\n", rightLine.preTheta, rightLine.curTheta, rightLine.bvTheta, rightLine.bvEnd.x);

	// 如果两条线都存在，进行平行判断
	if (!leftFlag && !rightFlag)
	{
		if (abs(abs(leftLine.bvTheta) - abs(rightLine.bvTheta)) > CV_PI / 12)
		{
			memset(&leftLine, 0, sizeof(leftLine));
			memset(&rightLine, 0, sizeof(rightLine));
			leftLine.tracked = false;
			rightLine.tracked = false;
			leftFlag = true;
			rightFlag = true;
		}

		if (abs(leftLine.bvEnd.x - rightLine.bvEnd.x) < stThres.left2right_Low || abs(leftLine.bvEnd.x - rightLine.bvEnd.x) > stThres.left2right_High)
		{
			memset(&leftLine, 0, sizeof(leftLine));
			memset(&rightLine, 0, sizeof(rightLine));
			leftLine.tracked = false;
			rightLine.tracked = false;
			leftFlag = true;
			rightFlag = true;
		}


		printf("%.2f \n", leftLine.curTheta - rightLine.curTheta);

		if (abs(leftLine.curTheta - rightLine.curTheta) < CV_PI / 4)
		{
			memset(&leftLine, 0, sizeof(leftLine));
			memset(&rightLine, 0, sizeof(rightLine));
			leftLine.tracked = false;
			rightLine.tracked = false;
			leftFlag = true;
			rightFlag = true;
		}
	}

	if (!leftFlag)
	{
		leftLine.tracked = true;
	}
	if (!rightFlag)
	{
		rightLine.tracked = true;
	}

//	if (leftFlag || rightFlag)
//		waitKey(0);

	return true;
}

bool  Ldw::LaneStat(stLane& leftLane, stLane& rightLane)
{
	/* 如果左右车道线没有判定为跟踪成功，则不进行变线判断 */
	if (!leftLane.tracked || !rightLane.tracked)
		return false;

	/* 获取车辆角度 */
	//	float carThera = (leftLane.bvTheta + rightLane.bvTheta) / 2;

	/* 上一帧变线趋势判断 */

	// 如果上一帧向左偏转
	if (preWarnFlag == -1)
	{
		if (leftLane.bvEnd.x - leftLane.preBvEnd.x > 0 &&
			leftLane.bvEnd.x > stThres.left_start_th)
		{
			warnFlag = -1;
		}
		else
			warnFlag = 0;
	}
	// 如果上一帧判断为向右偏转
	else if (preWarnFlag == 1)
	{
		if (rightLane.preBvEnd.x - rightLane.bvEnd.x > 0 &&
			rightLane.bvEnd.x < stThres.right_start_th)
		{
			warnFlag = 1;
		}
		else
			warnFlag = 0;
	}
	// 如果上一帧未发生偏转
	else if (preWarnFlag == 0)  
	{
		// 向左偏转判断
		if (leftLane.bvEnd.x - leftLane.preBvEnd.x > 0 &&
			rightLane.bvEnd.x - rightLane.preBvEnd.x > 0 &&
			leftLane.bvEnd.x > stThres.left_start_th)
		{
			warnFlag = -1;
		}
		// 向右偏转判断
		else if (rightLane.preBvEnd.x - rightLane.bvEnd.x > 0 &&
			leftLane.preBvEnd.x - leftLane.bvEnd.x > 0 &&
			rightLane.bvEnd.x < stThres.right_start_th)
		{
			warnFlag = 1;
		}
		// 正常行驶
		else
		{
			warnFlag = 0;
		}
	}

	/* 偏转完成信息判断 */
	if (leftLane.preBvEnd.x > stThres.left_start_th &&
		rightLane.preBvEnd.x < stThres.right_end_th)
	{
		departFlag = 0;
		warnFlag = 0;
	}
	// 向左偏转完成信息判断
	else if (leftLane.bvEnd.x - leftLane.preBvEnd.x > 0 &&
		rightLane.bvEnd.x - rightLane.preBvEnd.x > 0 &&
		leftLane.bvEnd.x > stThres.left_end_th)
	{
		departFlag = -1;
		warnFlag = 0;
	}
	else if (rightLane.preBvEnd.x - rightLane.bvEnd.x > 0 &&
		leftLane.preBvEnd.x - leftLane.bvEnd.x > 0 &&
		rightLane.bvEnd.x < stThres.right_end_th)
	{
		departFlag = 1;
		warnFlag = 0;
	}

	if (leftLane.tracked && rightLane.tracked)
	{
		preWarnFlag = warnFlag;

		leftLane.preLane = leftLane.curLane;
		leftLane.preTheta = leftLane.curTheta;
		leftLane.preBvLane = leftLane.bvLane;
		leftLane.preBvEnd = leftLane.bvEnd;

		rightLane.preLane = rightLane.curLane;
		rightLane.preTheta = rightLane.curTheta;
		rightLane.preBvLane = rightLane.bvLane;
		rightLane.preBvEnd = rightLane.bvEnd;
	}
	else
	{
		preWarnFlag = warnFlag = 0;
	}
}

/***************************************************************************************************
函数名：ADS_LaneDetec_get_other_lane
主要功能：//拟合另外一边车道线
*       one_line           -I    已知车道线
*       dir_flag           -O    拟合直线方向
*       other_line         -O    未知车道线
*       trans_Matrix       -I    转换矩阵
*       back_trans_Matrix  -I    反向转换矩阵

备注：无返回 VOID
***************************************************************************************************/
void Ldw::ADS_LaneDetect_get_other_lane(Vec4f& one_line, int dir_flag, Vec4f& other_line)
{
	Point t_point = { 0 };
	vector<Point2f> be_point;
	vector<Point2f> af_point;

	GetPointInLine(one_line, frame.rows * 0.5, t_point);
	be_point.push_back(t_point);
	t_point.x = one_line[2];
	t_point.y = one_line[3];
	be_point.push_back(t_point);

	perspectiveTransform(be_point, af_point, trans_Matrix);

	Point p1_BV = { 0 };
	Point p2_BV = { 0 };
	Point p3_BV = { 0 };
	Point p4_BV = { 0 };

	p1_BV = af_point[0];
	p2_BV = af_point[1];
	if (dir_flag == -1)
	{
		p3_BV.x = p1_BV.x - 360;
		p4_BV.x = p2_BV.x - 360;
	}
	else if (dir_flag == 1)
	{
		p3_BV.x = p1_BV.x + 360;
		p4_BV.x = p2_BV.x + 360;
	}

	p3_BV.y = p1_BV.y;
	p4_BV.y = p2_BV.y;

	af_point[0] = p3_BV;
	af_point[1] = p4_BV;
	perspectiveTransform(af_point, be_point, back_trans_Matrix);
	//ADS_LaneDetec_perspective_trans_single_point(af_point[0], &be_point[0], back_trans_Matrix);
	//ADS_LaneDetec_perspective_trans_single_point(af_point[1], &be_point[1], back_trans_Matrix);

	float dist = sqrt((be_point[0].x - be_point[1].x)*(be_point[0].x - be_point[1].x)
		+ (be_point[0].y - be_point[1].y)*(be_point[0].y - be_point[1].y));

	Vec4f addLane;

	addLane[0] = (be_point[0].x - be_point[1].x) / dist;
	addLane[1] = (be_point[0].y - be_point[1].y) / dist;
	addLane[2] = be_point[0].x;
	addLane[3] = be_point[0].y;

	if (addLane[0] < 0)
	{
		addLane[0] = -addLane[0];
		addLane[1] = -addLane[1];
	}

	other_line = addLane;
}

void Ldw::Detect(void)
{

	// 左线检测
	memset(leftMaskBuff, 0, sizeof(leftMaskBuff));
	pos0 = 0; pos1 = 0;
	LaneDet(leftLaneRoi, leftMask, leftCannyRoi, leftLanePoints, leftLane);

	// 右线检测
	memset(rightMaskBuff, 0, sizeof(rightMaskBuff));
	pos0 = 0; pos1 = 0;
 	LaneDet(rightLaneRoi, rightMask, rightCannyRoi, rightLanePoints, rightLane);

	// 确认两条直线的问题
	CheckLane(leftLane, rightLane);

	// 根据车头的斜率与变化确认是否变线
	if (!LaneStat(leftLane, rightLane))
	{
		warnFlag = 0;
		preWarnFlag = 0;
	}

	if (warnFlag == -1 && preWarnFlag == -1)
	{
		putText(frame, "LEFT", cvPoint(50, 50), CV_FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 0), 3);
		printf("\a");
	}
	else if (warnFlag == 1 && preWarnFlag == 1)
	{
		putText(frame, "RIGHT", cvPoint(1100, 50), CV_FONT_HERSHEY_COMPLEX, 1.0, Scalar(0, 0, 0), 3);
		printf("\a");
	}

	// 将左右车道线的点转换成鸟瞰车道线
	if (leftLane.tracked && rightLane.tracked)
	{ 
		preWarnFlag = warnFlag;

		leftLane.preLane   = leftLane.curLane;
		leftLane.preTheta  = leftLane.curTheta;
		leftLane.preBvLane = leftLane.bvLane;
		leftLane.preBvEnd  = leftLane.bvEnd;
		
		rightLane.preLane   = rightLane.curLane;
		rightLane.preTheta  = rightLane.curTheta;
		rightLane.preBvLane = rightLane.bvLane;
		rightLane.preBvEnd  = rightLane.bvEnd;

		rightGetFlag = 0;
		leftGetFlag  = 0;

	}
	else if (leftLane.tracked)
	{
		preWarnFlag = 0;
		warnFlag = 0;
		leftLane.preLane = leftLane.curLane;
		leftLane.preTheta = leftLane.curTheta;
		leftLane.preBvLane = leftLane.bvLane;
		leftLane.preBvEnd = leftLane.bvEnd;


		if (!rightGetFlag)
		{
			ADS_LaneDetect_get_other_lane(leftLane.curLane, 1, rightLane.curLane);
			rightLane.tracked = 1;
			rightLane.curTheta = atan(rightLane.curLane[1] / rightLane.curLane[0]);
			Lane2BvLane(rightLane);

			rightLane.preLane = rightLane.curLane;
			rightLane.preTheta = rightLane.curTheta;
			rightLane.preBvLane = rightLane.bvLane;
			rightLane.preBvEnd = rightLane.bvEnd;
			rightGetFlag = 1;
		}
		else
		{
			rightGetFlag = 0;
		}
	}
	else if (rightLane.tracked)
	{
		preWarnFlag = 0;
		warnFlag = 0;
		rightLane.preLane = rightLane.curLane;
		rightLane.preTheta = rightLane.curTheta;
		rightLane.preBvLane = rightLane.bvLane;
		rightLane.preBvEnd = rightLane.bvEnd;

		if (!leftGetFlag)
		{
			ADS_LaneDetect_get_other_lane(rightLane.curLane, -1, leftLane.curLane);
			leftLane.tracked = 1;
			leftLane.curTheta = atan(leftLane.curLane[1] / leftLane.curLane[0]);
			Lane2BvLane(leftLane);

			leftLane.preLane = leftLane.curLane;
			leftLane.preTheta = leftLane.curTheta;
			leftLane.preBvLane = leftLane.bvLane;
			leftLane.preBvEnd = leftLane.bvEnd;
			leftGetFlag = 1;
		}
		else
		{
			leftGetFlag = 0;
		}
		
	}

	// 如果向左偏移，右线=左线，左线通过右线拟合
	if (departFlag == -1)
	{
		rightLane = leftLane;
		memset(&leftLane, 0, sizeof(leftLane));

		departFlag = 0;
	}
	// 如果向左偏移，左线=右线，右线通过左线拟合
	else if (departFlag == 1)
	{
		leftLane = rightLane;
		memset(&rightLane, 0, sizeof(rightLane));

		departFlag = 0;
	}
}

inline void Ldw::GetPointInLine(Vec4f line, int y, Point& point)
{
	if (line[0] * line[1] == 0)
	{
		point.x = 0;
		point.y = 0;
	}

	point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
	point.y = y;
}


void Ldw::InitMatrixData(void)        //途安最新摄像头标定（自动标定） 20161206
{
	vector<Point2f> objPts(9), imgPts(9);

	imgPts[0] = Point2f(448, 456);
	imgPts[1] = Point2f(658, 453);
	imgPts[2] = Point2f(863, 451);
	imgPts[3] = Point2f(388, 520);
	imgPts[4] = Point2f(663, 519);
	imgPts[5] = Point2f(940, 515);
	imgPts[6] = Point2f(310, 598);
	imgPts[7] = Point2f(664, 600);
	imgPts[8] = Point2f(1017, 591);

	objPts[0] = Point2f(280, 1200);
	objPts[1] = Point2f(430, 1200);
	objPts[2] = Point2f(580, 1200);
	objPts[3] = Point2f(280, 700);
	objPts[4] = Point2f(430, 700);
	objPts[5] = Point2f(580, 700);
	objPts[6] = Point2f(280, 500);
	objPts[7] = Point2f(430, 500);
	objPts[8] = Point2f(580, 500);

	trans_Matrix = findHomography(imgPts, objPts);
	back_trans_Matrix = findHomography(objPts, imgPts);

}

//void Ldw::InitMatrixData(void)        //陕汽摄像头标定
//{
//	Point2f objPts[8], imgPts[8];
//
//	imgPts[0] = Point2f(478, 466);
//	imgPts[1] = Point2f(764, 466);
//	imgPts[2] = Point2f(433, 522);
//	imgPts[3] = Point2f(813, 522);
//
//	imgPts[4] = Point2f(343, 647);
//	imgPts[5] = Point2f(911, 647);
//	imgPts[6] = Point2f(289, 707);
//	imgPts[7] = Point2f(963, 707);
//
//	objPts[0] = Point2f(261, 2366);
//	objPts[1] = Point2f(599, 2366);
//	objPts[2] = Point2f(261, 1766);
//	objPts[3] = Point2f(599, 1766);
//
//	objPts[4] = Point2f(261, 1166);
//	objPts[5] = Point2f(599, 1166);
//	objPts[6] = Point2f(261, 980);
//	objPts[7] = Point2f(599, 980);
//
//	trans_Matrix = Mat::zeros(3, 3, CV_8U);
//	back_trans_Matrix = Mat::zeros(3, 3, CV_8U);
//
//	trans_Matrix = getPerspectiveTransform(imgPts, objPts);
//	back_trans_Matrix = getPerspectiveTransform(objPts, imgPts);
//
//		//int height = trans_Matrix.rows;
//		//int width = trans_Matrix.cols;
//		//printf("trans:    \n");
//		//for (int i = 0; i < height; i++)
//		//{
//		//	for (int j = 0; j < width; j++)
//		//	{
//		//		printf("%f  ", (float)trans_Matrix.ptr<float>(i)[j]);
//		//		//不加类型转换用txt打开是字符  
//		//	}
//		//	printf("\n");
//		//}
//	
//	
//		//printf("\n \n");
//		//printf("back_trans:    \n");
//		//for (int i = 0; i < height; i++)
//		//{
//		//	for (int j = 0; j < width; j++)
//		//	{
//		//		printf("%f  ", (float)back_trans_Matrix.ptr<float>(i)[j]);
//		//		//不加类型转换用txt打开是字符  
//		//	}
//		//	printf("\n");
//		//}
//
//		//FileStorage fs("transMatrix.xml", FileStorage::WRITE);
//		//fs <<"trans"<< trans_Matrix;
//		//fs.release();
//		////fs("transMatrix_back.xml", FileStorage::READ);
//		////fs["H"] >> bTransMat;
//}

//void Ldw::InitMatrixData(void)        //陕汽摄像头标定(新摄像头)
//{
//	Point2f objPts[8], imgPts[8];
//
//	imgPts[0] = Point2f(522, 476);
//	imgPts[1] = Point2f(762, 478);
//	imgPts[2] = Point2f(480, 532);
//	imgPts[3] = Point2f(799, 533);
//
//	imgPts[4] = Point2f(444, 582);
//	imgPts[5] = Point2f(837, 586);
//	imgPts[6] = Point2f(374, 670);
//	imgPts[7] = Point2f(896, 674);
//
//	imgPts[8] = Point2f(642, 476);
//	imgPts[9] = Point2f(640, 532);
//	imgPts[10] = Point2f(639, 587);
//	imgPts[11] = Point2f(639, 673);
//
//	objPts[0] = Point2f(280, 2000);
//	objPts[1] = Point2f(580, 2000);
//	objPts[2] = Point2f(280, 1500);
//	objPts[3] = Point2f(580, 1500);
//
//	objPts[4] = Point2f(280, 1200);
//	objPts[5] = Point2f(580, 1200);
//	objPts[6] = Point2f(280, 900);
//	objPts[7] = Point2f(580, 900);
//
//	objPts[8] = Point2f(430, 2000);
//	objPts[9] = Point2f(430, 1500);
//	objPts[10] = Point2f(430, 1200);
//	objPts[11] = Point2f(430, 900);
//
//	trans_Matrix = Mat::zeros(3, 3, CV_8U);
//	back_trans_Matrix = Mat::zeros(3, 3, CV_8U);
//
//	trans_Matrix = getPerspectiveTransform(imgPts, objPts);
//	back_trans_Matrix = getPerspectiveTransform(objPts, imgPts);
//
//}

void Ldw::PlotSingleLine(Mat& frame, Vec4f& plotLine, double dL)
{
	Point p1, p2;
	//p1.x = int(plotLine[2] - dL*plotLine[0]);
	//p1.y = int(plotLine[3] - dL*plotLine[1]);
	//p2.x = int(plotLine[2] + dL*plotLine[0]);
	//p2.y = int(plotLine[3] + dL*plotLine[1]);

	GetPointInLine(plotLine, frame.rows * 0.5, p1);
	GetPointInLine(plotLine, frame.rows, p2);
	line(frame, p1, p2, Scalar(255, 255, 0), 10);
}

void Ldw::AsertROI(Rect &roi)
{
	int width = 1280 - 10;
	int height = 720 - 10;

	if (roi.x < 0)
		roi.x = 0;
	else if (roi.x > width)
		roi.x = width;

	if (roi.y < 0)
		roi.y = 0;
	else if (roi.y > height)
		roi.y = height;

	if ((roi.x + roi.width) > width)
		roi.width = width - roi.x;

	if ((roi.y + roi.height) > height)
		roi.height = height - roi.y;
}

