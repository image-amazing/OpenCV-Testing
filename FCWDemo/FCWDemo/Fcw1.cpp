#include "Fcw1.h"


Fcw::Fcw()
{
	/* 加载分类器 */
	if (!cascCar.load("cascade_995_50.xml"))
	{
		cout << "Cannot load the cascade." << endl;
		return;
	}

	/* 不同尺度需要的搜索参数 */
	InitAllParas();

}

/* 不同尺度需要的搜索参数 */
uint8 Fcw::InitAllParas(void)
{
	/* 初始化参数设置 */
	frameWidth = 1280;
	frameHeight = 720;
	SKY_LINE = 0.4;

	/* 重合率 */
	overRate = 0.6;

	int minSize[3] = { 30, 40, 80};
	int maxSize[3] = { 80, 80, 200};


	Rect roi[3] =
	{
		// 最小图搜索区域
		Rect(0.25*frameWidth, SKY_LINE*frameHeight - 0.6 * maxSize[0],
		0.5*frameWidth, 2 * maxSize[0]),		

		// 中等尺度搜索区域
		Rect((0.2*frameWidth) / 2, SKY_LINE*frameHeight / 2 - 0.5*maxSize[1],
		(0.6*frameWidth) / 2, 1.5 * maxSize[1]),

		// 最大尺度搜索区域
		Rect((0.15*frameWidth) / 2, SKY_LINE*frameHeight / 2 - 0.6*maxSize[2],
		(0.7*frameWidth) / 2, 1.5 * maxSize[2]),
	};


	stParas[0].multiNum = 1;
	stParas[0].roi = roi[0];
	stParas[0].scale = 1.3;
	stParas[0].minSize = Size(minSize[0], minSize[0]);
	stParas[0].maxSize = Size(maxSize[0], maxSize[0]);

	stParas[1].multiNum = 2;
	stParas[1].roi = roi[1];
	stParas[1].scale = 1.3;
	stParas[1].minSize = Size(minSize[1], minSize[1]);
	stParas[1].maxSize = Size(maxSize[1], maxSize[1]);

	stParas[2].multiNum = 2;
	stParas[2].roi = roi[2];
	stParas[2].scale = 1.3;
	stParas[2].minSize = Size(minSize[2], minSize[2]);
	stParas[2].maxSize = Size(maxSize[2], maxSize[2]);

	/*  */
	stPreCar.flag = false;

	for (int i = 0; i < 3; i++)
	{
	//	printf("%d-%d-%d-%d\n", stParas[i].roi.x, stParas[i].roi.y, stParas[i].roi.width, stParas[i].roi.height);
	}

	return 0;
}

void Fcw::Detect(void)
{
	/* 如果上一帧没有跟踪到车辆 */
	if (stPreCar.flag == false)
	{
		targetCars.clear();
		// 首先在最大区，搜索危险车辆
		for (int i = 0; i < 3; i++)
		{
			LbpDetectCars(stParas[i]);
		}
	}
	else
	{
	//	printf("aaaaaaaaaaaa\n");
		targetCars.clear();

		/*  跟踪  */
		InitTrackedPara();
		LbpDetectCars(stTrackedPara);

		for (int i = 0; i < lbpCars.size(); i++)
		{
			targetCars[i].trackFlag = 1;
		}

		/*  检测  */
		LbpDetectCars(stParas[2]);
	}

	/* 合并CARS */
	MergeTargetCars();

	/* 找到关键CAR */
	FindKeyCar();

	// DEBUG MSG
	Rect roi(0, 0, 1, 1);
	DrawAngle(stParas[0].roi, Scalar(255, 0, 0), roi,  1);
	DrawAngle(stParas[1].roi, Scalar(0, 255, 0), roi, 2);
	DrawAngle(stParas[2].roi, Scalar(255, 0, 255), roi, 2);

	for (int i = 0; i < targetCars.size(); i++)
	{
		if (targetCars[i].flag)
		{
			DrawAngle(targetCars[i].rect, Scalar(0, 0, 255), roi, 1);
		}
	}

	if (stKeyCar.flag == 0x02)
	{
		DrawAngle(stKeyCar.rect, Scalar(255, 255, 255), roi, 1);
		DrawAngle(stTrackedPara.roi, Scalar(100, 100, 100), roi, 1);
	}
	
}


uint8 Fcw::LbpDetectCars(stDetectParas stPara)
{
	Mat pFrame;
	if (stPara.multiNum == 0x01)
		pFrame = matFrame;
	else
		pFrame = smallFrame;

//	printf("stPara:\n");
	stCarTarget tmpCar;
	pLdw->AsertROI(stPara.roi);
	cascCar.detectMultiScale(pFrame(stPara.roi), lbpCars, stPara.scale, 1, 0, stPara.minSize, stPara.maxSize);
//	printf("size=%d\n", lbpCars.size());
	for (int i = 0; i < lbpCars.size(); i++)
	{
		tmpCar.rect.x = (stPara.roi.x + lbpCars[i].x)*stPara.multiNum;
		tmpCar.rect.y = (stPara.roi.y + lbpCars[i].y)*stPara.multiNum;
		tmpCar.rect.width = lbpCars[i].width * stPara.multiNum;
		tmpCar.rect.height = lbpCars[i].height * stPara.multiNum;
		tmpCar.flag = 0x01;
		tmpCar.position = tmpCar.rect.x + tmpCar.rect.width / 2;
		tmpCar.dist = tmpCar.rect.y + tmpCar.rect.height;

		targetCars.insert(targetCars.end(), tmpCar);
	}

	return lbpCars.size();
}


void Fcw::MergeTargetCars(void)
{
	if (targetCars.size() < 2)
		return;

	for (int i = 0; i < targetCars.size(); i++)
	{
		for (int j = i + 1; j < targetCars.size(); j++)
		{
			ConisideRate(targetCars[i], targetCars[j]);
		}
	}
}

inline void GetPointInLine(Vec4f line, int y, Point& point)
{
	if (line[0] * line[1] == 0)
	{
		point.x = 0;
		point.y = 0;
	}

	point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
	point.y = y;
}

void Plot_SingleLine(Mat& frame, Vec4f& plotLine, double dL)
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

float Fcw::ConisideRate(stCarTarget& carA, stCarTarget& carB)
{
	if (carA.flag == 0 || carB.flag == 0)
		return 0.0f;

	int flag = -1;
	Rect bigRect, smallRect;
	float rate = 0.0;

	// 判断两个的大小
	if (carA.rect.width > carB.rect.width)
	{
		bigRect = carA.rect;
		smallRect = carB.rect;
		flag = 0;
	}
	else
	{
		bigRect = carB.rect;
		smallRect = carA.rect;
		flag = 1;
	}

	// 求出两个矩形的左上角的交叉顶点
	// 左上角交叉顶点
	Point p0(max(smallRect.x, bigRect.x), max(smallRect.y, bigRect.y));

	// 右下角交叉顶点
	Point p1(min(smallRect.x + smallRect.width, bigRect.x + bigRect.width),
		min(smallRect.y + smallRect.height, bigRect.y + bigRect.width));

	if (p0.x > p1.x || p0.y > p1.y)
		rate = 0.0f;

	float area = (p1.x - p0.x)*(p1.y - p0.y);

	rate = area / (smallRect.width*smallRect.height);
	if (rate > overRate)
	{
		switch (flag)
		{
		case 0:
			carA.flag = 0;
			break;
		case 1:
			carB.flag = 0;
			break;
		}
	}

//	printf("%f\n", rate);
	return rate;
}

void Fcw::FindKeyCar(void)
{
	Point left_end;
	Point right_end;

	/* 车道线 */
	if (!pLdw->leftLane.tracked || !pLdw->leftLane.tracked || 1)
	{
		Vec4f BvLeftLane = { 0.01f, 1, 150, 100 };
		Vec4f leftLane;
		Vec4f BvrightLane = { 0.01f, 1, 750, 100 };
		Vec4f rightLane;

		BvLane2Lane(BvLeftLane, pLdw->back_trans_Matrix, leftLane);
		BvLane2Lane(BvrightLane, pLdw->back_trans_Matrix, rightLane);

		pLdw->leftLane.curLane = leftLane;
		pLdw->rightLane.curLane = rightLane;

		Plot_SingleLine(matFrame, leftLane, 1000);
		Plot_SingleLine(matFrame, rightLane, 1000);
	}

//	printf("targetCars.size=%d\n", targetCars.size());

	/* 筛选车道线内部的点 */
	for (int i = 0; i < targetCars.size(); i++)
	{
		if (targetCars[i].flag == 0x00)
			continue;

		int y = targetCars[i].rect.y + targetCars[i].rect.height / 2;
		y = max(y, 320);
		GetPointInLine(pLdw->leftLane.curLane, y, left_end);
		GetPointInLine(pLdw->rightLane.curLane, y, right_end);

		//printf("left=%d, rifht=%d, pos=%f\n, dist=%f\n ", 
		//	left_end.x, right_end.x, targetCars[i].position, targetCars[i].dist);

		if ((float)targetCars[i].position > left_end.x - 10 && (float)targetCars[i].position < right_end.x + 10)
		{
			targetCars[i].flag = 0x02; 
		//	printf("targetCars[%d]=%f\n", i, targetCars[i].dist);
		}
	}

	/* 找到关键目标 */
	stKeyCar.dist = 0;
	stKeyCar.flag = 0x00;
	for (int i = 0; i < targetCars.size(); i++)
	{
		if (targetCars[i].flag != 0x02)
			continue;

		if (stKeyCar.dist < targetCars[i].dist)
		{
			stKeyCar = targetCars[i];
			stKeyCar.flag = 0x02;
		//	printf("targetCars[%d]=%f\n", i, targetCars[i].dist);
		}
	}

	/* 进行跟踪的车 */
	if (stKeyCar.dist > 0 )
	{
		CalCarDistance(stPreCar, stKeyCar);
		if (stKeyCar.trackFlag == 1)
		{
			CheckState(10);
		}
		stPreCar = stKeyCar;
		stPreCar.flag = 0x02;
		stKeyCar.flag = 0x02;
	}
	else
	{
		stPreCar.flag = false;
	}
}

void Fcw::CalCarDistance(stCarTarget preCar, stCarTarget& keyCar)
{
	float distWidth = 5136 * pow(keyCar.rect.width, -1.09);
	float distEdge = 0;
	int flag = 0;
//	printf("distance=%f\n", distance);

	char dist[400];
	string distPrint;
	sprintf(dist, "%d", int(distWidth));
	distPrint = dist;
	Point pos(keyCar.rect.x, keyCar.rect.y - 50);
	//putText(matFrame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);

	vector<Point2f> points, points_trans;
	points.push_back(Point2f(keyCar.rect.x, keyCar.rect.y + keyCar.rect.height));
	points.push_back(Point2f(keyCar.rect.x + keyCar.rect.width, keyCar.rect.y + keyCar.rect.height));
	perspectiveTransform(points, points_trans, transMatrix);
	distEdge = float(points_trans[0].y * 0.005f + points_trans[1].y * 0.005f);
	sprintf(dist, "%.3f m", distEdge);
	distPrint = dist;
	pos = Point(keyCar.rect.x, keyCar.rect.y - 20);
	//putText(matFrame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);

	if (abs(distWidth - preCar.dist) < 5 || abs(distEdge - preCar.dist) < 5)
	{
		if (abs(distEdge - distWidth) < 5)
		{
			keyCar.dist = (distWidth + distEdge) * 0.5f;
		}
		else 
		{
			if (preCar.dist > 50)
			{
				keyCar.dist = distWidth;
			}
			else
			{
				keyCar.dist = distEdge;
			}
		}
	}
	else
	{
		if (preCar.dist > 50)
		{
			keyCar.dist = distWidth;
		}
		else
		{
			keyCar.dist = distEdge;
		}
	}
	if (preCar.trackFlag == 1)
	{
		keyCar.dist = (preCar.dist + distWidth) * 0.5f;
	}
	else
	{
		keyCar.dist = distWidth;
	}
	
	sprintf(dist, "%.1f m", keyCar.dist);
	distPrint = dist;
	pos = Point(keyCar.rect.x, keyCar.rect.y - 80);
	putText(matFrame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);

}

void Fcw::InitTrackedPara(void)
{
	stTrackedPara.multiNum = 1;
	stTrackedPara.roi.x = stPreCar.rect.x - stPreCar.rect.width/2;
	stTrackedPara.roi.y = stPreCar.rect.y - stPreCar.rect.height/2;
	stTrackedPara.roi.width = stPreCar.rect.width * 2;
	stTrackedPara.roi.height = stPreCar.rect.height * 2;

	stTrackedPara.minSize = Size(stPreCar.rect.width / 1.5, stPreCar.rect.height / 1.5);
	stTrackedPara.maxSize = Size(stPreCar.rect.width * 1.5, stPreCar.rect.height * 1.5);

	stTrackedPara.scale = 1.2;
}

float calculateSafeDist(unsigned short Speed)
{
	float iCarRange = 0;
	float iManRange = 0;
	float iManTime = 1.f;
	float iCarU = 0.8f;
	float iSpeed = Speed / 10.f;

	iCarRange = (float)((iSpeed*iSpeed) / (3.6 * 2 * 9.8 * 3.6) / iCarU);
	iManRange = iSpeed * iManTime / 3.6f;

	return (iCarRange + iManRange);
}

void Fcw::CheckState(unsigned short speed)
{
	//int stateFlag = 0;  //报警状态为0（无危险）  报警状态为1（有危险）
	float safeDist = 0;
	float disChange = 0;
	countFlag[2]++;
	cout << stKeyCar.dist << endl;
	if (stPreCar.flag = 0x01 && stKeyCar.flag == 0x02)
	{
		disChange = abs(stPreCar.dist - stKeyCar.dist);
		if (disChange > 10)
		{
			stKeyCar.flag = 0x02;
			stateFlag = 0;
		}
		else
		{
			//cout << stKeyCar.dist << endl;
			safeDist = calculateSafeDist(speed);
			safeDist = 30;
			if (stKeyCar.dist < 10)
			{
				//stateFlag = 1;    // 报警
				
				cout << "WARNING" << endl;
			}
			else if (stKeyCar.dist < safeDist && stKeyCar.dist < 30)
			{
				if (stKeyCar.dist < stPreCar.dist)
				{
					countFlag[0]++;
				}
				else
				{
					stateFlag = 2; // 变红
					//countFlag = 0;
				}
				countFlag[1]++;
			}
			else if (stKeyCar.dist > 30 || stKeyCar.dist > safeDist)
			{
				stateFlag = 3; // 变绿
				countFlag[0] = 0;
				countFlag[1] = 0;
			}
		}
	}
	else
	{
		stateFlag = 0;
	}

	if (countFlag[2] > 7)
	{
		countFlag[2] %= 8;
		countFlag[1] = 0;
		countFlag[0] = 0;
	}

	if (countFlag[1] > 6 && countFlag[0] > 4)
	{
		stateFlag = 1;
	}

	if (stateFlag == 1)
	{
		char dist[100];
		string distPrint = "WARNING";
		putText(matFrame, distPrint, Point(600, 100), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);
		countFlag[1] = 0;
		countFlag[0] = 0;
	}
	else if (stateFlag == 2)
	{
		char dist[100];
		string distPrint = "RED";
		putText(matFrame, distPrint, Point(600, 100), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);
	}
	else if (stateFlag == 3)
	{
		char dist[100];
		string distPrint = "GREEN";
		putText(matFrame, distPrint, Point(600, 100), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);
	}

}

void Fcw::GetPointInLine(Vec4f line, int y, Point& point)
{
	if (line[0] * line[1] == 0)
	{
		point.x = 0;
		point.y = 0;
	}

	point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
	point.y = y;
}

void Fcw::BvLane2Lane(Vec4f BvLane, Mat backTransMatrix, Vec4f& lane)
{
	Point2f point = { 0 };
	vector<Point2f> be_point;
	vector<Point2f> af_point;
	Vec4f addLane;

	// 计算左线的鸟瞰线
	point.x = (0 - BvLane[3]) / BvLane[1] * BvLane[0] + BvLane[2];
	point.y = 0;
	be_point.push_back(point);

	point.x = BvLane[2];
	point.y = BvLane[3];
	be_point.push_back(point);

	perspectiveTransform(be_point, af_point, backTransMatrix);

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

	lane = addLane;
}

void Fcw::DrawAngle(Rect rect, Scalar color, Rect roi, int multiNum)
{
	Rect tmpRect((rect.x+roi.x)*multiNum, (rect.y+roi.y)*multiNum, 
		         rect.width*multiNum, rect.height*multiNum);

	rectangle(matFrame, tmpRect, color, 5);
}

Fcw::~Fcw(){}
