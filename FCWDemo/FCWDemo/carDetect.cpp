/***********************************************************************************************************************
...............................苏州清研微视电子科技有限公司    ©版权所有   未经许可 不得使用...............................
文件名：cardetect.cpp
主要功能：
作者：
起草时间：
修订记录：
备注：
***********************************************************************************************************************/
#include "carDetect.h"
#include "Ldw.h"

void initTrackingConfig(CAR_TARGET preCarTarget, Rect& trackRoi, Size& minSearchSize, Size& maxSearchSize)
{
	trackRoi.x = MAX(preCarTarget.rect.x - preCarTarget.rect.width / 2, 0);
	trackRoi.y = MAX(preCarTarget.rect.y - preCarTarget.rect.height / 2, 0);
	trackRoi.width = MIN(preCarTarget.rect.width * 2, 1280 - trackRoi.x);
	trackRoi.height = MIN(preCarTarget.rect.height * 2, 720 - trackRoi.y);

	minSearchSize = Size(preCarTarget.rect.width / 1.5, preCarTarget.rect.height / 1.5);
	maxSearchSize = Size(preCarTarget.rect.width * 1.5, preCarTarget.rect.height * 1.5);
}

void searchDownArea(Mat& frame , CascadeClassifier& cascade, Mat& transM, CAR_CANDIDATE *carCandidate) // 陕汽
{
	float skyLine = SKYLINE;
	int minSize = 20;
	int maxSize = 320;
	double scale = 1.3;
	vector<Rect> cars;
	int nRow = frame.rows;
	int nCol = frame.cols;

	int sizeD[4];
	int sizeU[4];
	Rect roi[4];
	Scalar color[4];

	// group 0
	sizeD[0] = 20;
	sizeU[0] = 40;
	roi[0] = Rect(0.3* nCol + 64, skyLine*nRow + 0.5 * sizeU[0], 0.3*nCol, 2 * sizeU[0]);
	color[0] = Scalar(255, 0, 0);

	// group 1
	sizeD[1] = 40;
	sizeU[1] = 80;
	roi[1] = Rect(0.25* nCol + 64, skyLine*nRow - 0.5*sizeU[1] + 0.5 * sizeU[0], 0.4*nCol, 2 * sizeU[1]);
	color[1] = Scalar(0, 255, 0);

	// group 2
	sizeD[2] = 80;
	sizeU[2] = 160;
	roi[2] = Rect(0.2* nCol + 64, skyLine*nRow - 0.5*sizeU[2] + 0.5 * sizeU[0], 0.5 * nCol, 1.8 * sizeU[2]);
	color[2] = Scalar(0, 0, 255);

	// group 3
	sizeD[3] = 160;
	sizeU[3] = 320;
	roi[3] = Rect(0.15* nCol + 64, skyLine*nRow - 0.5*sizeU[3] + 0.5 * sizeU[0], 0.6 * nCol, 1.5 * sizeU[3]);
	color[3] = Scalar(255, 0, 255);

	for (int i = 3; i >= 0; i--)
	{
		rectangle(frame, roi[i], color[i], 1);
	}

	cars.clear();

	for (int i = 3; i > 1; i--)
	{
		vector<Rect> cars_roi;

		cascade.detectMultiScale(frame(roi[i]), cars_roi, 1.3, 1, 0, Size(sizeD[i], sizeD[i]), Size(sizeU[i], sizeU[i]));

		for (unsigned int j = 0; j < cars_roi.size(); j++)
		{
			cars_roi[j] = Rect(cars_roi[j].x + roi[i].x, cars_roi[j].y + roi[i].y, cars_roi[j].width, cars_roi[j].height);
		}

		cars.insert(cars.end(), cars_roi.begin(), cars_roi.end());
	}

	mergeCarTarget(cars, 0.8f);

	vector<Point2f> p(2);		// 投影前下边缘点
	vector<Point2f> pp(2);		// 投影后下边缘点
	//<Vec4f> targetPos(size);
	Mat position = Mat::zeros(Size(860, 1000), CV_8U);

	carCandidate[0].candidateNum = cars.size();

	for (unsigned int i = 0; i < cars.size(); i++)
	{
		rectangle(frame, cars[i], Scalar(255, 0, 0), 1, 8, 0);

		carCandidate[i].car_rect = cars[i];
		carCandidate[i].img_dist = cars[i].y + cars[i].height / 2;
		carCandidate[i].position = cars[i].x + cars[i].width / 2;
		carCandidate[i].candidateNum = cars.size();
	}
}
int compare_dist(const void *a, const void *b)
{
	return ((CAR_TARGET*)a)->preDist > ((CAR_TARGET*)b)->preDist ? 1 : 0;
}

float calculateDistPoint(Point p1, Point p2)
{
	float dist;
	dist = abs(p1.x - p2.x) + abs(p1.y - p2.y);
	return dist;
}

void trackPreTarget(Mat& frame, CascadeClassifier& cascade2, CAR_TARGET preCarTarget, vector<Rect>& carsTracked) // 陕汽
{
	//跟踪
	Rect trackRoi;
	Size minSearchSize;
	Size maxSearchSize;
	CAR_TARGET candidate[20];
	Point preCenter = { 0 };
	Point trackCenter = { 0 };

	preCenter.x = preCarTarget.rect.x + preCarTarget.rect.width / 2;
	preCenter.y = preCarTarget.rect.y + preCarTarget.rect.height / 2;

	initTrackingConfig(preCarTarget, trackRoi, minSearchSize, maxSearchSize);

	cascade2.detectMultiScale(frame(trackRoi), carsTracked, 1.2, 1, 0, minSearchSize, maxSearchSize);


	for (unsigned int j = 0; j < carsTracked.size() && j < 20; j++)
	{
		carsTracked[j] = Rect(carsTracked[j].x + trackRoi.x, carsTracked[j].y + trackRoi.y, carsTracked[j].width, carsTracked[j].height);
	}

	mergeCarTarget(carsTracked, 0.8f);

	for (unsigned int j = 0; j < carsTracked.size() && j < 20; j++)
	{
		candidate[j].rect = carsTracked[j];

		trackCenter.x = carsTracked[j].x + carsTracked[j].width / 2;
		trackCenter.y = carsTracked[j].y + carsTracked[j].height / 2;

		candidate[j].preDist = calculateDistPoint(preCenter, trackCenter);
	}

	if (carsTracked.size() > 1)
	{
		qsort(candidate, carsTracked.size(), sizeof(CAR_TARGET), compare_dist);
	}

	carsTracked.clear();

	carsTracked.push_back(candidate[0].rect);   //跟踪目标

}

void searchDivided(Mat& frame, CAR_TARGET preCarTarget, CascadeClassifier& cascade1, CascadeClassifier& cascade2, 
	               vector<Rect>& cars, Mat& transM, CAR_CANDIDATE *carCandidate) // 陕汽
{

	if (preCarTarget.flag == 0)
	{
		float skyLine = SKYLINE;
		int minSize = 20;
		int maxSize = 320;
		double scale = 1.3;

		int nRow = frame.rows;
		int nCol = frame.cols;

		int sizeD[4];
		int sizeU[4];
		Rect roi[4];
		Scalar color[4];

		// group 0
		sizeD[0] = 20;
		sizeU[0] = 40;
		roi[0] = Rect(0.3* nCol + 64, skyLine*nRow - 0.5 * sizeU[0], 0.3*nCol, 2 * sizeU[0]);
		color[0] = Scalar(255, 0, 0);

		// group 1
		sizeD[1] = 40;
		sizeU[1] = 80;
		roi[1] = Rect(0.25* nCol + 64, skyLine*nRow - 0.5*sizeU[1] - 0.5 * sizeU[0], 0.4*nCol, 2 * sizeU[1]);
		color[1] = Scalar(0, 255, 0);

		// group 2
		sizeD[2] = 80;
		sizeU[2] = 160;
		roi[2] = Rect(0.2* nCol + 64, skyLine*nRow - 0.5*sizeU[2] - 0.5 * sizeU[0], 0.5*nCol, 1.8 * sizeU[2]);
		color[2] = Scalar(0, 0, 255);

		// group 3
		sizeD[3] = 160;
		sizeU[3] = 320;
		roi[3] = Rect(0.15* nCol + 64, skyLine*nRow - 0.5*sizeU[3] - 0.5 * sizeU[0], 0.6*nCol, 1.5 * sizeU[3]);
		color[3] = Scalar(255, 0, 255);

		for (int i = 3; i >= 0; i--)
		{
			rectangle(frame, roi[i], color[i], 1);
		}

		cars.clear();

		for (int i = 3; i >= 0; i--)
		{
			rectangle(frame, roi[i], color[i], 1);

			vector<Rect> cars_roi;

			cascade1.detectMultiScale(frame(roi[i]), cars_roi, 1.3, 1, 0, Size(sizeD[i], sizeD[i]), Size(sizeU[i], sizeU[i]));

			for (unsigned int j = 0; j < cars_roi.size(); j++)
			{
				cars_roi[j] = Rect(cars_roi[j].x + roi[i].x, cars_roi[j].y + roi[i].y, cars_roi[j].width, cars_roi[j].height);
			}

			cars.insert(cars.end(), cars_roi.begin(), cars_roi.end());
		}

		mergeCarTarget(cars, 0.8f);

		vector<Point2f> p(2);		// 投影前下边缘点
		vector<Point2f> pp(2);		// 投影后下边缘点
		//<Vec4f> targetPos(size);
		Mat position = Mat::zeros(Size(860, 1000), CV_8U);

		carCandidate[0].candidateNum = cars.size();

		for (unsigned int i = 0; i < cars.size(); i++)
		{

			rectangle(frame, cars[i], Scalar(255, 255, 0), 1);
			carCandidate[i].car_rect = cars[i];
			carCandidate[i].position = cars[i].x + cars[i].width / 2;
			carCandidate[i].img_dist = cars[i].y + cars[i].height / 2;
			carCandidate[i].candidateNum = cars.size();
		}
	}
	else if (preCarTarget.flag == 1)
	{
		CAR_TARGET trackTarget;
		vector<Rect> carsTracked;
		trackPreTarget(frame, cascade2, preCarTarget, carsTracked);

		//危险区域
		CAR_CANDIDATE carDown[100];
		vector<Rect> carsDown;
		searchDownArea(frame, cascade1, transM, carDown);

		for (int i = 0; i < carsTracked.size(); i++)
		{
			carCandidate[i].car_rect = carsTracked[i];
			carCandidate[i].position = carsTracked[i].x + carsTracked[i].width / 2;
			carCandidate[i].candidateNum = carsTracked.size() + carDown[0].candidateNum;
			carCandidate[i].img_dist = carsTracked[i].y + carsTracked[i].height / 2;
		}

		if (carDown[0].candidateNum > 0)
		{
			for (int i = carsTracked.size(), j = 0; i < carsTracked.size() + carDown[0].candidateNum; i++, j++)
			{
				carCandidate[i].car_rect = carDown[j].car_rect;
				carCandidate[i].position = carDown[j].position;
				carCandidate[i].candidateNum = carsTracked.size() + carDown[0].candidateNum;
				carCandidate[i].img_dist = carDown[j].img_dist;
			}
		}
	}

	cars.clear();
}

void mergeCarTarget(vector<Rect>& cars, float thres)
{
	if (cars.size() > 1)
	{
		cars.push_back(Rect(2000, 2000, 10, 10));
		sort(cars.begin(), cars.end(), CompLess());

		vector<Rect> cars_sl;
		Rect rectTemp;
		bool useTemp = false;
		for (unsigned int i = 1; i < cars.size(); i++)
		{
			if (useTemp)
			{
				float rate = coincideRate(rectTemp, cars[i]);
				if (rate > thres)
				{
					rectTemp = rectTemp.width > cars[i].width ? rectTemp : cars[i];
				}
				else
				{
					cars_sl.push_back(rectTemp);
					useTemp = false;
				}
			}
			else
			{
				float rate = coincideRate(cars[i - 1], cars[i]);
				//cout << rate << endl;
				if (rate > thres)
				{
					rectTemp = cars[i - 1].width > cars[i].width ? cars[i - 1] : cars[i];
					useTemp = true;
				}
				else
				{
					cars_sl.push_back(cars[i - 1]);
				}
			}
		}
		cars = cars_sl;
	}
}

float coincideRate(Rect rect1, Rect rect2)
{
	Rect rect_s;
	Rect rect_l;

	if (rect1.width < rect2.width)
	{
		rect_s = rect1;
		rect_l = rect2;
	}
	else
	{
		rect_s = rect2;
		rect_l = rect1;
	}

	int x_min = rect_s.x < rect_l.x ? rect_s.x : rect_l.x;
	int x_max = (rect_s.x + rect_s.width) >(rect_l.x + rect_l.width) ? (rect_s.x + rect_s.width) : (rect_l.x + rect_l.width);
	int y_min = rect_s.y < rect_l.y ? rect_s.y : rect_l.y;
	int y_max = (rect_s.y + rect_s.height) >(rect_l.y + rect_l.height) ? (rect_s.y + rect_s.height) : (rect_l.y + rect_l.height);

	int dx = x_max - x_min;
	int dy = y_max - y_min;

	int x_c = rect_s.width  + rect_l.width  - dx;
	int y_c = rect_s.height + rect_l.height - dy;

	if (x_c <= 0 || y_c <= 0)
	{
		return 0;
	}
	else
	{
		float rate = (float)(x_c*y_c) / (float)(rect_s.width * rect_s.height);

		return rate;
	}
}

int compare(const void *a, const void *b)
{
	return ((CAR_CANDIDATE*)a)->img_dist < ((CAR_CANDIDATE*)b)->img_dist ? 1 : 0;
}


void getPointInLine(Vec4f line, int y, Point& point)
{
	if (line[0] * line[1] == 0)
	{
		point.x = 0;
		point.y = 0;
	}

	point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
	point.y = y;
}

void BvLane2Lane(Vec4f BvLane, Mat backTransMatrix, Vec4f& lane)
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


void plotSingleLine(Mat& frame, Vec4f& plotLine, double dL)
{
	Point p1, p2;

	getPointInLine(plotLine, frame.rows * 0.5, p1);
	getPointInLine(plotLine, frame.rows, p2);

	line(frame, p1, p2, Scalar(255, 255, 0), 10);
}

/************************************************** 获取关键目标 *****************************************************/
CAR_TARGET getKeyTarget(Mat& frame, CAR_CANDIDATE* carCandidate, Mat& transM, CAR_TARGET& preKeyTarget, Ldw cldw)
{
	CAR_TARGET keyTarget = preKeyTarget;
	Rect pre_rect = preKeyTarget.rect;

	// 根据投影后结果筛选
	int candidateNum = carCandidate->candidateNum;
	//printf("\n%d\n", candidateNum);
	if (candidateNum > 1)
	{
		//printf("\n---------------------------------\n");
		qsort(carCandidate, candidateNum, sizeof(CAR_CANDIDATE), compare);
	}
	
	float minDist = 100000;
	float curDist = 0;
	CAR_CANDIDATE temp_car;
	int j = 0;
	int reset_flag = 0;
	Point left_end;
	Point right_end;
	int y = 0;

	if (!cldw.leftLane.tracked || !cldw.leftLane.tracked || 1)
	{
		Vec4f BvLeftLane = { 0.01f, 1, 200, 100 };
		Vec4f leftLane;
		Vec4f BvrightLane = { 0.01f, 1, 700, 100 };
		Vec4f rightLane;

		BvLane2Lane(BvLeftLane, cldw.back_trans_Matrix, leftLane);
		BvLane2Lane(BvrightLane, cldw.back_trans_Matrix, rightLane);

		cldw.leftLane.curLane = leftLane;
		cldw.rightLane.curLane = rightLane;

		/*plotSingleLine(frame, leftLane, 1000);
		plotSingleLine(frame, rightLane, 1000);*/

	}

	if (preKeyTarget.flag == 1)
	{

		y = carCandidate[0].car_rect.y + carCandidate[0].car_rect.height;
		y = max(y, 320);

		getPointInLine(cldw.leftLane.curLane, y, left_end);
		getPointInLine(cldw.rightLane.curLane, y, right_end);

		if (abs(carCandidate[0].img_dist - preKeyTarget.dist) > 4 || 
			abs(carCandidate[0].position - keyTarget.position) > 10 ||
			carCandidate[0].position < left_end.x ||
			carCandidate[0].position > right_end.x)
		{
			reset_flag = 1;
		}
		else
		{
			keyTarget.dist = carCandidate[0].img_dist;	
			keyTarget.position = carCandidate[0].position;	
			keyTarget.flag = 1;	
			keyTarget.rect = carCandidate[0].car_rect;
		}
	}
	else 
	{
		reset_flag = 1;
	}

	if (reset_flag == 1)
	{
		for (int i = 0; i < candidateNum; i++)
		{
			y = carCandidate[i].car_rect.y + carCandidate[i].car_rect.height / 2;
			y = max(y, 360);
			getPointInLine(cldw.leftLane.curLane, y, left_end);
			getPointInLine(cldw.rightLane.curLane, y, right_end);

			if (carCandidate[i].position > left_end.x && carCandidate[i].position < right_end.x)
			{
				j = i;
				reset_flag = 0;
				break;
			}
		}

		if (reset_flag == 0)
		{
			keyTarget.dist = carCandidate[j].img_dist;
			keyTarget.position = carCandidate[j].position;
			keyTarget.flag = 1;
			keyTarget.rect = carCandidate[j].car_rect;
		}
		else
		{
			preKeyTarget.flag == 0;
			clearTarget(keyTarget);
		}
		
	}
	return keyTarget;
}


Rect fixCarRect(double car_Shape[])
{
	Rect carRect;

	double up = car_Shape[1] > car_Shape[3] ? car_Shape[1] : car_Shape[3];
	up = up > car_Shape[5] ? up : car_Shape[5];
	up = up > car_Shape[7] ? up : car_Shape[7];

	double down = car_Shape[13] < car_Shape[15] ? car_Shape[13] : car_Shape[15];
	down = down < car_Shape[17] ? down : car_Shape[17];
	down = down < car_Shape[19] ? down : car_Shape[19];

	double left = car_Shape[0] < car_Shape[22] ? car_Shape[0] : car_Shape[22];
	left = left < car_Shape[20] ? left : car_Shape[20];
	left = left < car_Shape[18] ? left : car_Shape[18];

	double right = car_Shape[6] > car_Shape[8] ? car_Shape[6] : car_Shape[8];
	right = right > car_Shape[10] ? right : car_Shape[10];
	right = right > car_Shape[12] ? right : car_Shape[12];

	carRect.x = left + FRAME_WIDTH / 2 - 4;
	carRect.y = FRAME_HEIGHT / 2 - up - 4;
	carRect.width = right - left + 8;
	carRect.height = up - down + 12;

	return carRect;
}

CAR_TARGET targetTracking(CAR_TARGET& curTarget, CAR_TARGET& preTarget)
{
	CAR_TARGET car;
	float rate = 0;

	if (preTarget.dist > 0)			// 存在上一帧目标
	{
		if (curTarget.dist > 0)		// 存在当前目标
		{
			preTarget.predict = false;
			
			rate = coincideRate(curTarget.rect, preTarget.rect);
			cout << rate << endl;

			if (rate > 0.1)			// 跟踪成功
			{
				car.rect.x = curTarget.rect.x;
				car.rect.y = curTarget.rect.y;
				car.rect.width = 0.5*curTarget.rect.width + 0.5*preTarget.rect.width;
				car.rect.height = 0.5*curTarget.rect.height + 0.5*preTarget.rect.height;
			}
			else                    // 新目标出现
			{
				car = curTarget;
			}	
		}
		else						// 无当前目标
		{
			if (preTarget.predict)	// 上一帧也是跟踪预测
			{
				clearTarget(preTarget);
				clearTarget(curTarget);
			}
			else					// 上一帧为检测结果
			{
				car = preTarget;
				preTarget.predict = true;
			}			
		}
	}
	else	// 无上一帧目标
	{
		car = curTarget;		// 新检测目标作为当前目标
		preTarget = curTarget;	// 更新跟踪目标
	}

	return car;
}

void clearTarget(CAR_TARGET& target)
{
	target.dist = 0;
	target.rect = Rect(0, 0, 0, 0);
	target.predict = false;
	target.flag = 0;
}

void clearCarCandidate(CAR_CANDIDATE* carCandidate)
{
	for (int i = 0; i < carCandidate->candidateNum; i++)
	{
		carCandidate[i].candidateNum = 0;
		carCandidate[i].car_rect = { 0, 0, 0, 0 };
		carCandidate[i].img_dist = 0;
		carCandidate[i].overlapRate = 0;
		carCandidate[i].position = 0;
	}
}

void calTargetDistance(CAR_TARGET& target)
{
	target.dist = 5136 * pow(target.rect.width, -1.09);

	if (target.dist > 150 || target.dist < 0)
	{
		target.dist = 0;
	}
}


void getTargetDistance(Mat& frame, CAR_TARGET& target, Mat& transM)
{
	calTargetDistance(target);
	rectangle(frame, target.rect, Scalar(0, 255, 255), 3, 8, 0);

	target.flag = 1;

	char dist[400];
	sprintf(dist, "%d", int(target.dist));
	string distPrint = dist;
	Point pos(target.rect.x, target.rect.y - 30);
	putText(frame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 3);

	vector<Point2f> points, points_trans;
	points.push_back(Point2f(target.rect.x, target.rect.y + target.rect.height));
	points.push_back(Point2f(target.rect.x + target.rect.width, target.rect.y + target.rect.height));
	perspectiveTransform(points, points_trans, transM);

	sprintf(dist, "%.3f m", float(points_trans[0].y * 0.005f + points_trans[1].y * 0.005f));
	distPrint = dist;
	pos = Point(target.rect.x, target.rect.y - 20);
	putText(frame, distPrint, pos, CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3);
}

void forwardColWarning(Mat& frame, CAR_TARGET preCarTarget, CascadeClassifier& cascade1, CascadeClassifier& cascade2,
	                   Mat& transM, CAR_TARGET& curTarget, Ldw cLdw, unsigned char* buf)
{
	vector<Rect> cars;
	CAR_CANDIDATE carCandidate[100];

	searchDivided(frame, curTarget, cascade1, cascade2, cars, transM, carCandidate);

	if (carCandidate[0].candidateNum > 0)
	{
		curTarget = getKeyTarget(frame, carCandidate, transM, preCarTarget, cLdw);

		if (curTarget.flag == 1)
		{
			//curTarget = getCarContour(frame, curTarget, DetParams, CurImg, car_Model, buf);

			getTargetDistance(frame, curTarget, cLdw.back_trans_Matrix);
		}
	}
	else
	{
		curTarget.flag = 0;
	}

	clearCarCandidate(carCandidate);
}
