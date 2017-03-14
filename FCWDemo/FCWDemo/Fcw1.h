#pragma once

#include <opencv.hpp>
#include "core.hpp"
#include "highgui.hpp"
#include "imgproc.hpp"
#include "objdetect.hpp"
#include "Ldw.h"

using namespace cv;
using namespace std;

typedef unsigned char uint8;

typedef struct
{
	int		multiNum;		// ���ű���
	Rect	roi;			// �������� 
	Size	minSize;		// ��Χ��Сֵ
	Size	maxSize;		// ��Χ���ֵ
	float	scale;
}stDetectParas;

typedef struct 
{
	uint8 flag;
	uint8 trackFlag;  // 1���� 0���

	float dist;
	float preDist;
	float position;

	Rect rect;
}stCarTarget;




class Fcw
{
public:
	Fcw();
	~Fcw();

	Ldw *pLdw;

	void Detect(void);
	void CheckState(unsigned short speed);
	Mat matFrame;
	Mat smallFrame;
	Mat transMatrix;
	Mat backTransMatrix;
	stDetectParas stParas[3];
	uint8 InitAllParas(void);

	stCarTarget stPreCar;
	stCarTarget stKeyCar;
	vector<Rect> lbpCars;
	vector<int> carNums;
	uint8 LbpDetectCars(stDetectParas stPara);

	int stateFlag;   // ����״̬Ϊ0����Σ�գ�  ����״̬Ϊ1����Σ�գ� ����״̬Ϊ2 (��ɫ)  ����״̬3����ɫ��
	int countFlag[3];   // �ۼƽӽ�����

	vector<stCarTarget> targetCars;
	float overRate;
	void MergeTargetCars(void);
	float ConisideRate(stCarTarget& carA, stCarTarget& carB);

	stDetectParas stTrackedPara;
	void InitTrackedPara(void);
	void FindKeyCar(void);
	void BvLane2Lane(Vec4f BvLane, Mat backTransMatrix, Vec4f& lane);
	void GetPointInLine(Vec4f line, int y, Point& point);
	void CalCarDistance(stCarTarget preCar, stCarTarget& keyCar);

	void DrawAngle(Rect rect, Scalar color, Rect roi, int multiNum);
private:
	float SKY_LINE;
	int frameWidth, frameHeight;
	CascadeClassifier cascCar;
};

