/***********************************************************************************************************************
...............................苏州清研微视电子科技有限公司    ©版权所有   未经许可 不得使用...............................
文件名：cardetect.h
主要功能：
作者：
起草时间：
修订记录：
备注：
***********************************************************************************************************************/
#include <opencv.hpp>
#include "Ldw.h"

using namespace cv;
using namespace std;

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define SKYLINE 0.4
#define DET_FACE_WIDTH 200

struct CAR_TARGET
{
	Rect rect;
	float dist;
	float preDist;
	float position;
	int flag;
	
	bool predict;	// 是否是根据上一帧位置进行预测
};

typedef struct tag_CAR_CANDIDATE
{
	Rect car_rect;
	float img_dist;
	float position;
	int candidateNum;
	float overlapRate;
}CAR_CANDIDATE;

//Class Rect for sorting
class CompLess
{
public:
	bool operator()(const Rect& stItem1, const Rect& stItem2)
	{
		return stItem1.x < stItem2.x;
	}
};

class CompGreater
{
public:
	bool operator ()(const Rect& stItem1, const Rect& stItem2)
	{
		return stItem1.x > stItem2.x;
	}
};

void searchDivided(Mat& frame, CAR_TARGET preCarTarget, CascadeClassifier& cascade1, CascadeClassifier& cascade2,
	               vector<Rect>& cars, Mat& transM, CAR_CANDIDATE *carCandidate);

void mergeCarTarget(vector<Rect>& cars, float thres);

float coincideRate(Rect rect1, Rect rect2);

CAR_TARGET getKeyTarget(Mat& frame, CAR_CANDIDATE* carCandidate, Mat& transM, CAR_TARGET& preKeyTarget, Ldw cldw);


Rect fixCarRect(double car_Shape[]);

CAR_TARGET targetTracking(CAR_TARGET& curTarget, CAR_TARGET& preTarget);

void clearTarget(CAR_TARGET& target);

void clearCarCandidate(CAR_CANDIDATE* carCandidate);

void getTargetDistance(Mat& frame, CAR_TARGET& target, Mat& transM);

void forwardColWarning(Mat& frame, CAR_TARGET preCarTarget, CascadeClassifier& cascade1, CascadeClassifier& cascade2,
	                   Mat& transM, CAR_TARGET& curTarget, Ldw cLdw, unsigned char* buf);
