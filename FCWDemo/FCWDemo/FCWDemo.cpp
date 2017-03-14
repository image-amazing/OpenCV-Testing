/***********************************************************************************************************************
...............................苏州清研微视电子科技有限公司    ©版权所有   未经许可 不得使用...............................
文件名：carDetectGlobal.cpp
主要功能：
作者：
起草时间：
修订记录：
备注：
***********************************************************************************************************************/
#include <stdio.h>
#include <queue>
#include <windows.h>
#include "Fcw1.h"
#include "carDetect.h"

#pragma comment(lib, "winmm.lib")

#define CAL_SPEED_NUMBER 10

//#define VIDEO_NAME "20170307110558.avi"
#define VIDEO_NAME "20170307105552.avi"

#define USE_RADAR 0
#define USE_IMAGE 1

int main() {

    VideoCapture capture;
    if (!capture.open(VIDEO_NAME)) {
        cout << "Cannot open the video." << endl;
        return -1;
    }
    CascadeClassifier cascade1;
    if (!cascade1.load("cascade_995_50.xml")) {
        cout << "Cannot load the cascade." << endl;
        return -1;
    }

    CascadeClassifier cascade2;
    if (!cascade2.load("cascade_995_50.xml")) {
        cout << "Cannot load the cascade." << endl;
        return -1;
    }

    Ldw cLdw;
    cLdw.InitMatrixData();

    Fcw cFcw;
    cFcw.pLdw = &cLdw;

    Mat frame;
    int frameNum = 0;
    double t;
    CAR_TARGET car, carPre;

    Mat framePlot;
    unsigned char* buf = (unsigned char*)malloc(4000 * 4000);

    int c = 0;

    CAR_TARGET curTarget;
    CAR_TARGET preTarget;
    Mat greyFrame;
    //preTarget.flag = 0;
    char saveName[400];
    int size = 0;
    char num_name[20];
    unsigned short radar_speed = 0;
    float radar_dist = 0;
    float safeDist = 0;

    float img_speed = 0;
    float img_dist = 0;

    unsigned int WarnFlag = 0;
    string save_path = "D:\\dataSet\\";
    string save_name;
    //curTarget.flag = 0;
    t = (double)getTickCount();

    Mat smallFrame;
    cFcw.transMatrix = cLdw.trans_Matrix;
    cFcw.backTransMatrix = cLdw.back_trans_Matrix;

    int sleep = 0;

    while (c != 27) {
        capture >> frame;

        if (sleep > 0) {
            sleep--;
        } else {
            frameNum++;
            //sprintf(num_name, "%.6d", frameNum);
            cFcw.matFrame = frame;
            resize(frame, smallFrame, Size(640, 360));
            cFcw.smallFrame = smallFrame;
            save_name = save_path + "Img_" + num_name + ".jpg";

            if (!frame.empty()) {
                cvtColor(frame, greyFrame, CV_RGB2GRAY);
                cout << "Frame No. " << frameNum << endl;

                if (frameNum == 253) {
                    frameNum = 253;
                }
                ////LDW
                //cLdw.frame = greyFrame;
                //cLdw.Detect();
                //PlotSingleLine(frame, cLdw.leftLane.curLane, 300.0);
                //PlotSingleLine(frame, cLdw.rightLane.curLane, 300.0);

                //FCW
                //forwardColWarning(frame, preTarget, cascade1, cascade2, cLdw.trans_Matrix, curTarget, cLdw, buf);

                cFcw.Detect();

                //报警状态为0（无危险）  报警状态为1（有危险）
                if (cFcw.stateFlag == 1) {
                    cout << "Wanning" << endl;
                    //cFcw.stateFlag = 0;
                    //cFcw.countFlag = 0;
                    //sleep = 30;
                } else if (cFcw.stateFlag == 2) {
                    cout << "RED" << endl;
                }
            }
        }

        imshow("result", frame);
        //imwrite(save_name, frame);
        if (cFcw.stateFlag == 1) {
            c = cvWaitKey(0);
            cout << "Wanning" << endl;
            cFcw.stateFlag = 0;
            cFcw.countFlag[0] = 0;
            sleep = 60;
        } else {
            c = cvWaitKey(5);
        }

        cout << "state1:   " << cFcw.countFlag[0] << endl;
        cout << "state2:   " << cFcw.countFlag[1] << endl;
        if (c == 27)
            c = cvWaitKey(0);
    }

    t = (double)getTickCount() - t;
    t = t / ((double)getTickFrequency() / 1000.);
    cout << endl << "Time Used:" << t << endl << endl;

    free(buf);

    return 0;
}

