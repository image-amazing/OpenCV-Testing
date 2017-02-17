// RelativeVelocity.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv.hpp"
//#include <iostream>
#include <io.h>

using namespace cv;
using namespace std;

#define VIDEO_DIR "..\\Video"

void keyPointMatch();
Rect moveRect(Rect src, int x, int y);
void getFiles(string path, vector<string>& files);

int main() {
    Mat frame;
    Mat frame_gray;
    CascadeClassifier cascade("LBPcascade_car.xml");
    vector<Rect> cars;
    Rect search_roi(500, 150, 450, 350);
    Rect track_area;
    Rect target;
    Rect target_pre;
    bool tracked = false;

    VideoCapture capture;
    vector<string> video_files;
    getFiles(VIDEO_DIR, video_files);
    for (size_t i = 0; i < video_files.size(); i++) {
        capture.open(video_files[i]);
        int total_frame = capture.get(CV_CAP_PROP_FRAME_COUNT);
        int number_frame = 30;
        capture.set(CV_CAP_PROP_POS_FRAMES, 30);
        while (true) {
            capture >> frame;
            number_frame++;
            if (frame.empty() || (number_frame > total_frame - 15)) {
                capture.release();
                break;
            }
            cvtColor(frame, frame_gray, CV_BGR2GRAY);
            cars.clear();
            if (tracked) {    // target_pre exists
                track_area = Rect(target_pre.x - 0.3*target_pre.width, target_pre.y - 0.3*target_pre.height, 1.6*target_pre.width, 1.6*target_pre.height);
                cascade.detectMultiScale(frame_gray(track_area), cars, 1.1, 1, 0, Size(30,30));
                if (cars.size() > 0) {
                    target = moveRect(cars[0], track_area.x, track_area.y);
                    keyPointMatch();
                    rectangle(frame, target, Scalar(0, 255, 255), 2);
                    tracked = true;
                    target_pre = target;
                } else {
                    tracked = false;
                }
                rectangle(frame, track_area, Scalar(255, 0, 0));
            } else {          // no target_pre
                cascade.detectMultiScale(frame_gray(search_roi), cars, 1.2, 3);
                if (cars.size() > 0) {
                    target = moveRect(cars[0], search_roi.x, search_roi.y);
                    rectangle(frame, target, Scalar(0, 255, 255), 2);
                    tracked = true;
                    target_pre = target;
                } else {
                    tracked = false;
                }
            }
            rectangle(frame, search_roi, Scalar(255, 0, 0));
            imshow("R", frame);
            waitKey(20);
        }
    }


    return 0;
}


void keyPointMatch() {

}

Rect moveRect(Rect src, int x, int y) {
    Rect dst;
    dst.x = src.x + x;
    dst.y = src.y + y;
    dst.width = src.width;
    dst.height = src.height;
    return dst;
}


void getFiles(string path, vector<string>& files) {
    long hFile = 0;
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
        do {
            if ((fileinfo.attrib & _A_SUBDIR)) {
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                    getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
            } else {
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
}
