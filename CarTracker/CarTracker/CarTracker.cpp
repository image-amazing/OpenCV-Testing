// CarTracker.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv.hpp"
#include "opencv2/tracking.hpp"
#include <io.h>

using namespace cv;
using namespace std;

#define VIDEO_DIR "..\\Video"

void getFiles(string path, vector<string>& files);

int main() {
    Rect2d roi;
    Mat frame;
    Ptr<Tracker> tracker = Tracker::create("MEDIANFLOW");
    string video = "..\\Video\\20170307110558.avi";
    VideoCapture cap(video);
    cap.set(CV_CAP_PROP_POS_FRAMES, 240);
    cap >> frame;
    roi = selectROI("tracker", frame);
    tracker->init(frame, roi);
    int frame_num = 0;
    for (;; ) {
        cap >> frame;
        if (frame_num < 10000) {
            tracker->update(frame, roi);
            rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
            imshow("tracker", frame);
            if (waitKey(20) == 27) break;
            frame_num++;
        } else {
            roi = selectROI("tracker", frame);
            tracker->init(frame, roi);
            frame_num = 0;
        }
    }
    return 0;
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

