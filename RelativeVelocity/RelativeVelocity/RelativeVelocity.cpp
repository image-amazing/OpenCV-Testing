// RelativeVelocity.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include <iostream>
#include <io.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

#define VIDEO_DIR "..\\Video"

//void keyPointMatch(vector<KeyPoint>& corners, vector<KeyPoint>& corners_pre);
void getKeyPoints(Mat& frame, Rect target, vector<KeyPoint>& corners);
void plotKeyPoints(Mat& frame, Rect target, vector<KeyPoint>& corners);
Rect moveRect(Rect src, int x, int y);
Rect enlargeRect(Rect src, float scale);
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

    vector<KeyPoint> corners;
    vector<KeyPoint> corners_pre;

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
                track_area = enlargeRect(target, 0.3);
                cascade.detectMultiScale(frame_gray(track_area), cars, 1.1, 1, 0, Size(30,30));
                if (cars.size() > 0) {
                    target = moveRect(cars[0], track_area.x, track_area.y);
                    if (true) {
                        Rect area = enlargeRect(target, 0.1);
                        getKeyPoints(frame_gray, area, corners);
                        plotKeyPoints(frame, area, corners);
                        //keyPointMatch(corners, corners_pre);
                    }
                    rectangle(frame, target, Scalar(0, 255, 255), 2);
                    tracked = true;
                    target_pre = target;
                    corners_pre = corners;
                } else {
                    tracked = false;
                }
                rectangle(frame, track_area, Scalar(255, 0, 0));
            } else {          // no target_pre
                cascade.detectMultiScale(frame_gray(search_roi), cars, 1.2, 3);
                if (cars.size() > 0) {
                    target = moveRect(cars[0], search_roi.x, search_roi.y);
                    Rect area = enlargeRect(target, 0.1);
                    getKeyPoints(frame_gray, area, corners);
                    plotKeyPoints(frame, area, corners);
                    rectangle(frame, target, Scalar(0, 255, 255), 2);
                    tracked = true;
                    target_pre = target;
                    corners_pre = corners;
                } else {
                    tracked = false;
                }
            }
            rectangle(frame, search_roi, Scalar(255, 0, 0));
            imshow("R", frame);
            waitKey(50);
        }
    }


    return 0;
}


//void keyPointMatch(vector<KeyPoint>& corners, vector<KeyPoint>& corners_pre) {
//
//}


void getKeyPoints(Mat& frame, Rect target, vector<KeyPoint>& corners) {
    Ptr<Feature2D> b = SURF::create();
    Mat descImg;
    b->detect(frame(target), corners, Mat());
    b->compute(frame(target), corners, descImg);

    /*corners.clear();
    goodFeaturesToTrack(frame(target), corners, 100, 0.01, 5, Mat(), 3, false, 0.04);
    for (size_t i = 0; i < corners.size(); i++) {
        corners[i].x += target.x;
        corners[i].y += target.y;
    }*/
}


void plotKeyPoints(Mat& frame, Rect target, vector<KeyPoint>& corners) {
    Point p;
    for (size_t i = 0; i < corners.size(); i++) {
        p.x = corners[i].pt.x + target.x;
        p.y = corners[i].pt.y + target.y;
        circle(frame, p, 3, Scalar(0, 0, 255), -1);
    }
}


Rect moveRect(Rect src, int x, int y) {
    Rect dst;
    dst.x = src.x + x;
    dst.y = src.y + y;
    dst.width = src.width;
    dst.height = src.height;
    return dst;
}


Rect enlargeRect(Rect src, float scale) {
    Rect dst;
    dst.x = src.x - scale*src.width;
    dst.y = src.y - scale*src.height;
    dst.width = (1 + 2 * scale)*src.width;
    dst.height = (1 + 2 * scale)*src.height;
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
