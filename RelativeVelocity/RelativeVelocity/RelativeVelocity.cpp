// RelativeVelocity.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include <iostream>
#include <io.h>
#include <fstream>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

#define VIDEO_DIR "..\\Video"


class OptFlowlLK {
  public:
    OptFlowlLK();
    void processLK(Mat& frame);
    void sysInit(Mat& frame, Rect target);
    void sysUpdate(Mat& frame, Rect target);
    void plotKeyPoints(Mat& frame);
    void write2file(ofstream& ofile);
    vector<Vec4f> point_offset;     // [特征点相对目标框的位置(x,y)，特征点偏移的位移(x,y)]

  private:
    TermCriteria termcrit;
    Size subPix_winSize;
    Size winSize;
    Mat frame_pre;
    Rect target_pre;
    vector<Point2f> corners;
    vector<Point2f> corners_pre;
    vector<uchar> status;
    vector<float> err;

    void calcPointsOffset();
};

OptFlowlLK::OptFlowlLK() {
    termcrit = TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
    subPix_winSize = Size(10, 10);
    winSize = Size(21, 21);
}

void OptFlowlLK::sysInit(Mat& frame, Rect target) {
    corners_pre.clear();
    goodFeaturesToTrack(frame(target), corners_pre, 80, 0.01, 5, Mat(), 3, false, 0.04);
    cornerSubPix(frame(target), corners_pre, subPix_winSize, Size(-1, -1), termcrit);
    target_pre = target;
    frame_pre = frame.clone();
}

void OptFlowlLK::sysUpdate(Mat& frame, Rect target) {
    corners_pre.clear();
    goodFeaturesToTrack(frame(target), corners_pre, 80, 0.01, 5, Mat(), 3, false, 0.04);
    cornerSubPix(frame(target), corners_pre, subPix_winSize, Size(-1, -1), termcrit);
    target_pre = target;
    frame_pre = frame.clone();
}

void OptFlowlLK::processLK(Mat& frame) {
    corners.clear();
    calcOpticalFlowPyrLK(frame_pre(target_pre), frame(target_pre), corners_pre, corners, status,
                         err, winSize, 3, termcrit, 0, 0.001);
    //cout << corners_pre.size() << ", " << corners.size() << endl;
    calcPointsOffset();
}

void OptFlowlLK::plotKeyPoints(Mat& frame) {
    Point2f p;
    for (size_t i = 0; i < corners.size(); i++) {
        p.x = corners[i].x + target_pre.x;
        p.y = corners[i].y + target_pre.y;
        circle(frame, p, 2, Scalar(0, 0, 255), -1);
    }
}

void OptFlowlLK::calcPointsOffset() {
    point_offset.resize(corners.size());
    for (size_t i = 0; i < corners.size(); i++) {
        if (status[i] == 1) {
            point_offset[i][0] = corners[i].x - float(target_pre.width) / 2.0f;
            point_offset[i][1] = corners[i].y - float(target_pre.height) / 2.0f;
            point_offset[i][2] = corners[i].x - corners_pre[i].x;
            point_offset[i][3] = corners[i].y - corners_pre[i].y;
        } else {
            point_offset[i][0] = 0;
            point_offset[i][1] = 0;
            point_offset[i][2] = 0;
            point_offset[i][3] = 0;
        }
    }
    cout << endl;
}

void OptFlowlLK::write2file(ofstream& fout) {
    for (size_t i = 0; i < point_offset.size(); i++) {
        fout << point_offset[i][0] << ", ";
        fout << point_offset[i][1] << ", ";
        fout << point_offset[i][2] << ", ";
        fout << point_offset[i][3] << endl;
    }
}


//void keyPointMatch(vector<KeyPoint>& corners, vector<KeyPoint>& corners_pre);
//void getKeyPoints(Mat& frame, Rect target, vector<Point2f>& corners);
//void plotKeyPoints(Mat& frame, Rect target, vector<Point2f>& corners);
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

    vector<Point2f> corners;
    vector<Point2f> corners_pre;

    OptFlowlLK optflow;

    VideoCapture capture;
    vector<string> video_files;
    getFiles(VIDEO_DIR, video_files);
    for (size_t i = 0; i < video_files.size(); i++) {
        capture.open(video_files[i]);
        int total_frame = capture.get(CV_CAP_PROP_FRAME_COUNT);
        int number_frame = 30;
        capture.set(CV_CAP_PROP_POS_FRAMES, 30);
        string ofile_name = video_files[i] + ".txt";
        ofstream ofile(ofile_name);
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
                    rectangle(frame, target, Scalar(0, 255, 255), 1);
                    if (true) {
                        Rect area = enlargeRect(target, 0.1);
                        optflow.processLK(frame_gray);
                        optflow.write2file(ofile);
                        optflow.sysUpdate(frame_gray, area);
                        optflow.plotKeyPoints(frame);
                    }
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
                    optflow.sysUpdate(frame_gray, area);
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
            if (waitKey(10) == 'q') {
                break;
            }
        }
        capture.release();
        ofile.close();
    }


    return 0;
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
