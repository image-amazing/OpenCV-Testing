// LidarData.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "opencv.hpp"
#include <io.h>
#include <fstream>

#define HORIZONTAL_RANGE 90    // degree
#define VERTICAL_RANGE 26       // degree
#define FRAME_WIDTH 720
#define FRAME_HEIGHT 480
#define INTENS_THRES 1.0
#define DIST_THRES 5
#define Z_THRES -1.5

using namespace cv;
using namespace std;

Point transCoordinate(float x, float y, float z, float intens, float dist) {
    Point p(-1, -1);
    if (y > 0 && z > Z_THRES) {
        if (intens > INTENS_THRES && dist > DIST_THRES) {
            float alfa = atan(x / y) / CV_PI * 180;
            if (abs(alfa) < HORIZONTAL_RANGE / 2) {
                p.x = alfa * FRAME_WIDTH / HORIZONTAL_RANGE + FRAME_WIDTH / 2;
            }
            float beta = atan(z / y) / CV_PI * 180;
            if (beta > -24 && beta < 2) {
                p.y = (2 - beta)*FRAME_HEIGHT / VERTICAL_RANGE ;
            }
        }
    }
    return p;
}


int dist2pixel(float dist, float intens) {
    int pixel = (255 - 2 * dist)*intens / 255;
    if (pixel < 0) {
        pixel = 0;
    }
    return pixel;
}


int main() {
    ifstream fin("point_data.txt");
    Mat frame = Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8U);
    float info[7];
    Point p;
    int pixel_dist;
    while (!fin.eof()) {
        for (size_t i = 0; i < 7; i++) {
            fin >> info[i];
        }
        p = transCoordinate(info[0], info[1], info[2], info[3], info[6]);
        if (p.x > 0 && p.y > 0) {
            pixel_dist = dist2pixel(info[6], info[3]);
            if (pixel_dist > frame.at<uchar>(p.y, p.x)) {
                frame.at<uchar>(p.y, p.x) = pixel_dist;
            }
        }
    }

    imshow("Result", frame);
    imwrite("frame.jpg", frame);
    waitKey();
    return 0;
}

