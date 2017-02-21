#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include <iostream>
#include <io.h>

using namespace cv;
using namespace cv::face;
using namespace std;

#define TRAIN_NEW_MODEL 1
#define VIDEO_TEST 1
#define VIDEO_DIR "..\\Video"
#define TEST_DIR "..\\VideoBack"

void getFiles(string path, vector<string>& files);

int main(int argc, const char *argv[]) {
    CascadeClassifier cascade("lbpcascade_frontalface_improved.xml");
    Ptr<BasicFaceRecognizer> model1 = createEigenFaceRecognizer(80);
    Ptr<BasicFaceRecognizer> model2 = createFisherFaceRecognizer(80);
    Ptr<LBPHFaceRecognizer> model3 = createLBPHFaceRecognizer();
    Mat frame;
    Mat frameGray;
    vector<Rect> recs;
    Mat face;

    if (TRAIN_NEW_MODEL) {
        vector<Mat> images;
        vector<int> labels;

        vector<string> files;
        getFiles(VIDEO_DIR, files);
        VideoCapture capture;
        for (int j = 0; j < files.size(); j++) {
            capture.open(files[j]);
            cout << files[j] << endl;
            int frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);
            int frame_count = 0;
            int face_count = 0;
            while (frame_count < frame_num) {
                capture >> frame;
                if (face_count > 30) {
                    capture.release();
                    cout << endl;
                    break;
                }
                if (frame_count % 3 == 0) {
                    cvtColor(frame, frameGray, CV_BGR2GRAY);
                    recs.clear();
                    cascade.detectMultiScale(frameGray, recs, 1.3, 3, 0, Size(100, 100));
                    if (recs.size() > 0) {
                        resize(frameGray(recs[0]), face, Size(160, 160));
                        //normalize(face, face, 0, 255, NORM_MINMAX, CV_8UC1);
                        images.push_back(face);
                        labels.push_back(j+1);
                        cout << ".";
                        face_count++;
                        rectangle(frame, recs[0], Scalar(255, 0, 0), 2);
                        imshow("R", frame);
                        waitKey(10);
                    }
                }
                frame_count++;
            }
        }
        cout << "Training eigenfaces." << endl;
        model1->train(images, labels);
        model1->save("eigenfaces.yml");
        cout << "Training fisherfaces." << endl;
        model2->train(images, labels);
        model2->save("fisherfaces.yml");
        cout << "Training lbphfaces." << endl;
        model3->train(images, labels);
        model3->save("lbphfaces.yml");
    } else {
        model1->load("eigenfaces.yml");
        model2->load("fisherfaces.yml");
        model3->load("lbphfaces.yml");
    }

    if (VIDEO_TEST) {
        //VideoCapture capture(0);
        vector<string> files;
        getFiles(VIDEO_DIR, files);
        VideoCapture capture;

        for (size_t i = 1; i < files.size(); i++) {
            capture.open(files[i]);
            cout << files[i] << endl;
            int frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);
            int frame_count = 0;
            while (frame_count < frame_num) {
                capture >> frame;
                if (!frame.empty()) {
                    cvtColor(frame, frameGray, CV_BGR2GRAY);
                    recs.clear();
                    cascade.detectMultiScale(frameGray, recs, 1.3, 3, 0, Size(100, 100));
                    if (recs.size() > 0) {
                        resize(frameGray(recs[0]), face, Size(160, 160));
                        //normalize(face, face, 0, 255, NORM_MINMAX, CV_8UC1);
                        int predictedLabel = -1;
                        //double confidence = 0.0;
                        predictedLabel = model1->predict(face);
                        cout << "1: PredictedLabel = " << predictedLabel << endl;
                        predictedLabel = model2->predict(face);
                        cout << "2: PredictedLabel = " << predictedLabel << endl;
                        //model3->setThreshold(0.0);
                        predictedLabel = model3->predict(face);
                        cout << "3: PredictedLabel = " << predictedLabel << endl;
                        rectangle(frame, recs[0], Scalar(255, 0, 0), 2);
                        imshow("Face", face);
                        imshow("R", frame);
                        waitKey(10);
                    }
                }
                frame_count++;
            }
            capture.release();
        }
        return 0;
    }
}

void getFiles(string path, vector<string>& files) {
    long   hFile = 0;
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