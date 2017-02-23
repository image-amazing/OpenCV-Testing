#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include <iostream>
#include <io.h>
#include <fstream>

using namespace cv;
using namespace cv::face;
using namespace std;

#define TRAIN_NEW_MODEL 0
#define VIDEO_TEST 1
#define VIDEO_DIR "..\\Video"
#define TEST_DIR "..\\VideoBack"

#define DATABASE_SIZE 20    // 数据库人数
#define WINDOW_SIZE 30      // 判断结果的滑动时间窗长度
#define EIGEN_RESULT 1
#define FISHER_RESULT 2
#define LBPH_RESULT 3


class MyRecognizer {
  public:
    MyRecognizer();
    void setResult(int r, int mode);
    void updataInfo();
    int getRecognizeResult();

  private:
    deque<int> eigen_result;
    deque<int> fisher_result;
    deque<int> lbph_result;
    deque<Vec2i> frame_result;  // 当前帧的识别结果和可信度
};

MyRecognizer::MyRecognizer():
    eigen_result(WINDOW_SIZE), fisher_result(WINDOW_SIZE), lbph_result(WINDOW_SIZE),
    frame_result(WINDOW_SIZE) {
    for (size_t i = 0; i < WINDOW_SIZE; i++) {
        eigen_result[i] = -1;
        fisher_result[i] = -1;
        lbph_result[i] = -1;
        frame_result[i][0] = -1;
        frame_result[i][1] = 0;
    }
}

void MyRecognizer::setResult(int r, int mode) {
    switch (mode) {
    case EIGEN_RESULT: {
        eigen_result.pop_front();
        eigen_result.push_back(r);
        break;
    }
    case FISHER_RESULT: {
        fisher_result.pop_front();
        fisher_result.push_back(r);
        break;
    }
    case LBPH_RESULT: {
        lbph_result.pop_front();
        lbph_result.push_back(r);
        break;
    }
    default:
        cout << "Wrong type!" << endl;
        break;
    }
}

void MyRecognizer::updataInfo() {
    int r1 = eigen_result[WINDOW_SIZE - 1];
    int r2 = fisher_result[WINDOW_SIZE - 1];
    int r3 = lbph_result[WINDOW_SIZE - 1];
    frame_result.pop_front();

    Vec2i result;
    if (r1 == r2 && r2 == r3) {
        result = { r1, 3 };
        frame_result.push_back(result);
    } else if (r1 == r2) {
        result = { r1, 1 };
        frame_result.push_back(result);
    } else if (r1 == r3) {
        result = { r1, 1 };
        frame_result.push_back(result);
    } else if (r2 == r3) {
        result = { r2, 1 };
        frame_result.push_back(result);
    } else {
        result = { DATABASE_SIZE, 3 };
        frame_result.push_back(result);
    }
}

int MyRecognizer::getRecognizeResult() {
    int hist[DATABASE_SIZE + 1] = {0};
    for (size_t i = 0; i < WINDOW_SIZE; i++) {
        hist[frame_result[i][0]-1] += frame_result[i][1];
    }

    int max_hist = -1;
    int temp_hist;
    int max_index;
    for (size_t i = 0; i < DATABASE_SIZE+1; i++) {
        temp_hist = hist[i];
        if (temp_hist > max_hist) {
            max_hist = temp_hist;
            max_index = i;
        }
    }

    /*for (size_t i = 0; i < DATABASE_SIZE; i++) {
        cout << hist[i] << ", ";
    }
    cout << hist[DATABASE_SIZE] << endl;*/

    if (max_hist > 2 * WINDOW_SIZE) {
        return max_index + 1;
    } else {
        return DATABASE_SIZE + 1;
    }
}

void getFiles(string path, vector<string>& files);
Rect enlargeRect(Rect src, float scale);

int main(int argc, const char *argv[]) {
    CascadeClassifier cascade("lbpcascade_frontalface_improved.xml");
    Ptr<BasicFaceRecognizer> model1 = createEigenFaceRecognizer();
    Ptr<BasicFaceRecognizer> model2 = createFisherFaceRecognizer();
    Ptr<LBPHFaceRecognizer> model3 = createLBPHFaceRecognizer();
    Mat frame;
    Mat frameGray;
    vector<Rect> recs;
    Mat face;
    Rect face_rect;
    Size face_size(100, 100);

    if (TRAIN_NEW_MODEL) {
        vector<Mat> images;
        vector<int> labels;
        vector<string> files;
        getFiles(VIDEO_DIR, files);
        VideoCapture capture;
        for (int j = 0; j < DATABASE_SIZE; j++) {
            capture.open(files[j]);
            cout << files[j] << endl;
            int frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);
            int frame_count = 0;
            int face_count = 0;
            while (frame_count < frame_num) {
                capture >> frame;
                if (face_count > 40) {
                    capture.release();
                    cout << endl;
                    break;
                }
                if (frame_count % 3 == 0) {
                    cvtColor(frame, frameGray, CV_BGR2GRAY);
                    recs.clear();
                    cascade.detectMultiScale(frameGray, recs, 1.3, 3, 0, Size(120, 120));
                    if (recs.size() > 0) {
                        face_rect = enlargeRect(recs[0], 0.1);
                        face = frameGray(face_rect).clone();
                        resize(face, face, face_size);
                        normalize(face, face, 0, 255, NORM_MINMAX, CV_8UC1);
                        images.push_back(face);
                        labels.push_back(j+1);
                        cout << ".";
                        face_count++;
                        rectangle(frame, face_rect, Scalar(255, 0, 0), 2);
                        imshow("R", frame);
                        imshow("Face", face);
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

        ofstream fout("Result.txt");

        for (size_t i = 0; i < files.size(); i++) {
            capture.open(files[i]);
            cout << files[i] << endl;
            MyRecognizer recog;
            int frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);
            int frame_count = 0;
            int detect_count = 0;
            int wrong_count[3];
            int time_count[3];
            double confidence_count[6];
            int t1, t2;
            while (frame_count < frame_num) {
                capture >> frame;
                if (!frame.empty()) {
                    cvtColor(frame, frameGray, CV_BGR2GRAY);
                    recs.clear();
                    cascade.detectMultiScale(frameGray, recs, 1.3, 3, 0, Size(120, 120));
                    if (recs.size() > 0) {
                        face_rect = enlargeRect(recs[0], 0.1);
                        face = frameGray(face_rect).clone();
                        resize(face, face, face_size);
                        normalize(face, face, 0, 255, NORM_MINMAX, CV_8UC1);
                        int predictedLabel = -1;
                        double confidence = 0.0;

                        // EigenFace
                        t1 = getTickCount();
                        model1->predict(face, predictedLabel, confidence);
                        recog.setResult(predictedLabel, EIGEN_RESULT);
                        if (predictedLabel != i) {
                            wrong_count[0]++;
                            //fout << 0 << "," << confidence << ",";
                        } else {
                            //fout << confidence << "," << 0 << ",";
                        }
                        t2 = getTickCount();
                        time_count[0] += (t2 - t1);
                        //cout << (t2 - t1) / getTickFrequency() * 1000 << ", ";
                        //cout << "1: P = " << predictedLabel << ", C = " << confidence << endl;

                        // FisherFace
                        t1 = getTickCount();
                        model2->predict(face, predictedLabel, confidence);
                        recog.setResult(predictedLabel, FISHER_RESULT);
                        if (predictedLabel != i) {
                            wrong_count[1]++;
                            //fout << 0 << "," << confidence << ",";
                        } else {
                            //fout << confidence << "," << 0 << ",";
                        }
                        t2 = getTickCount();
                        time_count[1] += (t2 - t1);
                        //cout << (t2 - t1) / getTickFrequency() * 1000 << ", ";
                        //cout << "2: P = " << predictedLabel << ", C = " << confidence << endl;

                        // LBPHFace
                        t1 = getTickCount();
                        model3->predict(face, predictedLabel, confidence);
                        recog.setResult(predictedLabel, LBPH_RESULT);
                        if (predictedLabel != i) {
                            wrong_count[2]++;
                            //fout << 0 << "," << confidence << ",";
                        } else {
                            //fout << confidence << "," << 0 << ",";
                        }
                        t2 = getTickCount();
                        time_count[2] += (t2 - t1);
                        //cout << (t2 - t1) / getTickFrequency() * 1000 << endl;
                        //cout << "3: P = " << predictedLabel << ", C = " << confidence << endl;
                        //fout << endl;

                        recog.updataInfo();

                        if (detect_count % (WINDOW_SIZE/2) == 0 && detect_count>WINDOW_SIZE) {
                            int final_result = recog.getRecognizeResult();
                            if (final_result == DATABASE_SIZE+1) {
                                cout << "Not passed." << endl;
                            } else {
                                cout << "Result = " << final_result << endl;
                            }
                        }

                        detect_count++;
                        rectangle(frame, recs[0], Scalar(255, 0, 0), 2);
                        imshow("Face", face);
                        imshow("R", frame);
                        waitKey(10);
                    }
                }
                frame_count++;
            }
            capture.release();

            // 错误率及时间消耗统计
            /*fout << "No. " << i << " error rate = " << float(wrong_count[0]) / detect_count << ", "
                 << float(wrong_count[1]) / detect_count << ", " << float(wrong_count[2]) / detect_count << endl;
            fout << "No. " << i << " time = " << time_count[0] / detect_count << ", "  << time_count[1] / detect_count
                 << ", " << time_count[2] / detect_count << endl;*/

        }
        fout.close();
        waitKey();
        return 0;
    }
}


Rect enlargeRect(Rect src, float scale) {
    Rect dst;
    dst.x = src.x - scale*src.width;
    dst.y = src.y - scale*src.height;
    dst.width = (1 + 2 * scale)*src.width;
    dst.height = (1 + 2 * scale)*src.height;
    if (dst.x <= 0 || dst.y <= 0 || dst.x + dst.width >= 704 || dst.y + dst.height >= 480) {
        dst = src;
    }
    return dst;
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