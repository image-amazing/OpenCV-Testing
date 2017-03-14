#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include <iostream>
#include <io.h>
#include <fstream>

using namespace cv;
using namespace cv::face;
using namespace std;

//#define TRAIN_NEW_MODEL 0
//#define VIDEO_TEST 1
//#define USE_VIDEO 1
//#define VIDEO_DIR "..\\Video"
//#define TEST_DIR "..\\VideoBack"

#define FACE_SIZE 80
//#define DATABASE_SIZE 32    // 数据库人数
#define WINDOW_SIZE 40      // 判断结果的滑动时间窗长度
#define EIGEN_RESULT 1
#define FISHER_RESULT 2
#define LBPH_RESULT 3

Ptr<BasicFaceRecognizer> model1 = createEigenFaceRecognizer();
Ptr<BasicFaceRecognizer> model2 = createFisherFaceRecognizer();
Ptr<LBPHFaceRecognizer> model3 = createLBPHFaceRecognizer();
CascadeClassifier cascade("lbpcascade_frontalface_improved.xml");

Rect enlargeRect(Rect src, float scale, Size frameSize);

class MyRecognizer {
  public:
    MyRecognizer(int s);
    void setResult(int r, int mode);
    void updataInfo();
    int getRecognizeResult();

  private:
    int database_size;
    deque<int> eigen_result;
    deque<int> fisher_result;
    deque<int> lbph_result;
    deque<Vec2i> frame_result;  // 当前帧的识别结果和可信度
};

MyRecognizer::MyRecognizer(int s):
    eigen_result(WINDOW_SIZE), fisher_result(WINDOW_SIZE), lbph_result(WINDOW_SIZE),
    frame_result(WINDOW_SIZE) {
    database_size = s;
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
        result = { 0, 3 };
        frame_result.push_back(result);
    }
}

int MyRecognizer::getRecognizeResult() {
    vector<int> hist(database_size + 1);
    for (size_t i = 0; i < WINDOW_SIZE; i++) {
        hist[frame_result[i][0]] += frame_result[i][1];
    }

    int max_hist = -1;
    int temp_hist;
    int max_index;
    for (size_t i = 0; i < database_size+1; i++) {
        temp_hist = hist[i];
        if (temp_hist > max_hist) {
            max_hist = temp_hist;
            max_index = i;
        }
    }

    if (max_hist > 2 * WINDOW_SIZE) {
        return max_index;
    } else {
        return 0;
    }
}

Rect maxRect(vector<Rect>& recs);

int main(int argc, const char *argv[]) {
    // 系统初始化
    Mat frame;
    Mat frameGray;
    vector<Rect> recs;
    Mat face;
    Rect face_rect;
    Size face_size(FACE_SIZE, FACE_SIZE);
    string kInput;

    // 获取和显示当前数据库状态
    ifstream fin("FaceDatabase.txt");
    if (!fin.is_open()) {
        cout << "Load data-base err." << endl;
        return -1;
    }
    vector<string> video_dirs;
    vector<string> names;
    string cur_video;
    string cur_name;
    while (!fin.eof()) {
        fin >> cur_video;
        fin >> cur_name;
        video_dirs.push_back(cur_video);
        names.push_back(cur_name);
    }

    // 训练或载入人脸数据库
    cout << "是否重新训练人脸数据库？（Y/N）" << endl;
    cin >> kInput;
    if (kInput == "Y" || kInput == "y") {
        int data_size = names.size();
        cout << "当前共计 " << data_size << " 人。" << endl;
        cout << "名单：" << endl;
        for (size_t i = 0; i < data_size; i++) {
            cout << names[i] << " ";
        }
        cout << endl;
        vector<Mat> images;
        vector<int> labels;
        VideoCapture capture;
        Size frameSize;
        namedWindow("R");
        moveWindow("R", 200, 30);
        for (int j = 0; j < data_size; j++) {
            capture.open(video_dirs[j]);
            cout << "当前训练文件： " << video_dirs[j] << endl;
            int frame_num = capture.get(CV_CAP_PROP_FRAME_COUNT);
            int frame_count = 0;
            int face_count = 0;
            do {
                capture >> frame;
                frame_count++;
            } while (frame.empty());
            frameSize = frame.size();
            while (frame_count < frame_num-1) {
                if (face_count > 35) {
                    capture.release();
                    cout << endl;
                    break;
                }
                capture >> frame;
                if (frame_count % 3 == 0) {
                    cvtColor(frame, frameGray, CV_BGR2GRAY);
                    recs.clear();
                    cascade.detectMultiScale(frameGray, recs, 1.3, 5, 0, Size(120, 120), Size(400, 400));
                    if (recs.size() > 0) {
                        face_rect = maxRect(recs);
                        face_rect = enlargeRect(face_rect, 0.1, frameSize);
                        if (face_rect.width > 0) {
                            face = frameGray(face_rect).clone();
                            resize(face, face, face_size);
                            normalize(face, face, 0, 255, NORM_MINMAX, CV_8UC1);
                            images.push_back(face);
                            labels.push_back(j + 1);
                            cout << ".";
                            face_count++;
                            rectangle(frame, face_rect, Scalar(255, 0, 0), 2);
                        }
                        imshow("R", frame);
                        waitKey(10);
                    }
                }
                frame_count++;
            }
        }
        cout << "训练图片数量： " << images.size() << endl;
        cout << "开始训练：" << endl;
        cout << "Training eigenfaces ..." << endl;
        model1->train(images, labels);
        model1->save("eigenfaces.yml");
        cout << "Training fisherfaces ..." << endl;
        model2->train(images, labels);
        model2->save("fisherfaces.yml");
        cout << "Training lbphfaces ..." << endl;
        model3->train(images, labels);
        model3->save("lbphfaces.yml");
        cout << "模型训练完成。" << endl;
    } else {
        cout << "Loading eigenfaces.yml ..." << endl;
        model1->load("eigenfaces.yml");
        cout << "Loading fisherfaces.yml ..." << endl;
        model2->load("fisherfaces.yml");
        cout << "Loading lbphfaces.yml ..." << endl;
        model3->load("lbphfaces.yml");
        cout << "模型载入完成。" << endl;
    }

    cout << "是否测试开始？（Y/N）" << endl;
    cin >> kInput;
    if (kInput == "Y" || kInput == "y") {
        VideoCapture capture(0);
        if (!capture.isOpened()) {
            cout << "Camera err." << endl;
        }
        MyRecognizer recog(names.size());
        int frame_count = 0;
        int detect_count = 0;
        do {
            capture >> frame;
        } while (frame.empty());
        Size frameSize = frame.size();
        int h_roi = 0.8*frameSize.height;
        Rect roi = Rect(frameSize.width / 2 - h_roi / 2, frameSize.height / 2 - h_roi / 2, h_roi, h_roi);
        Scalar roi_color;
        while (1) {
            capture >> frame;
            if (!frame.empty()) {
                roi_color = Scalar(0, 0, 255);
                cvtColor(frame, frameGray, CV_BGR2GRAY);
                recs.clear();
                cascade.detectMultiScale(frameGray, recs, 1.2, 1, 0, Size(80, 80));
                if (recs.size() > 0) {
                    face_rect = maxRect(recs);
                    face_rect = enlargeRect(face_rect, 0.1, frameSize);
                    if (face_rect.width > 0) {
                        roi_color = Scalar(0, 255, 0);
                        face = frameGray(face_rect).clone();
                        resize(face, face, face_size);
                        normalize(face, face, 0, 255, NORM_MINMAX, CV_8UC1);
                        int predictedLabel = -1;
                        double confidence = 0.0;
                        // EigenFace
                        model1->predict(face, predictedLabel, confidence);
                        //cout << predictedLabel << "  ";
                        recog.setResult(predictedLabel, EIGEN_RESULT);
                        // FisherFace
                        model2->predict(face, predictedLabel, confidence);
                        //cout << predictedLabel << "  ";
                        recog.setResult(predictedLabel, FISHER_RESULT);
                        // LBPHFace
                        model3->predict(face, predictedLabel, confidence);
                        //cout << predictedLabel << endl;
                        recog.setResult(predictedLabel, LBPH_RESULT);
                        recog.updataInfo();
                        if (detect_count % (WINDOW_SIZE / 2) == 0 && detect_count>WINDOW_SIZE) {
                            int final_result = recog.getRecognizeResult();
                            if (final_result == 0) {
                                cout << "未找到匹配结果." << endl;
                            } else {
                                cout << "识别结果：" << names[final_result-1] << endl;
                            }
                        }
                        detect_count++;
                        rectangle(frame, face_rect, Scalar(255, 0, 0), 2);
                        //imshow("Face", face);

                    }
                }
                rectangle(frame, roi, roi_color, 1);
                imshow("R", frame);
                if (waitKey(10) == 'q') {
                    break;
                };
            }
            frame_count++;
        }
        capture.release();
    }
}

Rect maxRect(vector<Rect>& recs) {
    int size = recs.size();
    int max_width = 0;
    Rect max_rect;
    if (size>0) {
        for (size_t i = 0; i < size; i++) {
            if (recs[i].width > max_width) {
                max_width = recs[i].width;
                max_rect = recs[i];
            }
        }
    }
    return max_rect;
}

Rect enlargeRect(Rect src, float scale, Size frameSize) {
    Rect dst;
    dst.x = src.x - scale*src.width;
    dst.y = src.y - scale*src.height;
    dst.width = (1 + 2 * scale)*src.width;
    dst.height = (1 + 2 * scale)*src.height;
    if (dst.x > 0 && dst.y > 0 && dst.x + dst.width < frameSize.width && dst.y + dst.height < frameSize.height) {
        return dst;
    } else {
        return Rect(0, 0, 0, 0);
    }
}
