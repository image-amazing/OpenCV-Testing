#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include <iostream>
#include <io.h>

using namespace cv;
using namespace cv::face;
using namespace std;

#define TRAIN_NEW_MODEL false
#define VIDEO_TEST true

void getFiles(string path, vector<string>& files);

int main(int argc, const char *argv[]) {
	CascadeClassifier cascade("lbpcascade_frontalface_improved.xml");	
	Ptr<LBPHFaceRecognizer> model = createLBPHFaceRecognizer();
	Mat frame;
	Mat frameGray;
	vector<Rect> recs;

	if (TRAIN_NEW_MODEL) {
		vector<Mat> images;
		vector<int> labels;

		string imageDir[3];
		imageDir[0] = ".\\Image\\1";
		imageDir[1] = ".\\Image\\2";
		imageDir[2] = ".\\Image\\3";
	
		Mat face;	
		for (int j = 0; j < 3; j++) {
			vector<string> files;
			getFiles(imageDir[j], files);
			for (int i = 0; i < files.size(); i++) {
				frame = imread(files[i], 0);
				recs.clear();
				cascade.detectMultiScale(frame, recs, 1.2, 3, 0, Size(100, 100));
				if (recs.size() > 0) {
					face = frame(recs[0]);
					images.push_back(face);
					labels.push_back(j + 1);
				}
			}
		}
		model->train(images, labels);
		model->save("lbphfaces.yml");
	}
	else {
		model->load("lbphfaces.yml");
	}
	
	if (VIDEO_TEST) {
		VideoCapture cap(0);
		while (1)
		{
			cap >> frame;
			cvtColor(frame, frameGray, CV_BGR2GRAY);
			recs.clear();
			cascade.detectMultiScale(frameGray, recs, 1.2, 3, 0, Size(100, 100));
			if (recs.size() > 0) {
				Mat face = frameGray(recs[0]);
				int predictedLabel = model->predict(face);
				rectangle(frame, recs[0], Scalar(255, 0, 0), 2);				
				cout << "R = " << predictedLabel << endl;
			}
			else {
				cout << "Not found." << endl;
			}
			imshow("Result", frame);
			waitKey(40);
		}
	}
	else {
		string testDir[3];
		testDir[0] = ".//Image//test1.jpg";
		testDir[1] = ".//Image//test2.jpg";
		testDir[2] = ".//Image//test3.jpg";

		for (int i = 0; i < 3; i++) {
			frame = imread(testDir[i], 0);
			recs.clear();
			cascade.detectMultiScale(frame, recs, 1.2, 3, 0, Size(100, 100));
			if (recs.size() > 0) {
				Mat testSample = frame(recs[0]);
				int predictedLabel = model->predict(testSample);
				imshow("Result", testSample);
				cout << "R = " << predictedLabel << endl;
				waitKey();
			}
		}
	}

	return 0;
}


void getFiles(string path, vector<string>& files)
{ 
	long   hFile = 0; 
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}