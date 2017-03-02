// OmniViewJoint.cpp : Defines the entry point for the console application.
//

#include <fstream>
#include "OmniViewJoint.h"

inline void GetPointInLine(Vec4f line, int y, Point& point) {
    if (line[0] * line[1] == 0) {
        point.x = 0;
        point.y = 0;
    }

    point.x = (int)((y - line[3]) / line[1] * line[0] + line[2]);
    point.y = y;
}

void PlotSingleLine(Mat& frame, Vec4f& plotLine, double dL) {
    Point p1, p2;

    GetPointInLine(plotLine, frame.rows * 0.5, p1);
    GetPointInLine(plotLine, frame.rows, p2);
    line(frame, p1, p2, Scalar(255, 255, 0), 10);
}

void getMask(Mat& mask, vector<Point> pointVec, int flag) {
    Mat temp_mask(480, 320, CV_8U, Scalar(0, 0, 0));;

    if (flag == 0) { // 四边形
        Point root_points[1][4];
        root_points[0][0] = pointVec[0];
        root_points[0][1] = pointVec[1];
        root_points[0][2] = pointVec[2];
        root_points[0][3] = pointVec[3];
        const Point* ppt[1] = { root_points[0] };
        int npt[] = { 4 };
        polylines(temp_mask, ppt, npt, 1, 1, Scalar(255), 1, 8, 0);
        fillPoly(temp_mask, ppt, npt, 1, Scalar(255, 255, 255));
    } else if (flag == 1) { // 五边形
        Point root_points[1][5];
        root_points[0][0] = pointVec[0];
        root_points[0][1] = pointVec[1];
        root_points[0][2] = pointVec[2];
        root_points[0][3] = pointVec[3];
        root_points[0][4] = pointVec[4];

        const Point* ppt[1] = { root_points[0] };
        int npt[] = { 5 };
        polylines(temp_mask, ppt, npt, 1, 1, Scalar(255), 1, 8, 0);
        fillPoly(temp_mask, ppt, npt, 1, Scalar(255, 255, 255));
    } else if (flag == 2) { // 六边形

        Point root_points[1][6];
        root_points[0][0] = pointVec[0];
        root_points[0][1] = pointVec[1];
        root_points[0][2] = pointVec[2];
        root_points[0][3] = pointVec[3];
        root_points[0][4] = pointVec[4];
        root_points[0][5] = pointVec[5];

        const Point* ppt[1] = { root_points[0] };
        int npt[] = { 6 };
        polylines(temp_mask, ppt, npt, 1, 1, Scalar(255), 1, 8, 0);
        fillPoly(temp_mask, ppt, npt, 1, Scalar(255, 255, 255));
    }


    mask = temp_mask;
}

void SaveMapXY(Mat mapx, string name) {
    //写mapx,mapy为xml文件
    Mat map_x;
    string mapx_xml = name;
    string mapx_xml_name = ".\\" + mapx_xml + ".xml";

    FileStorage save_mpx(mapx_xml_name.c_str(), FileStorage::WRITE);

    save_mpx << mapx_xml.c_str() << mapx;

    save_mpx.release();
}


//加载mapx.xml到Mat
void ReadMapXY(Mat& mapx, string name) {
    //读mapx,mapy为xml文件
    string mapx_xml = name;

    string mapx_xml_name = ".\\" + mapx_xml + ".xml";

    FileStorage read_mpx(mapx_xml_name.c_str(), FileStorage::READ);

    read_mpx[mapx_xml.c_str()] >> mapx;
}



void onMouse(int event, int x, int y, int flags, void* ustc) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        Point pt = Point(x, y);
        cout << "x = " << pt.x << " , y = " << pt.y << endl;
        circle(imgShow, pt, 1, Scalar(0, 255, 255), 3, 8, 0);
        imshow("Image", imgShow);
        calibPoints.push_back(pt);
    }
}

void getCalibPoints(Mat frame, vector<Point2f>& points) {
    cout << "Front Image:" << endl;
    imgShow = frame.clone();
    calibPoints.clear();
    //resize(frame, frame, Size(1280, 720));
    imshow("Image", frame);
    setMouseCallback("Image", onMouse, 0);
    waitKey();
    cout << "New calibPoints got." << endl << endl;

    if (calibPoints.size() > 3) {
        points.insert(points.end(), calibPoints.begin(), calibPoints.end());
    }
}

void calib(int image_count, Size board_size) {

    Size image_size;                                          //     图像的尺寸
    vector<Point2f> corners;                                  //     缓存每幅图像上检测到的角点
    vector<vector<Point2f>>  corners_Seq;                     //     保存检测到的所有角点
    vector<Mat>  image_Seq;
    cv::Mat image;
    Mat image_gray;
    int successImageNum = 0;                                  //     检测角度成功图片个数
    int count = 0;                                            //     角点个数

    for (int i = 0; i < image_count; i++) {
        cout << "Frame #" << i + 1 << "..." << endl;
        string imageFileName;;
        std::stringstream StrStm;
        StrStm << i + 1;
        StrStm >> imageFileName;
        imageFileName += ".jpg";
        image = imread("..\\data\\calib\\" + imageFileName);
        Mat image_gray;
        cvtColor(image, image_gray, CV_RGB2GRAY);
        vector<Point2f> corners;
        bool patternFound = findChessboardCorners(image_gray, board_size, corners,
                            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (!patternFound) {
            cout << "can not find chessboard corners!\n";
            continue;
        } else {
            //cornerSubPix(image_gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            count = count + corners.size();
            corners_Seq.push_back(corners);
            successImageNum = successImageNum + 1;
            image_Seq.push_back(image);
        }
    }
    /************************************************************************
    摄像机定标
    *************************************************************************/
    vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/

    Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   保存提取的所有角点   *****/
    vector<int>  point_counts;
    /* 初始化定标板上角点的三维坐标 */
    for (int t = 0; t < successImageNum; t++) {
        vector<Point3f> tempPointSet;
        for (int i = 0; i<board_size.height; i++) {
            for (int j = 0; j < board_size.width; j++) {
                /* 假设定标板放在世界坐标系中z=0的平面上 */
                Point3f tempPoint;
                tempPoint.x = i*board_size.width;
                tempPoint.y = j*board_size.height;
                tempPoint.z = 0;
                tempPointSet.push_back(tempPoint);
            }
        }
        object_Points.push_back(tempPointSet);
    }

    for (int i = 0; i < successImageNum; i++) {
        point_counts.push_back(board_size.width * board_size.height);
    }

    /* 开始定标 */
    image_size = image_Seq[0].size();
    cv::Mat intrinsic_matrix;                                          /*****    摄像机内参数矩阵    ****/
    cv::Mat distortion_coeffs;                                         /* 摄像机的4个畸变系数：k1,k2,k3,k4*/
    std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
    std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
    int flags = 0;
    flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= cv::fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
    fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors,
                       translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));

    /************************************************************************
    显示定标结果
    *************************************************************************/
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    cout << "保存矫正图像" << endl;
    for (int i = 0; i != image_count; i++) {
        cout << "Frame #" << i + 1 << "..." << endl;
        Matx33d newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
        newCameraMatrix = intrinsic_matrix;

        fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
        cout << intrinsic_matrix << endl;

        SaveMapXY(intrinsic_matrix, "intrinsic_matrix");
        SaveMapXY(distortion_coeffs, "distortion_coeffs");
        SaveMapXY(R, "R");

    }
    cout << "保存结束" << endl;
}

void InitMatrixData(Mat frame, Mat& trans_Matrix, int flag) { // 前
    vector<Point2f> objPts(4), imgPts(4);
    if (flag == 0) {

        imgPts.clear();
        /*imgPts[0] = Point2f(706, 410);
        imgPts[1] = Point2f(831, 408);
        imgPts[2] = Point2f(708, 546);
        imgPts[3] = Point2f(951, 536);*/

        objPts[0] = Point2f(148.4, 53.6);
        objPts[1] = Point2f(171.6, 53.6);
        objPts[2] = Point2f(148.4, 76.8);
        objPts[3] = Point2f(171.6, 76.8);
        /*Point2f tempPoint = { 153, 58.24f };
        for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.x += 4.64f;
        }
        tempPoint.x -= 4 * 4.64f;
        tempPoint.y += 4.64f;
        }*/
        getCalibPoints(frame, imgPts);

    } else if (flag == 1) {
        imgPts.clear();
        /*imgPts[0] = Point2f(654, 424);
        imgPts[1] = Point2f(783, 426);
        imgPts[2] = Point2f(579, 564);
        imgPts[3] = Point2f(868, 574);*/

        objPts[0] = Point2f(226.4, 148.4);
        objPts[1] = Point2f(226.4, 171.6);
        objPts[2] = Point2f(203.2, 148.4);
        objPts[3] = Point2f(203.2, 171.6);
        /*Point2f tempPoint = { 221.76f, 153.04f };
        for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.y += 4.64f;
        }
        tempPoint.x -= 4.64f;
        tempPoint.y -= 4 * 4.64f;
        }*/
        getCalibPoints(frame, imgPts);

    } else if (flag == 2) {
        imgPts.clear();
        getCalibPoints(frame, imgPts);
        /*imgPts[0] = Point2f(640, 424);
        imgPts[1] = Point2f(766, 426);
        imgPts[2] = Point2f(553, 569);
        imgPts[3] = Point2f(843, 566);*/

        objPts[0] = Point2f(226.4, 300.4);
        objPts[1] = Point2f(226.4, 331.6);
        objPts[2] = Point2f(203.2, 300.4);
        objPts[3] = Point2f(203.2, 331.6);
        /*Point2f tempPoint = { 221.76f, 313.04f };
        for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.y += 4.64f;
        }
        tempPoint.x -= 4.64f;
        tempPoint.y -= 4 * 4.64f;
        }*/

    } else if (flag == 3) {
        imgPts.clear();
        getCalibPoints(frame, imgPts);
        /*imgPts[0] = Point2f(640, 424);
        imgPts[1] = Point2f(789, 429);
        imgPts[2] = Point2f(485, 606);
        imgPts[3] = Point2f(931, 611);*/

        objPts[0] = Point2f(202, 426.4);
        objPts[1] = Point2f(178.8, 426.4);
        objPts[2] = Point2f(202, 403.2);
        objPts[3] = Point2f(178.8, 403.2);
        /*Point2f tempPoint = { 197.36f, 421.76f };
        for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.x -= 4.64f;
        }
        tempPoint.y -= 4.64f;
        tempPoint.x += 4 * 4.64f;
        }*/
    } else if (flag == 4) {
        imgPts.clear();
        getCalibPoints(frame, imgPts);
        /*imgPts[0] = Point2f(640, 424);
        imgPts[1] = Point2f(789, 429);
        imgPts[2] = Point2f(485, 606);
        imgPts[3] = Point2f(931, 611);*/

        objPts[0] = Point2f(141, 426.4);
        objPts[1] = Point2f(117.8, 426.4);
        objPts[2] = Point2f(141, 403.2);
        objPts[3] = Point2f(117.8, 403.2);
        /*Point2f tempPoint = { 136.36f, 421.76f };
        for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.x -= 4.64f;
        }
        tempPoint.y -= 4.64f;
        tempPoint.x += 4 * 4.64f;
        }*/

    } else if (flag == 5) {
        imgPts.clear();

        /*imgPts[0] = Point2f(640, 424);
        imgPts[1] = Point2f(789, 429);
        imgPts[2] = Point2f(485, 606);
        imgPts[3] = Point2f(931, 611);*/

        objPts[0] = Point2f(93.6, 331.6);
        objPts[1] = Point2f(93.6, 300.4);
        objPts[2] = Point2f(116.8, 331.6);
        objPts[3] = Point2f(116.8, 300.4);

        /*Point2f tempPoint = { 88.96f, 326.96f };
        for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.y -= 4.64f;
        }
        tempPoint.y += 4 * 4.64f;
        tempPoint.x += 4.64f;
        }*/

        getCalibPoints(frame, imgPts);
    } else if (flag == 6) {
        imgPts.clear();
        getCalibPoints(frame, imgPts);
        /*imgPts[0] = Point2f(640, 424);
        imgPts[1] = Point2f(789, 429);
        imgPts[2] = Point2f(485, 606);
        imgPts[3] = Point2f(931, 611);*/
        Point2f tempPoint = { 88.96f, 166.96f };
        objPts[0] = Point2f(93.6, 171.6);
        objPts[1] = Point2f(93.6, 140.4);
        objPts[2] = Point2f(116.8, 171.6);
        objPts[3] = Point2f(116.8, 140.4);

        /*	for (int j = 0; j < 4; j++)
        {
        for (int i = 0; i < 4; i++)
        {
        objPts[j * 4 + i] = tempPoint;
        tempPoint.y -= 4.64f;
        }
        tempPoint.y += 4 * 4.64f;
        tempPoint.x += 4.64f;
        }*/
    }
    trans_Matrix = findHomography(imgPts, objPts);
}

double getYValue(double x, Point p1, Point p2) {
    double y;
    y = (p1.y - p2.y)*x / (p1.x - p2.x) + (p1.x*p2.y - p1.y*p2.x) / (p1.x - p2.x);
    return y;
}

int dist2line(Point pt, Point p1, Point p2) {	//判断点在直线的左侧还是右侧
    int A = p1.y - p2.y;
    int B = p2.x - p1.x;
    int C = p1.x*p2.y - p2.x*p1.y;
    int num = A*pt.x + B*pt.y + C;

    if (A*num < 0) {
        return -1;
    } else {
        return 1;
    }
}

void seamRefine(Mat& OFrame, Mat& frame1, Mat& frame2, int rowB, int rowE, int colB, int colE, const Point pM[]) {
    uchar* p;
    for (unsigned j = rowB; j < rowE; j++) {
        p = OFrame.ptr<uchar>(j);

        for (unsigned i = colB * 3; i < colE * 3; i += 3) {
            int index1 = dist2line(Point(i / 3, j), pM[1], pM[4]);
            int index2 = dist2line(Point(i / 3, j), pM[3], pM[5]);

            if (index1 < 0 && index2 > 0) {
                double y1 = getYValue(i / 3, pM[1], pM[4]);
                double y2 = getYValue(i / 3, pM[3], pM[5]);
                double mu1 = abs((j - y2) / (y1 - y2));
                double mu2 = abs((j - y1) / (y1 - y2));

                p[i] = mu1 * frame1.at<uchar>(j, i) + mu2 * frame2.at<uchar>(j, i);
                p[i + 1] = mu1 * frame1.at<uchar>(j, i + 1) + mu2 * frame2.at<uchar>(j, i + 1);
                p[i + 2] = mu1 * frame1.at<uchar>(j, i + 2) + mu2 * frame2.at<uchar>(j, i + 2);
            }
        }
    }
}

void seamRefine1(Mat& OFrame, Mat& frame1, Mat& frame2, int rowB, int rowE, int colB, int colE, int flag) {
    uchar* p;

    if (flag == 0) { // 上下拼接
        for (int j = rowB; j < rowE; j++) {
            p = OFrame.ptr<uchar>(j);

            for (int i = colB * 3; i < colE * 3; i += 3) {
                double mu1 = abs((j - SCREEN_H / 2) *1.0f / ((rowB - rowE) / 2));
                double mu2 = 1 - mu1;

                p[i] = mu1 * frame1.at<uchar>(j, i) + mu2 * frame2.at<uchar>(j, i);
                p[i + 1] = mu1 * frame1.at<uchar>(j, i + 1) + mu2 * frame2.at<uchar>(j, i + 1);
                p[i + 2] = mu1 * frame1.at<uchar>(j, i + 2) + mu2 * frame2.at<uchar>(j, i + 2);
            }
        }
    } else if (flag == 1) { // 左右拼接
        for (int j = rowB; j < rowE; j++) {
            p = OFrame.ptr<uchar>(j);

            for (int i = colB * 3; i < colE * 3; i += 3) {
                double mu1 = abs((i / 3 - SCREEN_W / 2) * 1.0f / ((colB - colE) / 2));
                double mu2 = 1 - mu1;

                p[i] = mu1 * frame1.at<uchar>(j, i) + mu2 * frame2.at<uchar>(j, i);
                p[i + 1] = mu1 * frame1.at<uchar>(j, i + 1) + mu2 * frame2.at<uchar>(j, i + 1);
                p[i + 2] = mu1 * frame1.at<uchar>(j, i + 2) + mu2 * frame2.at<uchar>(j, i + 2);
            }
        }
    }

}


void unDistort(Mat frame, Mat& distortFrame) {
    Mat R = Mat::eye(3, 3, CV_32F);
    Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));             // 摄像机内参数矩阵
    Mat distortion_coeffs = Mat(1, 4, CV_32FC1, Scalar::all(0));            // 摄像机的4个畸变系数：k1,k2,p1,p2
    Size image_size = Size(704, 480);                                       //     图像的尺寸
    mapx = Mat::zeros(image_size, CV_32FC1);
    mapy = Mat::zeros(image_size, CV_32FC1);

    ReadMapXY(intrinsic_matrix, "intrinsic_matrix");
    ReadMapXY(distortion_coeffs, "distortion_coeffs");
    ReadMapXY(R, "R");

    double time = (double)getTickCount();
    Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    newCameraMatrix = intrinsic_matrix.clone();

    if (!newCameraMatrix.empty()) {
        newCameraMatrix.at<double>(0, 2) *= 2.0f;
        newCameraMatrix.at<double>(1, 2) *= 2.0f;
    }

    fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, newCameraMatrix, Size(2 * image_size.width, 2 * image_size.height),
                                     CV_32FC1, mapx, mapy);

    cv::remap(frame, distortFrame, mapx, mapy, INTER_LINEAR);
    //imwrite("111.jpg", distortFrame);
    //waitKey();
}

void proBird() {
    if (NEW_MATRIX) {
        FileStorage fs("transMatrix.xml", FileStorage::WRITE);
        //前
        InitMatrixData(disFront, trans_Matrix, 0);
        warpPerspective(disFront, proFront, trans_Matrix, Size(320, 480));
        fs << "front" << trans_Matrix;
        //右1
        InitMatrixData(disRight1, trans_Matrix, 1);
        warpPerspective(disRight1, proRight1, trans_Matrix, Size(320, 480));
        fs << "right1" << trans_Matrix;
        //右2
        InitMatrixData(disRight2, trans_Matrix, 2);
        warpPerspective(disRight2, proRight2, trans_Matrix, Size(320, 480));
        fs << "right2" << trans_Matrix;
        //后1
        InitMatrixData(disBack1, trans_Matrix, 3);
        warpPerspective(disBack1, proBack1, trans_Matrix, Size(320, 480));
        fs << "back1" << trans_Matrix;
        //后2
        InitMatrixData(disBack2, trans_Matrix, 4);
        warpPerspective(disBack2, proBack2, trans_Matrix, Size(320, 480));
        fs << "back2" << trans_Matrix;
        //左1
        InitMatrixData(disLeft1, trans_Matrix, 5);
        warpPerspective(disLeft1, proLeft1, trans_Matrix, Size(320, 480));
        fs << "left1" << trans_Matrix;
        //左2
        InitMatrixData(disLeft2, trans_Matrix, 6);
        warpPerspective(disLeft2, proLeft2, trans_Matrix, Size(320, 480));
        fs << "left2" << trans_Matrix;
        fs.release();
    } else {
        FileStorage fs("transMatrix.xml", FileStorage::READ);
        fs["front"] >> trans_Matrix;
        warpPerspective(disFront, proFront, trans_Matrix, Size(320, 480));
        fs["right1"] >> trans_Matrix;
        warpPerspective(disRight1, proRight1, trans_Matrix, Size(320, 480));
        fs["right2"] >> trans_Matrix;
        warpPerspective(disRight2, proRight2, trans_Matrix, Size(320, 480));
        fs["back1"] >> trans_Matrix;
        warpPerspective(disBack1, proBack1, trans_Matrix, Size(320, 480));
        fs["back2"] >> trans_Matrix;
        warpPerspective(disBack2, proBack2, trans_Matrix, Size(320, 480));
        fs["left1"] >> trans_Matrix;
        warpPerspective(disLeft1, proLeft1, trans_Matrix, Size(320, 480));
        fs["left2"] >> trans_Matrix;
        warpPerspective(disLeft2, proLeft2, trans_Matrix, Size(320, 480));
        fs.release();
    }
}

void initPoint() {
    //右前
    pMerge[0][0] = Point((SCREEN_W + SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2);
    pMerge[0][3] = Point(SCREEN_W, -8);
    pMerge[0][2] = Point(SCREEN_W, 0);
    pMerge[0][1] = Point(SCREEN_W, 8);
    pMerge[0][4] = Point((SCREEN_W + SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 + 8);
    pMerge[0][5] = Point((SCREEN_W + SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2);

    //右后
    pMerge[1][0] = Point((SCREEN_W + SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2);
    pMerge[1][1] = Point(SCREEN_W, SCREEN_H - 8);
    pMerge[1][2] = Point(SCREEN_W, SCREEN_H);
    pMerge[1][3] = Point(SCREEN_W - 8, SCREEN_H);
    pMerge[1][4] = Point((SCREEN_W + SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 - 8);
    pMerge[1][5] = Point((SCREEN_W + SCAR_W) / 2 - 8, (SCREEN_H + SCAR_H) / 2);

    //左前
    pMerge[2][0] = Point((SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2);
    pMerge[2][3] = Point(0, 8);
    pMerge[2][2] = Point(0, 0);
    pMerge[2][1] = Point(8, 0);
    pMerge[2][5] = Point((SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 + 8);
    pMerge[2][4] = Point((SCREEN_W - SCAR_W) / 2 + 8, (SCREEN_H - SCAR_H) / 2);

    //左后
    pMerge[3][0] = Point((SCREEN_W - SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2);
    pMerge[3][3] = Point(0, SCREEN_H - 10);
    pMerge[3][2] = Point(0, SCREEN_H);
    pMerge[3][1] = Point(0, SCREEN_H + 10);
    pMerge[3][5] = Point((SCREEN_W - SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 - 8);
    pMerge[3][4] = Point((SCREEN_W - SCAR_W) / 2 + 10, (SCREEN_H + SCAR_H) / 2 + 8);

}

void joint() {
    Point p1 = { 0, 0 }, p2 = { SCREEN_W, 0 }, p3 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 },
          p4 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 };
    vector<Point>pt_vector = { p1, p2, p4, p3 };
    getMask(maskFront, pt_vector, 0);
    proFront.copyTo(omniView, maskFront);

    p1 = p4;
    p2 = { SCREEN_W, 0 };
    p3 = { (SCREEN_W + SCAR_W) / 2, SCREEN_H / 2 };
    p4 = { SCREEN_W, SCREEN_H / 2 };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskRight1, pt_vector, 0);
    proRight1.copyTo(omniView, maskRight1);

    p1 = p3;
    p2 = p4;
    p3 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
    p4 = { SCREEN_W, SCREEN_H };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskRight2, pt_vector, 0);
    proRight2.copyTo(omniView, maskRight2);

    p1 = { SCREEN_W / 2, (SCREEN_H + SCAR_H) / 2 };
    p2 = p3;
    p3 = { SCREEN_W / 2, SCREEN_H };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskBack1, pt_vector, 0);
    proBack1.copyTo(omniView, maskBack1);

    p2 = p1;
    p4 = p3;
    p1 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
    p3 = { 0, SCREEN_H };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskBack2, pt_vector, 0);
    proBack2.copyTo(omniView, maskBack2);

    p1 = { 0, SCREEN_H / 2 };
    p2 = { (SCREEN_W - SCAR_W) / 2, SCREEN_H / 2 };
    p3 = { 0, SCREEN_H };
    p4 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskLeft1, pt_vector, 0);
    proLeft1.copyTo(omniView, maskLeft1);

    p1 = { 0, 0 };
    p2 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 };
    p3 = { 0, SCREEN_H / 2 };
    p4 = { (SCREEN_W - SCAR_W) / 2, SCREEN_H / 2 };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskLeft2, pt_vector, 0);
    proLeft2.copyTo(omniView, maskLeft2);

    imshow("omn", omniView);
    waitKey();
}

void extendMask() {
    // 前 缩小
    /*Point p1 = { 10, 0 }, p2 = { SCREEN_W - 10, 0 }, p3 = { (SCREEN_W - SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2 - 10},
    p4 = { (SCREEN_W + SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2 - 10 }, p5, p6;*/
    Point p1 = { 0, -10 }, p2 = { SCREEN_W - 10, 0 }, p3 = { (SCREEN_W - SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2 - 10 },
          p4 = { (SCREEN_W + SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2 - 10 }, p5, p6;
    vector<Point>pt_vector = { p1,p2, p4, p3 };
    getMask(maskCUTFront, pt_vector, 0);

    // 前 扩大
    p1 = { 0, 0 }, p2 = { SCREEN_W, 0 }, p3 = { SCREEN_W, 10 };
    p4 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 + 10 },
    p5 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 + 10 };
    p6 = { 0, 10 };
    pt_vector = { p1, p2, p3, p4, p5, p6 };
    getMask(maskETDFront, pt_vector, 2);

    // 右1 缩小
    p1 = { (SCREEN_W + SCAR_W) / 2 + 10, (SCREEN_H - SCAR_H) / 2 + 10 };
    p2 = { SCREEN_W, 10 };
    p3 = { (SCREEN_W + SCAR_W) / 2 + 10, SCREEN_H / 2 - 10 };
    p4 = { SCREEN_W, SCREEN_H / 2 - 10 };
    pt_vector = { p1, p2, p4, p3 };
    getMask(maskCUTRight1, pt_vector, 0);

    // 右1 扩大
    p1 = { (SCREEN_W + SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2 - 10 };
    p2 = { SCREEN_W - 10, 0 };
    p3 = { SCREEN_W, 0 };
    p4 = { SCREEN_W, SCREEN_H / 2 + 10 };
    p5 = { (SCREEN_W + SCAR_W) / 2 - 10, SCREEN_H / 2 + 10 };
    pt_vector = { p1, p2, p3, p4, p5 };
    getMask(maskETDRight1, pt_vector, 1);

    // 右2 缩小
    p1 = { (SCREEN_W + SCAR_W) / 2 + 10, SCREEN_H / 2 + 10 };
    p2 = { SCREEN_W, SCREEN_H / 2 + 10 };
    p3 = { SCREEN_W, SCREEN_H - 10 };
    p4 = { (SCREEN_W + SCAR_W) / 2 + 10, (SCREEN_H + SCAR_H) / 2 - 10 };
    pt_vector = { p1, p2, p3, p4 };
    getMask(maskCUTRight2, pt_vector, 0);

    // 右2 扩大
    p1 = { (SCREEN_W + SCAR_W) / 2 - 10, SCREEN_H / 2 - 10 };
    p2 = { SCREEN_W, SCREEN_H / 2 - 10 };
    p3 = { SCREEN_W, SCREEN_H };
    p4 = { SCREEN_W - 10, SCREEN_H };
    p5 = { (SCREEN_W + SCAR_W) / 2 - 10, (SCREEN_H + SCAR_H) / 2 + 10 };
    pt_vector = { p1, p2, p3, p4, p5 };
    getMask(maskETDRight2, pt_vector, 1);

    // 后1 缩小
    p1 = { SCREEN_W / 2 + 10, (SCREEN_H + SCAR_H) / 2 + 10 };
    p2 = { (SCREEN_W + SCAR_W) / 2 - 10, (SCREEN_H + SCAR_H) / 2 + 10 };
    p3 = { SCREEN_W - 10, SCREEN_H };
    p4 = { SCREEN_W / 2 + 10, SCREEN_H };
    pt_vector = { p1, p2, p3, p4 };
    getMask(maskCUTBack1, pt_vector, 0);

    // 后1 扩大
    p1 = { SCREEN_W / 2 - 10, (SCREEN_H + SCAR_H) / 2 - 10 };
    p2 = { (SCREEN_W + SCAR_W) / 2 + 10, (SCREEN_H + SCAR_H) / 2 - 10 };
    p3 = { SCREEN_W, SCREEN_H - 10 };
    p4 = { SCREEN_W, SCREEN_H };
    p5 = { SCREEN_W / 2 - 10, SCREEN_H };
    pt_vector = { p1, p2, p3, p4, p5 };
    getMask(maskETDBack1, pt_vector, 1);

    // 后2 缩小
    p1 = { SCREEN_W / 2 - 10, (SCREEN_H + SCAR_H) / 2 + 10 };
    p2 = { SCREEN_W / 2 - 10, SCREEN_H };
    p3 = { 10, SCREEN_H };
    p4 = { (SCREEN_W - SCAR_W) / 2 - 10, (SCREEN_H + SCAR_H) / 2 + 10 };
    pt_vector = { p1, p2, p3, p4 };
    getMask(maskCUTBack2, pt_vector, 0);

    // 后2 扩大
    p1 = { SCREEN_W / 2 + 10, (SCREEN_H + SCAR_H) / 2 - 10 };
    p2 = { SCREEN_W / 2 + 10, SCREEN_H };
    p3 = { 0, SCREEN_H };
    p4 = { 0, SCREEN_H - 10 };
    p5 = { (SCREEN_W - SCAR_W) / 2 - 10, (SCREEN_H + SCAR_H) / 2 - 10 };
    pt_vector = { p1, p2, p3, p4, p5 };
    getMask(maskETDBack2, pt_vector, 1);

    // 左1 缩小
    p1 = { (SCREEN_W - SCAR_W) / 2 - 10, (SCREEN_H + SCAR_H) / 2 - 10 };
    p2 = { 0, SCREEN_H - 10 };
    p3 = { 0, SCREEN_H / 2 + 10 };
    p4 = { (SCREEN_W - SCAR_W) / 2 - 10, SCREEN_H / 2 + 10 };
    pt_vector = { p1, p2, p3, p4 };
    getMask(maskCUTLeft1, pt_vector, 0);

    // 左1 扩大
    p1 = { 0, SCREEN_H / 2 - 10 };
    p2 = { (SCREEN_W - SCAR_W) / 2 + 10, SCREEN_H / 2 - 10 };
    p3 = { (SCREEN_W - SCAR_W) / 2 + 10, (SCREEN_H + SCAR_H) / 2 + 10 };
    p4 = { 10, SCREEN_H };
    p5 = { 0, SCREEN_H };
    pt_vector = { p1, p2, p3, p4, p5 };
    getMask(maskETDLeft1, pt_vector, 1);

    // 左2 缩小
    p1 = { 0, 10 };
    p2 = { (SCREEN_W - SCAR_W) / 2 - 10, (SCREEN_H - SCAR_H) / 2 - 10 };
    p3 = { (SCREEN_W - SCAR_W) / 2 - 10, SCREEN_H / 2 - 10 };
    p4 = { 0, SCREEN_H / 2 - 10 };
    pt_vector = { p1, p2, p3, p4 };
    getMask(maskCUTLeft2, pt_vector, 0);

    // 左2 扩大
    p1 = { 0, 0 };
    p2 = { 10, 0 };
    p3 = { (SCREEN_W - SCAR_W) / 2 + 10, (SCREEN_H - SCAR_H) / 2 - 10 };
    p4 = { (SCREEN_W - SCAR_W) / 2 + 10, SCREEN_H / 2 + 10 };
    p5 = { 0, SCREEN_H / 2 + 10 };
    pt_vector = { p1, p2, p3, p4, p5 };
    getMask(maskETDLeft2, pt_vector, 1);
}


void refine() {
    initPoint();
    seamRefine(omniView, proRight1, proFront, 0, (SCREEN_H - SCAR_H) / 2 + 8, (SCREEN_W + SCAR_W) / 2 - 10, SCREEN_W, pMerge[0]);
    seamRefine(omniView, proRight2, proBack1, (SCREEN_H + SCAR_H) / 2 - 8, SCREEN_H, (SCREEN_W + SCAR_W) / 2 - 10, SCREEN_W, pMerge[1]);
    seamRefine(omniView, proFront, proLeft2, 0, (SCREEN_H - SCAR_H) / 2 + 8, 0, (SCREEN_W - SCAR_W) / 2 + 10, pMerge[2]);
    seamRefine(omniView, proBack2, proLeft1, (SCREEN_H + SCAR_H) / 2 - 8, SCREEN_H, 0, (SCREEN_W - SCAR_W) / 2 + 10, pMerge[3]);
    seamRefine1(omniView, proRight1, proRight2, SCREEN_H / 2 - 10, SCREEN_H / 2 + 10, (SCREEN_W + SCAR_W) / 2, SCREEN_W, 0);
    seamRefine1(omniView, proLeft1, proLeft2, SCREEN_H / 2 - 10, SCREEN_H / 2 + 10, 0, (SCREEN_W - SCAR_W) / 2, 0);
    seamRefine1(omniView, proBack1, proBack2, (SCREEN_H + SCAR_H) / 2, SCREEN_H, SCREEN_W / 2 - 8, SCREEN_W / 2 + 8, 1);
    Rect roi = { 110, 70, 100, 340 };
    imgCar.copyTo(omniView(roi));
}

void undistort() {
    unDistort(imgFront, disFront);
    unDistort(imgRight1, disRight1);
    unDistort(imgRight2, disRight2);
    unDistort(imgBack1, disBack1);
    unDistort(imgLeft1, disLeft1);
    unDistort(imgLeft2, disLeft2);
    unDistort(imgBack2, disBack2);
}

// 利用table生成图像
void getImage(Mat& frameFV, Mat& frameBV, Mat& table) {
    int* pTable;
    for (unsigned i = 1; i < table.rows; i++) {
        pTable = table.ptr<int>(i);
        int xBV = pTable[0];
        int yBV = pTable[1];
        int xFV = pTable[2];
        int yFV = pTable[3];
        uchar rValue = frameFV.at<uchar>(yFV, 3 * xFV);
        uchar gValue = frameFV.at<uchar>(yFV, 3 * xFV + 1);
        uchar bValue = frameFV.at<uchar>(yFV, 3 * xFV + 2);
        frameBV.at<uchar>(yBV, 3 * xBV) = rValue;
        frameBV.at<uchar>(yBV, 3 * xBV + 1) = gValue;
        frameBV.at<uchar>(yBV, 3 * xBV + 2) = bValue;
    }
}

// tables鱼眼部分
void fisheyeTables(Mat& table) {
    Mat frame = imread("..\\data\\img\\1.BMP");
    Mat frameP(2 * frame.rows, 2 * frame.cols, CV_8UC3);
    float *pX;
    float *pY;
    uchar *pP;
    for (size_t j = 0; j < mapx.rows; j++) {
        pX = mapx.ptr<float>(j);
        pY = mapy.ptr<float>(j);
        pP = frameP.ptr<uchar>(j);
        for (size_t i = 0; i < mapx.cols; i++) {
            int x = pX[i];
            int y = pY[i];
            if (x >= 0 && x < 704 && y >= 0 && y < 480) {
                pP[3 * i] = frame.at<uchar>(y, 3 * x);
                pP[3 * i + 1] = frame.at<uchar>(y, 3 * x + 1);
                pP[3 * i + 2] = frame.at<uchar>(y, 3 * x + 2);
            }
        }
    }
    imshow("RR", frameP);
    waitKey();
}


int calcSeamWeight(int x, int y, int flag) {
    int w;
    int x_line;
    int y_line;
    Point p1, p2, p3, p4;
    switch (flag) {
    case 1: {
        p1 = { 0, 0 };
        p2 = { SCREEN_W, 0 };
        p3 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 },
        p4 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 };
        if (x < SCREEN_W / 2) { // left
            y_line = getYValue(x, p1, p3);

        } else {              // right
            y_line = getYValue(x, p2, p4);
        }
        w = 5 * (y_line - y) + 50;
        break;
    }

    case 2: {
        p1 = { 0, 0 };
        p2 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 };
        p3 = { 0, SCREEN_H / 2 };
        p4 = { (SCREEN_W - SCAR_W) / 2, SCREEN_H / 2 };
        if (y < SCREEN_H / 2 - 30) {    // up
            y_line = getYValue(x, p1, p2);
            w = 5 * (y - y_line) + 50;
        } else {                      // down
            y_line = SCREEN_H / 2;
            w = 5 * (y_line - y) + 50;
        }
        break;
    }

    case 3: {
        p1 = { 0, SCREEN_H / 2 };
        p2 = { (SCREEN_W - SCAR_W) / 2, SCREEN_H / 2 };
        p3 = { 0, SCREEN_H };
        p4 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
        if (y<SCREEN_H / 2 + 30) {
            y_line = SCREEN_H / 2;
            w = 5 * (y - y_line) + 50;
        } else {
            y_line = getYValue(x, p3, p4);
            w = 5 * (y_line - y) + 50;
        }
        break;
    }

    case 4: {
        p1 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H - SCAR_H) / 2 };;
        p2 = { SCREEN_W, 0 };
        p3 = { (SCREEN_W + SCAR_W) / 2, SCREEN_H / 2 };
        p4 = { SCREEN_W, SCREEN_H / 2 };
        if (y < SCREEN_H / 2 - 30) {
            y_line = getYValue(x, p1, p2);
            w = 5 * (y - y_line) + 50;
        } else {
            y_line = SCREEN_H / 2;
            w = 5 * (y_line - y) + 50;
        }
        break;
    }

    case 5: {
        p1 = { (SCREEN_W + SCAR_W) / 2, SCREEN_H / 2 };
        p2 = { SCREEN_W, SCREEN_H / 2 };
        p3 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
        p4 = { SCREEN_W, SCREEN_H };
        if (y<SCREEN_H / 2 + 30) {
            y_line = SCREEN_H / 2;
            w = 5 * (y - y_line) + 50;
        } else {
            y_line = getYValue(x, p3, p4);
            w = 5 * (y_line - y) + 50;
        }
        break;
    }

    case 6: {
        p1 = { (SCREEN_W - SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
        p2 = { SCREEN_W / 2, (SCREEN_H + SCAR_H) / 2 };
        p3 = { 0, SCREEN_H };
        p4 = { SCREEN_W / 2, SCREEN_H };
        if (x < SCREEN_W / 2 - 30) {
            y_line = getYValue(x, p1, p3);
            w = 5 * (y - y_line) + 50;
        } else {
            x_line = SCREEN_W / 2;
            w = 5 * (x_line - x) + 50;
        }
        break;
    }

    case 7: {
        p1 = { SCREEN_W / 2, (SCREEN_H + SCAR_H) / 2 };
        p2 = { (SCREEN_W + SCAR_W) / 2, (SCREEN_H + SCAR_H) / 2 };
        p3 = { SCREEN_W / 2, SCREEN_H };
        p4 = { SCREEN_W, SCREEN_H };
        if (x < SCREEN_W / 2 + 30) {
            x_line = SCREEN_W / 2;
            w = 5 * (x - x_line) + 50;
        } else {
            y_line = getYValue(x, p2, p4);
            w = 5 * (y - y_line) + 50;
        }
        break;
    }

    default:
        break;
    }
    return w;
}

// 单个table
void calcTable(Mat& transM, Mat& maskB, Mat& maskS, int flag) {
    cout << "Flag = " << flag << endl;
    // table文件名生成
    char t[10];
    sprintf(t, "%d", flag);
    string table_name = t;
    table_name += ".txt";
    ofstream fout(table_name);
    Mat transM_inv = transM.inv();

    // 需要填充的点的列表
    uchar *pMaskB;
    //uchar *pMaskS;
    vector<Point2f> pBV;
    vector<Point2f> pPV;
    for (size_t j = 0; j < maskB.rows; j++) {
        pMaskB = maskB.ptr<uchar>(j);
        //pMaskS = maskS.ptr<uchar>(j);
        for (size_t i = 0; i < maskB.cols; i++) {
            if (pMaskB[i]>0) {
                pBV.push_back(Point2f(i, j));
            }
        }
    }
    perspectiveTransform(pBV, pPV, transM_inv);

    // 寻找主体对应点
    for (size_t i = 0; i < pPV.size(); i++) {
        if (pPV[i].x >= 0 && pPV[i].x <= 1407 && pPV[i].y >= 0 && pPV[i].y <= 959) {
            int xx = pBV[i].x;
            int yy = pBV[i].y;
            int x = mapx.at<float>(pPV[i].y, pPV[i].x);
            int y = mapy.at<float>(pPV[i].y, pPV[i].x);
            if (x >= 0 && x <= 703 && y >= 0 && y <= 479) {
                int weight = calcSeamWeight(xx, yy, flag);
                if (weight < 0) {
                    weight = 0;
                } else if (weight > 100) {
                    weight = 100;
                }
                fout << xx << " " << yy << " " << x << " " << y << " " << weight << endl;
            } /*else {
              cout << "err: x = " << x << ", y = " << y << endl;
              }*/
        }
    }

    fout.close();
    cout << endl;
}


// 生成查找表
void generateTables() {
    extendMask();
    FileStorage fs("transMatrix.xml", FileStorage::READ);
    Mat maskB;
    Mat maskS;
    for (size_t i = 0; i < 7; i++) {
        switch (i + 1) {
        case 1: {
            fs["front"] >> trans_Matrix;
            maskB = maskETDFront;
            maskS = maskCUTFront;
            break;
        }
        case 2: {
            fs["left2"] >> trans_Matrix;
            maskB = maskETDLeft2;
            maskS = maskCUTLeft2;
            break;
        }
        case 3: {
            fs["left1"] >> trans_Matrix;
            maskB = maskETDLeft1;
            maskS = maskCUTLeft1;
            break;
        }
        case 4: {
            fs["right1"] >> trans_Matrix;
            maskB = maskETDRight1;
            maskS = maskCUTRight1;
            break;
        }
        case 5: {
            fs["right2"] >> trans_Matrix;
            maskB = maskETDRight2;
            maskS = maskCUTRight2;
            break;
        }
        case 6: {
            fs["back2"] >> trans_Matrix;
            maskB = maskETDBack2;
            maskS = maskCUTBack2;
            break;
        }
        case 7: {
            fs["back1"] >> trans_Matrix;
            maskB = maskETDBack1;
            maskS = maskCUTBack1;
            break;
        }
        default:
            break;
        }
        calcTable(trans_Matrix, maskB, maskS, i + 1);
    }
    fs.release();
    cout << "generate table finished" << endl;
}

void getFrameFromTable(Mat& frame, Mat& oframe, int flag) {
    char t[10];
    sprintf(t, "%d", flag);
    string table_name = t;
    table_name += ".txt";
    ifstream fin(table_name);
    while (true) {
        if (fin.eof()) {
            break;
        }
        int p[5];
        for (size_t i = 0; i < 5; i++) {
            fin >> p[i];
        }
        if (p[4] == 100) {
            oframe.at<uchar>(p[1], 3 * p[0]) = frame.at<uchar>(p[3], 3 * p[2]);
            oframe.at<uchar>(p[1], 3 * p[0] + 1) = frame.at<uchar>(p[3], 3 * p[2] + 1);
            oframe.at<uchar>(p[1], 3 * p[0] + 2) = frame.at<uchar>(p[3], 3 * p[2] + 2);
        } else if (p[4] > 0) {
            float w = float(p[4]) / 100;
            oframe.at<uchar>(p[1], 3 * p[0]) += w*frame.at<uchar>(p[3], 3 * p[2]);
            oframe.at<uchar>(p[1], 3 * p[0] + 1) += w*frame.at<uchar>(p[3], 3 * p[2] + 1);
            oframe.at<uchar>(p[1], 3 * p[0] + 2) += w*frame.at<uchar>(p[3], 3 * p[2] + 2);
        }
    }
}

// 验证查找表
void checkTables() {
    Mat origin_frame;
    Mat out_frame = Mat::zeros(480, 320, CV_8UC3);
    for (size_t i = 0; i < 7; i++) {
        switch (i + 1) {
        case 1: {
            origin_frame = imgFront;
            break;
        }
        case 2: {
            origin_frame = imgLeft2;
            break;
        }
        case 3: {
            origin_frame = imgLeft1;
            break;
        }
        case 4: {
            origin_frame = imgRight1;
            break;
        }
        case 5: {
            origin_frame = imgRight2;
            break;
        }
        case 6: {
            origin_frame = imgBack2;
            break;
        }
        case 7: {
            origin_frame = imgBack1;
            break;
        }
        default:
            break;
        }
        getFrameFromTable(origin_frame, out_frame, i + 1);
    }
    imshow("Check", out_frame);
    waitKey();
}

int main() {
    //calib(6, Size(4, 4));

    //读图
    imgCar = imread("..\\img\\car.jpg");
    resize(imgCar, imgCar, Size(100, 340));
    imgFront = imread("..\\img\\1.bmp");
    imgRight1 = imread("..\\img\\2.bmp");
    imgRight2 = imread("..\\img\\3.bmp");
    imgBack1 = imread("..\\img\\4.bmp");
    imgBack2 = imread("..\\img\\5.bmp");
    imgLeft1 = imread("..\\img\\6.bmp");
    imgLeft2 = imread("..\\\img\\7.bmp");

    // 鱼眼矫正
    undistort();

    // 投影
    proBird();

    // 拼合图像
    joint();

    // 优化接缝
    refine();

    // 生成查找表
    generateTables();

    // 验证查找表
    checkTables();

    //imshow("3", omniView);
    waitKey();

    return 0;
}



