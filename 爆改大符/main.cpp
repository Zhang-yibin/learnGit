////
////#include "HaiKangCamera.h"
////#include "serial.h"
////#include "1111.h"
////#include "new.cpp"
////#define OPEN_SERIAL
////#include <chrono>
////#include <thread>
////using namespace std;
////using namespace std::chrono;
////SendData send_data;
////SerialPort SP;
////#include<iostream>
////#include <opencv2/opencv.hpp>
////#include <opencv2/core/core.hpp>
////#include <opencv2/highgui/highgui.hpp>
////using namespace std;
////using namespace cv;
////#define LENGTH 4000
//
//int main() {
//    HaiKangCamera HaiKang;
//    HaiKang.StartDevice(0);
//    HaiKang.SetResolution(640,480);
//    HaiKang.SetFPS(400);
//    HaiKang.SetStreamOn();
//    HaiKang.SetExposureTime(22600);
//    HaiKang.SetGAIN(0, 10);
//    HaiKang.GetImageParam(HaiKang.m_param);
//
//#ifdef OPEN_SERIAL
//    while( bool usb = SP.initSerial("/dev/ttyUSB0", 115200, 'N', 8, 1) == false);
//    send_data.clear();
//#endif
//    while (true) {
//        Mat src;
//        HaiKang.GetMat(src);
//        RuneDetector rune_detector = RuneDetector(RuneDetector::RuneParam());
//        //1
//        rune_detector.imageProcess(src);
//        //2
//        rune_detector.fillContour();
//        //3
//        rune_detector.fanSizer(fans);
//        rune_detector.findCenter(src);
////        imshow("1",src);
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//#ifdef OPEN_SERIAL
//        if(!SP.sendData(7,send_data,0xAA,0xAF)){
//            while(!SP.initSerial("/dev/ttyUSB0", 115200, 'N', 8, 1));
//        }
//
//        cout << "start sleeping..." << endl;
//        std::this_thread::sleep_for(milliseconds(221)
//        );
//        cout << "sleeping finished." << endl;
//#endif
//        if (waitKey(1) == 'q')
//            break;
//    }
//    return 0;
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////
////#include "HaiKangCamera.h"
////#include "serial.h"
////#define OPEN_SERIAL
////#include <chrono>
////#include <thread>
////using namespace std;
////using namespace std::chrono;
////SendData send_data;
////SerialPort SP;
////#include<iostream>
////#include <opencv2/opencv.hpp>
////#include <opencv2/core/core.hpp>
////#include <opencv2/highgui/highgui.hpp>
////using namespace std;
////using namespace cv;
////#define LENGTH 4000
////void calculateAngle();
////void swapped();
////void getpic();
////double yaw_angle;
////double pitch_angle;
////Point2f points[4];
////void swip(Point2f a,Point2f b)
////{auto temp=a;
////    a=b;
////    b=temp;}
////vector<vector<Point>> contours;
////Mat src;
////Scalar backgroundColor(28,38,45);
////Scalar lowerBound(175, 129, 22);
////Scalar upperBound(182, 255, 255);
////vector<Vec4i> hierarchy;
////const int kMaxVal = 255;
////Point2f dian[2][4];
////Point2f red_center=Point(318.7,240+19.67);
////Point2f zhondian[4];
////void judgecenter(Point2f n,Point2f cpt);
////Point2f zup=(zhondian[0]+zhondian[1])/2;
////Point2f zright=(zhondian[1]+zhondian[2])/2;
////Point2f zdown=(zhondian[2]+zhondian[3])/2;
////Point2f zleft=(zhondian[3]+zhondian[1])/2;
////int yuzhi=180;
////void swapped(Point2f cpt);
////Mat newBackground;
////int budin=3;
////void swappp(Point2f a[4]);
////const Size kGaussianBlueSize = Size(5, 5);
////void paidian()
////{
////    for(int q=0;q<4;q++) {
////        zhondian[q] = ((dian[0][q] + dian[1][q]) / 2);//xierudian}
////        Point2f zup=(zhondian[0]+zhondian[1])/2;
////        Point2f zright=(zhondian[1]+zhondian[2])/2;
////        Point2f zdown=(zhondian[2]+zhondian[3])/2;
////        Point2f zleft=(zhondian[3]+zhondian[1])/2;
////    }
////}
////Mat changeBackground(const Mat& inputImage, const Scalar& backgroundColor, const Scalar& lowerBound, const Scalar& upperBound)
////{
////    Mat hsv;
////    cvtColor(inputImage, hsv, COLOR_BGR2HSV);
////    Mat mask;
////    inRange(hsv, lowerBound, upperBound, mask);
////    bitwise_not(mask, mask);
////    Mat ele= getStructuringElement(0,Size(7,7));
////    morphologyEx(mask,mask,MORPH_OPEN,ele);
////    imshow("m",mask);
////    newBackground = Mat::zeros(inputImage.size(), inputImage.type());
////    newBackground = backgroundColor;
////    inputImage.copyTo(newBackground, mask);
////    return newBackground;
////}
////int main() {
////    HaiKangCamera HaiKang;
////    HaiKang.StartDevice(0);
////    HaiKang.SetResolution(640,480);
////    HaiKang.SetFPS(400);
////    HaiKang.SetStreamOn();
////    HaiKang.SetExposureTime(21800);
////    HaiKang.SetGAIN(0, 10);
////    HaiKang.GetImageParam(HaiKang.m_param);
////#ifdef OPEN_SERIAL
////    while( bool usb = SP.initSerial("/dev/ttyUSB0", 115200, 'N', 8, 1) == false);
////    send_data.clear();
////#endif
////    while (true) {
//////      hk_camera.readImage(src);
////        HaiKang.GetMat(src);
////        Mat gray=src.clone();
////        Mat frame,channels[3],binary,Gaussian;
////        vector<Point2f> boxPts(4);
////        changeBackground(src,backgroundColor,lowerBound,upperBound);
////        frame=newBackground.clone();
////        split(frame,channels);
////        threshold(channels[1],binary,150,255,0);
////        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
////        imshow("233", Gaussian);
////        int k=0;
////        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
////        for (int n = 0; n < contours.size(); n++) {
////            if (cv::contourArea(contours[n]) < 19700) {//||cv::contourArea(contours[n]) > 17999){
////                continue;
////            }
////            if(arcLength(contours[n],true)>900)
////            { continue;}
////            RotatedRect rrect;
////            rrect = minAreaRect(contours[n]);
////            if (std::max(rrect.size.width, rrect.size.height) / std::min(rrect.size.width, rrect.size.height) >
////                1.29)       // 长宽比约束 1.42
////            {
////                continue;
////            }
////            rrect.points(points);
////            Point2f cpt = rrect.center;
////            circle(frame, cpt, 2, Scalar(0, 255, 0), 2, 8, 0);
////            circle(frame, red_center, 2, Scalar(0, 122, 222), 2, 8, 0);
////            swapped(cpt);
//////            judgecenter(red_center,cpt);
////            for (int i = 0; i < 4; i++) {
////                line(frame, points[i], points[(i + 1) % 4], Scalar(0, 155, 255), 2);
//////                 cout << vertices[i] << endl<<endl;
//////                dian[k][i]=vertices[i];
//////                cout<<points[i]<<endl;
////                dian[k][i]=points[i];
////            }
////            k++;
////        }
////        paidian();
////        for(int i=0;i<4;i++) {
////            line(frame, zhondian[i], zhondian[(i + 1) % 4], Scalar(0, 155, 255), 2);
////        }
////        calculateAngle();
////        imshow("max", frame);
//////        sleep(500);
////#ifdef OPEN_SERIAL
////        if(!SP.sendData(7,send_data,0xAA,0xAF)){
////            while(!SP.initSerial("/dev/ttyUSB0", 115200, 'N', 8, 1));
////        }
////        cout << "start sleeping..." << endl;
////        std::this_thread::sleep_for(seconds(1));
////        cout << "sleeping finished." << endl;
////#endif
////        if (waitKey(1) == 'q')
////            break;
////    }
////    return 0;
////}
////void swapped(Point2f cpt)
////{
////    double width=cpt.x;
////    double height =cpt.y;
////    int TL,TR,BR,BL;
////    for(int i=0;i<4;i++) {
////        if ((points[i].x< width)&&(points[i].y < height))
////        { TL = i;
////            continue; }
////        if ((points[i].x >width)&&(points[i].y<height))
////        {TR=i;
////            continue;}
////        if ((points[i].x >width)&&(points[i].y>height))
////        {BR=i;
////            continue;}
////        if ((points[i].x <width)&&(points[i].y>height))
////        {BL=i;
////            continue;}
////    }
////    Point2f t[4];
////    t[0]=points[TL];
////    t[1]=points[TR];
////    t[2]=points[BR];
////    t[3]=points[BL];
////    for(int i=0;i<4;i++) {
////        points[i] = t[i];
////    }
////}
////double angle[2];
////int count1=0;
////int value=10;
////void calculateAngle() {
////    double dist1;
////    double dist2;
////    double len_y1;
////    double len_y2;
////    double angle_x1;
////    double angle_x2;
////    double angle_y1;
////    double angle_y2;
////    if (count1 == 0) {
////        dist1 = 4.8 * (red_center.x - src.cols / 2);
////        dist2 = 4.8 * (zhondian[0].x - src.cols / 2);
////        angle_x1 = atan(dist1 / LENGTH) * 180.0 / CV_PI;
////        angle_x2 = atan(dist2 / LENGTH) * 180.0 / CV_PI;
////        angle[0] = angle_x1 - angle_x2;   //  >0  zuo  <0  you
////        len_y1 = 4.8 * (red_center.y - src.rows / 2);
////        len_y2 = 4.8 * (zhondian[0].y - src.rows / 2);
////        angle_y1 = atan(len_y1 / LENGTH) * 180.0 / CV_PI;
////        angle_y2 = atan(len_y2 / LENGTH) * 180.0 / CV_PI;
////        angle[1] = angle_y1 - angle_y2;  // >0  shang  <0  xia
////    }
////    if (count1 == 1) {
////        dist1 = 4.8 * (red_center.x - src.cols / 2);
////        dist2 = 4.8 * (zhondian[1].x - src.cols / 2);
////        angle_x1 = atan(dist1 / LENGTH) * 180.0 / CV_PI;
////        angle_x2 = atan(dist2 / LENGTH) * 180.0 / CV_PI;
////        angle[0] = angle_x2 - angle_x1;   //  >0  zuo  <0  you
//////        angle[0] = angle_x1 - angle_x2;   //  >0  zuo  <0  you
////        len_y1 = 4.8 * (red_center.y - src.rows / 2);
////        len_y2 = 4.8 * (zhondian[1].y - src.rows / 2);
////        angle_y1 = atan(len_y1 / LENGTH) * 180.0 / CV_PI;
////        angle_y2 = atan(len_y2 / LENGTH) * 180.0 / CV_PI;
////        angle[1] = angle_y1 - angle_y2;  // >0  shang  <0  xia
////    }
////    if (count1 == 2) {
////        dist1 = 4.8 * (red_center.x - src.cols / 2);
////        dist2 = 4.8 * (zhondian[2].x - src.cols / 2);
////        angle_x1 = atan(dist1 / LENGTH) * 180.0 / CV_PI;
////        angle_x2 = atan(dist2 / LENGTH) * 180.0 / CV_PI;
//////        angle[0] = angle_x1 - angle_x2;   //  >0  zuo  <0  you
////        angle[0] = angle_x2 - angle_x1;   //  >0  zuo  <0  you
////        len_y1 = 4.8 * (red_center.y - src.rows / 2);
////        len_y2 = 4.8 * (zhondian[2].y - src.rows / 2);
////        angle_y1 = atan(len_y1 / LENGTH) * 180.0 / CV_PI;
////        angle_y2 = atan(len_y2 / LENGTH) * 180.0 / CV_PI;
////        angle[1] = angle_y1 - angle_y2;  // >0  shang  <0  xia
////    }
////    if (count1 == 3) {
////        dist1 = 4.8 * (red_center.x - src.cols / 2);
////        dist2 = 4.8 * (zhondian[3].x - src.cols / 2);
////        angle_x1 = atan(dist1 / LENGTH) * 180.0 / CV_PI;
////        angle_x2 = atan(dist2 / LENGTH) * 180.0 / CV_PI;
//////        angle[0] = angle_x1 - angle_x2;   //  >0  zuo  <0  you
////        angle[0] = angle_x2 - angle_x1;   //  >0  zuo  <0  you
////        len_y1 = 4.8 * (red_center.y - src.rows / 2);
////        len_y2 = 4.8 * (zhondian[3].y - src.rows / 2);
////        angle_y1 = atan(len_y1 / LENGTH) * 180.0 / CV_PI;
////        angle_y2 = atan(len_y2 / LENGTH) * 180.0 / CV_PI;
////        angle[1] = angle_y1 - angle_y2;  // >0  shang  <0  xia
////    }
////    if (count1 == 4) {
////        dist1 = 4.8 * (red_center.x - src.cols / 2);
////        dist2 = 4.8 * (zhondian[0].x - src.cols / 2);
////        angle_x1 = atan(dist1 / LENGTH) * 180.0 / CV_PI;
////        angle_x2 = atan(dist2 / LENGTH) * 180.0 / CV_PI;
//////        angle[0] = angle_x1 - angle_x2;   //  >0  zuo  <0  you
////        angle[0] = angle_x2 - angle_x1;   //  >0  zuo  <0  you
////        len_y1 = 4.8 * (red_center.y - src.rows / 2);
////        len_y2 = 4.8 * (zhondian[0].y - src.rows / 2);
////        angle_y1 = atan(len_y1 / LENGTH) * 180.0 / CV_PI;
////        angle_y2 = atan(len_y2 / LENGTH) * 180.0 / CV_PI;
////        angle[1] = angle_y1 - angle_y2;  // >0  shang  <0  xia
////    }
//////    angle[0]=-angle[0];
////    send_data.pitch_angle=angle[1];//+90;
////    send_data.yaw_angle=angle[0];//+180;
////
//////    for (int i = 0; i < 3; ++i) {
//////        send_data.pitch_angle=i;//+90;
//////        send_data.yaw_angle=i;//+180;
//////
//////    }
////
////    cout<<angle[1]<<endl;  // >0  shang  <0  xia
////    cout<<angle[0]<<endl;  // >0  zuo  <0  you
////
////    if (abs(red_center.x - zhondian[0].x) < value && abs(red_center.y - zhondian[0].y) < value) {
////        count1 = 1;
////    } else if (abs(red_center.x - zhondian[1].x) < value && abs(red_center.y - zhondian[1].y) < value) {
////        count1 = 2;
////    } else if (abs(red_center.x - zhondian[2].x) < value && abs(red_center.y - zhondian[2].y) < value) {
////        count1 = 3;
////    } else if (abs(red_center.x - zhondian[3].x) < value && abs(red_center.y - zhondian[3].y) < value) {
////        count1 = 4;
////    }
////}