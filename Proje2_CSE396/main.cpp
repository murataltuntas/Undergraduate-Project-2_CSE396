#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#define MINUTE 7			// video baslangic dakikasi (1, 4, 9, 14, 15, 16, 16.9, 17, 18, 19 serit degistirme for Yol01.avi) (10, 17.5 Serit degistirme ;; 21 emniyet seridi AnkaraYolu)
#define MIN_LINE_LENGTH 20  // canny icin min line uzunlugu
#define MAX_LINE_GAP 5      // canny icin max gap size
#define THRESHOLD_VALUE 100 // FOR LINE
#define SPEED_CROP_H 100	// yukarýdan kesilen alan speed icin
#define CROP_H 130			// yukarýdan kesilen alan line icin
#define CROP_W 0			// sagdan ve soldan kesilen alan
#define LANE_CHANGE_H 50	// yukarýdan kesilen alan
#define LANE_CHANGE_W 150	// sagdan ve soldan kesilen alan
#define RESIZE_WIDTH 360    // resize genisligi
#define RESIZE_HEIGHT 240   // resize yuksekligi
typedef struct
{
	vector<float> Operatos;

}Operators;

double findLineLength(const Point pt1, const Point pt2);
bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f& r);
bool intersection(Vec4i line1, Vec4i line2, Point2f& iPoint);
static double drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, double, const Scalar& color);

int main()
{
	VideoCapture capture;
	Mat frame, tempframe, cdst;
	Mat prevgray, gray, flow, cflow;
	capture.open("C:/Users/Murat/Desktop/Proje2/AnkaraYolu.avi");
	capture.set(CV_CAP_PROP_POS_MSEC, MINUTE * 60 * 1000);
	namedWindow("original", 0);
	//namedWindow("producted", 0);
	int allFrameTotalMagnitude = 0;
	int magnitudeArray[30];
	string speedText = "";

	Mat roi;
	Mat roi2;
	for (int i = 0; true; i++)
	{
		capture >> frame;
		//if(i % 3 != 0) continue;
		//resize(frame, frame, Size(frame.cols / 2, frame.rows / 2));
		resize(frame, frame, Size(RESIZE_WIDTH, RESIZE_HEIGHT));
		roi = frame.rowRange(CROP_H, RESIZE_HEIGHT).colRange(CROP_W, frame.cols - CROP_W).clone();
		roi2 = frame.rowRange(SPEED_CROP_H, RESIZE_HEIGHT).colRange(CROP_W, frame.cols - CROP_W).clone();
		//tempframe = roi;// (Rect(0, 0, frame.cols, frame.rows * 3 / 6));

		//cvtColor(tempframe, tempframe, CV_BGR2GRAY); // line icin
		cvtColor(roi, gray, CV_BGR2GRAY);            // hiz icin
		//GaussianBlur(tempframe, tempframe, Size(3, 3), 5, 5);
		threshold(gray, tempframe, THRESHOLD_VALUE, 255, CV_THRESH_BINARY);
//		imshow("threshold", tempframe); // threshold ekrani
		Canny(tempframe, tempframe, 50, 255, 3);
//		imshow("Canny", tempframe);
		
		//cout << "after canny" << endl;

#if 1
		vector<Vec4i> lines, fitLines, intersectLines;
		HoughLinesP(tempframe, lines, 1, CV_PI / 180, 30, MIN_LINE_LENGTH, MAX_LINE_GAP);
		Point sou(LANE_CHANGE_W, LANE_CHANGE_H); // sol ust
		Point soa(LANE_CHANGE_W, frame.rows); // sol alt
		Point sau(frame.cols - LANE_CHANGE_W, LANE_CHANGE_H); // sag ust
		Point saa(frame.cols - LANE_CHANGE_W, frame.rows); // sag alt
		//line(frame, sou, soa, Scalar(0, 255, 255), 2, CV_AA); // sol
		//line(frame, sau, saa, Scalar(0, 255, 255), 2, CV_AA); // sag
		//line(frame, sou, sau, Scalar(0, 255, 255), 2, CV_AA); // ust
		//line(frame, soa, saa, Scalar(0, 255, 255), 2, CV_AA); // alt

		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4i l = lines[i];
			Point pt1, pt2;
			//int length = findLineLength(pt1, pt2);

			pt1 = Point(l[0] + CROP_W, l[1] + CROP_H);
			pt2 = Point(l[2] + CROP_W, l[3] + CROP_H);

			double angle = atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180.0 / CV_PI;
			//cout << "Angle: " << angle << "\n";
			if (angle < -21 && angle > -60) // soldaki seritler (kirmizi)
			{
				//line( cdst,  pt1, pt2, Scalar(0,0,255), 8, CV_AA );
				line(frame, pt1, pt2, Scalar(0, 0, 255), 2, CV_AA);
				fitLines.push_back(lines[i]);
			}

			if ((angle < 150 && angle > 70) || (angle < -70 && angle > -150))
			{
				//line( cdst,  pt1, pt2, Scalar(0,0,255), 8, CV_AA );
				double length = findLineLength(pt1, pt2);
				if (sou.x < pt1.x && sou.y < pt1.y && saa.x > pt1.x && saa.y > pt1.y && length > 2) {

					if (length < 26)
						cout << "Serit Degistiriyor.\n";
					else if (length > 30)
						cout << "DANGER! Emniyet Seridine Girdi.\n";
					cout << "Length of Lane : " << length << endl;
					line(frame, pt1, pt2, Scalar(225, 10, 15), 2, CV_AA);
				}
				fitLines.push_back(lines[i]);
			}

			if (angle > 21 && angle < 60) // sagdaki seritler (yesil)
			{
				//line( cdst,  pt1, pt2, Scalar(0,255,0), 3, CV_AA );
				line(frame, pt1, pt2, Scalar(0, 255, 0), 2, CV_AA);
				fitLines.push_back(lines[i]);
			}
		}


		for (size_t i = 0; i < fitLines.size(); i++)
		{
			for (size_t j = 0; j < fitLines.size(); j++)
			{
				Point2f p;
				if (intersection(fitLines[i], fitLines[j], p))
				{
					intersectLines.push_back(fitLines[i]);
					intersectLines.push_back(fitLines[j]);

					//cout << "intersection point is : " << p << endl;					
					//					circle(frame, p, 3, Scalar(111, 111, 11), 2);
				}
			}
		}

		cout << intersectLines.size() << endl;
#endif

#if 1
		if (prevgray.data)
		{
			calcOpticalFlowFarneback(prevgray, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
			cvtColor(prevgray, cflow, CV_GRAY2BGR);

//			cout << "magnitude" << drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0)) << endl;
//			cout << "index " << i << endl;

			allFrameTotalMagnitude += drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
//			cout << "allframeMagnitude" << allFrameTotalMagnitude << endl;
//			cout << allFrameTotalMagnitude / i << endl;

			if (i == 30){

				if (allFrameTotalMagnitude / i < 20)
					speedText = "Stop";
				else if (allFrameTotalMagnitude / i < 50)
					speedText = "Slow";
				else if (allFrameTotalMagnitude / i < 100)
					speedText = "Medium";
				else if (allFrameTotalMagnitude / i < 200)
					speedText = "Fast";
				else
					speedText = "Very Fast";
				if (i % 30 == 0)
				{
					i = 0;
					allFrameTotalMagnitude = 0;
				}
			}
			putText(frame, "speed: " + speedText, cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 255, 255), 1);

//			imshow("frame", frame);
			//imshow("flow", cflow);

			//waitKey(0); 

		}
		if (waitKey(1) == 27)
			break;
		std::swap(prevgray, gray);
#endif
		
		imshow("original", frame);
//		imshow("roi", roi);
//		imshow("roi2", roi2);
		if (waitKey(30) >= 0) break;

	}

	return 0;
}

/**/
double findLineLength(const Point pt1, const Point pt2)
{
	double result = 0.0;
	double x = 0.0, y = 0.0;

	x = abs(pt1.x - pt2.x);
	y = abs(pt2.y - pt2.y);

	result = x*x + y*y;
	result = sqrt(result);

	return result;
}

bool intersection(Vec4i line1, Vec4i line2, Point2f& iPoint)
{

	Point2f line1p1 = Point2f(line1[0], line1[1]);
	Point2f line1p2 = Point2f(line1[2], line1[3]);

	Point2f line2p1 = Point2f(line2[0], line2[1]);
	Point2f line2p2 = Point2f(line2[2], line2[3]);

	return intersection(line1p1, line2p1, line1p2, line2p2, iPoint);
}

bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r)
{
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

static double drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, double, const Scalar& color)
{
	double totalMagnitude = 0;
	for (int y = 0; y < cflowmap.rows; y += step)
		for (int x = 0; x < cflowmap.cols; x += step)
		{
		const Point2f& fxy = flow.at<Point2f>(y, x);
		line(cflowmap, Point(x, y), Point(cvRound(x + fxy.x), cvRound(y + fxy.y)),
			color);
		totalMagnitude += sqrt(pow((x - cvRound(x + fxy.x)), 2) + pow((y - cvRound(y + fxy.y)), 2));
		circle(cflowmap, Point(x, y), 2, color, -1);
		}
	return totalMagnitude;
}