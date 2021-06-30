#include <opencv2/opencv.hpp>
#include <iostream>
#include "detect.h"

using namespace cv;
using namespace std;

void main() {
	Mat img, processing_img;
	Point2f mid_point_ratio;
	img = imread("./img/car.jpg");
	mid_point_ratio = detect(img, processing_img);
	if (mid_point_ratio.x != -9999)
		cout << "x: " << mid_point_ratio.x << " y: " << mid_point_ratio.y << endl;
	system("pause");
}