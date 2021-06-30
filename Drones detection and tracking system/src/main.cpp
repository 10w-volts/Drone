#include <opencv2/opencv.hpp>
#include <iostream>
#include "detect.h"

using namespace cv;
using namespace std;

void main() {
	Mat img;
	Point2f mid_point_ratio;
	img = imread("./img/car.jpg");
	mid_point_ratio = detect(img);
	if (mid_point_ratio.x != -2)
		cout << "x: " << mid_point_ratio.x * 4 << " y: " << mid_point_ratio.y * 3 << endl;
	system("pause");
}