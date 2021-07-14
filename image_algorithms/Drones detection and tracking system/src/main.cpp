#include <opencv2/opencv.hpp>
#include <iostream>
#include "detect.h"

using namespace cv;
using namespace std;

void main() {
	Mat img, processing_img;
	Point2f mid_point_ratio;
	img = imread("./img/car.jpg");
	struct result Result = detect(img);
	mid_point_ratio = Result.result_point;
	if (mid_point_ratio.x != -9999)
		cout << "x: " << mid_point_ratio.x << " y: " << mid_point_ratio.y << endl;
	system("pause");
}
