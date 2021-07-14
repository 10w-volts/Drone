#include <opencv2/opencv.hpp>
#include "detect.h"

using namespace cv;

int main()
{
	Mat img = imread("./img/test2.jpg");
	// bar_code_detect(img);
	qr_code_detect(img);
	system("pause");
	return 0;
}