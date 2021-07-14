#include <opencv2/opencv.hpp>
#include "detect.h"

using namespace cv;
using namespace std;

Mat get_yellow(Mat img, int min_thred);
Mat dilate_erode(Mat img, int kernel_size);
Mat erode_dilate(Mat img, int kernel_size);
Rect code_find(Mat gray, Mat binary, int size_threshold, int gradient_threshold);
Mat get_x_diff(Mat img);

/*************************************************
Function:       qr_detect
Author:			Junpeng Chen
Description:    Detect qr code
Input:          Standard image
Output:         Rect of qr code
*************************************************/
void qr_code_detect(Mat img)
{
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	Mat x_diff_img = get_x_diff(gray);
	threshold(x_diff_img, x_diff_img, 50, 255, THRESH_BINARY);
	Mat erode = erode_dilate(dilate_erode(x_diff_img, 10), 10);
	Rect code_rect = code_find(gray, erode, 100, 10);

	if (code_rect.x != -1)
		cout << "the rect is : " << code_rect.x << " " << code_rect.y << " " << code_rect.width << " " << code_rect.height << endl;
	else
		cout << "there is no code in the image" << endl;
}

/*************************************************
Function:       bar_code_detect
Author:			Junpeng Chen
Description:    Detect yellow bar code
Input:          Standard image
Output:         Rect of yellow bar code
*************************************************/
void bar_code_detect(Mat img)
{
	Mat binary = get_yellow(img, 180);
	Mat dilate = dilate_erode(binary, 10);
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	Rect code_rect =  code_find(gray, dilate, 100, 10);

	if (code_rect.x != -1)
		cout << "the rect is : " << code_rect.x << " " << code_rect.y << " " << code_rect.width << " " << code_rect.height << endl;
	else
		cout << "there is no code in the image" << endl;
}

/*************************************************
Function:       get_yellow
Author:			Junpeng Chen
Description:    Get yellow area
Input:          Standard image
Output:         Yellow binary image
*************************************************/
Mat get_yellow(Mat img, int min_thred)
{
	Mat hsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	vector<Mat> hsv_channels;
	Mat h_channel, s_channel;
	split(hsv, hsv_channels);
	h_channel = hsv_channels[2];
	s_channel = hsv_channels[1];

	Mat h_binary, s_binary;
	threshold(h_channel, h_binary, min_thred, 255, THRESH_BINARY);
	threshold(s_channel, s_binary, min_thred, 255, THRESH_BINARY);

	Mat dst;
	bitwise_and(h_binary, s_binary, dst);

	return dst;
}

/*************************************************
Function:       dilate_erode
Author:			Junpeng Chen
Description:    Dilate and erode the image
Input:          Binary image
Output:         Dilate and erode image
*************************************************/
Mat dilate_erode(Mat img, int kernel_size)
{
	Mat dst;
	Mat kernel;
	Size size;
	size.width = kernel_size;
	size.height = kernel_size;
	kernel = getStructuringElement(MORPH_RECT, size);
	dilate(img, dst, kernel);
	erode(dst, dst, kernel);
	return dst;
}

/*************************************************
Function:       erode_dilate
Author:			Junpeng Chen
Description:    Erode and dilate the image
Input:          Binary image
Output:         Erode and dilate image
*************************************************/
Mat erode_dilate(Mat img, int kernel_size)
{
	Mat dst;
	Mat kernel;
	Size size;
	size.width = kernel_size;
	size.height = kernel_size;
	kernel = getStructuringElement(MORPH_RECT, size);
	erode(img, dst, kernel);
	dilate(dst, dst, kernel);
	return dst;
}

/*************************************************
Function:       code_find
Author:			Junpeng Chen
Description:    Find bar code in the image
Input:          Binary image
Output:         The bar code point
*************************************************/
Rect code_find(Mat gray, Mat binary, int size_threshold, int gradient_threshold)
{
	Mat labels, stats, centroids;
	int bar_index = -1;
	int num = connectedComponentsWithStats(binary, labels, stats, centroids);
	Rect code_rect;
	if (num > 0)
	{
		for (int i = 1; i < num; i++)
		{
			if (stats.at<int>(i, CC_STAT_AREA) > size_threshold)
			{
				// obtain a partial image
				Rect rect(stats.at<int>(i, CC_STAT_LEFT), stats.at<int>(i, CC_STAT_TOP), stats.at<int>(i, CC_STAT_WIDTH), stats.at<int>(i, CC_STAT_HEIGHT));
				Mat temp_gray = gray(rect);
				// calculate the gradient
				Mat x_gradient, y_gradient;
				Mat x_kernel = (Mat_<char>(1, 2) << -1, 1);
				Mat y_kernel = (Mat_<char>(2, 1) << -1, 1);
				filter2D(temp_gray, x_gradient, -1, x_kernel);
				filter2D(temp_gray, y_gradient, -1, y_kernel);
				// calculate the sum of gradient
				int x_gradient_sum = sum(x_gradient)[0];
				int y_gradient_sum = sum(y_gradient)[0];

				float average_gradient = (float)(x_gradient_sum + y_gradient_sum) / (float)stats.at<int>(i, CC_STAT_AREA);
				if (average_gradient > gradient_threshold)
				{
					code_rect = rect;
					break;
				}
			}
			if (i == num - 1)
				code_rect = Rect(-1, -1, -1, -1);
		}
	}
	else
		code_rect = Rect(-1, -1, -1, -1);
	return code_rect;
}

/*************************************************
Function:       get_x_diff
Author:			Junpeng Chen
Description:    Find x_diff image
Input:          Gray image
Output:         The x_diff image
*************************************************/
Mat get_x_diff(Mat img)
{
	Mat x_gradient;
	Mat x_kernel = (Mat_<char>(1, 2) << -1, 1);
	filter2D(img, x_gradient, -1, x_kernel);
	return x_gradient;
}