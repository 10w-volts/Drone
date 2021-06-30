#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat background_filter(Mat img, int low_threshold, int high_threshold);
Mat erode_dilate(Mat img, int kernel_size);
Point2i get_mid_point(Mat img, int size_threshold);
Point2i get_mid_point2(Mat img, int size_threshold);
Point2f get_ratio(Mat img, Point2i mid_point);

/*************************************************
Function:       detect
Author:			Junpeng Chen
Description:    If there is car in the image ,return mid point ratio, else return (-9999, -9999)
Input:          Standard image
Output:         Mid point ratio, x in (-1, 1)��y in (-1, 1)
*************************************************/
Point2f detect(Mat img) {

	Mat Lab, lab_a_channel, lab_a_binary, lab_a_dst;
	cvtColor(img, Lab, cv::COLOR_RGB2Lab);
	vector<Mat> Lab_channels;
	split(Lab, Lab_channels);
	lab_a_channel = Lab_channels[1];
	lab_a_binary = background_filter(lab_a_channel, 120, 255);
	lab_a_dst = erode_dilate(lab_a_binary, 5);

	Mat hsv, hsv_s_channel, hsv_binary, hsv_dst;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	vector<Mat> hsv_channels;
	split(hsv, hsv_channels);
	hsv_s_channel = hsv_channels[1];
	hsv_binary = background_filter(hsv_s_channel, 120, 255);
	hsv_dst = erode_dilate(hsv_binary, 5);

	Mat dst = hsv_dst.mul(lab_a_dst);
	Point2i mid_point;
	//mid_point = get_mid_point(dst, 100);
	mid_point = get_mid_point2(dst, 100);
	Point2f mid_point_ratio;
	if (mid_point.x != -1)
		mid_point_ratio = get_ratio(img, mid_point);
	else
		mid_point_ratio = Point2f(-9999, -9999);
	return mid_point_ratio;
}

/*************************************************
Function:       background_filter
Author:			Junpeng Chen
Description:    Filter out the background
Input:          Standard image and lowest and highest threshold
Output:         Result image
*************************************************/
Mat background_filter(Mat img, int low_threshold, int high_threshold) {
	const int max_val = 255;
	Mat dst1, dst2, dst;
	threshold(img, dst1, low_threshold, max_val, THRESH_BINARY);
	threshold(img, dst2, high_threshold, max_val, THRESH_BINARY_INV);
	bitwise_and(dst1, dst2, dst);
	return dst;
}

/*************************************************
Function:       erode_dilate
Author:			Junpeng Chen
Description:    Erode and dilate the image
Input:          Standard image and kernel size
Output:         Result image
*************************************************/
Mat erode_dilate(Mat img, int kernel_size) {
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
Function:       get_mid_point
Author:			Junpeng Chen
Description:    Find the mid point
Input:          Standard image and the threshold of contour size
Output:         Mid point
*************************************************/
/*
Point2i get_mid_point(Mat img, int size_threshold) {
	vector<vector<Point>> contours;
	Mat hierarchy;
	findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	int contour_num = contours.size();
	int max_index = 0;
	int max_size = 0;
	Point2i mid_point;
	if (contour_num > 0) {
		for (int i = 0; i < contour_num; i++) {
			if (contours[i].size() > max_size) {
				max_size = contours[i].size();
				max_index = i;
			}
		}
		if (max_size > size_threshold) {
			Moments moment;
			moment = moments(contours[max_index], false);
			mid_point = Point2i(moment.m10 / moment.m00, moment.m01 / moment.m00);
		}
		else
			mid_point = Point2i(-1, -1);
	}
	else
		mid_point = Point2i(-1, -1);
	return mid_point;
}
*/

/*************************************************
Function:       get_mid_point2
Author:			Junpeng Chen
Description:    Find the mid point
Input:          Standard image and the threshold of contour size
Output:         Mid point
*************************************************/
Point2i get_mid_point2(Mat img, int size_threshold) {
	Mat labels, stats, centroids;
	int max_index = 0;
	int max_size = 0;
	int num = connectedComponentsWithStats(img, labels, stats, centroids);
	Point2i mid_point;
	if (num > 0)
	{
		for (int i = 1; i < num; i++)
		{
			if (stats.at<int>(i, CC_STAT_AREA) > max_size)
			{
				max_index = i;
				max_size = stats.at<int>(i, CC_STAT_AREA);
			}
		}
		if (max_size > size_threshold)
			mid_point = Point2i(centroids.at<Point2d>(max_index, 0).x, centroids.at<Point2d>(max_index, 0).y);
		else
			mid_point = Point2i(-1, -1);
	}
	else
		mid_point = Point2i(-1, -1);
	return mid_point;
}

/*************************************************
Function:       get_ratio
Author:			Junpeng Chen
Description:    Get the mid point ratio, x in (-1, 1)��y in (-1, 1)
Input:          Standard image and the mid point
Output:         Mid point ratio, x in (-1, 1)��y in (-1, 1)
*************************************************/
Point2f get_ratio(Mat img, Point2i mid_point) {
	int col, row;
	col = img.cols;
	row = img.rows;
	Point2f mid_point_ratio;
	mid_point_ratio = Point2f((float)mid_point.x / col * 2 - 1, -((float)mid_point.y / row * 2 - 1));
	return mid_point_ratio;
}