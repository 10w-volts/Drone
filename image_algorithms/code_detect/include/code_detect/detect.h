#pragma once
// #include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace cv;

struct result{
    Point2f result_point;
    Mat result_mat;
};

result bar_code_detect(Mat img);
result qr_code_detect(Mat img);