#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

struct result{
    Point2f result_point;
    Mat result_mat;
};

result detect(Mat img);

