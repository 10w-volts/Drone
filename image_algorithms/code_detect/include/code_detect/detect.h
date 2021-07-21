#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

struct result{
    Point2f result_point;
    Mat result_mat;
};

result bar_code_detect(Mat img);
result qr_code_detect(Mat img);
