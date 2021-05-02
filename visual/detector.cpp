#include "detector.h"

using namespace cv;

void Detector::cameraCaptureCb(const sensor_msgs::ImageConstPtr &data)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(img_);
        ROS_INFO("Transform SUCCESS!"); //打印消息
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    return;
}

void Detector::detectLightCb(const std_msgs::String::ConstPtr &msg)
{
    switch (msg.data)
    {
    case 'a':
        current_location_ = 1;
        break;
    case 'b':
        current_location_ = 2;
        break;
    case 'c':
        current_location_ = 3;
        break;
    default:
        current_location_ = 0;
        break;
    }

    if (current_location_ != 0 && img_.data != NULL)
    {
        circle_center_.data = detectCircle(img_);
        cir_center_pub_.publish(circle_center_);
        current_location_ = 0;
    }
    else
        current_location_ = 0;
}

std::vector<double> Detector::detectCircle(Mat img)
{
    std::vector<double> data;
    data.push_back(0);
    data.push_back(0);
    int ERODE_KERNEL_SIZE = 33;
    int MIN_CIRCLE_DISTANCE = int(pow((pow(img.rows, 2) + pow(img.cols, 2)), 0.5));
    int MIN_RADIUS = 150;
    int MAX_RADIUS = 600;

    Mat img_gray, img_thresh, img_erode, img_canny;

    cvtColor(img, img_gray, CV_BGR2GRAY);
    threshold(img_gray, img_thresh, 1000, 255, THRESH_BINARY);
    Mat kernel_erode = getStructuringElement(MORPH_RECT, Size(ERODE_KERNEL_SIZE, ERODE_KERNEL_SIZE));
    erode(img_thresh, img_erode, kernel_erode);

    cvCanny(img_erode, img_canny, 50, 150, 3);

    std::vector<Vec3f> cir;
    /*
    param1 - 第一个方法特定的参数。在CV_HOUGH_GRADIENT的情况下， 两个传递给Canny（）边缘检测器的阈值较高（较小的两个小于两倍）。
    param2 - 第二种方法参数。在CV_HOUGH_GRADIENT的情况下，它是检测阶段的圆心的累加器阈值。越小，可能会检测到越多的虚假圈子。首先返回对应于较大累加器值的圈子。
    */
    HoughCircles(img_canny, cir, CV_HOUGH_GRADIENT, 1, MIN_CIRCLE_DISTANCE, 100, 20, MIN_RADIUS, MAX_RADIUS);
    if (cir.empty())
        return data;
    data.push_back(cir[0][0]);
    data.push_back(cir[0][1]);
    // for (size_t i = 0; i < cir.size(); i++)
    // {
    //     Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    //     circle(t1, Point(cir[i][0], cir[i][1]), cir[i][2], color, 1, 8);
    // }
    return data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector");
    Detector d;
    ros::spin();
    return 0;
}