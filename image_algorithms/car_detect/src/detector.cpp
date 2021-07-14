#include "detector.h"
#include <ctime>

using namespace cv;
clock_t start, end;

void Detector::cameraCaptureCb(const sensor_msgs::ImageConstPtr &data)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(img_);
        //ROS_INFO("Transform SUCCESS!"); //打印消息
        //ROS_INFO("Img Rows: %d. Img Cols: %d", img_.rows, img_.cols);
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
    if (msg->data == "a")
    {
        current_location_ = 1;
        ROS_INFO("Detector has heard \"Pos.a\"!");
    }
    else if (msg->data == "b")
    {
        current_location_ = 2;
        ROS_INFO("Detector has heard \"Pos.b\"!");
    }
    else if (msg->data == "c")
    {
        current_location_ = 3;
        ROS_INFO("Detector has heard \"Pos.c\"!");
    }
    else
    {
        current_location_ = 0;
        ROS_INFO("Detector has not heard!");
    }

    if (current_location_ != 0 && img_.data != NULL)
    {
        circle_center_.data = detectCircle(img_);
        cir_center_pub_.publish(circle_center_);
        ROS_INFO("Detector has published the center point!");
        current_location_ = 0;
    }
    else
        current_location_ = 0;
}

std::vector<double> Detector::detectCircle(Mat img)
{
    start = clock();

    std::vector<double> data;
    int MIN_CIRCLE_DISTANCE = int(pow((pow(img.rows, 2) + pow(img.cols, 2)), 0.5));
    int MIN_RADIUS = 50;
    int MAX_RADIUS = 200;

    Mat img_gray;

    cvtColor(img, img_gray, CV_BGR2GRAY);

    std::vector<Vec3f> cir;
    /*
    param1 - 第一个方法特定的参数。在CV_HOUGH_GRADIENT的情况下， 两个传递给Canny（）边缘检测器的阈值较高（较小的小于两倍）。
    param2 - 第二种方法参数。在CV_HOUGH_GRADIENT的情况下，它是检测阶段的圆心的累加器阈值。越小，可能会检测到越多的虚假圈子。首先返回对应于较大累加器值的圈子。
    */
    HoughCircles(img_gray, cir, CV_HOUGH_GRADIENT, 1, MIN_CIRCLE_DISTANCE, 200, 30, MIN_RADIUS, MAX_RADIUS);
    if (cir.empty())
    {
        ROS_INFO("No circle detected!");
        return data;
    }

    ROS_INFO("Circle num: %d", cir.size());
    ROS_INFO("Center Point: (%d, %d).", int(cir[0][0]), int(cir[0][1]));

    int PIC_HEIGHT = img.rows;
    int PIC_WIDTH = img.cols;

    data.push_back((cir[0][0] - PIC_WIDTH / 2) / (PIC_WIDTH / 2));
    data.push_back(-(cir[0][1] - PIC_HEIGHT / 2) / (PIC_HEIGHT / 2));

    end = clock();
    double endtime = (double)(end - start) / CLOCKS_PER_SEC;
    ROS_INFO("Run time: %f ms.", endtime * 1000);

    // return [normalize_x, normalize_y]
    return data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector");
    Detector d;
    ros::spin();
    return 0;
}