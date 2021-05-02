#ifndef DETECTOR
#define DETECTOR

#include <ros/ros.h>
// 用image_transport软件包发布和订阅ROS中的图像
#include <image_transport/image_transport.h>
// 这两个头文件包含了CvBridge类以及与图像编码相关的函数
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// 这两个头文件包含了OpenCV图像处理模块和GUI模块
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include <math.h>

using namespace cv;

class Detector
{
private:
    Mat img_;
    char current_location_;
    std_msgs::Float64MultiArray circle_center_;
    //msg.data = array_test;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber img_sub_;
    ros::Subscriber locate_sub_;
    ros::Publisher cir_center_pub_;

    Detector() : it_(nh_)
    {
        current_location_ = 0;
        circle_center_.data.push_back(0);
        circle_center_.data.push_back(0);
        img_sub_ = it_.subscribe("/usb_cam_nodelet/image_raw", 1, &ImageConverter::cameraCaptureCb, this);
        locate_sub_ = nh_.subscribe("gap_type_req", 10, detectLightCb);
        cir_center_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("gap", 10);
    }
    ~Detector()
    {
    }

    void cameraCaptureCb(const sensor_msgs::ImageConstPtr &data);
    void detectLightCb(const std_msgs::String::ConstPtr &msg);
    std::vector<double> detectCircle(Mat);

public:
}

#endif // !DETECTOR
