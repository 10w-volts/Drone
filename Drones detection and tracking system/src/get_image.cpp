#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "detect.h"

using namespace cv;

class CGet_image
{
    private:
        ros::Subscriber mStartProcess;
        ros::Publisher mRatioPub;
        ros::NodeHandle mNh;
        Mat mImage;
        Point2f mMidPointRatio;
        image_transport::ImageTransport mImageTrans;
        image_transport::Subscriber mImageRecv;
    public:
        CGet_image()
        :mImageTrans(mNh)
        {
            mImageRecv = mImageTrans.subscribe("/usb_cam/image_raw", 1, &CGet_image::image_transform, this);
            mStartProcess = mNh.subscribe("start", 1, &CGet_image::image_process, this);
            mRatioPub = mNh.advertise<std_msgs::Float64MultiArray>("result", 10);
        }

        ~CGet_image()
        {}

        /*************************************************
        Function:       image_transform
        Author:			Junpeng Chen
        Description:    Transform image from ros to opencv
        Input:          Ros image message
        Output:         Store opencv image into mImage
        *************************************************/
        void image_transform(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                cv_ptr->image.copyTo(mImage);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }

        /*************************************************
        Function:       image_process
        Author:			Junpeng Chen
        Description:    Detect image and return the mid point ratio
        Input:          Ros bool message
        Output:         Publish result into ros topic
        *************************************************/
        void image_process(const std_msgs::Bool::ConstPtr &msg)
        {
            if(msg->data == true)
            {
                mMidPointRatio = detect(mImage);
                std::vector<double> data;
                data.push_back(mMidPointRatio.x);
                data.push_back(mMidPointRatio.y);
                std_msgs::Float64MultiArray mid_point_ratio;
                mid_point_ratio.data = data;
                mRatioPub.publish(mid_point_ratio);
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_image");
    CGet_image obj;
    ros::spin();
}
