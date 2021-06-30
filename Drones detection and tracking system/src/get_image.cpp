#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "detect.h"

using namespace cv;

class CGet_image
{
    private:
        ros::Subscriber mStartProcess;
        ros::Subscriber mStartPubImage;
        ros::Publisher mRatioPub;
        ros::NodeHandle mNh;
        Mat mImage;
        Mat mProcessingImage;
        Point2f mMidPointRatio;
        image_transport::ImageTransport mImageTrans;
        image_transport::Subscriber mImageRecv;
        image_transport::Publisher mImagePub;
        ros::Timer mImagePubTimer;
        int mStartPub;
    public:
        CGet_image()
        :mImageTrans(mNh)
        {
            mImageRecv = mImageTrans.subscribe("/usb_cam/image_raw", 1, &CGet_image::image_transform, this);
            mStartPubImage = mNh.subscribe("/imagepub/start", 1, &CGet_image::image_pub, this);
            mStartProcess = mNh.subscribe("/vision/type", 1, &CGet_image::image_process, this);
            mRatioPub = mNh.advertise<std_msgs::Float64MultiArray>("/vision/gap", 10);
            mImagePub = mImageTrans.advertise("/processing_image", 10);
            mImagePubTimer = mNh.createTimer(ros::Duration(0.1), &CGet_image::image_pub_timer, this);
            mStartPub = 0;
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
        void image_process(const std_msgs::String::ConstPtr &msg)
        {
            if(msg->data == "car")
            {
                mMidPointRatio = detect(mImage, mProcessingImage);
                std::vector<double> data;
                if(mMidPointRatio.x != -9999)
                {
                    data.push_back(mMidPointRatio.x);
                    data.push_back(mMidPointRatio.y);
                }
                std_msgs::Float64MultiArray mid_point_ratio;
                mid_point_ratio.data = data;
                mRatioPub.publish(mid_point_ratio);
            }
        }

        /*************************************************
        Function:       image_pub
        Author:			Junpeng Chen
        Description:    Publish processing image
        Input:          Opencv image
        Output:         Change mStartPub parameter
        *************************************************/
        void image_pub(const std_msgs::Bool::ConstPtr &msg)
        {
            if(msg->data == true)
                mStartPub = 1;
            else
                mStartPub = 0;
        }

        /*************************************************
        Function:       image_pub_timer
        Author:			Junpeng Chen
        Description:    Publish processing image timer
        Input:          Timer event
        Output:         Publish image topic
        *************************************************/
        void image_pub_timer(const ros::TimerEvent &event)
        {
            if(mStartPub)
            {
                cv_bridge::CvImage out_msg;
                out_msg.header.stamp = ros::Time::now();
                out_msg.encoding = sensor_msgs::image_encodings::TYPE_8SC1;
                out_msg.image = mProcessingImage;
                mImagePub.publish(out_msg.toImageMsg());
            }
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_image");
    CGet_image obj;
    ros::spin();
}
