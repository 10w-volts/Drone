#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

using namespace std;

class nav_client{
    private:
        ros::NodeHandle mNh;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mNavClient;
        tf::TransformListener mTfListener;
        ros::Subscriber mNavVelSub;
        ros::Publisher mCmdVelPub;
        ros::Publisher mNavComplete;
    public:
        nav_client()
        {
            mNavVelSub = mNh.subscribe("/nav_point", 1, &nav_client::nav_point_cb, this);
            mCmdVelPub = mNh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
            mNavComplete = mNh.advertise<std_msgs::Bool>("/nav_complete", 1);
            mNavClient.waitForServer();
            mTfListener.waitForTransform("/map", "/nav_link", ros::Time(0), ros::Duration(0, 0));
        }
        ~nav_client()
        {}

        void nav_point_cb(const std_msgs::Float64MultiArray::ConstPtr &msg)
        {
            geometry_msgs::Pose set_pose;
            set_pose.position.x = msg->data[0];
            set_pose.position.y = msg->data[1];
            set_pose.position.z = 0;
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            set_pose.orientation.x = q.getX();
            set_pose.orientation.y = q.getY();
            set_pose.orientation.z = q.getZ();
            set_pose.orientation.w = q.getW();
            move_base_msgs::MoveBaseGoal goal = GoalSerialization(set_pose);
            mNavClient.sendGoal(
                goal,
                std::bind(&nav_client::done_cb, this, std::placeholders::_1,
                            std::placeholders::_2),
                std::bind(&nav_client::active_cb, this),
                std::bind(&nav_client::feedback_cb, this, std::placeholders::_1));
            }
        }

        move_base_msgs::MoveBaseGoal GoalSerialization(geometry_msgs::Pose set_pose)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = map_frame_id_;
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = set_pose;
            return goal;
        }
        void done_cb(const actionlib::SimpleClientGoalState &state,
              const move_base_msgs::MoveBaseResultConstPtr &result)
        {
            std_msgs::Bool complete;
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                complete.data = true;
                mNavComplete.publish(complete);
            }
        }
        void active_cb()
        {}
        void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
        {}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_client");
    nav_client obj;
    ros::spin();
}