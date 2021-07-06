#include <mavros/mavros_plugin.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

namespace mavros{
    namespace std_plugins{
        class WhudTakeoffLandPlugin : public plugin::PluginBase{
            public:
                WhudTakeoffLandPlugin() : PluginBase(), mav_control_nh("~whud_takeoff_land"){}
                void initialize(UAS &uas_) override {
                    PluginBase::initialize(uas_);

                    takeoff_sub = mav_control_nh.subscribe("takeoff_height", 1, &WhudTakeoffLandPlugin::takeoff_cb, this);
                    land_sub = mav_control_nh.subscribe("land", 1, &WhudTakeoffLandPlugin::land_cb, this);
                    unlock_sub = mav_control_nh.subscribe("unlock", 1, &WhudTakeoffLandPlugin::unlock_cb, this);
                    lock_sub = mav_control_nh.subscribe("lock", 1, &WhudTakeoffLandPlugin::lock_cb, this);

                    takeoff_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("takeoff_progress", 1);
                    land_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("land_progress", 1);
                    unlock_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("unlock_progress", 1);
                    lock_progress_pub = mav_control_nh.advertise<std_msgs::Int32>("lock_progress", 1);
                }
                Subscriptions get_subscriptions() override
                {
                    return {
                            make_handler(&WhudTakeoffLandPlugin::handle_progress),
                    };
                }
            private:
                ros::NodeHandle mav_control_nh;
                ros::Subscriber takeoff_sub;
                ros::Subscriber land_sub;
                ros::Subscriber unlock_sub;
                ros::Subscriber lock_sub;

                ros::Publisher takeoff_progress_pub;
                ros::Publisher land_progress_pub;
                ros::Publisher unlock_progress_pub;
                ros::Publisher lock_progress_pub;

                void takeoff_cb(const std_msgs::Float64MultiArray::ConstPtr &takeoff_msg)
                {
                    mavlink::common::msg::COMMAND_LONG msg;
                    // MAV_CMD_NAV_TAKEOFF_LOCAL
                    msg.command = 24;
                    // set z axis speed, default unit m/s
                    msg.param3 = takeoff_msg->data[0];
                    // set height, default unit m
                    msg.param7 = takeoff_msg->data[1];

                    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
                }

                void land_cb(const std_msgs::Float64::ConstPtr &land_msg)
                {
                    mavlink::common::msg::COMMAND_LONG msg;
                    // MAV_CMD_NAV_LAND_LOCAL
                    msg.command = 23;
                    // set z axis speed, default unit m/s
                    msg.param3 = land_msg->data;
                    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
                }

                void unlock_cb(const std_msgs::Bool::ConstPtr &unlock_msg)
                {
                    mavlink::common::msg::COMMAND_LONG msg;
                    msg.command = 31011;
                    if(unlock_msg->data == true)
                        UAS_FCU(m_uas)->send_message_ignore_drop(msg);
                }

                void lock_cb(const std_msgs::Bool::ConstPtr &lock_msg)
                {
                    mavlink::common::msg::COMMAND_LONG msg;
                    msg.command = 31012;
                    if(lock_msg->data == true)
                        UAS_FCU(m_uas)->send_message_ignore_drop(msg);
                }

                void handle_progress(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_ACK &progress_msg)
                {
                    auto progress = boost::make_shared<std_msgs::Int32>();
                    progress->data = progress_msg.result;

                    if(progress_msg.command == 24)
                        takeoff_progress_pub.publish(progress);
                    else if(progress_msg.command == 23)
                        land_progress_pub.publish(progress);
                    else if(progress_msg.command == 31011)
                        unlock_progress_pub.publish(progress);
                    else if(progress_msg.command == 31012)
                        lock_progress_pub.publish(progress);
                }
        };
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WhudTakeoffLandPlugin,
                       mavros::plugin::PluginBase)