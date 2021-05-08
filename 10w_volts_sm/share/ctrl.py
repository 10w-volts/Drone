#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math

import rospy
import tf

from tf import TransformListener
from tf.msg import tfMessage

from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist, PoseStamped, Quaternion

from std_msgs.msg import Bool, String, Float64, Float64MultiArray, Int32
from sensor_msgs.msg import Imu


get_h_pub = None
get_p_pub = None
height_control_pub = None
h_done_pub = None

tf_listener = None
goal_pub = None
cmd_vel_pub = None

get_yaw_pub = None
init_yaw = None
cur_yaw = None
yaw_control_pub = None
yaw_done_pub = None

move = False

takeoff_height_pub = None
lift_done_pub = None
land_pub = None
land_done_pub = None


def move_base_topic(goalx = 0, goaly = 0, goaltheta = 0):
    goal = MoveBaseActionGoal() 
    goal.goal.target_pose.header.frame_id = 'map'
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose.position.x = goalx
    goal.goal.target_pose.pose.position.y = goaly
    goal.goal.target_pose.pose.position.z = 0.0
    q_angle = tf.transformations.quaternion_from_euler(0.0, 0.0, goaltheta)
    q = Quaternion(*q_angle)
    goal.goal.target_pose.pose.orientation = q
    return goal

# 监听/mavros/local_position/pose并转发到/get_h
def pose_cb(msg):
    global get_h_pub
    get_h_pub.publish(msg.pose.position.z)

# 监听/set_h，并发送到/height_control
def set_h_cb(msg):
    global height_control_pub
    height_control = Float64MultiArray(data=[0, msg.data])
    height_control_pub.publish(height_control)

# 监听/mavros/whud_height_control/height_control_progress并发布到/ctrl/get/h_done
def height_control_progress_cb(msg):
    global h_done_pub
    if msg.data == 0:
        h_done_pub.publish(True)
    elif msg.data == 5:
        h_done_pub.publish(False)


# 监听/tf并发布当前位置到/get_p
def tf_cb(msg):
    global tf_listener, get_p_pub
    tf_translation, tf_rotation = tf_listener.lookupTransform('/map', '/nav_link', rospy.Time())
    pos = Float64MultiArray(data=[tf_translation[0], tf_translation[1]])
    get_p_pub.publish(pos)

# 监听/set_p，处理后发送到/move_base/goal
def set_p_cb(msg):
    global goal_pub
    goal = move_base_topic(msg.data[0], msg.data[1], 0)
    goal_pub.publish(goal)
# 监听/move_base/cmd_vel，直接发到/mavros/whud_navigation/cmd_vel
def cmd_vel_cb(msg):
    global cmd_vel_pub, move
    if move:
        cmd_vel_pub.publish(msg)


# 监听/mavros/imu/data，计算后发布到/ctrl/get/yaw
def imu_data_cb(msg):
    global get_yaw_pub, init_yaw, cur_yaw
    o = msg.orientation
    point = tf.transformations.euler_from_quaternion((o.x, o.y, o.z, o.w))
    if init_yaw is None:
        init_yaw = point[2]*180/math.pi
    cur_yaw = (point[2]*180/math.pi - init_yaw) % 360
    get_yaw_pub.publish(cur_yaw)

# 监听/set_yaw并发布到/mavros/whud_yaw_control/yaw_control
def set_yaw_cb(msg):
    global yaw_control_pub, cur_yaw
    yaw = (msg.data-cur_yaw)%360
    if yaw > 180:
        yaw = yaw-360
    yaw_control = Float64MultiArray(data=[yaw*math.pi/180, 1])
    yaw_control_pub.publish(yaw_control)

# 监听/mavros/whud_yaw_control/yaw_control_progress并发布到/ctrl/get/yaw_done
def yaw_control_progress_cb(msg):
    global yaw_done_pub
    if msg.data == 0:
        yaw_done_pub.publish(True)
    elif msg.data == 5:
        yaw_done_pub.publish(False)


# 监听/move，控制无人机是否允许移动
def set_move_cb(msg):
    global move
    move = msg.data


# 监听起飞消息/lift并发送到/mavros/whud_takeoff_land/takeoff_height
def set_lift_cb(msg):
    global takeoff_height_pub
    takeoff_height = Float64MultiArray(data=[0, msg.data])
    takeoff_height_pub.publish(takeoff_height)

# 监听/mavros/whud_takeoff_land/takeoff_progress并发布到/ctrl/get/lift_done
def takeoff_progress_cb(msg):
    global lift_done_pub
    if msg.data == 0:
        lift_done_pub.publish(True)
    elif msg.data == 5:
        lift_done_pub.publish(False)


# 监听降落的消息/land并发送到/mavros/whud_takeoff_land/land
def set_land_cb(msg):
    global land_pub
    land_pub.publish(msg.data)

# 监听/mavros/whud_takeoff_land/land_progress并发布到/ctrl/get/land_done
def land_progress_cb(msg):
    global land_done_pub
    if msg.data == 0:
        land_done_pub.publish(True)
    elif msg.data == 5:
        land_done_pub.publish(False)


def main():
    global get_h_pub, height_control_pub, h_done_pub
    global tf_listener, get_p_pub, goal_pub, cmd_vel_pub
    global get_yaw_pub, yaw_control_pub, yaw_done_pub
    global move, takeoff_height_pub, lift_done_pub, land_pub, land_done_pub

    rospy.init_node('ctrl', anonymous=True)
    
    tf_listener = TransformListener()
    time.sleep(3)

    # 监听并发布当前位置到/get_h
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    get_h_pub = rospy.Publisher('/ctrl/get/h', Float64, queue_size=5)
    # 监听/set_h并发布给下位机
    rospy.Subscriber('/ctrl/set/h', Float64, set_h_cb)
    height_control_pub = rospy.Publisher('/mavros/whud_height_control/height_control', Float64MultiArray, queue_size=5)
    # 监听/mavros/whud_height_control/height_control_progress并发布到/ctrl/get/h_done
    rospy.Subscriber('/mavros/whud_height_control/height_control_progress', Int32, height_control_progress_cb)
    h_done_pub = rospy.Publisher('/ctrl/get/h_done', Bool, queue_size=5)

    # 监听/tf并发布当前位置到/get_p
    rospy.Subscriber('/tf', tfMessage, tf_cb)
    get_p_pub = rospy.Publisher('/ctrl/get/p', Float64MultiArray, queue_size=5)
    
    # 监听/set_p并控制下位机
    rospy.Subscriber('/ctrl/set/p', Float64MultiArray, set_p_cb)
    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=5)
    rospy.Subscriber('/move_base/cmd_vel', Twist, cmd_vel_cb)
    cmd_vel_pub = rospy.Publisher('/mavros/whud_navigation/cmd_vel', Twist, queue_size=5)

    # 监听/mavros/imu/data，计算后发布到/ctrl/get/yaw
    rospy.Subscriber('/mavros/imu/data', Imu, imu_data_cb)
    get_yaw_pub = rospy.Publisher('/ctrl/get/yaw', Float64, queue_size=5)
    # 监听/set_yaw并发布到/mavros/whud_yaw_control/yaw_control
    rospy.Subscriber('/ctrl/set/yaw', Float64, set_yaw_cb)
    yaw_control_pub = rospy.Publisher('/mavros/whud_yaw_control/yaw_control', Float64MultiArray, queue_size=5)
    # 监听/mavros/whud_yaw_control/yaw_control_progress并发布到/ctrl/get/yaw_done
    rospy.Subscriber('/mavros/whud_yaw_control/yaw_control_progress', Int32, yaw_control_progress_cb)
    yaw_done_pub = rospy.Publisher('/ctrl/get/yaw_done', Bool, queue_size=5)

    # 监听/move，控制无人机是否允许移动
    rospy.Subscriber('/ctrl/set/move', Bool, set_move_cb)

    # 监听起飞消息/lift并发送到/mavros/whud_takeoff_land/takeoff_height
    rospy.Subscriber('/ctrl/set/lift', Float64, set_lift_cb)
    takeoff_height_pub = rospy.Publisher('/mavros/whud_takeoff_land/takeoff_height', Float64MultiArray, queue_size=5)
    # 监听/mavros/whud_takeoff_land/takeoff_progress并发布到/ctrl/get/lift_done
    rospy.Subscriber('/mavros/whud_takeoff_land/takeoff_progress', Int32, takeoff_progress_cb)
    lift_done_pub = rospy.Publisher('/ctrl/get/lift_done', Bool, queue_size=5)
    
    # 监听降落的消息/land并发送到/mavros/whud_takeoff_land/land
    rospy.Subscriber('/ctrl/set/land', Float64, set_land_cb)
    land_pub = rospy.Publisher('/mavros/whud_takeoff_land/land', Float64, queue_size=5)
    # 监听/mavros/whud_takeoff_land/land_progress并发布到/ctrl/get/land_done
    rospy.Subscriber('/mavros/whud_takeoff_land/land_progress', Int32, land_progress_cb)
    land_done_pub = rospy.Publisher('/ctrl/get/land_done', Bool, queue_size=5)

    print 'begin...'

    rospy.spin()


if __name__ == '__main__':
    main()
