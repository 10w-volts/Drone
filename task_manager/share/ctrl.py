#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import time
import math

import rospy
import tf

from tf import TransformListener
from tf.msg import tfMessage

from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist, PoseStamped, Quaternion

from std_msgs.msg import Bool, String, Float64, Float64MultiArray, Int32

get_h_pub = None
get_p_pub = None
cur_h = 0.0
height_control_pub = None
h_ack_pub = None

tf_listener = None
goal_pub = None
cmd_vel_pub = None

get_yaw_pub = None
cur_yaw = None
yaw_control_pub = None
yaw_ack_pub = None
yaw_end_pub = None

nav_en = False

takeoff_height_pub = None
lift_ack_pub = None

land_pub = None
land_ack_pub = None
land_end_pub = None

unlock_pub = None
lock_pub = None
lock_ack_pub = None


def move_base_Part(goalx=0.0, goaly=0.0, goaltheta=0.0):
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
    global get_h_pub, cur_h
    cur_h = msg.pose.position.z
    get_h_pub.publish(cur_h)


# 监听/set_h，并发送到/height_control
def set_h_cb(msg):
    global cur_h, height_control_pub
    dh = msg.data - cur_h
    height_control = Float64MultiArray(data=[0, dh])
    height_control_pub.publish(height_control)


# 监听/mavros/whud_height_control/height_control_progress并发布到/ctrl/get/h_ack
def height_control_progress_cb(msg):
    global h_ack_pub
    if msg.data == 5:
        h_ack_pub.publish(True)


# 监听/set_p，处理后发送到/move_base/goal，启用导航
def set_p_cb(msg):
    global goal_pub, nav_en
    goal = move_base_Part(msg.data[0], msg.data[1], 0)
    goal_pub.publish(goal)
    nav_en = True


# 监听/move_base/cmd_vel，直接发到/mavros/whud_navigation/cmd_vel
def cmd_vel_cb(msg):
    global cmd_vel_pub, nav_en
    if nav_en:
        cmd_vel_pub.publish(msg)


# 限制速度范围
def limit(vel):
    if vel > 0.3:
        return 0.3
    if vel < -0.3:
        return -0.3
    return vel


# 监听/set_v，处理后发送到/mavros/whud_navigation/cmd_vel，禁用导航
def set_v_cb(msg):
    global cmd_vel_pub, nav_en
    nav_en = False
    vel = Twist()
    vel.linear.x = limit(msg.data[0])
    vel.linear.y = limit(msg.data[1])
    cmd_vel_pub.publish(vel)


# 监听/set_yaw并发布到/mavros/whud_yaw_control/yaw_control，禁用导航
def set_yaw_cb(msg):
    global nav_en, yaw_control_pub, cur_yaw
    nav_en = False
    yaw = (msg.data-cur_yaw) % 360
    if yaw > 180:
        yaw = yaw-360
    yaw_control = Float64MultiArray(data=[yaw*math.pi/180, 1])
    yaw_control_pub.publish(yaw_control)


# 监听/mavros/whud_yaw_control/yaw_control_progress并发布到/ctrl/get/yaw_ack(yaw_pub)
def yaw_control_progress_cb(msg):
    global yaw_ack_pub, yaw_end_pub
    if msg.data == 0:
        yaw_end_pub.publish(True)
    elif msg.data == 5:
        yaw_ack_pub.publish(True)


# 监听/tf并发布当前位置到/get_p，发布当前偏航到/yaw
def tf_cb(msg):
    global tf_listener, get_p_pub, get_yaw_pub, cur_yaw
    try:
        tf_translation, tf_rotation = tf_listener.lookupTransform(
            '/map', '/nav_link', rospy.Time())
    except:
        rospy.logwarn('no tf transform')
    else:
        pos = Float64MultiArray(data=[tf_translation[0], tf_translation[1]])
        get_p_pub.publish(pos)

        point = tf.transformations.euler_from_quaternion(tf_rotation)
        cur_yaw = point[2]*180.0/math.pi
        get_yaw_pub.publish(cur_yaw)


# 监听起飞消息/lift并发送到/mavros/whud_takeoff_land/takeoff_height
def set_lift_cb(msg):
    global takeoff_height_pub
    takeoff_height = Float64MultiArray(data=[0, msg.data])
    takeoff_height_pub.publish(takeoff_height)


# 监听/mavros/whud_takeoff_land/takeoff_progress并发布到/ctrl/get/lift_ack
def takeoff_progress_cb(msg):
    global lift_ack_pub
    if msg.data == 5:
        lift_ack_pub.publish(True)


# 监听降落的消息/land并发送到/mavros/whud_takeoff_land/land
def set_land_cb(msg):
    global land_pub
    land_pub.publish(msg.data)


# 监听/mavros/whud_takeoff_land/land_progress并发布到/ctrl/get/land_ack(land_end)
def land_progress_cb(msg):
    global land_ack_pub, land_end_pub
    if msg.data == 0:
        land_end_pub.publish(True)
    elif msg.data == 5:
        land_ack_pub.publish(True)


# 监听/ctrl/set/lock并发布到/mavros/whud_takeoff_land/(un)lock
def set_lock_cb(msg):
    global unlock_pub, lock_pub
    if msg.data:
        lock_pub.publish(True)
    else:
        unlock_pub.publish(True)


# 监听/mavros/whud_takeoff_land/unlock_progress并发布到/ctrl/get/lock_ack
def unlock_progress_cb(msg):
    global lock_ack_pub
    if msg.data == 5:
        lock_ack_pub.publish(True)


# 监听/mavros/whud_takeoff_land/lock_progress并发布到/ctrl/get/lock_ack
def lock_progress_cb(msg):
    global lock_ack_pub
    if msg.data == 5:
        lock_ack_pub.publish(True)


def main():

    # lift, h只有ack信号，end条件由任务管理器自行判断，加快速度
    global get_h_pub, height_control_pub, h_ack_pub
    global tf_listener, get_p_pub, goal_pub, cmd_vel_pub
    global get_yaw_pub, yaw_control_pub, yaw_ack_pub, yaw_end_pub
    global takeoff_height_pub, lift_ack_pub
    global land_pub, land_ack_pub, land_end_pub
    global unlock_pub, lock_pub, lock_ack_pub

    rospy.init_node('ctrl', anonymous=True)

    tf_listener = TransformListener()
    time.sleep(3)

    # 监听并发布当前位置到/get_h
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    get_h_pub = rospy.Publisher('/ctrl/get/h', Float64, queue_size=5)
    # 监听/set_h并发布给下位机
    rospy.Subscriber('/ctrl/set/h', Float64, set_h_cb)
    height_control_pub = rospy.Publisher(
        '/mavros/whud_height_control/height_control', Float64MultiArray, queue_size=5)
    # 监听/mavros/whud_height_control/height_control_progress并发布到/ctrl/get/h_ack
    rospy.Subscriber('/mavros/whud_height_control/height_control_progress',
                     Int32, height_control_progress_cb)
    h_ack_pub = rospy.Publisher('/ctrl/get/h_ack', Bool, queue_size=5)
    rospy.loginfo('Enable H Part...')

    # 监听/tf并发布当前位置到/get_p
    get_p_pub = rospy.Publisher('/ctrl/get/p', Float64MultiArray, queue_size=5)

    # 监听/set_p并控制下位机
    rospy.Subscriber('/ctrl/set/p', Float64MultiArray, set_p_cb)
    goal_pub = rospy.Publisher(
        '/move_base/goal', MoveBaseActionGoal, queue_size=5)
    rospy.Subscriber('/move_base/cmd_vel', Twist, cmd_vel_cb)
    cmd_vel_pub = rospy.Publisher(
        '/mavros/whud_navigation/cmd_vel', Twist, queue_size=5)
    rospy.loginfo('Enable Pos Part...')

    # 监听/set_v并控制下位机
    rospy.Subscriber('/ctrl/set/v', Float64MultiArray, set_v_cb)
    rospy.loginfo('Enable V Part...')

    # 监听/tf，计算后发布到/ctrl/get/yaw
    get_yaw_pub = rospy.Publisher('/ctrl/get/yaw', Float64, queue_size=5)
    # 监听/set_yaw并发布到/mavros/whud_yaw_control/yaw_control
    rospy.Subscriber('/ctrl/set/yaw', Float64, set_yaw_cb)
    yaw_control_pub = rospy.Publisher(
        '/mavros/whud_yaw_control/yaw_control', Float64MultiArray, queue_size=5)
    # 监听/mavros/whud_yaw_control/yaw_control_progress并发布到/ctrl/get/yaw_ack(yaw_pub)
    rospy.Subscriber('/mavros/whud_yaw_control/yaw_control_progress',
                     Int32, yaw_control_progress_cb)
    yaw_ack_pub = rospy.Publisher('/ctrl/get/yaw_ack', Bool, queue_size=5)
    yaw_end_pub = rospy.Publisher('/ctrl/get/yaw_end', Bool, queue_size=5)
    rospy.loginfo('Enable Yaw Part...')

    # 监听/tf并发布当前位置到/get_p，发布当前偏航到/yaw
    rospy.Subscriber('/tf', tfMessage, tf_cb)

    # 监听起飞消息/lift并发送到/mavros/whud_takeoff_land/takeoff_height
    rospy.Subscriber('/ctrl/set/lift', Float64, set_lift_cb)
    takeoff_height_pub = rospy.Publisher(
        '/mavros/whud_takeoff_land/takeoff_height', Float64MultiArray, queue_size=5)
    # 监听/mavros/whud_takeoff_land/takeoff_progress并发布到/ctrl/get/lift_ack
    rospy.Subscriber('/mavros/whud_takeoff_land/takeoff_progress',
                     Int32, takeoff_progress_cb)
    lift_ack_pub = rospy.Publisher('/ctrl/get/lift_ack', Bool, queue_size=5)
    rospy.loginfo('Enable Lift Part...')

    # 监听降落的消息/land并发送到/mavros/whud_takeoff_land/land
    rospy.Subscriber('/ctrl/set/land', Float64, set_land_cb)
    land_pub = rospy.Publisher(
        '/mavros/whud_takeoff_land/land', Float64, queue_size=5)
    # 监听/mavros/whud_takeoff_land/land_progress并发布到/ctrl/get/land_ack(land_end)
    rospy.Subscriber('/mavros/whud_takeoff_land/land_progress',
                     Int32, land_progress_cb)
    land_ack_pub = rospy.Publisher('/ctrl/get/land_ack', Bool, queue_size=5)
    land_end_pub = rospy.Publisher('/ctrl/get/land_end', Bool, queue_size=5)
    rospy.loginfo('Enable Land Part...')

    # 监听/ctrl/set/lock并发布到/mavros/whud_takeoff_land/(un)lock
    rospy.Subscriber('/ctrl/set/lock', Bool, set_lock_cb)
    unlock_pub = rospy.Publisher(
        '/mavros/whud_takeoff_land/unlock', Bool, queue_size=5)
    lock_pub = rospy.Publisher(
        '/mavros/whud_takeoff_land/lock', Bool, queue_size=5)
    # 监听/mavros/whud_takeoff_land/(un)lock_progress并发布到/ctrl/get/lock_ack
    rospy.Subscriber('/mavros/whud_takeoff_land/unlock_progress',
                     Int32, unlock_progress_cb)
    rospy.Subscriber('/mavros/whud_takeoff_land/lock_progress',
                     Int32, lock_progress_cb)
    lock_ack_pub = rospy.Publisher('/ctrl/get/lock_ack', Bool, queue_size=5)
    rospy.loginfo('Enable Lock Part...')

    rospy.loginfo('Begin...')

    rospy.spin()


if __name__ == '__main__':
    main()
