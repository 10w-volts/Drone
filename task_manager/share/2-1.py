# -*- coding: utf-8 -*-
from std_msgs.msg import Bool
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone
from task_vision import Track, bind_vision


tm = TaskManager()
tm.debug = True

bind_drone(tm)
bind_vision(tm)

tm.add([
    Tip('2-1'),
    Lift(1.0),
    Nav((1.0, 0.0)),
    Track('car', pid=(0.7, 0.0005, 0.7), num=50, eps=0.1),
    Land()
])

tm.loop()
