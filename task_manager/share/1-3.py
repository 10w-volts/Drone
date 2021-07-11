# -*- coding: utf-8 -*-
from std_msgs.msg import Bool
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone
from task_vision import Track, bind_vision


tm = TaskManager()
tm.debug = True

tm.add_pub('/gpio/led', Bool)

bind_drone(tm)
bind_vision(tm)

tm.add([
    Tip('1-3'),
    Lift(1.0),
    Nav((1.0, 0.0), eps=0.1),
    Track('car', (0.7, 0.0005, 0.7), 60, eps=0.10),
    Land(),
    Tip('End')
])

tm.loop()
