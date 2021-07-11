# -*- coding: utf-8 -*-
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone
from task_vision import Track, bind_vision


tm = TaskManager()
tm.debug = True

bind_drone(tm)

tm.add([
    Tip('1-1'),
    Lift(1.2),
    Wait(5.0),
    Land()
])

tm.loop()
