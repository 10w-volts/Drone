# -*- coding: utf-8 -*-
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone
from task_vision import Track, bind_vision


class Test(Task):
    def run(self, tm):
        tm.call(Track(
            vtype   =   'car',
            pid     =   (0.5, 0.0, 0.5),
            timeout =   3.0,
            eps     =   0.05,
            times   =   5
        ))


tm = TaskManager()
tm.debug = True

bind_drone(tm)
bind_vision(tm)

tm.add([
    Wait(1),
    Lift(1)
])

tm.add(Test())

tm.loop()
