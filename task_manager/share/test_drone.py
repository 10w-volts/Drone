# -*- coding: utf-8 -*-
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, Unlock, Lock, bind_drone

tm = TaskManager()
tm.debug = True

bind_drone(tm)

tm.add([
    Tip('Action A'),
    Unlock(),
    Lift(1),
    Wait(1),
    Land(),
    Lock()
    #Yaw(-45),
    #Nav((1, -1))
])

#tm.add([
#    Tip('Action B'),
#    H(0.5),
#    Yaw(45),
#    Nav((0, 0)),
#    H(-1),
#    Land()
#])

tm.loop()
