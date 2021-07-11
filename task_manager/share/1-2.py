# -*- coding: utf-8 -*-
from std_msgs.msg import Bool
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone
from task_vision import Track, bind_vision
from math import pi, sin, cos, fabs, sqrt


class MyTask(Task):

    def __init__(self, vtype='car'):
        self.vtype = vtype

        # 一米高时半视野视角对应XY轴的长度
        self.vax, self.vay = (0.8, 0.6)

    def run(self, tm):

        # 检测出目标
        if tm.sub['/vision/gap'] and len(tm.sub['/vision/gap'].data) == 2:

            [x, y] = tm.sub['/ctrl/get/p'].data
            h = tm.sub['/ctrl/get/h'].data
            a = tm.sub['/ctrl/get/yaw'].data*pi/180.0

            # 目标位置相对于相机中心的偏移
            [dx, dy] = tm.sub['/vision/gap'].data
            dx = dx * h * self.vax
            dy = dy * h * self.vay
            distance = sqrt(dx**2 + dy**2 + h**2)

            if tm.debug:
                tm.log('({:.3f}, {:.3f}, {:.3f})\t{:.3f}'.format(
                    dx, dy, h, distance))

            if distance > 0.5 and distance < 1.5:
                tm.pub['/gpio/led'].publish(True)
                tm.pub['/gpio/buzz'].publish(True)
                tm.log('OK')
            else:
                tm.pub['/gpio/led'].publish(False)
                tm.pub['/gpio/buzz'].publish(False)

        else:  # 没检测到目标

            tm.pub['/gpio/led'].publish(False)
            tm.pub['/gpio/buzz'].publish(False)

        tm.pub['/vision/type'].publish(self.vtype)


tm = TaskManager()
tm.debug = True

tm.add_pub('/gpio/led', Bool)
tm.add_pub('/gpio/buzz', Bool)

bind_drone(tm)
bind_vision(tm)

tm.add([
    Tip('1-2'),
    MyTask()
])

tm.loop()
