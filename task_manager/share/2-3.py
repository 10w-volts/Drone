# -*- coding: utf-8 -*-
from std_msgs.msg import Bool
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone, Unlock, Lock
from task_vision import Track, bind_vision
from math import pi, sin, cos, fabs, sqrt, asin


class MyTask(Task):
    def __init__(self, vtype='type', gap=(-0.08, -0.08), vrange=(0.8, 0.6), timeout=3.0):

        self.vtype = vtype

        # 真正的中心相对于摄像头中心的坐标
        self.gap_x, self.gap_y = gap

        # 一米高时半视野视角对应XY轴的长度
        self.vrange_x, self.vrange_y = vrange

        # 目标超时时间与此时的已超时时间
        self.timeout = timeout
        self.time = 0

    def run(self, tm):

        # 检测出目标
        if tm.sub['/vision/gap'] and len(tm.sub['/vision/gap'].data) == 2:

            # 取消超时
            self.time = 0.0

            [x, y] = tm.sub['/ctrl/get/p'].data
            h = tm.sub['/ctrl/get/h'].data
            a = tm.sub['/ctrl/get/yaw'].data*pi/180.0

            # 目标位置相对于相机中心的偏移
            [dx, dy] = tm.sub['/vision/gap'].data
            dx = dx * h * self.vrange_x - self.gap_x
            dy = dy * h * self.vrange_y - self.gap_y

            if tm.debug:
                tm.log('(dx, dy): {:.3f}, {:.3f}'.format(dx, dy))

            # 目标位置相对于绝对坐标的偏移
            ex = dy*cos(a) + dx*sin(a)
            ey = dy*sin(a) - dx*cos(a)

            angle = asin(ey/sqrt(ex**2+ey**2))*180/pi

            if angle < 15:
                tm.pub['/gpio/led'].publish(True)
                tm.pub['/gpio/buzz'].publish(True)
                tm.log('OK')
            else:
                tm.pub['/gpio/led'].publish(False)
                tm.pub['/gpio/buzz'].publish(False)

            tm.pub['/ctrl/set/yaw'].publish(angle)

        else:  # 没检测到目标

            tm.pub['/gpio/led'].publish(False)
            tm.pub['/gpio/buzz'].publish(False)

            # 等待超时
            self.time += 1.0/tm.freq
            if self.time > self.timeout:
                tm.next()

        tm.pub['/vision/type'].publish(self.vtype)


tm = TaskManager()
tm.debug = True

tm.add_pub('/gpio/led', Bool)
tm.add_pub('/gpio/buzz', Bool)

bind_drone(tm)
bind_vision(tm)

tm.add([
    Tip('2-3'),
    Unlock(),
    Lift(1.0),
    MyTask(vtype='car', timeout=5.0),
    Land(),
    Lock()
])

tm.loop()
