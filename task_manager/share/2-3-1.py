# -*- coding: utf-8 -*-
from std_msgs.msg import Bool
from std_msgs.msg import String, Float64MultiArray
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, bind_drone, Unlock, Lock
from task_vision import Track, bind_vision
from task_other import Alarm, Back_R, bind_other
from math import pi, sin, cos, fabs, sqrt, asin


class Track_R(Task):
    def __init__(self, vtype='type', pid=(0.7, 0.0005, 0.7), num=5, eps=0.05, gap=(-0.08, -0.08), vrange=(0.8, 0.6), timeout=3.0):

        self.vtype = vtype
        self.kp, self.ki, self.kd = pid

        # 到达位置的次数
        self.num = num
        self.idx = 0

        # 在eps误差范围内认为到达
        self.eps = eps

        # 真正的中心相对于摄像头中心的坐标
        self.gap_x, self.gap_y = gap

        # 一米高时半视野视角对应XY轴的长度
        self.vrange_x, self.vrange_y = vrange

        # 目标超时时间与此时的已超时时间
        self.timeout = timeout
        self.time = 0

        # PID参数
        self.e_sum_x, self.e_sum_y = (None, None)
        self.e_last_x, self.e_last_y = (None, None)

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

            # 到达目标num次后，结束任务
            if fabs(ex) < self.eps and fabs(ey) < self.eps:
                self.idx += 1
                if tm.debug:
                    tm.log('OK {}'.format(self.idx))
                if self.idx >= self.num:
                    tm.pub['/ctrl/set/v'].publish(Float64MultiArray(data=[0.0, 0.0]))
                    tm.data.pos_list.append((x, y))
                    tm.next()
            else:
                self.idx = 0

            # 初始化PID参数
            if self.e_sum_x is None:
                self.e_sum_x, self.e_sum_y = (ex, ey)
                self.e_last_x, self.e_last_y = (ex, ey)

            # 计算并更新积分项
            self.e_sum_x += ex
            self.e_sum_y += ey

            # 计算微分项
            e_diff_x = ex - self.e_last_x
            e_diff_y = ey - self.e_last_y

            # 更新上一项
            self.e_last_x = ex
            self.e_last_y = ey

            # 计算速度
            vx = self.kp * ex + self.ki * self.e_sum_x + self.kd * e_diff_x
            vy = self.kp * ey + self.ki * self.e_sum_y + self.kd * e_diff_y

            tm.pub['/ctrl/set/v'].publish(Float64MultiArray(data=[vx, vy]))

        else:  # 没检测到目标

            # 等待超时
            self.time += 1.0/tm.freq
            if self.time > self.timeout:
                tm.next()

            self.idx = 0

            # 清除PID参数
            self.e_sum_x, self.e_sum_y = (None, None)
            self.e_last_x, self.e_last_y = (None, None)

        tm.pub['/vision/type'].publish(self.vtype)


class Back_R(Task):
    def run(self, tm):
        task_list = [Nav(p) for p in reversed(tm.data.pos_list)]
        tm.jump(task_list)


tm = TaskManager()
tm.debug = True

tm.data.pos_list = []

tm.add_pub('/gpio/led', Bool)
tm.add_pub('/gpio/buzz', Bool)

bind_drone(tm)
bind_vision(tm)
bind_other(tm)

tm.add([
    Tip('2-3'),
    Unlock(),
    Lift(1.0),

    Track_R(vtype='car', timeout=5.0),
    Alarm(),
    Alarm(0, False),

    Track_R(vtype='car', timeout=5.0),
    Alarm(),
    Alarm(0, False),

    Track_R(vtype='car', timeout=5.0),
    Alarm(),
    Alarm(0, False),

    Track_R(vtype='car', timeout=5.0),
    Alarm(),
    Alarm(0, False),

    Track_R(vtype='car', timeout=5.0),
    Alarm(),
    Alarm(0, False),

    Alarm(0, True),
    Back_R(),
    Alarm(0, False),

    Land(),
    Lock()
])

tm.loop()
