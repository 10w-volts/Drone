# -*- coding: utf-8 -*-
from task import Task
from std_msgs.msg import Bool, Float64, Float64MultiArray
from math import pi, sin, cos, fabs


class Tip(Task):
    def __init__(self, name='Name'):
        self.name = name

    def run(self, tm):
        tm.log(self.name)
        tm.next()


class Wait(Task):
    def __init__(self, time=1.0):
        self.time = time

    def run(self, tm):
        self.time -= 1.0 / tm.freq
        if self.time <= 0:
            tm.next()


class Lift(Task):
    def __init__(self, h=1.0, eps=0.05):
        self.h = h
        self.eps = eps

    def run(self, tm):
        ack = tm.sub['/ctrl/get/lift_ack']
        if not ack:
            tm.pub['/ctrl/set/lift'].publish(self.h)
        else:
            h = tm.sub['/ctrl/get/h'].data
            # tm.log('{}'.format(h))
            if fabs(self.h - h) <= self.eps:
                tm.sub['/ctrl/get/lift_ack'] = None
                tm.next()


class H(Task):
    def __init__(self, h=1.0, eps=0.05):
        self.h = h
        self.eps = eps

    def run(self, tm):
        ack = tm.sub['/ctrl/get/h_ack']
        if not ack:
            tm.pub['/ctrl/set/h'].publish(self.h)
        else:
            h = tm.sub['/ctrl/get/h'].data
            if fabs(self.h - h) <= self.eps:
                tm.sub['/ctrl/get/h_ack'] = None
                tm.next()


class Land(Task):
    def __init__(self, v=0.0):
        self.v = v

    def run(self, tm):
        ack = tm.sub['/ctrl/get/land_ack']
        end = tm.sub['/ctrl/get/land_end']
        if not ack:
            tm.pub['/ctrl/set/land'].publish(self.v)
        elif end:
            tm.sub['/ctrl/get/land_ack'] = None
            tm.sub['/ctrl/get/land_end'] = None
            tm.next()


class Route(Task):
    def __init__(self, pos=(0.0, 0.0), eps=0.05):
        self.x, self.y = pos
        self.eps = eps
        self.ready = False

    def run(self, tm):
        if not self.ready:
            tm.pub['/ctrl/set/p'].publish(
                Float64MultiArray(data=[self.x, self.y]))
            self.ready = True
        else:
            [x, y] = tm.sub['/ctrl/get/p'].data
            if fabs(x-self.x) < self.eps and fabs(y-self.y) < self.eps:
                tm.next()


class Nav(Task):
    def __init__(self, pos=(0.0, 0.0), pid=(0.7, 0.0005, 0.7), num=5, eps=0.1, gap=(-0.0, -0.0)):

        self.x, self.y = pos
        self.kp, self.ki, self.kd = pid

        # 到达位置的次数
        self.num = num
        self.idx = 0

        # 在eps误差范围内认为到达
        self.eps = eps

        # 真正的中心相对于飞控的坐标
        self.gap_x, self.gap_y = gap

        # PID参数
        self.e_sum_x, self.e_sum_y = (None, None)
        self.e_last_x, self.e_last_y = (None, None)

    def run(self, tm):

        [cx, cy] = tm.sub['/ctrl/get/p'].data
        a = tm.sub['/ctrl/get/yaw'].data*pi/180.0

        # 目标位置相对于绝对坐标
        dx = self.x - cx - self.gap_x
        dy = self.y - cy - self.gap_y

        if tm.debug:
            tm.log('(dx, dy): {:.3f}, {:.3f}'.format(dx, dy))

        # 目标位置相对于自身坐标的偏移
        ex = dy*sin(a) + dx*cos(a)
        ey = dy*cos(a) - dx*sin(a)

        # 到达目标num次后，结束任务
        if fabs(ex) < self.eps and fabs(ey) < self.eps:
            self.idx += 1
            if tm.debug:
                tm.log('OK {}'.format(self.idx))
            if self.idx >= self.num:
                tm.pub['/ctrl/set/v'].publish(
                    Float64MultiArray(data=[0.0, 0.0]))
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


class Yaw(Task):
    def __init__(self, angle=0.0):
        self.angle = angle
        self.pos = None

    def run(self, tm):
        if self.pos is None:
            self.pos = tm.sub['/ctrl/get/p']

        ack = tm.sub['/ctrl/get/yaw_ack']
        end = tm.sub['/ctrl/get/yaw_end']
        if not ack:
            tm.pub['/ctrl/set/yaw'].publish(self.angle)
        elif end:
            tm.sub['/ctrl/get/yaw_ack'] = None
            tm.sub['/ctrl/get/yaw_end'] = None
            tm.jump(Nav((self.pos.data[0], self.pos.data[1])))


class Unlock(Task):
    def __init__(self):
        pass

    def run(self, tm):
        ack = tm.sub['/ctrl/get/lock_ack']
        if not ack:
            tm.pub['/ctrl/set/lock'].publish(False)
        else:
            tm.sub['/ctrl/get/lock_ack'] = None
            tm.next()


class Lock(Task):
    def __init__(self):
        pass

    def run(self, tm):
        ack = tm.sub['/ctrl/get/lock_ack']
        if not ack:
            tm.pub['/ctrl/set/lock'].publish(True)
        else:
            tm.sub['/ctrl/get/lock_ack'] = None
            tm.next()


def bind_drone(tm):

    tm.add_pub('/ctrl/set/p', Float64MultiArray)
    tm.add_sub('/ctrl/get/p', Float64MultiArray)

    tm.add_pub('/ctrl/set/v', Float64MultiArray)

    tm.add_pub('/ctrl/set/h', Float64)
    tm.add_sub('/ctrl/get/h', Float64)
    tm.add_sub('/ctrl/get/h_ack', Bool)

    tm.add_pub('/ctrl/set/yaw', Float64)
    tm.add_sub('/ctrl/get/yaw', Float64)
    tm.add_sub('/ctrl/get/yaw_ack', Bool)
    tm.add_sub('/ctrl/get/yaw_end', Bool)

    tm.add_pub('/ctrl/set/lift', Float64)
    tm.add_sub('/ctrl/get/lift_ack', Bool)

    tm.add_pub('/ctrl/set/land', Float64)
    tm.add_sub('/ctrl/get/land_ack', Bool)
    tm.add_sub('/ctrl/get/land_end', Bool)

    tm.add_pub('/ctrl/set/lock', Bool)
    tm.add_sub('/ctrl/get/lock_ack', Bool)
