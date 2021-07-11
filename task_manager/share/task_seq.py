# -*- coding: utf-8 -*-
from task import Task
from task_ros import TaskManager
from task_drone import Tip, Wait, Lift, Land, H, Nav, Yaw, Unlock, Lock, bind_drone
from task_vision import Track, bind_vision
from task_other import Task_12, Alarm, Track_R, Back_R, bind_other


def task_seq_11():
    return [
        Tip('1-1'),
        Wait(5.0),
        Unlock(),
        Lift(1.2),
        Wait(5.0),
        Land(),
        Lock()
    ]

def task_seq_12():
    return [
        Tip('1-2'),
        Wait(5.0),
        Task_12()
    ]

def task_seq_13():
    return [
        Tip('1-3'),
        Wait(5.0),
        Unlock(),
        Lift(1.0),
        Nav((1.0, 0.0), eps=0.1),
        Track('car', (0.7, 0.0005, 0.7), 60, eps=0.10),
        Land(),
        Lock()
    ]

def task_seq_21():
    return [
        Tip('2-1'),
        Wait(5.0),
        Unlock(),
        Lift(1.0),
        Nav((1.0, 0.0)),
        Track('car', pid=(0.7, 0.0005, 0.7), num=50, eps=0.1),
        Land(),
        Lock()
    ]

def task_seq_22():
    return [
        Tip('2-2'),
        Wait(5.0),
        Unlock(),
        Lift(1.0),
        Nav((1.0, 0.0)),
        Track('car', pid=(0.7, 0.0005, 0.7), num=50, eps=0.1, timeout=5.0),
        Land(),
        Lock()
    ]

def task_seq_23():
    return [
        Tip('2-3'),
        Wait(5.0),
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
    ]


def bind_all(tm):

    bind_drone(tm)
    bind_vision(tm)
    bind_other(tm)
