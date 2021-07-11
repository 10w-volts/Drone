#!/usr/bin/env python
# -*- coding: utf-8 -*-
from std_msgs.msg import Int32
from task import Task
from task_ros import TaskManager
from task_seq import task_seq_11, task_seq_12, task_seq_13, task_seq_21, task_seq_22, task_seq_23
from task_seq import bind_all


class Main(Task):
    def __init__(self):
        self.prev_start = None

    def run(self, tm):

        if tm.sub['/mode'] and tm.sub['/start']:

            mode = tm.sub['/mode'].data
            start = tm.sub['/start'].data

            if mode == 0:
                print('mode 0')
            elif mode == 1:
                tm.call(task_seq_11())
            elif mode == 2:
                tm.call(task_seq_12())
            elif mode == 3:
                tm.call(task_seq_13())
            elif mode == 4:
                tm.call(task_seq_21())
            elif mode == 5:
                tm.call(task_seq_22())
            elif mode == 6:
                tm.call(task_seq_23())

        tm.sub['/start'] = None


tm = TaskManager()
bind_all(tm)

tm.add_sub('/mode', Int32)
tm.add_sub('/start', Int32)

tm.add(Main())

tm.loop()
