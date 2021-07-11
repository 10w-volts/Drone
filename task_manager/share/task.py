# -*- coding: utf-8 -*-
from __future__ import print_function
import time


class Data(object):

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Task:

    def __init__(self):
        pass

    def run(self, tm):
        tm.next()


class TaskManager:

    def __init__(self):
        self.task = Task()
        self.main_task = []
        self.side_task = []

        self.debug = False
        self.enabled = False
        self.data = Data()
        self.freq = 10.0

    def add(self, task_or_list):
        if type(task_or_list) == type([]):
            for task in reversed(task_or_list):
                self.main_task.append(task)
        else:
            self.main_task.append(task_or_list)

    def log(self, text='', log_task=False, to_file=False):
        if to_file:
            with open('/home/nvidia/.task.log', 'a') as f:
                if not log_task:
                    print('{}\t\t{}'.format(time.strftime('%d %H:%M:%S'), text), file=f)
                else:
                    print('{}\t{}\t{}'.format(time.strftime('%d %H:%M:%S'),
                    self.task.__class__.__name__, text), file=f)
        else:
            if not log_task:
                print('{}\t\t{}'.format(time.strftime('%d %H:%M:%S'), text))
            else:
                print('{}\t{}\t{}'.format(time.strftime('%d %H:%M:%S'),
                self.task.__class__.__name__, text))

    def loop(self):
        self.enabled = True
        self.log('Begin...')
        while self.enabled:
            t = time.time()
            self.task.run(self)
            dt = time.time() - t
            if dt < 1.0 / self.freq:
                time.sleep(1.0 / self.freq - dt)
        self.log('End...')

    def next(self):
        self.log('NEXT', True)
        if self.side_task:
            self.task = self.side_task.pop()
        elif self.main_task:
            self.task = self.main_task.pop()
        else:
            self.enabled = False
        self.log('', True)

    def jump(self, task_or_list):
        self.log('JUMP', True)
        if type(task_or_list) == type([]):
            for task in reversed(task_or_list):
                self.side_task.append(task)
        else:
            self.side_task.append(task_or_list)
        self.task = self.side_task.pop()
        self.log('', True)

    def call(self, task_or_list):
        self.log('CALL', True)
        self.side_task.append(self.task)
        if type(task_or_list) == type([]):
            for task in reversed(task_or_list):
                self.side_task.append(task)
        else:
            self.side_task.append(task_or_list)
        self.task = self.side_task.pop()
        self.log('', True)
