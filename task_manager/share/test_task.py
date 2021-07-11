# -*- coding: utf-8 -*-
from task import Task, TaskManager
from task_drone import Wait, Tip


class A(Task):
    def run(self, tm):
        tm.log('AAA')
        tm.next()


class B(Task):
    def run(self, tm):
        tm.log('BBB')
        tm.jump([
            Tip('000'),
            Tip('222'),
            Tip('333')
        ])


tm = TaskManager()
tm.add(Wait(1))

tm.add([
    A(),
    B()
])


tm.loop()
