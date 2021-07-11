# -*- coding: utf-8 -*-
import task
import rospy
import threading


class TaskManager(task.TaskManager):

    def __init__(self, node_name='task_ros'):
        task.TaskManager.__init__(self)
        self.thread = None
        self.sub = {}
        self.pub = {}
        rospy.init_node(node_name)

    def add_sub(self, topic, msg_type):
        def cb(msg):
            self.sub[topic] = msg
        rospy.Subscriber(topic, msg_type, callback=cb, queue_size=10)
        self.sub[topic] = None

    def add_pub(self, topic, msg_type):
        self.pub[topic] = rospy.Publisher(topic, msg_type, queue_size=10)

    def begin_thread(self):
        self.thread = threading.Thread(target=task.TaskManager.loop, args=(self,))
        self.thread.setDaemon(True)
        self.thread.start()

    def loop(self):
        if self.thread is None:
            self.begin_thread()
            rospy.spin()
        elif not self.thread.is_alive():
            self.begin_thread()
