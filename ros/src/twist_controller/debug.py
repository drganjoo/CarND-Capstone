#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log

class Debug:
    def __init__(self, sleep_time):

        self.msg = []

        rospy.init_node('debug_node')
        rospy.Subscriber('/rosout', Log, self.msg_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            for m in self.msg:
                print(m)

            self.msg = []
            rate.sleep()

    def msg_cb(self, msg):
        self.msg.append(msg.msg)


if __name__ == '__main__':
    try:
        Debug(1.0)
    except rospy.ROSInterruptException:
        pass