import rospy

class Debug:
    def __init__(self, sleep_time):

        self.msg = []

        rospy.init_node('debug_node')
        rospy.subscribe('/ros_topic', self.msg_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(1)  # 1Hz

        while not rospy.is_shutdown():
            for m in self.msg:
                print(m)

            self.msg = []
            rate.sleep()

    def msg_cb(self, msg):
        self.msg.append(msg)
