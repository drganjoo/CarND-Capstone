import rospy

class TimedLogger():
    def __init__(self, delay):
        self.duration = rospy.Duration(delay)
        self.last_time = rospy.Time().now()

    def log(self, msg, *args):
        if rospy.Time().now() - self.last_time > self.duration:
            rospy.loginfo(msg, *args)
            self.last_time = rospy.Time().now()
