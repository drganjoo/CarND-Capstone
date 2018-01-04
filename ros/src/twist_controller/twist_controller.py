import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

THROTTLE_PID = (1.0, 0.0, 0.0, 0.0, 1.0)
#STEERING_PID = (0.195, 0.0079, 0.2, -1.0, 1.0)
# STEERING_PID = (2, 0, 0, -1.0, 1.0)


class Controller(object):
    def __init__(self, yaw_controller):
        self.throttle_pid = None
        # self.steering_pid = None
        self.yaw_controller = yaw_controller

    def control(self, target_velocity, current_velocity, target_angular_velocity, current_angular_velocity):
        # Return throttle, brake, steer
        throttle_error = target_velocity - current_velocity
        # steering_error = target_angular_velocity - current_angular_velocity

        throttle = self.throttle_pid.step(throttle_error, 1.0 / 50.0)
        #steer = self.steering_pid.step(steering_error, 1.0 / 50.0)
        steer = self.yaw_controller.get_steering(target_velocity, target_angular_velocity, current_velocity)
        brake = 0

        # rospy.loginfo('Steering:%.3f,%.3f,%.3f', steering_error, target_angular_velocity, current_angular_velocity)
        return throttle, brake, steer

    def reset(self):
        #rospy.loginfo('Controller: reset')
        self.throttle_pid = None

    def start(self):
        self.throttle_pid = PID(*THROTTLE_PID)
        # self.steering_pid = PID(*STEERING_PID)
        # rospy.loginfo('Controller Started. Throttle PID: %f,%f,%f', THROTTLE_PID[0], THROTTLE_PID[1], THROTTLE_PID[2])
