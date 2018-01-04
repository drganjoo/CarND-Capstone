#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from timed_logger import TimedLogger

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.final_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.way_points = None
        self.last_point = 0
        self.points_to_look = 0      # how many points to look at

        self.timed_logger = TimedLogger(1)

        print('WaypointUpdater about to go for spinning')
        rospy.spin()

    def pose_cb(self, msg):
        if self.way_points is None:
            return

        position = msg.pose.position
        # angle_q = msg.pose.orientation

        # compute distance to the next way point and figure out which is the closest
        least_index = self.last_point
        least_distance = float('+inf')
        total_wp = len(self.way_points)

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(self.points_to_look):
            wp_index = (self.last_point + i) % total_wp
            wp = self.way_points[wp_index]
            distance = dl(position, wp.pose.pose.position)

            if distance < least_distance:
                least_distance = distance
                least_index = wp_index

        # publish as many as LOOKAHEAD points
        lane = Lane()
        for i in range(LOOKAHEAD_WPS):
            index = (i + least_index) % total_wp
            lane.waypoints.append(self.way_points[index])

        # next time we would like to look closeby to where we found the vehicle
        # to be for time t-1.
        self.points_to_look = 100
        self.last_point = least_index

        # publish the points to the rest of the nodes
        self.final_pub.publish(lane)

        self.timed_logger.log('[waypoint_updater] start_index:%d, points_to_look_at:%s', self.last_point, self.points_to_look)


    def waypoints_cb(self, waypoints):
        self.way_points = waypoints.waypoints
        self.points_to_look = len(waypoints.waypoints)

        print('Waypoints received')

        # for waypoint in waypoints.waypoints:
        #     position = waypoint.pose.pose.position
            # angle_q = waypoint.pose.pose.orientation
            # euler_angle = tf.transformations.euler_from_quaternion(angle_q)
            # euler_angle = 0
            # velocity = waypoint.twist.twist.linear.x

            #self.way_points.append([position, euler_angle, angle_q, velocity])

            # rospy.loginfo('%s, %s, %s, %s', position.x, position.y, position.z, euler_angle)
            # self.way_points.append(position.x, position.y, position.z)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
