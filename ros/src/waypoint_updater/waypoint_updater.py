#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math
import numpy as np

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
CONSTANT_DECEL = 1 / LOOKAHEAD_WPS  # Deceleration constant for smoother braking
PUBRATE = 10  # Rate (Hz) of waypoint publishing
STOP_LINE_MARGIN = 4  # Distance in waypoints to pad in front of the stop line
MAX_DECEL = 0.5

class WaypointUpdater(object):
    # cb = callback
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.stopline_wp_idx = -1
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=2)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=8)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=2)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.loop()
#         rospy.spin()
    
    def loop(self):
        rate = rospy.Rate(PUBRATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
#                 rospy.logwarn("hello")
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints()
            rate.sleep()
        
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # take closest index from the tree

        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val >0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx
    
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        rospy.logwarn("published new final waypoints")
      
    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            rospy.logwarn("stopline wp id = -1 or stopline wp idx too far ")
            lane.waypoints = base_waypoints
        else:
            rospy.logwarn("decelerate")
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - STOP_LINE_MARGIN, 0)
            rospy.logwarn("stopline: {}, closest_idx: {}, stop idx: {}, waypoint length: {}".format(self.stopline_wp_idx, closest_idx, stop_idx, len(waypoints)))
            try:
                dist = self.distance(waypoints, i, stop_idx)
            except:
                dist = 0.0
            # the closer we are to last waypoint, lower velocity
            vel = math.sqrt(2 * MAX_DECEL * dist) + (i * CONSTANT_DECEL)
            
            if vel < 1.0:
                vel = 0.0
            # sqrt might be very large if far from last waypoint, so just keep current vel in case
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp
    
    def pose_cb(self, msg):
        self.pose = msg        

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints 
        # make sure initialized first
       
        if not self.waypoints_2d:
            # only take the 2d coordinates
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
     

    def traffic_cb(self, msg):
#         if self.stopline_wp_idx != msg.data:
#             rospy.logwarn(
#                 "LIGHT: new stopline_wp_idx={}, old stopline_wp_idx={}".format(msg.data, self.stopline_wp_idx))
        self.stopline_wp_idx = msg.data

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
