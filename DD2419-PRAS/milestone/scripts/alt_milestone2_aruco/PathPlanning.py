#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 12:01:59 2020

@author: Fredrik Forsberg
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

###


# TODO Communication between PathPlanning and FlyControl, sending only each setpoint once


class PathPlanning:
    def __init__(self, node_name, fly_publishing_topic):
        # A list of desired targets
        self.target_list = []
        
        # Create node
        rospy.init_node(node_name)
        
        # Create publisher for outgoing positional commands
        self.fly_control_publisher = rospy.Publisher(fly_publishing_topic, PoseStamped, queue_size=25)
        
        
    def add_target_coords(self, coords):
        # Translates the coordinates to a PoseStamped
        # coords = [x, y, z, yaw, (frame)]
        pose_stamped = PoseStamped()
        
        pose_stamped.header.stamp = rospy.Time.now()
        
        if len(coords) > 4:
            pose_stamped.header.frame_id = coords[4]
        else:
            pose_stamped.header.frame_id = 'map'
        
        pose_stamped.pose.position.x = coords[0]
        pose_stamped.pose.position.y = coords[1]
        pose_stamped.pose.position.z = coords[2]
        
        yaw = np.radians(coords[3])
        
        (pose_stamped.pose.orientation.x,
         pose_stamped.pose.orientation.y,
         pose_stamped.pose.orientation.z,
         pose_stamped.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
        
        self.fly_control_publisher.publish(pose_stamped)  # TODO Can't send just once

###
        
    
if __name__ == '__main__':
    path_planning = PathPlanning('PathPlanning', '/fly_pose')
    
    # This code is specifically for taking off and following an ArUco marker
    
    n = 1  # ArUco marker id to follow
    
    for item in [[0.5, 0.5, 0.4, 0, 'cf1/odom']]: 
                [0.4, 0, 0, np.pi, 'aruco/marker' + str(n)]]:  # x, y, z, yaw, (frame)
        # Add coordinates as map-based poses
        path_planning.add_target_coords(item)
        
    rospy.spin()