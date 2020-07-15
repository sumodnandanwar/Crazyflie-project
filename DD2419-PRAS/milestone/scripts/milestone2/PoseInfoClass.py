#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 25 11:25:09 2020

@author: Fredrik Forsberg
"""

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class PoseInfoCLass:
    def __init__(self, pose_subscription_topic='/cf1/pose'):
        self.pose = PoseStamped()
        self.pose_listener = rospy.Subscriber(pose_subscription_topic, PoseStamped, self.pose_callback)
        
        
    def pose_callback(self, pose):
        self.pose = pose
        
        
    def get_angles(self):
        # Roll, Pitch, Yaw
        return euler_from_quaternion((self.pose.pose.orientation.x,
                                      self.pose.pose.orientation.y,
                                      self.pose.pose.orientation.z,
                                      self.pose.pose.orientation.w))
        
        
    def get_position(self):
        return self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z
    
    
    def get_pose_info(self):
        return self.get_position(), self.get_angles()
        