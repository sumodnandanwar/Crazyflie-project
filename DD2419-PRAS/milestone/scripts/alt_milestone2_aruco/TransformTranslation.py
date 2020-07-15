#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 13:20:13 2020

@author: Fredrik Forsberg
"""

import rospy
import tf2_ros
import tf2_geometry_msgs  # Needed
from geometry_msgs.msg import TransformStamped, PoseStamped

###


class TransformTranslation:
    def __init__(self, node_name):
        # Create node
        rospy.init_node(node_name)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
        self.number_of_aruco_markers = 10
        
        self.last_timestamp = 0
        self.latest_transform = None
        self.latest_map_to_odom = None
        
        
    def run(self):
        rate = rospy.Rate(10)  # Hz
        
        while not rospy.is_shutdown():
            
            detected_objects = []
            
            for i in range(0, self.number_of_aruco_markers):
                if self.tf_buf.can_transform("cf1/odom", "aruco/detected" + str(i), rospy.Time(0.0)):
                    detected_objects.append(self.tf_buf.lookup_transform("aruco/detected" + str(i), "cf1/odom", 
                                                                         rospy.Time(0.0)))  # TransformStamped
                    
            if detected_objects:
                
                timestamps = [rospy.Time.to_sec(t.header.stamp) for t in detected_objects]
                
                if max(timestamps) > self.last_timestamp:
                    self.latest_transform = detected_objects[timestamps.index(max(timestamps))]
                    
                    
            # Publish the transform in multiple steps
            if self.latest_transform is not None:
                aruco_marker_name = ('aruco/marker' + 
                                     self.latest_transform.header.frame_id.replace("aruco/detected", ""))
                # Link the ArUco marker in the map frame and the odom frame
                marker_to_odom_pose = PoseStamped()
                marker_to_odom_pose.header.frame_id = aruco_marker_name
                marker_to_odom_pose.pose.position= self.latest_transform.transform.translation
                marker_to_odom_pose.pose.orientation = self.latest_transform.transform.rotation
                
                # Test if it is possible to transform from map to the ArUco marker
                if self.tf_buf.can_transform("map", aruco_marker_name, rospy.Time(0.0)):
                    
                    map_to_odom_pose = self.tf_buf.transform(marker_to_odom_pose, "map")
                    
                    transform = TransformStamped()
                    transform.header.stamp = self.latest_transform.header.stamp
                    transform.header.frame_id = "map"
                    transform.child_frame_id = "cf1/odom"
                
                    transform.transform.translation = map_to_odom_pose.pose.position
                    transform.transform.rotation = map_to_odom_pose.pose.orientation
                    
                    self.tf_broadcaster.sendTransform(transform)
                    
                    self.latest_map_to_odom = transform
                
            # TODO Kalman filter if the time since the last grabbed transform is too long
            
            rate.sleep()
        
        # rospy.spin() shouldn't be needed since it sleeps until rospy.is_shutdown() returns True
        # Keeping it to be on the safe side
        rospy.spin()
    
###
        
    
if __name__ == '__main__':
    transform_translation = TransformTranslation('TransformTranslation')
    
    transform_translation.run()
