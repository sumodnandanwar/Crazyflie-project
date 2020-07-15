#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 23:15:51 2020

@author: Fredrik Forsberg
"""

import rospy
import tf2_ros
import tf2_geometry_msgs  # Needed
from geometry_msgs.msg import PoseStamped, TransformStamped
from aruco_msgs.msg import MarkerArray

###


class ArUcoDetect:
    def __init__(self, node_name, aruco_markers_subscription_topic):
        # Initiate node
        rospy.init_node(node_name)
        
        self.detected_markers = None  # Initiates as None
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        
        #Subscribe to image topic
        rospy.Subscriber(aruco_markers_subscription_topic, MarkerArray, self.markers_callback)
    
    #
    
    
    def markers_callback(self, markers):
        self.detected_markers = markers
     
    #
    
    
    def run(self):
        rate = rospy.Rate(10)  # Hz
        
        while not rospy.is_shutdown():
            
            if self.detected_markers is not None:
                
                transforms = [self.transform_from_marker(m) for m in self.detected_markers.markers]
                transforms = [t for t in transforms if t is not None]
                
                self.tf_broadcaster.sendTransform(transforms)
                
            rate.sleep()
            
        # rospy.spin() shouldn't be needed since it sleeps until rospy.is_shutdown() returns True
        # Keeping it to be on the safe side
        rospy.spin()
    
    #
    

    def transform_from_marker(self, m):
        m_camera = PoseStamped()
        m_camera.header.frame_id = "cf1/camera_link"
        m_camera.pose.position= m.pose.pose.position
        m_camera.pose.orientation = m.pose.pose.orientation
        
        # Translate the marker into the base_link frame
        if not self.tf_buf.can_transform(m_camera.header.frame_id, 'cf1/base_link', rospy.Time(0.0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % m_camera.header.frame_id)
            return
    

        m_base = self.tf_buf.transform(m_camera, 'cf1/base_link')
        
        # Translate the marker into the odom frame
        if not self.tf_buf.can_transform(m_base.header.frame_id, 'cf1/odom', m_base.header.stamp):
            if not self.tf_buf.can_transform(m_base.header.frame_id, 'cf1/odom', rospy.Time(0.0)):
                rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % m_base.header.frame_id)
                return
            else:
                odom_transform = self.tf_buf.lookup_transform(m_base.header.frame_id, "cf1/odom", rospy.Time(0.0))
                m_base.header.stamp = odom_transform.header.stamp
        else:
            m_base.header.stamp = m.header.stamp
                
    
        m_odom = self.tf_buf.transform(m_base,'cf1/odom')
        
        transform = TransformStamped()
        transform.header.stamp = odom_transform.header.stamp
        transform.header.frame_id = "cf1/odom"
        transform.child_frame_id = "aruco/detected" + str(m.id)
    
        transform.transform.translation = m_odom.pose.position
        transform.transform.rotation = m_odom.pose.orientation
        
        return transform
        

###
        

if __name__ == '__main__':
    aruco_transform_broadcaster = ArUcoDetect('ArUcoDetect', '/aruco/markers')
	
    aruco_transform_broadcaster.run()
