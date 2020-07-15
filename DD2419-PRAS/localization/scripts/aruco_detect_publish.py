#!/usr/bin/env python
"""
Created on Wednesday Apr 22 13:14:00 2020

@author: Akanshu Mahajan
"""

#import json
import sys
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
#import geometry_msgs.msg
import nav_msgs.msg

from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover

from std_msgs.msg import Empty
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker

#from binary_map_4n import localization_algo


class ArucoDetect:

    def __init__(self):
        
        ## Pull necessary ROS parameters from launch file:
        
        # Read ArUco message topic
        param1 = rospy.search_param("aruco_topic")
        self.aruco_msg_topic = rospy.get_param(param1)      # /aruco/markers

        # Read aruco_pose topic variable to publish to
        # param2 = rospy.search_param("aruco_pos_topic")
        # self.aruco_pos_topic = rospy.get_param(param2)      # aruco_odom_pose_tf
        
        # Read frames
        param3 = rospy.search_param("camera_frame")
        self.camera_frame = rospy.get_param(param3)
        # param4 = rospy.search_param("base_frame")
        # self.base_frame = rospy.get_param(param4)
        # param5 = rospy.search_param("odom_frame")
        # self.odom_frame = rospy.get_param(param5)
        
        # Read covariance values
        # param6 = rospy.search_param("lin_cov_aruco")
        # self.lin_cov = rospy.get_param(param6)
        # param7 = rospy.search_param("ang_cov_aruco")
        # self.ang_cov = rospy.get_param(param7)

        # Initialize callback variables
        self.detected_markers = None  # Initiates as None
         
        # Establish subscription to different messages
        ##ArUco message
        rospy.Subscriber(self.aruco_msg_topic, MarkerArray, self.aruco_msg_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(0.1)

        # Establish publisher of localization messages
        # self.aruco_pose_pub = rospy.Publisher(self.aruco_pos_topic, PoseStamped, queue_size=10)
        # self.pub = PoseWithCovarianceStamped()
        # self.pub.header.frame_id = self.map_frame
        # self.pub.pose.covariance = self.cov_matrix_build()
        # self.marker_camera_frame = PoseStamped()
        #self.marker_camera_frame.pose.covariance = self.cov_matrix_build()

        # self.marker_base_frame = PoseWithCovarianceStamped()
        # self.marker_base_frame.pose.covariance = self.cov_matrix_build()

        # self.marker_odom_frame = PoseWithCovarianceStamped()
        # self.marker_odom_frame.pose.covariance = self.cov_matrix_build()
         
         ## ArUco Markers transform variables

        self.tf_broadcaster = tf2_ros.TransformBroadcaster() 
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.transform = TransformStamped()
        rospy.sleep(0.1)


    def aruco_msg_callback(self, aruco_msg):
        self.detected_markers = aruco_msg

    def aruco_calc_tf_and_pub(self):
        
        if self.detected_markers is not None:

            transforms = [self.aruco_calc_tf(marker) for marker in self.detected_markers.markers]
            transforms = [trans for trans in transforms if trans is not None]
                
            self.tf_broadcaster.sendTransform(transforms)
            
    def aruco_calc_tf(self, marker):

        # self.marker_camera_frame.header.frame_id = self.camera_frame
        # self.marker_camera_frame.pose.position = marker.pose.pose.position
        # self.marker_camera_frame.pose.orientation = marker.pose.pose.orientation
        
        # # Translate the marker into the base_link frame
        # if not self.tf_buf.can_transform(self.marker_camera_frame.header.frame_id, self.base_frame, rospy.Time(0.0)):
        #     rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % self.marker_camera_frame.header.frame_id)
        #     return

        # self.marker_base_frame = self.tf_buf.transform(self.marker_camera_frame, self.base_frame)   

        # # Translate the marker into the odom frame
        # if not self.tf_buf.can_transform(self.marker_base_frame.header.frame_id, self.odom_frame, self.marker_base_frame.header.stamp):
        #     if not self.tf_buf.can_transform(self.marker_base_frame.header.frame_id, self.odom_frame, rospy.Time(0.0)):
        #         rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % self.marker_base_frame.header.frame_id)
        #         return
        #     else:
        #         odom_transform = self.tf_buf.lookup_transform(self.marker_base_frame.header.frame_id, self.odom_frame, rospy.Time(0.0))
        #         self.marker_base_frame.header.stamp = odom_transform.header.stamp
        # else:
        #     self.marker_base_frame.header.stamp = marker.header.stamp
                
        # rospy.sleep(0.5)                                                                        # Check into both these lines, 135 and 136, they are giving extrapolation error in past and future
        # self.marker_odom_frame = self.tf_buf.transform(self.marker_base_frame,self.odom_frame)
        # self.marker_odom_frame.header.stamp = rospy.Time.now()

        # #self.aruco_pose_pub.publish(self.marker_odom_frame)

        self.transform.header.stamp = rospy.Time.now()
        self.transform.header.frame_id = self.camera_frame
        self.transform.child_frame_id = "aruco/detected" + str(marker.id)
    
        self.transform.transform.translation = marker.pose.pose.position
        self.transform.transform.rotation = marker.pose.pose.orientation
        
        return self.transform

    # def cov_matrix_build(self):
    #     self.cov_matrix = [self.lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                         0.0, self.lin_cov, 0.0, 0.0, 0.0, 0.0,
    #                         0.0, 0.0, self.lin_cov, 0.0, 0.0, 0.0,
    #                         0.0, 0.0, 0.0, self.ang_cov, 0.0, 0.0,
    #                         0.0, 0.0, 0.0, 0.0, self.ang_cov, 0.0,
    #                         0.0, 0.0, 0.0, 0.0, 0.0, self.ang_cov]
    #     return self.cov_matrix

if __name__ == '__main__':
    
    #while not rospy.is_shutdown():

    rospy.init_node('aruco_detect_publish', anonymous=True)
    rospy.loginfo("Successful initilization of node aruco_detect_publish")
    rate = rospy.Rate(8)  # Hz
        
    while not rospy.is_shutdown():

        tf_ar = ArucoDetect()
        tf_ar.aruco_calc_tf_and_pub()

    rate.sleep()
