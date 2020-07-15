#!/usr/bin/env python
"""
Created on Wednesday Apr 23 11:15:00 2020

@author: Akanshu Mahajan
"""
import json
import sys
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
from numpy import mean

from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped ,TransformStamped
from crazyflie_driver.msg import Position

class Republish:
    
    def __init__(self, motion_model = 'moving_average'):
        
        ## Pull necessary ROS parameters from launch file:
        
        # Read pose message topic
        # param1 = rospy.search_param("pose_topic") 
        # self.pos_msg_topic = rospy.get_param(param1)       # /cf1/cmd_position

        self.pos_msg_topic = '/cf1/pose'       # /cf1/cmd_position

        self.pos_filtered_msg_topic = 'pose_filtered'       #/localize/pose_filtered

        # Read frame id 
        # param3 = rospy.search_param("frame_id")
        # self.frame_id = rospy.get_param(param3)
        self.frame_id = 'cf1/odom'

        # Read covariance values
        # param4 = rospy.search_param("linear_covariance")
        # self.lin_cov = rospy.get_param(param4)
        self.lin_cov = 0.01

        # param5 = rospy.search_param("angular_covariance")
        # self.ang_cov = rospy.get_param(param5)
        self.ang_cov = 0.01
     
        # Initialize callback variables
        self.pos_msg = None
        # Initialize class variables
        self.pose_filtered = None

        # Establish subscription to different messages

         ## Pose Message       
        rospy.Subscriber(self.pos_msg_topic,PoseStamped,self.pos_msg_callback)      
       # Delay briefly for subscriber to find message
        rospy.sleep(2)

        # Establish publisher of converted Twist message
        self.pub = rospy.Publisher(self.pos_filtered_msg_topic, PoseWithCovarianceStamped, queue_size=10)

    def pos_msg_callback(self, pos_msg):
        self.pos_msg = pos_msg

    def moving_ave(self, pos_list, window_size = 10):
        moving_ave_list = pos_list
        avg = 0.0
        # moving_ave_size = window_size
        # print(moving_ave_list)
        # cumsum, moving_aves = [0], []
        avg = mean(moving_ave_list)
        # for t in num:
        #     sum_num = sum_num + t           

        # avg = sum_num / len(num)
        # return avg


        # for i, x in enumerate(moving_ave_list, 1):
        #     cumsum.append(cumsum[i-1] + x)
        #     if i>= moving_ave_size:
        #         moving_average = (cumsum[i] - cumsum[i-moving_ave_size])/moving_ave_size
        #         moving_aves.append(moving_average)

        # print(mean)
        return avg


    def pos_calc_and_pub(self, size = 5):
        x, y, z, x_q, y_q, z_q, w_q = [], [], [], [], [], [], []

        self.pose_filtered = PoseWithCovarianceStamped()
        self.pose_filtered.header.frame_id = self.frame_id
        self.pose_filtered.pose.covariance = self.cov_matrix_build()

        if self.pos_msg is not None:

            for i in range(size):
                x.append(self.pos_msg.pose.position.x)
                y.append(self.pos_msg.pose.position.y)
                z.append(self.pos_msg.pose.position.z)

                x_q.append(self.pos_msg.pose.orientation.x)
                y_q.append(self.pos_msg.pose.orientation.y)
                z_q.append(self.pos_msg.pose.orientation.z)
                w_q.append(self.pos_msg.pose.orientation.w)
            
            x_pos = self.moving_ave(x)
            # x_pos = x_pos[0]
            y_pos = self.moving_ave(y)
            # y_pos = y_pos[0]
            z_pos = self.moving_ave(z)
            # z_pos = z_pos[0]

            x_q_pos = self.moving_ave(x_q)
            # x_q_pos = x_q_pos[0]
            y_q_pos = self.moving_ave(y_q)
            # y_q_pos = y_q_pos[0]
            z_q_pos = self.moving_ave(z_q)
            # z_q_pos = z_q_pos[0]
            w_q_pos = self.moving_ave(w_q)
            # w_q_pos = w_q_pos[0]
            
            self.pose_filtered.pose.pose.position.x = x_pos
            self.pose_filtered.pose.pose.position.y = y_pos
            self.pose_filtered.pose.pose.position.z = z_pos
                
            self.pose_filtered.pose.pose.orientation.x = x_q_pos
            self.pose_filtered.pose.pose.orientation.y = y_q_pos
            self.pose_filtered.pose.pose.orientation.z = z_q_pos
            self.pose_filtered.pose.pose.orientation.w = w_q_pos

            self.pose_filtered.pose.covariance = self.cov_matrix_build()

            self.pub.publish(self.pose_filtered)


            # rate.sleep

        # except:
        #     rospy.loginfo("Pose message not available")

    def cov_matrix_build(self):

        self.cov_matrix = [self.lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, self.lin_cov, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, self.lin_cov, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, self.ang_cov, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, self.ang_cov, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, self.ang_cov]
        return self.cov_matrix

if __name__ == '__main__':
    
    rospy.init_node('pose_filtered', anonymous=True)
    rospy.loginfo("Successful initilization of pose_filtered node")
    p_f = Republish()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        p_f.pos_calc_and_pub(size=5)
        rate.sleep()