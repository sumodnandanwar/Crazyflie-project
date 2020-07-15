#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 13:33:31 2020

@author: Fredrik Forsberg
"""

import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo

###


class CameraInfoClass(object):
    def __init__(self, camera_info_subscription_topic='/cf1/camera/camera_info'):
        self.image_height = None
        self.image_width = None
        self.camera_matrix = np.identity(3, dtype=np.float64)
        self.distortion = np.ones((1, 5), dtype=np.float64)
        # self.calibration_camera_matrix = np.identity(3, dtype=np.float64)
        # self.roi = (0, 0, 0, 0)
        
        # Camera info subscription
        rospy.Subscriber(camera_info_subscription_topic, CameraInfo, self.camera_info_callback)
    
    
    def camera_info_callback(self, camera_info):
        self.camera_matrix = np.asarray(camera_info.K, dtype=np.float64).reshape((3, 3))
        self.distortion = np.asarray(camera_info.D, dtype=np.float64)
        self.image_height = camera_info.height
        self.image_width = camera_info.width
        # self.calibration_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion, 
        #                                                                     (self.image_width, self.image_height), 1, 
        #                                                                     (self.image_width, self.image_height))
