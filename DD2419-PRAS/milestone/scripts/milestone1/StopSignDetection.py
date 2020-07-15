#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 23:15:51 2020

@author: Fredrik Forsberg
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

###


def get_color_mask(img, hsv_min, hsv_max):
    # # Color detection based on https://henrydangprg.com/2016/06/26/color-detection-in-python-with-opencv/
    # hue = hsv_color[0][0][0]

    # lower_hsv = np.asarray([max(hue - hue_tolerance, 0), lower_sv_threshold, lower_sv_threshold], dtype=np.uint8)
    # upper_hsv = np.asarray([min(hue + hue_tolerance, 179), 255, 255], dtype=np.uint8)
    
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    return cv2.inRange(hsv_img, hsv_min, hsv_max)

#


def bgr2hsv(bgr_color):
    # Returns values in ranges H: 0-179, S: 0-255, V: 0-255
    return cv2.cvtColor(np.array(bgr_color, dtype=np.uint8).reshape(1, 1, 3), cv2.COLOR_BGR2HSV)


###


class StopSignDetection:
    def __init__(self, node_name, raw_image_subscription_topic, image_publishing_topic, mask_publishing_topic, 
                 hsv_min, hsv_max, sign_edges=8, bgr_sign_color=(17, 15, 81), 
                 epsilon_length_modifier=0.03, opencv_imshow=True):
        # Create bridge between ROS and OpenCV
        self.bridge = CvBridge()
        # Create publisher
        self.publisher_img = rospy.Publisher(image_publishing_topic, Image, queue_size=2)
        self.publisher_mask = rospy.Publisher(mask_publishing_topic, Image, queue_size=2)
        
        if type(sign_edges) in [int, float]:
            self.sign_edges = [int(sign_edges)]
        else:  # Assuming tuple or list
            self.sign_edges = sign_edges
        
        self.epsilon_length_modifier = epsilon_length_modifier
        
        # Transforms BGR to HSV
        # hsv_sign_color = bgr2hsv(np.array(list(bgr_sign_color), dtype=np.uint8))
        
        # self.hsv_max = np.asarray([min(hsv_sign_color[0][0][0] + hue_tolerance, 179), 255, 255], dtype=np.float64)
        # self.hsv_min = np.asarray([max(hsv_sign_color[0][0][0] - hue_tolerance, 0), lower_sv_threshold, 
        #                            lower_sv_threshold], dtype=np.float64)
        
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max
        
        self.opencv_imshow = opencv_imshow
        if self.opencv_imshow:
            self.firstrun = True
        
        self.i = 0
        
        # Initiate node
        rospy.init_node(node_name)
        #Subscribe to image topic
        rospy.Subscriber(raw_image_subscription_topic, Image, self.callback)
        
        # Keep python from exiting
        try:
            rospy.spin()
        except KeyboardInterrupt as err:
            print(repr(err))
        # Close OpenCV windows once rospy.spin() release
        cv2.destroyAllWindows()
        
    #
    
    
    def callback(self, ros_img):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_img, 'bgr8')
        except CvBridgeError as err:
            print(repr(err))
            return
        
        drawn_img, mask = self.draw_colored_polygons(img, self.sign_edges, 
                                                     hsv_min=self.hsv_min, hsv_max=self.hsv_max, 
                                                     blur_kernel_size=5, morph_kernel_size=5, 
                                                     epsilon_length_modifier=self.epsilon_length_modifier, 
                                                     line_color=(0, 255, 0),  line_width=3)
        
        drawn_mask = np.full(drawn_img.shape, 255, dtype=np.uint8)
        drawn_mask = cv2.bitwise_and(drawn_img, drawn_mask, mask=mask)
        
        try:
            self.publisher_img.publish(self.bridge.cv2_to_imgmsg(drawn_img, "bgr8"))
            self.publisher_mask.publish(self.bridge.cv2_to_imgmsg(drawn_mask, "bgr8"))
        except CvBridgeError as err:
            print(repr(err))
            
        if self.opencv_imshow:
            cv2.imshow('img', img)
            cv2.imshow('mask', mask)
            if self.firstrun:
                self.firstrun = False
                cv2.createTrackbar("H min", "mask" , int(self.hsv_min[0]), 179, self.trackbar_h_min)
                cv2.createTrackbar("H max", "mask" , int(self.hsv_max[0]), 179, self.trackbar_h_max)
                cv2.createTrackbar("S min", "mask" , int(self.hsv_min[1]), 255, self.trackbar_s_min)
                cv2.createTrackbar("S max", "mask" , int(self.hsv_max[1]), 255, self.trackbar_s_max)
                cv2.createTrackbar("V min", "mask" , int(self.hsv_min[2]), 255, self.trackbar_v_min)
                cv2.createTrackbar("V max", "mask" , int(self.hsv_max[2]), 255, self.trackbar_v_max)
                # cv2.setMouseCallback('img', self.on_mouse_click, img)
            key = cv2.waitKey(1)
            if key == ord('s'):
                cv2.imwrite('/home/ff/Pictures/img2.jpg', img)
    
    #
                
                
    def draw_colored_polygons(self, img, n_edges_list, hsv_min, hsv_max, 
                              blur_kernel_size=5, morph_kernel_size=5, epsilon_length_modifier = 0.03, 
                              line_color=(0, 255, 0), line_width=3, min_size=2800):
        # Blur image
        blurred_img = cv2.GaussianBlur(img, (blur_kernel_size, blur_kernel_size), 0)
        
        # Get a mask based on a colour (within a set tolerance)
        mask = get_color_mask(blurred_img, hsv_min, hsv_max)
        
        # Morphological opening and closing
        kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
        # Open
        morphed_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # Close
        morphed_mask = cv2.morphologyEx(morphed_open, cv2.MORPH_CLOSE, kernel)
        
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(morphed_mask, connectivity=8)
        sizes = stats[1:, -1]
        nb_components = nb_components - 1
        
        mask2 = np.zeros((output.shape), np.uint8)
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                mask2[output == i + 1] = 255
        #         self.count += 1
        #         break
        
        # if self.count == 10:
        #       print("find image")
        #       self.count = 0
        
        # Find contours
        try:
            _, contours, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        except ValueError:
            contours, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # print(contour.shape)  # [~70, 1, 2]
            subimage_min = contour.min(axis=0)[0]
            subimage_max = contour.max(axis=0)[0]
            margin = 10
            subimage = img[max(subimage_min[1] - margin, 0):min(subimage_max[1] + margin, img.shape[1]), 
                           max(subimage_min[0] - margin, 0):min(subimage_max[0] + margin, img.shape[0]),:]
            cv2.imwrite('/home/ff/Pictures/subimages/%d.jpg' % self.i, subimage)
            self.i += 1
            
            # Polygon contour detection based on https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
            perimiter_length = cv2.arcLength(contour, closed=True)
            approx = cv2.approxPolyDP(contour, perimiter_length * epsilon_length_modifier, closed=True)
            # Check number of approximated corners
            for n_edges in n_edges_list:
                if n_edges != 0:
                    if len(approx) == n_edges:
                        # Draw on img
                        # cv2.drawContours(img, [contour], 0, line_color, line_width)
                        pass
                else:
                     # cv2.drawContours(img, [contour], 0, line_color, line_width)
                    pass
        
        return img, mask2
            
            
    # def on_mouse_click(self, event, x, y, flags, img):
    #     if event == cv2.EVENT_LBUTTONUP:
    #         hsv_color = bgr2hsv(np.array(list(img[y,x]), dtype=np.uint8))[0][0]
    #         self.hsv_min = hsv_color - 5
    #         self.hsv_max = hsv_color + 5
    #         cv2.setTrackbarPos("H min", "mask", int(self.hsv_min[0]))
    #         cv2.setTrackbarPos("H max", "mask", int(self.hsv_max[0]))
    #         cv2.setTrackbarPos("S min", "mask", int(self.hsv_min[1]))
    #         cv2.setTrackbarPos("S max", "mask", int(self.hsv_max[1]))
    #         cv2.setTrackbarPos("V min", "mask", int(self.hsv_min[2]))
    #         cv2.setTrackbarPos("V max", "mask", int(self.hsv_max[2]))
    
    #
    
    
    def trackbar_h_min(self, value):
        self.hsv_min[0] = min(value, self.hsv_max[0] - 1)
        cv2.setTrackbarPos("H min", "mask", int(self.hsv_min[0]))
    
    #
    
    
    def trackbar_h_max(self, value):
        self.hsv_max[0] = max(value, self.hsv_min[0] + 1)
        cv2.setTrackbarPos("H min", "mask", int(self.hsv_min[0]))
    
    #
    
    
    def trackbar_s_min(self, value):
        self.hsv_min[1] = min(value, self.hsv_max[1] - 1)
        cv2.setTrackbarPos("S min", "mask", int(self.hsv_min[1]))
    
    #
    
    
    def trackbar_s_max(self, value):
        self.hsv_max[1] = max(value, self.hsv_min[1] + 1)
        cv2.setTrackbarPos("S max", "mask", int(self.hsv_max[1]))
    
    #
    
    
    def trackbar_v_min(self, value):
        self.hsv_min[2] = min(value, self.hsv_max[2] - 1)
        cv2.setTrackbarPos("V min", "mask", int(self.hsv_min[2]))
    
    #
    
    
    def trackbar_v_max(self, value):
        self.hsv_max[2] = max(value, self.hsv_min[2] + 1)
        cv2.setTrackbarPos("V max", "mask", int(self.hsv_max[2]))

###


if __name__ == '__main__':
    # Set parameters
    # bgr_sign_color=(102, 132, 211)
    # hue_tolerance=25
    sign_edges=(0,)
    # lower_sv_threshold=35
    epsilon_length_modifier=0.03
    
    hsv_min = np.array([0, 35, 35], dtype=np.uint8)
    hsv_max = np.array([20, 255, 255], dtype=np.uint8)
    
    # Launch class
    stop_sign_detection = StopSignDetection('StopSignDetection', '/cf1/camera/image_raw', '/image_stop_sign', 
                                            '/mask_stop_sign', hsv_min, hsv_max,
                                            epsilon_length_modifier=epsilon_length_modifier)
