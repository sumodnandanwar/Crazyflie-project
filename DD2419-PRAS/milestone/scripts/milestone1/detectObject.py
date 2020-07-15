#! /usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from cv2 import aruco
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from crazyflie_gazebo.msg import Position
import matplotlib.pyplot as plt

class image_converter:

  def __init__ (self):
    self.image_pub = rospy.Publisher ("/myresult", Image, queue_size = 2)

    self.bridge = CvBridge ()
    self.image_sub = rospy.Subscriber ("/cf1/camera/image_raw", Image, self.callback)

    self.count = 0

  def callback (self, data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2 (data, "bgr8")
    except CvBridgeError as e:
      print (e)

    cv2.imwrite("img.bmp",cv_image)

    # define range of the color we look for in the HSV space
    frame_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    frame_threshold = cv2.inRange(frame_HSV, (0, 56, 175), (214, 201, 236))

    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(frame_threshold, connectivity=8)
    #connectedComponentswithStats yields every seperated component with information on each of them, such as size
    #the following part is just taking out the background which is also considered a component, but most of the time we don't want that.
    sizes = stats[1:, -1]; nb_components = nb_components - 1

    # minimum size of particles we want to keep (number of pixels)
    #here, it's a fixed value, but you can set it as you want, eg the mean of the sizes or whatever
    min_size = 2800  

    #your answer image
    img2 = np.zeros((output.shape),np.uint8)
    #for every component in the image, you keep it only if it's above min_size
    for i in range(0, nb_components):
        if sizes[i] >= min_size:
            img2[output == i + 1] = 255
            self.count +=1
            # print("find image")
            break
    
    if self.count == 10:
          print("find image")
          self.count = 0


    # cv2.imwrite("img_test.jpg",cv_image)

    # # Threshold the HSV image to get only the pixels in ranage
    # mask = cv2.inRange (cv_image, lower, upper)
    # # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    # Convert BGR to gray
    # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # _,thresh = cv2.threshold(gray,150,200,cv2.THRESH_BINARY_INV)
    # plt.imshow(thresh,cmap="gray"),plt.show()

    # cv2.imwrite("test_img.jpg",cv_image)

    # aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # parameters =  aruco.DetectorParameters_create()
    # corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # frame_markers = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
    
    # for i in range(len(ids)):
    #     c = corners[i][0]
    #     print("my id number: ",ids[i],c[:, 0].mean(), c[:, 1].mean())

    # # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img2, "mono8"))
    except CvBridgeError as e:
      print (e)

def detectX(args):
  rospy.init_node ('detectX', anonymous = True)

  ic = image_converter()

  print( "Running ...")
  try:
    rospy.spin ()
  except KeyboardInterrupt:
    print ("Shutting down")

  cv2.destroyAllWindows ()



if __name__ == '__main__':
    detectX(sys.argv)
    