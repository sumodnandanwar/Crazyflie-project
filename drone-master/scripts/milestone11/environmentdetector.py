#!/usr/bin/env python
from __future__ import print_function

from os.path import expanduser

import numpy as np
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # define range of the color we look for in the HSV space
    bottomRedLower = np.array([int(1/2-1), int(0.3*255), int(0.3*255)])
    bottomRedUpper = np.array([int(20/2-1), int(0.9*255), int(0.9*255)])
    topRedLower = np.array([int(340/2-1), int(0.3*255), int(0.3*255)])
    topRedUpper = np.array([int(360/2-1), int(0.9*255), int(0.9*255)])

    whiteLower = np.array([0, 0, 90]) # white pixel
    whiteUpper = np.array([255, 15, 120]) # white and grey pixels
    
    # Threshold the HSV image to get only the pixels in ranage
    bottomRedMask = cv2.inRange(hsv, bottomRedLower, bottomRedUpper)
    topRedMask = cv2.inRange(hsv, topRedLower, topRedUpper)
    whiteMask = cv2.inRange(hsv, whiteLower, whiteUpper)

    # Create final mask
    redMask = cv2.bitwise_or(bottomRedMask, topRedMask)
    stopSignMask = cv2.bitwise_or(redMask, whiteMask)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image, cv_image, mask= stopSignMask)

    # Check if sign exist and draw circle around it
    if np.count_nonzero(redMask) > 600 and np.count_nonzero(whiteMask) > 500:
      # Find center of stop sign
      signPixels = np.where(redMask==np.amax(redMask))
      x = int(np.median(signPixels[0]))
      y = int(np.median(signPixels[1]))
      
      # Draw circle onto image where sign is
      radius = x - signPixels[0][0]
      cv2.circle(cv_image, (y, x), radius, 255, 10) # Original image
      # cv2.circle(res, (y, x), radius, 255, 10) # Filtered image

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8")) # Original image
      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8")) # Filtered image
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('environmentdetector', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)