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
from crazyflie_gazebo.msg import Hover
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import math
import readchar
from std_msgs.msg import Empty


rospy.init_node("keyControl",anonymous=True)

pub_cmd = rospy.Publisher("keyControl",Position,queue_size=1)
pub_stop = rospy.Publisher('stop', Empty, queue_size=1)

if __name__ == '__main__':
    rate = rospy.Rate(10)
    cmd = Position()
    cmd.x = 0.5
    cmd.y = 0.5
    cmd.z = 0.3
    while not rospy.is_shutdown():
        k = readchar.readkey()
        k = k.lower()

        if k == "a":
            cmd.x -= 0.1
        elif k == "d":
            cmd.x += 0.1
        elif k == "w":
            cmd.y += 0.1
        elif k == "s":
            cmd.y -= 0.1
        elif k == "1":
            cmd.yaw += 90
        elif k == "2":
            cmd.yaw -= 90
        elif k== "3":
            cmd.z += 0.2
        elif k== "4":
            cmd.z -= 0.2
        elif k == 'q':
            pub_stop.publish(Empty())
            break
        pub_cmd.publish(cmd)
        rate.sleep()
