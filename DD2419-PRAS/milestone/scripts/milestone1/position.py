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

currentPose = None
goal = None

def setupGoal(msg):
    global goal
    goal = msg

def updateCurrentPose(msg):
    global currentPose
    currentPose = msg.pose
    # rospy.loginfo(currentPose)



def getCmdFromPose():
    global currentPose
    # rospy.loginfo(currentPose)
    while currentPose == None:
        continue
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "cf1/odom"

    cmd.x = currentPose.position.x
    cmd.y = currentPose.position.y
    cmd.z = currentPose.position.z
    cmd.z = 0.3

    _,_,yaw = euler_from_quaternion((currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w))
    cmd.yaw = math.degrees(yaw)

    return cmd

    
def experiment():
    rate = rospy.Rate(10)
    cmd = getCmdFromPose()

    count = 0
    while not rospy.is_shutdown():
        if count < 30:
            count+=1
            # cmd.x += 0.3
        elif count < 50:
            count+=1
            cmd.x = 0.8
        elif count < 80:
            count+=1
            cmd.yaw = 180
        elif count < 100:
            count += 1
            cmd.x = 0.5
        else:
            pub_stop.publish(Empty())
            break
        rospy.loginfo(cmd)
        pub_cmd.publish(cmd)
        rate.sleep()

def keys():
    global goal
    rate = rospy.Rate(10)
    cmd = Position()
    cmd.x = 0.5
    cmd.y = 0.5
    cmd.z = 0.3
    cmd.yaw = 0.0
    while not rospy.is_shutdown():
        if goal:
            pub_cmd.publish(goal)
        else:
            pub_cmd.publish(cmd)
        
        rate.sleep()
        
rospy.init_node("drone_position",anonymous=True)

pose_sub = rospy.Subscriber("/cf1/pose",PoseStamped,updateCurrentPose)
sub = rospy.Subscriber("keyControl",Position,callback=setupGoal)

pub_cmd = rospy.Publisher("/cf1/cmd_position",Position,queue_size=1)
pub_stop = rospy.Publisher('stop', Empty, queue_size=1)



if __name__ == '__main__':
    try:
        # keys() for keyboard control
        keys()
        # experiment for 
        # experiment()
    except rospy.ROSInterruptException:
        pass



    