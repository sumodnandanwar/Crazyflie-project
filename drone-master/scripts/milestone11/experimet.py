#!/usr/bin/env python

import json
import sys
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Vector3
from crazyflie_driver.msg import Position
from tf2_msgs.msg import TFMessage
from os.path import expanduser


# Global variables:

# Inital State
state = 0

# Distance to travel
X = 0.5

initalize = 0

# Rotations
N = 3
angle = 0
lap = 0

# Origin
origin = None

# First setpoint
setpoint = Position()
setpoint.header.frame_id = 'cf1/odom'
setpoint.x = X
setpoint.y = 0
setpoint.z = 0.4
setpoint.yaw = 0

# Rotation
rotation = Position()
rotation.header.frame_id = 'cf1/odom'
rotation.x = X
rotation.y = 0
rotation.z = 0.4
rotation.yaw = 0


def get_origin(pose):
    global origin
    origin = Position()
    origin.header.frame_id = 'cf1/odom'
    origin.x = pose.pose.position.x
    origin.y = pose.pose.position.y
    origin.z = 0.4
    origin.yaw = 0
    return origin


def publish_cmd(cmd):
    cmd.header.stamp = rospy.Time.now()
    # rospy.loginfo(cmd)
    pub_cmd.publish(cmd)


def rotate_cmd():
    global rotation, state, angle, lap, N, origin

    angle += 2
    rotation.x = setpoint.x
    rotation.y = setpoint.y
    rotation.yaw = angle
    rotation.header.stamp = rospy.Time.now()
    pub_cmd.publish(rotation)

    if angle == 360:
        lap += 1
        print('lap=', lap)
        angle = 0

    if lap == N and angle == 180:
        print('change to origin')
        origin.yaw = angle
        origin.z = 0.2
        state = 3
        angle = 0


def dist(myposex, myposey, markx, marky):
    d = math.sqrt((myposex-markx)**2+(myposey-marky)**2)
    return d


def new_pose(mypose):
    global origin, state, lap, initalize
    if initalize < 1:
        origin = get_origin(mypose)
        initalize += 1
    tol = 0.1
    if dist(mypose.pose.position.x, mypose.pose.position.y, origin.x, origin.y) < tol and (origin.z-mypose.pose.position.z) < tol and lap != N:
        state = 1
        lap = 0
    elif dist(mypose.pose.position.x, mypose.pose.position.y, setpoint.x, setpoint.y) < tol and lap == 0:
        state = 2


rospy.init_node('experimentTest')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, new_pose)
pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)


def main():
    rate = rospy.Rate(10)  # Hz

    # rospy.sleep(.5)
    while not rospy.is_shutdown():
        if origin:
            setpoint.x = origin.x + X
            setpoint.y = origin.y
            # print(state)
            if state == 0:
                publish_cmd(origin)
            elif state == 1:
                publish_cmd(setpoint)
            elif state == 2:
                rotate_cmd()
            elif state == 3:
                publish_cmd(origin)
            rate.sleep()


if __name__ == '__main__':
    main()
