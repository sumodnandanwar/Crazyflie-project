#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty

state = 0
angle = 0
thr = 0.1
# Current pose (global state)
#pose = None


""" takeoff_cmd = Position()
takeoff_cmd.header.frame_id = 'cf1/odom'
takeoff_cmd.x = 0
takeoff_cmd.y = 0
takeoff_cmd.z = 0.4


Rotate_cmd = Position()
Rotate_cmd.header.frame_id = 'cf1/odom'   
Rotate_cmd.x = 0
Rotate_cmd.y = 0
Rotate_cmd.z = 0.5
Rotate_cmd.yaw = 0 """

def originn(cu_pose):
    global origin

    origin = Position()
    origin.header.frame_id = 'cf1/odom'
    origin.x = cu_pose.pose.position.x
    origin.y = cu_pose.pose.position.y
    origin.z = 0.4
    origin.yaw = 000
    # origin = pose
    return origin

def Rotate_callback(origin):
    global state, angle, Rotate_cmd

    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 5 
    Rotate_cmd.x = origin.x
    Rotate_cmd.y = origin.y
    Rotate_cmd.z = origin.z
    Rotate_cmd.yaw = angle
    # rospy.loginfo('Rotating now angle:\n%s',Rotate_cmd.yaw)
    # pub_cmd.publish(Rotate_cmd)
    
    if angle == 360:
        angle = 0
        state = 3
    return Rotate_cmd
    
def dist(myposex, myposey, goalx, goaly):
    dis = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2)
    return dis
        
def state_machine(cf1_pose):
    global cu_pose, state
    cu_pose = cf1_pose
    # xval = round(cu_pose.pose.position.x,1)    
    # yval = round(cu_pose.pose.position.y,1)
    zval = round(cu_pose.pose.position.z,1)
    # print xval, yval, zval
     
    if zval < 0.15:
        state = 1
        print zval
    if  zval == 0.4:#dist(xval, yval,0.0,0.0) < thr and 
        state = 2
    

    # if angle == 360:
    #     angle = 0
    #     state = 3

rospy.init_node('Takeoff_n_Scout')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, state_machine)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

def main():
    rate = rospy.Rate(10)  # Hz
    print ('main')
    while not rospy.is_shutdown():
        if state == 1:
            originn(cu_pose)
            pub_cmd.publish(origin)
            print state
        elif state == 2:
            Rotate_callback(origin)
            pub_cmd.publish(Rotate_cmd)
            print angle
        elif state == 3:
            rospy.signal_shutdown()

        rate.sleep()

if __name__ == '__main__':
    main()